/*
 * arch/arm/plat-oxnas/backup.c
 *
 * Copyright (C) 2010 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <net/tcp.h>
#include <mach/oxnas_errors.h>
#include <mach/sata_helper.h>
#include <mach/oxnas_direct_disk.h>

#define SATA_ACQUIRE_TIMEOUT_JIFFIES (3*HZ)

/* These have to be a factor of FS Block Size */
#define NET_RX_CHUNK_SIZE 128*1024

/* It is assumed that the file will have 16 extents - if it grows more than
 * this, num of extents will be used accordingly
 */
#define FILE_SUB_EXTENTS 16

#define BMV_IF_PREALLOC	0x4 /* rtn status BMV_OF_PREALLOC if req */

int oxnas_init_backup(struct file *fp)
{
	struct socket               *socket = fp->private_data;
	oxnas_direct_disk_context_t *context;
	oxnas_net_rx_context_t      *net_rx_context;
	struct kmem_cache           *frag_cache;
	int                          retval = 0;
	int                          i;

	if (fp->backup_context) { /* duplicate call - ignore it */
		printk(KERN_INFO "Initialisation called more than once \n");
		goto out;
	}

	context = kzalloc(sizeof(oxnas_direct_disk_context_t), GFP_KERNEL);
	if (unlikely(!context)) {
		retval = -ENOMEM;
		printk(KERN_INFO "Alloc memory Socket context failed\n");
		goto out;
	}

	atomic_set(&context->sata_transfer_in_progress, 0);
	context->need_sata_completion_notification = 0;
	context->discontinuous_write = 0;
	context->fs_blocklog = 0;
	context->buffer = NULL;
	context->buffer_pa = 0;

	spin_lock_init(&context->sata_completion_lock);
	sema_init(&context->sata_completion_sem, 1);
	atomic_set(&context->free_sg, 0);
	
	net_rx_context = &context->net_rx_context;
	
	/* -2 to accnt for possible first and last partial FSB's
	 * -1 to accnt for 0 offset
	 */
	net_rx_context->max_frag_cnt = CONFIG_ODRB_NUM_SIMPLE_SG_INFOS - 3;

	/* Initialise the zero-copy context and sg index variables*/
	for (i=0; i < NUM_NET_RX_FRAG_LISTS; i++) {
		INIT_LIST_HEAD(&net_rx_context->page_info[i]);
		net_rx_context->frag_cnt[i] = 0;
	}

	/* Construct unique name for the fragment cache for this socket */
	snprintf(context->frag_cache_name, sizeof(context->frag_cache_name), "FRG%p", socket);
	frag_cache = kmem_cache_create(context->frag_cache_name,
		sizeof(frag_list_entry_t), 0, 0, frag_list_entry_ctor);

	if (unlikely(!frag_cache)) {
		printk (KERN_INFO "Frag Cache creation failed \n");
		retval = -ENOMEM;
		goto context_out;
	}

	BUG_ON(odrb_alloc_prd_array(&context->prd_list[0], NUM_NET_RX_FRAG_LISTS, 1));

	net_rx_context->frag_cache = frag_cache;
	/* Don't want any fragments releasing until 1st SATA write has completed */
	net_rx_context->release_frag_list_idx = -2;

	fp->backup_context = context;

out:
	return retval;

context_out:
	kfree(context);
	goto out;
}
EXPORT_SYMBOL(oxnas_init_backup);

int oxnas_exit_backup(struct file *fp)
{
	oxnas_direct_disk_context_t	*context =
		(oxnas_direct_disk_context_t *)fp->backup_context;
	oxnas_net_rx_context_t *net_rx_context = &context->net_rx_context;
	int retVal = 0;
	int i = 0;

	/* Wait for the last SATA transfer to finish before freeing the network
		   fragment storage */
	oxnas_direct_disk_complete(context);

	/* Free network fragments used in final SATA transfer */
	release_netdma_net_frags(net_rx_context);

	for (i = 0; i < NUM_NET_RX_FRAG_LISTS; i++) {
		if (context->prd_list[i] != NULL) {
			odrb_free_prd_array(context->prd_list[i]);
			context->prd_list[i] = NULL;
		}
	}

	/* Free the network fragment storage */
	kmem_cache_destroy(net_rx_context->frag_cache);

	if (context->buffer) 
		dma_free_coherent(0, sizeof(char) << context->fs_blocklog, context->buffer, context->buffer_pa);
	kfree(context);

	fp->backup_context = NULL;
	return retVal;
}
EXPORT_SYMBOL(oxnas_exit_backup);

static int backup_disk_write(
	oxnas_direct_disk_context_t *context,
	oxnas_filemap_offset_t      *filemap_offset,
	ssize_t                      length,
	int                          port,
	sector_t                     part_offset)
{
	oxnas_net_rx_context_t *net_rx_context = &context->net_rx_context;
	getbmapx_t             *map = context->map;
	frag_list_entry_t      *frag;
	unsigned int            frag_offset = 0;
	ssize_t                 remaining = length;

	/* Get the first network Rx data fragment */
	frag = (frag_list_entry_t*)container_of(
		net_rx_context->page_info[net_rx_context->fill_frag_list_idx].next,
		frag_list_entry_t, head);

	do {
		getbmapx_t *map_entry;
		long long   map_start;
		long long   map_len;
		long long   map_offset;
		sector_t    lba;
		long long   contig_len;
		long long   remaining_contig_len;
		prd_table_entry_t *prd = NULL;
		long long   total_len = 0;
		int         zero_length = 0;

		map_entry = &map[filemap_offset->cur_map_entry];
		map_offset = filemap_offset->cur_map_entry_offset;

		map_start  = map_entry->bmv_offset;
		map_len    = map_entry->bmv_length;

		/* Find the next block to write */
		lba = map_entry->bmv_block + map_offset + part_offset;

		/* Calculate remaining contiguous disk bytes at offset into mapping */
		contig_len = (map_len - map_offset) << SECTOR_SHIFT;
		if (contig_len > remaining) {
			contig_len = remaining;
			filemap_offset->cur_map_entry_offset += (contig_len >> SECTOR_SHIFT);
		} else {
			++filemap_offset->cur_map_entry;
			filemap_offset->cur_map_entry_offset = 0;
		}

		// Get hold of pointer to the first in the array of dedicated DMA PRDs
		if (context->prd_list[net_rx_context->fill_frag_list_idx] == NULL) {
			BUG_ON(odrb_alloc_prd_array(&context->prd_list[net_rx_context->fill_frag_list_idx], 1, 1));
		}
		prd = context->prd_list[net_rx_context->fill_frag_list_idx]->prds;

		/* Fill SG DMA descriptors with enough network Rx data to fill the
		   contiguous disk sectors */
		remaining_contig_len = contig_len;
		do {
			unsigned int frag_remaining_len;
			unsigned int len;

			frag_remaining_len = frag->bio_vec.bv_len - frag_offset;
			len = (frag_remaining_len > remaining_contig_len) ?
					remaining_contig_len : frag_remaining_len;

			// Fill the PRD with the DMAable address and length of network data
			prd->adr = virt_to_dma(0, page_address(frag->bio_vec.bv_page) +
				frag->bio_vec.bv_offset + frag_offset);
			prd++->flags_len = len;

			total_len += len;
			frag_offset += len;
			remaining_contig_len -= len;

			if (len == frag_remaining_len) {
				if (frag->head.next) {
					frag = (frag_list_entry_t*)container_of(frag->head.next,
						frag_list_entry_t,	head);
					frag_offset = 0;
				} else {
					BUG_ON(remaining_contig_len);
				}
			}
		} while (remaining_contig_len);

		/* partial last block handling */
		if (contig_len -
			((contig_len >> context->fs_blocklog) << context->fs_blocklog)) {

			zero_length = (1 << context->fs_blocklog) -
				(contig_len - ((contig_len >> context->fs_blocklog)
					<< context->fs_blocklog));

			/* need not do a memset of the buffer as its set to zero on creation */
			// Fill the SG entry with the DMAable address and length
			prd->adr = context->buffer_pa;
			prd++->flags_len = zero_length;
		}

		// Terminate the SG list
		(--prd)->flags_len |= PRD_EOF_MASK;

		// Wait for our last SATA transfer to complete
		fast_writes_wait_sata_complete(context);

		/* Busy wait for ownership of the SATA core from the Linux SATA stack */
		while (!acquire_sata_core_may_sleep(fast_writes_isr, (int)context,
			SATA_ACQUIRE_TIMEOUT_JIFFIES)) {
			printk(KERN_ERR "Sata acquire failed - return error \n");
//			return OXERR_DISKIO;
		}

		/* Setup SG DMA transfer */
		odrb_dma_sata_prd(OXNAS_DMA_TO_DEVICE, (total_len + zero_length) >> SECTOR_SHIFT,
			context->prd_list[net_rx_context->fill_frag_list_idx], 1);

		/* How much still to be written to SATA after transfer completes? */
		remaining -= contig_len;

		/* Need to wait for SATA to complete as we shall be re-using the same
		   DMA SG entry and fragment lists for subsequent transfer to next
		   contiguous region of disk. No need to lock access to flag as we know
		   there's no SATA transfer in progress and therefore the ISR cannot
		   get in */
		context->need_sata_completion_notification = remaining;
		atomic_set(&context->sata_transfer_in_progress, 1);

		context->cur_transfer_idx = net_rx_context->fill_frag_list_idx;
		atomic_set(&context->free_sg, 0);

		/* Issue SATA command to write contiguous sectors to disk */
		direct_sata_transfer(C_WRITE_DMA_EXT, port, lba,
			(contig_len + zero_length) >> SECTOR_SHIFT);

		if (context->need_sata_completion_notification) {
			// Wait for ISR to signal SATA transfer finished */
			while (down_interruptible(&context->sata_completion_sem));
		}

	} while (remaining);

	/* Update fill and release indices */
	update_context_indices(net_rx_context);

	BUG_ON(remaining);
	return length - remaining;
}

int do_xfsbackup(
	struct file	*in_fp,
	struct file	*fp,
    loff_t		 count)
{
	struct socket 				*socket = in_fp->private_data;
	oxnas_direct_disk_context_t	*context = (oxnas_direct_disk_context_t *)in_fp->backup_context;
	oxnas_net_rx_context_t      *net_rx_context = &context->net_rx_context;
	oxnas_filemap_info_t         filemap_info;
	oxnas_filemap_offset_t 		 filemap_offset;
	read_descriptor_t            desc;
	int                          finished;
	ssize_t                      received_from_net;
	ssize_t                      chunk_size = NET_RX_CHUNK_SIZE;
	loff_t                       total_transfered = 0;
	loff_t						 remaining_to_receive = 0;
	loff_t						 loop_to_receive = 0;
	int                          read_result = 0;
	int                          write_result = 0;
	struct inode                *inode;
	struct block_device         *bd;
	struct hd_struct            *partition;
	struct gendisk              *disk;
	getbmapx_t	  				*map;

	/* retrieve the file map from the fs */
	{
		int num_subexts = FILE_SUB_EXTENTS;
		loff_t total_file_size = count;
		int once = 1;

		read_result = fp->f_op->preallocate(fp, 0, total_file_size);
		if (unlikely(read_result < 0 )) {
			/* error on preallocate */
			printk(KERN_ERR "ERROR - PREALLOCATING SPACE - %d\n", read_result);
			goto out;
		}

		/* Truncate of the file has been removed from here. Without this, the
		 * file size will get updated only after the entire file is written 
		 * and so other readers cannot read it until write is completed
		 */

		map = kmalloc(sizeof(getbmapx_t) * (num_subexts + 1), GFP_KERNEL);
		if (unlikely(!map)) {
			printk(KERN_ERR "allocating file map failed\n");
			read_result = -ENOMEM;
			goto out;
		}

try_again:
		memset(map, 0, sizeof(getbmapx_t) * (num_subexts + 1));

		map->bmv_length = -1;
		map->bmv_count = num_subexts + 1;
		map->bmv_iflags = BMV_IF_PREALLOC;

		read_result = fp->inode->i_op->getbmapx(fp->inode, map);
		if (unlikely(read_result < 0)) {
			printk(KERN_ERR "retrieving file map failed - %d\n", read_result);
			goto out;
		}

		/* check whether we need more space */
		if ((map->bmv_entries == num_subexts) && once) {
			once = 0;
			/* retrieve the number of file extents */
			num_subexts = fp->inode->i_op->get_extents(fp->inode, 0);

			printk(KERN_INFO "Number of map entries - %d\n", num_subexts);

			map = krealloc(map, sizeof(getbmapx_t) * (num_subexts + 1), GFP_KERNEL);
			if (!map) {
				printk(KERN_ERR "Re-allocating file map failed\n");
				read_result = -ENOMEM;
				goto out;
			}
#ifdef DEBUG
			printk(KERN_INFO "Before trying again \n");
#endif
			goto try_again;
		}

#ifdef DEBUG
		printk(KERN_INFO "Number of map entries - %ld\n", map[0].bmv_entries);
		/* print the new file map info */
		{
			int temp = map[0].bmv_entries;
			getbmapx_t *map_entry = &map[1];

			while (temp) {
				printk (KERN_INFO "map - %ld, map start - %lld, map length - %lld\n",
						(map[0].bmv_entries - temp + 1), map_entry->bmv_block,
						map_entry->bmv_length);
				temp --;
				if (temp) map_entry ++;
			}
		}
#endif
	}

	if (context->fs_blocklog == 0) {
		context->fs_blocklog = oxnas_get_fs_blocksize(fp);
	}
	
	if (context->buffer == NULL) {
		context->buffer = dma_alloc_coherent(0, sizeof(char) << context->fs_blocklog, &context->buffer_pa, GFP_KERNEL);
		if (unlikely(!context->buffer)) {
			read_result = -ENOMEM;
			printk(KERN_ERR "allocating context.buffer failed\n");
			goto out;
		}

		memset(context->buffer, 0, sizeof(char) << context->fs_blocklog);
	}

	memset(&filemap_offset, 0, sizeof(oxnas_filemap_offset_t));
	memset(&filemap_info, 0, sizeof(filemap_info));
	filemap_info.map = &map[1];
	context->map = &map[1];

	/* Get block offset of the partition containing the output file */
	inode = fp->f_dentry->d_inode;
	BUG_ON(!inode);
	bd = bdget(MKDEV(MAJOR(inode->i_sb->s_dev), MINOR(inode->i_sb->s_dev)));
	BUG_ON(!bd);
	partition = bd->bd_part;
	if (!partition) {
		bd = bdget(MKDEV(8, MINOR(inode->i_sb->s_dev)));
		partition = bd->bd_part;
	}
	BUG_ON(!partition);
	disk = bd->bd_disk;
	BUG_ON(!disk);

//	filemap_info.port = MINOR(inode->i_sb->s_dev) / disk->minors;
	filemap_info.port = 0;
	filemap_info.part_offset = partition->start_sect;
	context->port = filemap_info.port; 

	desc.arg.data = net_rx_context;

	finished = 0;
	remaining_to_receive = count;
	do {
		/* Initialise the descriptor that will track progress of acquiring
		   network page references */
		desc.written = 0;
		desc.error = 0;

		loop_to_receive = chunk_size;

		if (remaining_to_receive < chunk_size) {
			loop_to_receive = remaining_to_receive;
		}

		/* Acquire references to the pages from the network */
		received_from_net = 0;
		while (loop_to_receive > 0) {
			desc.count = loop_to_receive;

			read_result = oxnas_net_read_sock(socket->sk, &desc, oxnas_net_rx_actor, 1);
			if ((read_result < 0) && (read_result != -EAGAIN)) {
#ifdef DEBUG
				printk(KERN_ERR "network read returned error - %d\n", read_result);
#endif
				read_result = OXERR_NETWORK;
				finished = 1;
				break;
			}
#ifdef DEBUG
			if (read_result != -EAGAIN) {
				printk(KERN_ERR "NETWORK READ RET - EAGAIN\n");
			}
#endif
			received_from_net += read_result;
			loop_to_receive -= read_result;
		
			if (unlikely(net_rx_context->frag_cnt[net_rx_context->fill_frag_list_idx] ==
					net_rx_context->max_frag_cnt)) {
				printk(KERN_INFO "exceeded num of frags to fill - breaking fill loop\n");
				break;
			}	
		}

		remaining_to_receive -= received_from_net;

		/* Write all received network data directly to disk */
		if (received_from_net) {
			write_result = backup_disk_write(context, &filemap_offset,
				received_from_net, filemap_info.port, filemap_info.part_offset);

			if (write_result < 0) {
				finished = 1;
			} else if (write_result != received_from_net) {
				finished = 1;
			} else {
				total_transfered +=received_from_net;
				if (total_transfered >= count) {
					finished = 1;
				}
			}
		}

		/* Release pages holding network data */
		release_netdma_net_frags(net_rx_context);
	} while (!finished);

	/* set file size and reset prealloc flag */
	oxnas_set_filesize(fp, total_transfered);
	oxnas_reset_extent_preallocate_flag(fp, 0, total_transfered, GETBMAPX_OF_PREALLOC);

	/* File system safety demands that stale on disk data is not exposed.
	 * hence the file needs to be truncated to the actual written size
	 */
	if (count != total_transfered) {
		fp->f_op->unpreallocate(fp, total_transfered, count - total_transfered);
	 }

	kfree(map);

out:
	return read_result ? : write_result;
}
