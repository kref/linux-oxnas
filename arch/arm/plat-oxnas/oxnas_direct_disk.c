/*
 * arch/arm/plat-oxnas/oxnas_direct_disk.c
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
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/oxnas_direct_disk.h>
#include <mach/desc_alloc.h>
#include <mach/oxnas_errors.h>
#include <mach/direct_writes.h>
#include <mach/ox820sata.h>
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
#include <mach/incoherent_sendfile.h>
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

//#define SHOW_WAITS

extern irqreturn_t (*ox810sata_isr_callback)(int, unsigned long);
extern unsigned long ox810sata_isr_arg;

/* file system helper routines */
int oxnas_get_fs_blocksize(struct file *file) {
	struct dentry *dentry;
	struct inode	*inode;

	dentry = file->f_path.dentry;
	inode = dentry->d_inode;

	return inode->i_blkbits;
}

int oxnas_set_filesize(
	struct file *file,
	loff_t       size) {
	struct dentry *dentry;
	struct inode  *inode;

	dentry = file->f_path.dentry;
	inode = dentry->d_inode;

	if(size > i_size_read(inode)) {
		if (inode->i_op->setsize)
			inode->i_op->setsize(inode, size);

		mutex_lock(&inode->i_mutex);
		i_size_write(inode, size);
		mutex_unlock(&inode->i_mutex);

		mark_inode_dirty(inode);
	}

	return 1;
}

int oxnas_reset_extent_preallocate_flag(
	struct file *file,
	loff_t       offset,
	size_t       length,
	int          extent_flag,
	int 		 disable_accumulation) {
#ifdef DEBUG
	printk (KERN_INFO "FAST Writes - reset_preallocate_flag - offset - %lld, length - %u \n", offset, length);
#endif
	if (length <= 0) {
		return 0;
	}

	if (extent_flag == GETBMAPX_OF_PREALLOC) {
		file->f_op->resetpreallocate(file, offset, length);
#ifdef DEBUG
		printk (KERN_INFO "FAST Writes - calling remap - offset - %lld, length - %u \n", offset, length);
#endif

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		incoherent_sendfile_remap_file(file->inode);
		if(disable_accumulation) {
			incoherent_sendfile_invalidate_cache(file->inode, offset, length);
		}
	} else {
#ifdef DEBUG
		printk (KERN_INFO "FAST Writes - calling Invalidate - offset - %lld, length - %u \n", offset, length);
#endif
		incoherent_sendfile_invalidate_cache(file->inode, offset, length);
#endif //CONFIG_OXNAS_FAST_READS_AND_WRITES
	}

	return 0;
}

/* end of fs helpers */

void fast_writes_isr(int error, void *arg)
{
	oxnas_direct_disk_context_t *context = (oxnas_direct_disk_context_t*)arg;
	struct inode *inode = context->inode;
	direct_access_context_t *sata_context = inode->writer_filemap_info.direct_access_context;
	void (*release_fn)(int) = sata_context->release;

	if (error) {
		// Record that the transfer failed
		printk(KERN_WARNING "fast_writes_isr() Transfer failed for file %s, error %d\n",
			context->file->f_path.dentry->d_name.name, error);
		set_write_error(inode);
	}

	// Relinquish ownership of the SATA core now we've finished touching it
	(*release_fn)(0);

	// Sort out DMA SG/PRD list used for the completed SATA transfer. The
	// variables for list handling are modified at task level when not under
	// the SATA completion spinlock
	smp_rmb();

	if(atomic_read(&context->free_sg)) {
		int cur_idx = atomic_read(&context->cur_transfer_idx);
		if(cur_idx != -1) {
			if (context->prd_list[cur_idx] != NULL) {
				odrb_free_prd_array(context->prd_list[cur_idx]);
				context->prd_list[cur_idx] = NULL;
				atomic_set(&context->cur_transfer_idx, -1);
				atomic_set(&context->cur_sg_status[cur_idx], 0);
			}
		}
		atomic_set(&context->free_sg, 0);
	}

	// Make sure DMA related changes above will be seen by the woken task
	// when it awakes due to seeing sata_in_progress become zero
	smp_wmb();

//printk("fast_writes_isr() Up for context %p\n", context);
	up(&context->sata_active_sem);
}
