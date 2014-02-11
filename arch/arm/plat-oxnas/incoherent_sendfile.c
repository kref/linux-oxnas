/*
 * arch/arm/plat-oxnas/incoherent_sendfile.c
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
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <mach/desc_alloc.h>
#include <mach/hardware.h>
#include <mach/incoherent_sendfile.h>
#include <mach/ox820sata.h>
#include <mach/direct-storage.h>
#include <mach/dma.h>

#ifdef CONFIG_OXNAS_FAST_WRITES
#include <net/tcp.h>
#include <mach/direct_writes.h>
#endif // CONFIG_OXNAS_FAST_WRITES

#define SATA_ACQUIRE_TIMEOUT_JIFFIES (HZ << 5)

#define INCOHERENT_ALLOC_BYTES ((INCOHERENT_ALLOC_PAGES) * (PAGE_SIZE))
#define INCOHERENT_ALLOC_SECTORS (INCOHERENT_ALLOC_BYTES >> SECTOR_SHIFT)
#define INCOHERENT_BLOCK_SHIFT ((CONFIG_OXNAS_FAST_READ_ALLOC_ORDER) + (PAGE_SHIFT))

#define INITIAL_NUM_MAP_ENTRIES	16

#define BLOCK_FREE_WAIT_JIFFIES (HZ >> 3)

#define FILL_TIMEOUT_JIFFIES (HZ << 5)
#define FILL_CHECK_INTERVAL_JIFFIES (HZ << 2)

#define LOCK_HOLD_LIMIT_JIFFIES (HZ >> 1)
#define LOCK_HOLD_INC_JIFFIES   (HZ >> 1)

#define QUEUE_SCHEDULE_LIMIT_JIFFIES	 (HZ >> 2)
#define QUEUE_SCHEDULE_LIMIT_INC_JIFFIES (HZ >> 2)

#define MAX_RA_SHIFT 2

// Global tasklet for all incoherent sendfile operations
static struct tasklet_struct tasklet_s;

// Fill info for the context currently having a fill performed against it to
// allow the ISR and tasklet to track fill progress
static fill_progress_t *current_fill_progress_s = 0;
static int sata_lock_held_s = 0;

// All the contexts who have pending fills against them to allow the tasklet to
// pick a next fill to start. Contains fill_progress_t* entries
static struct list_head pending_fills_s;

// Lock for manipulations of the global pending_fills_s list
static spinlock_t fill_lock_s = SPIN_LOCK_UNLOCKED;

// The global set of block caches
static incoherent_cache_t caches_s[CONFIG_OXNAS_FAST_READ_NUM_BLOCK_CACHES];
static struct list_head free_caches_s;
static spinlock_t caches_lock_s = SPIN_LOCK_UNLOCKED;

// Globally available file entries
static fill_que_entry_t fill_entries_s[CONFIG_OXNAS_FAST_READ_NUM_FILL_ENTRIES];
static struct list_head free_fill_entries_s;
static spinlock_t fill_entries_lock_s = SPIN_LOCK_UNLOCKED;

typedef struct zero_block {
	struct page *zero_page;
	struct page *ptbl[INCOHERENT_ALLOC_PAGES];
} zero_block_t;

static zero_block_t zero_block_s;

typedef struct read_frag {
	struct list_head head;
	int              is_hole;
	size_t           length;
} read_frag_t;

// Globally available block fragment descriptors
static struct kmem_cache *frag_cache_s = 0;
static spinlock_t frag_cache_lock_s = SPIN_LOCK_UNLOCKED;

static unsigned long lock_hold_limit_jiffies_s;

static unsigned long queue_schedule_limit_jiffies_s;

static void deferred_fill_work(struct work_struct *);
DECLARE_WORK(fast_reads_work, deferred_fill_work);

static void cache_add_blocks(
	incoherent_sendfile_context_t *context,
	fill_que_entry_t              *fill_entry)
{
	struct list_head *p, *n;

	spin_lock_bh(&context->lock);
	list_for_each_safe(p, n, &fill_entry->blocks) {
		incoherent_cache_block_t *block = list_entry(p, incoherent_cache_block_t, fill_head);
		list_del_init(p);
//printk("cache_add_blocks() Context %p, block %p, block->start %llu, block->count %u\n", context, block, block->start, block->count);
//printk("C %lld\n", block->start << SECTOR_SHIFT);

		// Block must be on the filling list, but not on the LRU list
		BUG_ON(list_empty(&block->head));
		BUG_ON(!list_empty(&block->lru_head));

		// Remove the block from the filling list
		list_del(&block->head);

		if (!block->stale) {
			// Add block to list of valid blocks - needs to be efficiently indexed
			// eventually
			list_add(&block->head, &context->cache->valid);
		} else {
//printk("cache_add_blocks() Context %p, block %p, stale\n", context, block);
			INIT_LIST_HEAD(&block->head);
			fill_entry->stale = 1;
		}

		// Make the newly added block the most recently used, i.e. add to the tail
		// of the LRU list
		list_add_tail(&block->lru_head, &context->cache->lru);
	}
	spin_unlock_bh(&context->lock);
}

#if 0
static void dump_filemap(
	incoherent_sendfile_context_t *context,
	loff_t                         offset)
{
	oxnas_filemap_info_t *filemap_info = &context->inode->filemap_info;
	sector_t sector_offset = offset >> SECTOR_SHIFT;
	sector_t where = 0;
	int i;

	printk("dump_filemap() offset in sectors %lld\n", sector_offset);
	for (i=1; i <= filemap_info->used_extents; i++) {
		getbmapx_t *map_entry = &filemap_info->map[i];
		loff_t      map_len = map_entry->bmv_length;
		int         is_hole = !!(map_entry->bmv_block == GETBMAPX_BLOCK_HOLE);
		int         is_prealloc = !!(map_entry->bmv_oflags & GETBMAPX_OF_PREALLOC);

		printk("dump_filemap() where %lld\n", where);
		printk("dump_filemap() entry %d, start %lld, len %lld, is_hole %d, is_prealloc %d\n",
			i, map_entry->bmv_offset, map_entry->bmv_length, is_hole, is_prealloc);

		where += map_len;
	}

	printk("dump_filemap() total length in sectors %lld\n", where);
}
#endif

static int index_filemap(
	incoherent_sendfile_context_t *context,
	loff_t                         offset)
{
	oxnas_filemap_info_t *filemap_info = &context->inode->filemap_info;
	sector_t              sector_offset = offset >> SECTOR_SHIFT;
	int                   retval = 0;
	getbmapx_t           *map_entry;

	/* Is the offset still within the current filemap extent? */
	map_entry = &filemap_info->map[context->cur_map_entry];
	if ((sector_offset >= map_entry->bmv_offset) &&
		(sector_offset < (map_entry->bmv_offset + map_entry->bmv_length))) {
		/* Yes, so update record of offset into current extent */
		context->cur_map_entry_offset = sector_offset - map_entry->bmv_offset;
		context->next_offset = offset;
		retval = 1;
	} else {
		int i;
		sector_t where = 0;

//printk("index_filemap() at %lld, sector %lld\n", offset, sector_offset);
		for (i=1; i <= filemap_info->used_extents; i++) {
			loff_t map_len;

			map_entry = &filemap_info->map[i];
			map_len   = map_entry->bmv_length;

//printk("index_filemap() where %lld\n", where);
//printk("index_filemap() entry %d, start %lld, len %lld\n", i, map_entry->bmv_offset, map_entry->bmv_length);
			if ((sector_offset >= where) && (sector_offset < (where + map_len))) {
				context->cur_map_entry = i;
				context->cur_map_entry_offset =
					sector_offset - map_entry->bmv_offset;
				context->next_offset = offset;
//printk("index_filemap() Found cur_map_entry %d, cur_map_entry_offset %lld, next_offset %lld\n", context->cur_map_entry, context->cur_map_entry_offset, context->next_offset);
				retval = 1;
				break;
			}

			where += map_entry->bmv_length;
		}
	}

	return retval;
}

/*
 * Allocate, or reallocate if there's an existing map_info pointer, the filemap
 * flag 0 = reader and flag 1 = writer
 */
int alloc_filemap(
	struct inode *inode,
	int           writer)
{
	int i;
#ifdef CONFIG_OXNAS_FAST_WRITES
	oxnas_filemap_info_t *map_info =
		writer ? &inode->writer_filemap_info : &inode->filemap_info;
#else // CONFIG_OXNAS_FAST_WRITES
	oxnas_filemap_info_t *map_info = &inode->filemap_info;
#endif // CONFIG_OXNAS_FAST_WRITES
	int retval = 0;

//printk(KERN_INFO "alloc_filemap() Inode %p %s for %s\n", inode, map_info->map ? "re-map" : "map" , writer ? "writer" : "reader");

	/* Should only enter with NULL map pointer on first read access to inode for
	 * newly opened fast mode file */
	if (!map_info->map) {
		/* Allocate a new map, making a guess at the number of extents the map
		   will need */
//printk(KERN_INFO "alloc_filemap() Allocating new filemap\n");
		map_info->alloced_extents = INITIAL_NUM_MAP_ENTRIES;
		map_info->map = kzalloc(sizeof(getbmapx_t) * (map_info->alloced_extents + 1), GFP_KERNEL);
		if (!map_info->map) {
			retval = -ENOMEM;
			goto out;
		}
	} else {
		// Re-use the existing map
//printk(KERN_INFO "alloc_filemap() Reusing filemap\n");
		map_info->map[0].bmv_offset = 0;
		map_info->map[0].bmv_block = 0;
		map_info->map[0].bmv_entries = 0;
	}

	/* Query for the file map */
	map_info->map[0].bmv_length = -1;
	map_info->map[0].bmv_count = map_info->alloced_extents + 1;

//printk("alloc_filemap() before getbmapx[1] - bmv_offset %lld, bmv_block 0x%llx, "
//	   "bmv_length %lld, bmv_count %ld, bmv_entries %ld, bmv_iflags 0x%lx, "
//	   "bmv_oflags 0x%lx\n", map_info->map[0].bmv_offset,
//	   map_info->map[0].bmv_block, map_info->map[0].bmv_length,
//	   map_info->map[0].bmv_count, map_info->map[0].bmv_entries,
//	   map_info->map[0].bmv_iflags, map_info->map[0].bmv_oflags);

	retval = inode->i_op->getbmapx(inode, map_info->map);
	if (retval < 0) {
		goto free_map;
	}

//printk("alloc_filemap() after getbmapx[1] - bmv_offset %lld, bmv_block 0x%llx, "
//	   "bmv_length %lld, bmv_count %ld, bmv_entries %ld, bmv_iflags 0x%lx, "
//	   "bmv_oflags 0x%lx\n", map_info->map[0].bmv_offset,
//	   map_info->map[0].bmv_block, map_info->map[0].bmv_length,
//	   map_info->map[0].bmv_count, map_info->map[0].bmv_entries,
//	   map_info->map[0].bmv_iflags, map_info->map[0].bmv_oflags);

	/* Work out how many map entries are really used by the file */
	map_info->used_extents = map_info->map[0].bmv_entries;
//printk(KERN_INFO "alloc_filemap() used_extents %d\n", map_info->used_extents);
	if (map_info->used_extents > map_info->alloced_extents) {
		retval = -EINVAL;
		goto free_map;
	} else if (map_info->used_extents == map_info->alloced_extents) {
		/* More entries than we initially provided are required to discribe the
		   entire file */
		map_info->used_extents = inode->i_op->get_extents(inode, 0);
//printk(KERN_INFO "alloc_filemap() used_extents %d, alloced_extents %d\n",
//	map_info->used_extents, map_info->alloced_extents);
		if (map_info->used_extents < 0) {
			retval = -EINVAL;
			goto free_map;
		}

		if (map_info->used_extents > map_info->alloced_extents) {
			/* Guess was wrong so use the correct value */
//printk(KERN_INFO "alloc_filemap() Need more map entires: %d\n", map_info->used_extents);
			kfree(map_info->map);
			map_info->alloced_extents = map_info->used_extents;
			map_info->map = kzalloc(sizeof(getbmapx_t) * (map_info->alloced_extents + 1), GFP_KERNEL);
			if (!map_info->map) {
				retval = -ENOMEM;
				goto out;
			}

			map_info->map[0].bmv_length = -1;
			map_info->map[0].bmv_count = map_info->alloced_extents + 1;

//printk("alloc_filemap() getbmapx[2] - bmv_offset %lld, bmv_block 0x%llx, "
//	   "bmv_length %lld, bmv_count %ld, bmv_entries %ld, bmv_iflags 0x%lx, "
//	   "bmv_oflags 0x%lx\n", map_info->map[0].bmv_offset,
//	   map_info->map[0].bmv_block, map_info->map[0].bmv_length,
//	   map_info->map[0].bmv_count, map_info->map[0].bmv_entries,
//	   map_info->map[0].bmv_iflags, map_info->map[0].bmv_oflags);

			retval = inode->i_op->getbmapx(inode, map_info->map);
			if (retval < 0) {
				goto free_map;
			}

//printk("alloc_filemap() after getbmapx[2] - bmv_offset %lld, bmv_block 0x%llx, "
//	   "bmv_length %lld, bmv_count %ld, bmv_entries %ld, bmv_iflags 0x%lx, "
//	   "bmv_oflags 0x%lx\n", map_info->map[0].bmv_offset,
//	   map_info->map[0].bmv_block, map_info->map[0].bmv_length,
//	   map_info->map[0].bmv_count, map_info->map[0].bmv_entries,
//	   map_info->map[0].bmv_iflags, map_info->map[0].bmv_oflags);

			if (map_info->map[0].bmv_entries > map_info->alloced_extents) {
				retval = -EINVAL;
				goto free_map;
			}
//printk(KERN_INFO "alloc_filemap() Reallocated -> used_extents %d, alloced_extents %d\n",
//	map_info->used_extents, map_info->alloced_extents);
		}
	}

	/* Calculate the file length by totalling up all the extent sizes */
	map_info->length = 0;
	for (i=1; i <= map_info->used_extents; i++) {
//printk("alloc_filemap() extent %d, bmv_offset %lld, bmv_length %lld, "
//	   "bmv_block 0x%llx, bmv_iflags 0x%lx, bmv_oflags 0x%lx\n", i,
//	   map_info->map[i].bmv_offset, map_info->map[i].bmv_length,
//	   map_info->map[i].bmv_block, map_info->map[i].bmv_iflags,
//	   map_info->map[i].bmv_oflags);

		map_info->length += map_info->map[i].bmv_length;
	}
	map_info->length *= SECTOR_SIZE;
//printk(KERN_INFO "alloc_filemap() total length %lld bytes, extents = %d\n",
//	map_info->length, map_info->used_extents);

	if (!map_info->direct_access_context) {
//printk(KERN_INFO "alloc_filemap() Inode %p SATA context for %s\n", inode, writer ? "writer" : "reader");
		retval = alloc_direct_sata_context(inode, !writer, &map_info->direct_access_context);
		if (retval < 0) {
			goto free_map;
		}
	}

	/* Success */
	goto out;

free_map:
	kfree(map_info->map);
	map_info->map = 0;

out:
//printk(KERN_INFO "alloc_filemap() Map entries used %d, alloced %d\n",
//	map_info->used_extents, map_info->alloced_extents);

	return retval;
}

static void add_frag(
	incoherent_cache_block_t *block,
	int                       is_hole,
	size_t                    length)
{
	read_frag_t   *frag;

	spin_lock_bh(&frag_cache_lock_s);
	frag = kmem_cache_alloc(frag_cache_s, GFP_ATOMIC);
	spin_unlock_bh(&frag_cache_lock_s);
	BUG_ON(!frag);

	frag->is_hole = is_hole;
	frag->length = length;
	INIT_LIST_HEAD(&frag->head);
	list_add_tail(&frag->head, &block->frags_head);
}

static void block_add_frag(
	incoherent_cache_block_t *block,
	int                       block_offset,
	int                       is_hole,
	size_t                    length)
{
	// Defer allocating frags until we know the block is not entirely data or
	// entirely hole. Relies on block fills always starting at beginning of
	// block
	if (!block_offset) {
		// Starting block fill so can't be any existing fragments yet so record
		// state of block fill and do not create a frag
		block->is_hole = is_hole;
	} else if (!list_empty(&block->frags_head)) {
		// Already frags, so add another for this block section fill
		add_frag(block, is_hole, length);
	} else if (block->is_hole != is_hole) {
		// No frags yet, but have found change in block fill type, so must
		// create frag to describe block upto current offset, then another frag
		// to describe this new section
		add_frag(block, block->is_hole, block_offset);
		add_frag(block, is_hole, length);
	}

	// Update record of total fill of the block
	block->count += length;
}

/*
 * Try to get filemap read access from atomic context. Fail if a filemap update
 * update is pending
 * Returns 1 on success
 */
static int trylock_filemap(struct inode *inode)
{
	int retval = 0;

	if (is_writer_flush_filemap_required(inode)) {
//printk("trylock_filemap() Inode %p writer filemap flush required\n", inode);
		goto out;
	}

	if (down_trylock(&inode->reader_sem)) {
//printk("trylock_filemap() Inode %p failed to grab semaphore\n", inode);
		goto out;
	}

	if (unlikely(inode->filemap_update_pending)) {
//printk("trylock_filemap() Inode %p has pending filemap update\n", inode);
		up(&inode->reader_sem);
		goto out;
	}

	retval = 1;

out:
//printk("trylock_filemap() Inode %p locking %s\n", inode, retval ? "successful" : "failed");
	return retval;
}

/* Bring the readers filemap upto date with any changes effected by the writers
 * Leave the function holding the read lock on the filemap
 */
static void lock_filemap(struct inode *inode)
{
//printk("lock_filemap() Entered for inode %p\n", inode);

	while (down_timeout(&inode->reader_sem, HZ)) {
		printk("lock_filemap() A second has elapsed while waiting, inode %p\n", inode);
	}

	smp_rmb();
	if (unlikely(inode->filemap_update_pending)) {
//printk("lock_filemap() Inode %p remap required\n", inode);
		if (alloc_filemap(inode, 0)) {
			printk("lock_filemap() filemap remap failed\n");
		} else {
			/* Mark all active fast readers as needing to re-index the filemap */
			struct list_head *p;
			read_lock(&inode->fast_files_lock);
			list_for_each(p, &inode->fast_files) {
				struct file *file = list_entry(p, struct file, fast_head);
	
				/* Assignment of context should be done with a single pointer
				 * setting so should be OK to test here without locks */
				void *ptr = file->fast_context;
				if (ptr) {
					/* Fast reader is active, mark for re-index */
					((incoherent_sendfile_context_t*)ptr)->need_reindex = 1;
				}
			}
			read_unlock(&inode->fast_files_lock);
		}

		inode->filemap_update_pending = 0;
		smp_wmb();
	}

//printk("lock_filemap() Leaving for inode %p \n", inode);
}

/*
 * SATA interrupt handler
 */
static void isr(int error, void *data)
{
	int  need_wmb = 0;

//printk("isr() error %d, data %p\n", error, data);

	// This interrupt should be for us and it's therefore a major problem if we
	// don't have the tracking for the in-progress fill setup
	smp_rmb();
	BUG_ON(!current_fill_progress_s);

	BUG_ON(!current_fill_progress_s->entry);

	if (current_fill_progress_s->is_sg) {
		// Relinquish ownership of the SG list used for the DMA transfer
		BUG_ON(!current_fill_progress_s->prd_list);
		odrb_reader_free_prd_array(current_fill_progress_s->prd_list);
		current_fill_progress_s->is_sg = 0;
		need_wmb = 1;
	}

	if (error) {
		// Record that the transfer failed, so the block(s) can be disgarded
		current_fill_progress_s->entry->failed = 1;
		++current_fill_progress_s->context->failure_count;
		need_wmb = 1;
printk("isr() Transfer failed total failures %d\n", current_fill_progress_s->context->failure_count);
	}

	if (likely(need_wmb)) {
		// Ensure tasklet will see changes to current_fill_progress_s and entry
		// that were made without fill lock held
		smp_wmb();
	}

//printk("isr() Schedule tasklet\n");
	tasklet_hi_schedule(&tasklet_s);
}

/*
 * filemap read lock must be held when calling this function and will be
 * dropped if a disk read is started
 */
static int direct_disk_read(fill_progress_t *fill_progress)
{
	incoherent_sendfile_context_t *context = fill_progress->context;
	struct inode                  *inode = context->inode;
	oxnas_filemap_info_t          *filemap_info = &inode->filemap_info;
	fill_que_entry_t              *entry = fill_progress->entry;
	loff_t                         offset = entry->pos;
	size_t                         remaining = entry->remaining;
	getbmapx_t                    *map_entry;
	long long                      map_start;
	long long	                   map_len;
	long long	                   map_offset;
	sector_t   		               lba;
	long long	                   contig_len;
	int                            is_prealloc = 0;
	int                            is_delalloc = 0;
	int                            is_hole = 0;
	direct_access_context_t       *sata_context;

//printk("direct_disk_read() Context %p, fill pos %lld, fill remaining %d\n", context, offset, remaining);
//printk("direct_disk_read() Context %p, remaining %d, used_extents %d, offset %lld, next_offset %lld\n", context, remaining, filemap_info->used_extents, offset, filemap_info->next_offset);

	if (unlikely(context->need_reindex) || (offset != context->next_offset)) {
		/* Read is discontiguous with previous one so need to re-index into the
		 * file map for the required sector offset */
		if (!index_filemap(context, offset)) {
printk("direct_disk_read() Filemap info %p failed to index, remaining %d, used_extents %d, offset %lld, next_offset %lld\n", filemap_info, remaining, filemap_info->used_extents, offset, context->next_offset);
			fill_progress->eof = 1;
			return -EIO;
		}
		context->need_reindex = 0;
	}

	map_entry = &filemap_info->map[context->cur_map_entry];
	map_offset = context->cur_map_entry_offset;

	is_hole = !!(map_entry->bmv_block == GETBMAPX_BLOCK_HOLE);
//if (is_hole) printk("direct_disk_read() Hole!\n");
	is_prealloc = !!(map_entry->bmv_oflags & GETBMAPX_OF_PREALLOC);
//if (is_prealloc) printk("direct_disk_read() Prealloc!\n");
	if (is_prealloc) {
//		dump_filemap(context, offset);
		is_hole = 1;
	}

	is_delalloc = !!(map_entry->bmv_oflags & GETBMAPX_OF_DELALLOC);
	BUG_ON(is_delalloc);

	/* Get the sector offset and sector count of the current file extent */
	map_start = map_entry->bmv_offset;
	map_len   = map_entry->bmv_length;

	/* Get the disk block from which to start the read */
	lba = map_entry->bmv_block + map_offset;

	/* Calculate remaining contiguous disk bytes at offset into mapping */
	contig_len = (map_len - map_offset) << SECTOR_SHIFT;
//printk("direct_disk_read() Context %p, map entry %d, map offset %lld, map_len %lld, map_start %lld, lba %llu, contig_len %lld, hole %d, prealloc %d, delalloc %d\n", context, context->cur_map_entry, context->cur_map_entry_offset, map_len, map_start, lba, contig_len, is_hole, is_prealloc, is_delalloc);
	if (!contig_len) {
		// Extent count wasn't right as we've hit the EOF now rather than during
		// the previous sub-fill
printk("direct_disk_read() Context %p unexpected EOF\n", context);
		fill_progress->eof = 1;
		return -EIO;
	} else if (contig_len > remaining) {
		contig_len = remaining;
		context->cur_map_entry_offset += (contig_len >> SECTOR_SHIFT);
	} else if (context->cur_map_entry == filemap_info->used_extents) {
//printk("direct_disk_read() Context %p, reached EOF\n", context);
		/* The current map entry cannot supply all the data requested and
		   there are no further map entries */                                         
		fill_progress->eof = 1;                                                           
		context->cur_map_entry_offset += (contig_len >> SECTOR_SHIFT);
	} else {
//printk("direct_disk_read() Context %p, next read will use next mapinfo entry\n", context);
		++context->cur_map_entry;
		context->cur_map_entry_offset = 0;
	}

	/* Remember where a subsequent contiguous read must start such that a
	 * re-indexing of the file map is not required
	 */
	context->next_offset += contig_len;

//printk("direct_disk_read() Context %p, lba %llu, contig_len %lld, eof %d\n", context, lba, contig_len, !!fill_progress->eof);
	/* Keep track of overall block fill progress assuming read completes OK */
	fill_progress->last_read_count = contig_len;

	/*
	 * contig_len may be greater than the remaining space in the current block
	 */
	if (is_hole) {
		incoherent_cache_block_t *block = fill_progress->block;
		int                       block_offset = fill_progress->block_offset;

		/* Associate the file hole with the blocks that are to map it */
		do {
			/* Calculate how much of the hole fits in the current block */
			unsigned int fill_len =
				((block_offset + contig_len) <= INCOHERENT_ALLOC_BYTES)
					? contig_len : (INCOHERENT_ALLOC_BYTES - block_offset);

			/* Record the extent of the hole that is associated with this block */
			block_add_frag(block, block_offset, is_hole, fill_len);

			/* Update the amount of the hole still to be mapped onto blocks */
			contig_len -= fill_len;

			/* Update the amount of the entire fill that still remains */
			remaining -= fill_len;

			if (contig_len ||
			    (remaining && ((block_offset + fill_len) == INCOHERENT_ALLOC_BYTES))) {
				/* This block is full and there is still more filling to be
				   done so get the next block associated with the fill */
				block = list_entry(block->fill_head.next, incoherent_cache_block_t, fill_head);
				fill_progress->block = block;
				block_offset = 0;
			} else {
				/* Update the offset into this block at which any subsequent
				   section of this fill should begin */
				block_offset += fill_len;
			}

			fill_progress->block_offset = block_offset;
		} while (contig_len);

		/* Indicate to caller that no disk read was required
		 *
		 * We don't drop the filemap read lock if only holes were found so the
		 * caller can go again for any following non-hole data
		 */
		return 1;
	} else {
		/* Fill SG DMA descriptors to describe the contiguous disk sectors and
		   the pages into which they are to be copied */
		incoherent_cache_block_t *block = fill_progress->block;
		int        block_offset = fill_progress->block_offset;
		dma_addr_t buffer_start = virt_to_dma(0, page_address(block->pages));
		size_t     remaining_contig_len = contig_len;
		prd_table_entry_t *prds, *prds_end;

		/* Finished all reads of the file map so can release access lock */
		up(&inode->reader_sem);

		// Remember that the DMA transfer is using SG
		fill_progress->is_sg = 1;

		BUG_ON(odrb_reader_alloc_prd_array(&fill_progress->prd_list));

		prds = fill_progress->prd_list->prds;
		prds_end = prds + CONFIG_ODRB_READER_PRD_ARRAY_SIZE;

		do {
			int    block_filled;
			size_t sg_len, block_len, fill_len;

			// Should never have more cache blocks to fill from a single
			// contiguous disk block than there are available DMA descriptors
			BUG_ON(prds == prds_end);

			/* Fill length cannot exceed remaining space in current block */
			block_len = ((block_offset + remaining_contig_len) <= INCOHERENT_ALLOC_BYTES)
				? remaining_contig_len : (INCOHERENT_ALLOC_BYTES - block_offset);

			/* Fill length cannot exceed the max allowed for SG entries */
			sg_len = (remaining_contig_len <= PRD_MAX_LEN)
				? remaining_contig_len : PRD_MAX_LEN;

			/* Fill block with amount consistent with both SG and block
			   limitations */
			fill_len = (block_len < sg_len) ? block_len : sg_len;

			/* Record the extent of the non-hole that is associated with
			   this block */
			block_add_frag(block, block_offset, is_hole, fill_len);

			/* Fill the SG entry with the DMAable address and length
			   accounting for the maximum allowed length being represented
			   by zero */
			prds->adr = buffer_start + block_offset;
			prds++->flags_len = (fill_len == PRD_MAX_LEN) ? 0 : fill_len;

			/* Update the amount of the entire fill that still remains */
			remaining -= fill_len;

			/* Is the current block filled to its end by this fill? */
			block_filled = ((block_offset + fill_len) == INCOHERENT_ALLOC_BYTES);

			/* Update the block ptr and offset dependent on whether the current
			   block has been filled but there is still more filling to be done */
			if (remaining && block_filled) {
				/* This block is full and there is still more filling to be
				   done so get the next block associated with the fill */
				block = list_entry(block->fill_head.next, incoherent_cache_block_t, fill_head);
				fill_progress->block = block;
				buffer_start = virt_to_dma(0, page_address(block->pages));
				block_offset = 0;
			} else {
				/* Update the offset into this block at which any subsequent
				   section of this fill should begin */
				block_offset += fill_len;
			}				

			/* Remember offset into current block is case cannot entirely
			   fulfill the entire read request this time */
			fill_progress->block_offset = block_offset;

			/* We only satisfy a read from disk of a single contiguous set of
			   disk sectors for each call to this function */
			remaining_contig_len -= fill_len;
		} while (remaining_contig_len);

		// Mark last used PRD
		(--prds)->flags_len |= PRD_EOF_MASK;

//printk("direct_disk_read() Context %p setting PRD DMA contig_len %lld\n", context, contig_len);
		sata_context = filemap_info->direct_access_context;

		(*sata_context->prepare_command)(sata_context, 0, lba,
			contig_len >> SECTOR_SHIFT, fill_progress->prd_list->phys, isr, 0);

		/* Issue SATA command to write contiguous sectors to disk */
//printk("direct_disk_read() Context %p SATA read lba %llu, contig_len %lld, eof %u\n", context, lba, contig_len, !!fill_progress->eof);
		entry->disk_read_started = 1;
		(*sata_context->execute_command)();

		return 0;
	}
}

/* Does not require filemap access lock to be held as will not attempt to
 * start a disk read
 */
static int fill_block_step_first(fill_progress_t *fill_progress)
{
	fill_que_entry_t *entry = fill_progress->entry;
	size_t            last_read_count;
	int               fill_finished = 0;

	BUG_ON(!entry->disk_read_started);

//printk("fill_block_step_first() Entered: Context %p, entry %p\n", fill_progress->context, entry);
	// Update fill entry progress assuming previous disk read completed OK
	last_read_count = fill_progress->last_read_count;
	entry->remaining -= last_read_count;
	entry->pos       += last_read_count;
//printk("fill_block_step_first() last_read_count %d, entry->remaining %d, entry->pos %lld\n", fill_progress->last_read_count, entry->remaining, entry->pos);

	if (entry->failed) {
//printk("fill_block_step_first() Fill failed for context %p - marking fill aborted\n", fill_progress->context);
		fill_finished = 1;
		entry->aborted = 1;
	} else if (!entry->remaining || fill_progress->eof) {
//printk("fill_block_step_first() Fill finished for context %p\n", fill_progress->context);
		// Make the filled block available for use, unless it has been marked
		// stale in which case only add to LRU list
		cache_add_blocks(fill_progress->context, entry);
		fill_finished = 1;
	}

	if (fill_finished) {
		// The fill is complete so reset flag indicating that a fill is in progress
		entry->disk_read_started = 0;
	}

	return fill_finished;
}

/* Called with filemap access lock held. Lock should be dropped before returning
 * regardless of whether a disk read was successfully started or not
 */
static int fill_block_step_next(fill_progress_t *fill_progress)
{
	fill_que_entry_t *entry = fill_progress->entry;
	int               disk_read_started = 0;
	int               fill_finished = 0;

	// We may or may not soon start a new disk fill, so make sure the flag is
	// clear first
	entry->disk_read_started = 0;

	while (!fill_finished) {
		size_t last_read_count;
		int    retval;

		// Start read of data from disk into the cache block, may hit a hole
//printk("fill_block_step_next() Start disk read: Context %p\n", fill_progress->context);
		retval = direct_disk_read(fill_progress);
//printk("fill_block_step_next() Finished disk read: Context %p, result %d\n", fill_progress->context, retval);

		switch (retval) {
			case -EIO:
				// No more data in file - should have been caught on previous
				// read but sometimes the XFS extent query function doesn't
				// quite work
				fill_progress->eof = 1;
				fill_finished = 1;
				entry->aborted = 1;
				goto out;
			case 1:
				// Found a hole so no disk fill to do so check if we've finished
				// filling this entry
				break;
			case 0:
				// Successfully queued the next disk read
				disk_read_started = 1;
				goto out;
			default:
				BUG();
		}

		// Update fill entry progress assuming previous disk read completed OK
		last_read_count = fill_progress->last_read_count;
		entry->remaining -= last_read_count;
		entry->pos       += last_read_count;
//printk("fill_block_step_next() last_read_count %d, entry->remaining %d, entry->pos %lld\n", fill_progress->last_read_count, entry->remaining, entry->pos);

		if (entry->failed) {
//printk("fill_block_step_next() Fill failed for context %p - marking fill aborted\n", fill_progress->context);
			fill_finished = 1;
			entry->aborted = 1;
		} else if (!entry->remaining || fill_progress->eof) {
//printk("fill_block_step_next() Fill finished for context %p\n", fill_progress->context);
			// Make the filled block available for use, unless it has been marked
			// stale in which case only add to LRU list
			cache_add_blocks(fill_progress->context, entry);
			fill_finished = 1;
		}
	}

out:
	if (!disk_read_started) {
		// Not starting disk read, so ensure we drop the filemap read lock before
		// returning
		up(&fill_progress->context->inode->reader_sem);
	}

//printk("fill_block_step_next() Leaving: Context %p, fill_finished %d\n", fill_progress->context, fill_finished);
	return fill_finished;
}

static void release_block_cache(incoherent_cache_t *cache)
{
//printk(KERN_INFO "release_block_cache() Entered for cache %p\n", cache);
	/* Return cache to list of free caches */
	spin_lock(&caches_lock_s);
	list_add(&cache->list, &free_caches_s);
	spin_unlock(&caches_lock_s);
}

static void deassign_block_cache(incoherent_sendfile_context_t *context)
{
//printk(KERN_INFO "deassign_block_cache() Entered for context %p with cache %p\n", context, context->cache);
	release_block_cache(context->cache);
	context->cache = NULL;
}

static void empty_frags_list(incoherent_cache_block_t *block)
{
	if (!list_empty(&block->frags_head)) {
		struct list_head *p, *n;

		// Return any frag structures attached to this block
		spin_lock_bh(&frag_cache_lock_s);
		list_for_each_safe(p, n, &block->frags_head) {
			read_frag_t *frag = list_entry(p, read_frag_t, head);
			list_del(p);
			kmem_cache_free(frag_cache_s, frag);
		}
		spin_unlock_bh(&frag_cache_lock_s);
	}
}

static void free_block_cache(incoherent_cache_t *cache)
{
	int i;
//printk(KERN_INFO "free_block_cache() Entered for cache %p\n", cache);

	for (i=0; i < CONFIG_OXNAS_FAST_READ_BLOCKS_PER_CACHE; i++) {
		incoherent_cache_block_t *block = &cache->blocks[i];
		unsigned long end = jiffies + BLOCK_FREE_WAIT_JIFFIES;

		while (atomic_read(&block->users) != 0) {
			if (time_after(jiffies, end)) {
				printk(KERN_WARNING "incoherent_sendfile_free_context() "
					"Timed-out of wait for block 0x%p to have no users\n", block);
				break;
			}
			msleep_interruptible(1);
		}

		if (block->pages) {
			int j;
			for (j=0; j < INCOHERENT_ALLOC_PAGES; j++) {
				// Mark the page as no longer in use by the fast reads code
				__ClearPageIncoherentSendfile(block->ptbl[j]);

				// Force the page count to the correct value prior to freeing
				// the page group
				atomic_set(&block->ptbl[j]->_count, !j);
			}
//printk(KERN_INFO "free_block_cache() Freeing pages for block 0x%p\n", block);
			__free_pages(block->pages, CONFIG_OXNAS_FAST_READ_ALLOC_ORDER);
		}

		empty_frags_list(block);
	}
}

/*
 * fill_lock_s spinlock must be held while calling this function
 */
static fill_que_entry_t* search_pending_fills_by_sector(
	fill_progress_t *fill_progress,
	sector_t         sector)
{
	struct list_head *p;
	fill_que_entry_t *entry = NULL;

	if (fill_progress->in_progress) {
		fill_que_entry_t *e = fill_progress->entry;
		sector_t fill_start_sector = e->sector;
		sector_t fill_end_sector =
			fill_start_sector + fill_progress->fill_len_sectors;

		if ((sector >= fill_start_sector) &&
		    (sector < fill_end_sector)) {
			entry = e;
			goto out;
		}
	}

	list_for_each(p, &fill_progress->queue) {
		fill_que_entry_t *e = list_entry(p, fill_que_entry_t, list);
		sector_t fill_start_sector = e->sector;
		sector_t fill_end_sector =
			fill_start_sector + (e->remaining >> SECTOR_SHIFT);

		if ((sector >= fill_start_sector) &&
		    (sector < fill_end_sector)) {
			entry = e;
			break;
		}
	}

out:
	return entry;
}

void incoherent_sendfile_free_context(void* ptr)
{
	incoherent_sendfile_context_t *context = (incoherent_sendfile_context_t*)ptr;
	LIST_HEAD(to_be_freed);
	struct list_head *p, *n;
//printk(KERN_INFO "incoherent_sendfile_free_context() Entered context %p, cache %p\n", context, context->cache);

	/*
	 * Prevent any further fills for this context other than for, perhaps, the
	 * currently in-progress fill
	 */

	spin_lock_bh(&fill_lock_s);
	// Abort any pending fills for this context
	list_for_each_safe(p, n, &context->fill_progress.queue) {
		fill_que_entry_t *entry = list_entry(p, fill_que_entry_t, list);
//printk(KERN_INFO "incoherent_sendfile_free_context() entry %p\n", entry);
		list_del(&entry->list);
		entry->filled = entry->aborted = 1;
		wake_up(&entry->wait_queue);
//printk(KERN_INFO "incoherent_sendfile_free_context() entry %p to be freed\n", entry);
		list_add_tail(&entry->list, &to_be_freed);
	}

	// Is this context queued with the global fill queue? 
	if (!list_empty(&context->fill_progress.list)) {
		// Yes, so remove it from the global queue
		list_del_init(&context->fill_progress.list);
	}
	spin_unlock_bh(&fill_lock_s);

	// Append the entries removed from the fill queue to the free entries list
	spin_lock_bh(&fill_entries_lock_s);
	list_for_each_safe(p, n, &to_be_freed) {
		fill_que_entry_t *entry = list_entry(p, fill_que_entry_t, list);
		list_del(&entry->list);
		list_add_tail(&entry->list, &free_fill_entries_s);
//printk(KERN_INFO "incoherent_sendfile_free_context() Moved entry %p from free_fill_entries_s list\n", entry);
	}
	spin_unlock_bh(&fill_entries_lock_s);

	/*
	 * If there is a currently in-progress fill for this context then wait for
	 * it to complete
	 */

	spin_lock_bh(&fill_lock_s);
	while (1) {
		if (!context->fill_progress.in_progress) {
//printk(KERN_INFO "incoherent_sendfile_free_context() No fill in progress\n");
			spin_unlock_bh(&fill_lock_s);
			break;
		} else {
			fill_que_entry_t *entry = context->fill_progress.entry;
			unsigned long end = jiffies + FILL_TIMEOUT_JIFFIES;
			DEFINE_WAIT(wait);

//printk(KERN_INFO "incoherent_sendfile_free_context() Waiting for entry %p...\n", entry);
			for (;;) {
				prepare_to_wait(&entry->wait_queue, &wait, TASK_UNINTERRUPTIBLE);
				if (entry->filled || entry->aborted) {
					break;
				}
				if (time_after(jiffies, end)) {
					printk(KERN_WARNING "incoherent_sendfile_free_context() Timed-out of wait for fill\n");
					break;
				}
				spin_unlock_bh(&fill_lock_s);
				if (!schedule_timeout(FILL_CHECK_INTERVAL_JIFFIES)) {
					printk(KERN_INFO "incoherent_sendfile_free_context() %d seconds have passed while waiting\n", FILL_CHECK_INTERVAL_JIFFIES/HZ);
				}
				spin_lock_bh(&fill_lock_s);
			}
			finish_wait(&entry->wait_queue, &wait);
//printk(KERN_INFO "incoherent_sendfile_free_context() Woken, entry filled = %d, aborted = %d\n", !!entry->filled, !!entry->aborted);
		}
	}

	// Deallocate context and associated data structures
	if (context->cache) {
		deassign_block_cache(context);
	}
	kfree(ptr);
}

static incoherent_cache_block_t* cache_get_block(
	incoherent_sendfile_context_t *context,
	loff_t                         file_offset,
	ssize_t                       *available,
	ssize_t                       *block_offset)
{
	incoherent_cache_block_t *block = NULL;
	struct list_head *p;

	// Need an efficiently indexed and searched data structure eventually
	spin_lock_bh(&context->lock);
//printk("cache_get_block() Context %p cache %p file_offset %lld\n", context, context->cache, file_offset);
	list_for_each(p, &context->cache->valid) {
		loff_t start, end;
		incoherent_cache_block_t *b =
			list_entry(p, incoherent_cache_block_t, head);

		// A stale block should never be seen on the valid list
		BUG_ON(b->stale);

		// Any already filled block should be on the LRU list, due to our
		// inability to reliably tell when the network Tx has finished with
		// any pages we give it
		BUG_ON(list_empty(&b->lru_head));

		start = b->start << SECTOR_SHIFT;
		end = start + INCOHERENT_ALLOC_BYTES;

//printk("cache_get_block() Context %p cache %p, try block %p, start %lld, end %lld\n", context, context->cache, b, start, end);
		if ((file_offset >= start) && (file_offset < end)) {
			*available = end - file_offset;
			*block_offset = file_offset - start;
			block = b;
//printk("cache_get_block() Context %p cache %p, found matching block %p, *block_offset %d, *available %d\n", context, context->cache, block, *block_offset, *available);

			// We are going to use pages from this already available, filled
			// block for network Tx, so we do not want it to be picked for
			// re-filling anytime soon
//printk("cache_get_block() Context %p cache %p, block %p moving to tail of LRU list\n", context, context->cache, block);
			list_del(&block->lru_head);
			list_add_tail(&block->lru_head, &context->cache->lru);
			break;
		}
	}
//printk("cache_get_block() Context %p cache %p returning block %p\n", context, context->cache, block);
	spin_unlock_bh(&context->lock);

	return block;
}

static int cache_is_block_available(
	incoherent_sendfile_context_t *context,
	sector_t                       sector)
{
	int available = 0;
	struct list_head *p;

	// Need an efficiently indexed and searched data structure eventually
	spin_lock_bh(&context->lock);
//printk("cache_is_block_available() Context %p sector %lld\n", context, sector);
	list_for_each(p, &context->cache->valid) {
		sector_t start, end;
		incoherent_cache_block_t *block =
			list_entry(p, incoherent_cache_block_t, head);

		// A stale block should never be seen on the valid list
		BUG_ON(block->stale);

		// Any already filled block should be on the LRU list, due to our
		// inability to reliably tell when the network Tx has finished with
		// any pages we give it
		BUG_ON(list_empty(&block->lru_head));

		start = block->start;
		end = start + INCOHERENT_ALLOC_SECTORS;
//printk("cache_is_block_available() Context %p, sector %lld, try block %p, start %lld, end %lld\n", context, sector, block, start, end);

		if ((sector >= start) && (sector < end)) {
			available = 1;
//printk("cache_is_block_available() Context %p, sector %lld, found matching block %p with start %lld, end %lld\n", context, sector, block, start, end);
			break;
		}
	}
//printk("cache_is_block_available() Context %p, sector %lld, block is %s\n", context, sector, available ? "available" : "not available");
	spin_unlock_bh(&context->lock);

	return available;
}

static incoherent_cache_block_t* cache_get_for_fill(
	incoherent_sendfile_context_t *context,
	sector_t                       start_sector)
{
	incoherent_cache_block_t *block = NULL;

	spin_lock_bh(&context->lock);

	if (!list_empty(&context->cache->free)) {
		// Get an unused, zeroised block
		block = list_entry(context->cache->free.next, incoherent_cache_block_t, head);
		BUG_ON(!list_empty(&block->lru_head));

		// Remove the unused block from the free list
		list_del(&block->head);
//printk("cache_get_for_fill() Context %p, returning block %p from free list, block->count %u\n", context, block, block->count);
	} else if (likely(!list_empty(&context->cache->lru))) {
		/*
 		 * Get the least recently used block
		 */
		struct list_head *p;
		list_for_each(p, &context->cache->lru) {
			block = list_entry(p, incoherent_cache_block_t, lru_head);
			if (likely(atomic_read(&block->users) == 0)) {
				// Found an unused block
				list_del_init(p);
				break;
			} else {
printk("cache_get_for_fill() Block %p still in use (count = %d)\n", block, atomic_read(&block->users));
			}
			block = NULL;
		}

		if (likely(block)) {
			// Is the block on the valid list?
			if (!list_empty(&block->head)) {
				// Remove block from the valid list
				list_del(&block->head);
			}

			// Forget about frags associated with the block's previous use
			empty_frags_list(block);

			// Zeroise the block
			block->stale = 0;
			block->is_hole = 0;
			block->count = 0;
//printk("cache_get_for_fill() Context %p, returning block %p from LRU list, block->count %u\n", context, block, block->count);
		}
	}

	if (block) {
		// Remember from where on disk this block is going to be filled so we
		// can search the filling list when invalidate is called
		block->start = start_sector;

		// Add block to filling list
		list_add(&block->head, &context->cache->filling);
//printk("cache_get_for_fill() Context %p, block %p start set to %llu, block->count %u\n", context, block, block->start, block->count);
	}

	spin_unlock_bh(&context->lock);

	return block;
}

static void cache_add_free(
	incoherent_sendfile_context_t *context,
	incoherent_cache_block_t      *block)
{
	spin_lock_bh(&context->lock);
//printk("cache_add_free() Context %p, block %p\n", context, block);
	// Remove the block from any lists it's currently on
	if (!list_empty(&block->head)) {
//printk("cache_add_free() Context %p, block %p, unhooked head\n", context, block);
		list_del(&block->head);
	}
	if (!list_empty(&block->lru_head)) {
//printk("cache_add_free() Context %p, block %p, unhooked lru_head\n", context, block);
		list_del_init(&block->lru_head);
	}
	spin_unlock_bh(&context->lock);

	// Forget about any frags describing the block
	empty_frags_list(block);

	// Zeroise the block
	block->stale = 0;
	block->is_hole = 0;
	block->count = 0;
	block->start = 0;

	// Add the block to the free list
	spin_lock_bh(&context->lock);
	list_add(&block->head, &context->cache->free);
	spin_unlock_bh(&context->lock);

}

/* Called with filemap read lock held. Lock should be dropped before returning
 * regardless of whether a disk read was successfully started or not
 */
static int fill_block_start(fill_progress_t *fill_progress)
{
	fill_que_entry_t              *entry = fill_progress->entry;
	incoherent_sendfile_context_t *context = fill_progress->context;
	size_t                         remaining = entry->remaining;
	sector_t		               sector = entry->sector;
	struct list_head              *p, *n;

//printk("fill_block_start() Context %p, entry %p, sector %lld, remaining %d\n", context, entry, sector, remaining);
	while (remaining) {
		incoherent_cache_block_t *block;

//printk(KERN_INFO "fill_block_start() Calling cache_get_for_fill, context %p, sector %lld\n", context, sector);
		// Get an unused block, recording with it the sector on disk it will be filled from
		block = cache_get_for_fill(context, sector);
		if (!block) goto free_blocks;

//printk(KERN_INFO "fill_block_start() Block to fill 0x%p, sector %lld\n", block, entry->sector);
		// Associate the block with the fill entry
		list_add_tail(&block->fill_head, &entry->blocks);

		sector += INCOHERENT_ALLOC_SECTORS;
		remaining -= INCOHERENT_ALLOC_BYTES;
	}

	// Keep a record of the total fill length for use during searches
	fill_progress->fill_len_sectors = entry->remaining >> SECTOR_SHIFT;

	// Record the first block to be filled
	fill_progress->block = list_entry(entry->blocks.next, incoherent_cache_block_t, fill_head);

	// Always fill entire blocks
	fill_progress->block_offset = 0;

	// Record that no progress has yet been made with the fill
	fill_progress->last_read_count = 0;

	// Read the first contiguous sectors from the disk
//printk("fill_block_start() Successfully allocated blocks for context %p, sector %lld, sector count %d\n",context, sector, fill_progress->fill_len_sectors);
	return fill_block_step_next(fill_progress);

free_blocks:
//printk("fill_block_start() Failed to allocate all blocks for context %p, sector %lld, count %d\n", context, sector, entry->remaining);
	// Free all blocks that were successfully allocated
	list_for_each_safe(p, n, &entry->blocks) {
		incoherent_cache_block_t *block = list_entry(p, incoherent_cache_block_t, fill_head);
		list_del_init(p);
		cache_add_free(context, block);
	}

	// Not starting disk read, so ensure we drop the filemap read lock before
	// returning
	up(&context->inode->reader_sem);

	return -ENOMEM;
}

static void deferred_fill_work(struct work_struct *not_used)
{
	struct inode *inode;
	direct_access_context_t *sata_context;

//printk("defer_fill() Fill start for entry %p, context %p\n", current_fill_progress_s->entry, current_fill_progress_s->context);

	// Spinlock not held so ensure this CPU correctly sees any changes made to
	// fill structures by other CPUs
	smp_rmb();
BUG_ON(sata_lock_held_s);
BUG_ON(!current_fill_progress_s);

	inode = current_fill_progress_s->context->inode;

	write_flush_filemap_if_required(inode);
//printk("deferred_fill_work() Any write flushing completed %p \n", inode);

	// Acquire SATA core
	sata_context = inode->filemap_info.direct_access_context;

	if ((*sata_context->acquire)(sata_context, SATA_ACQUIRE_TIMEOUT_JIFFIES,
		current_fill_progress_s->context, 1)) {
		goto acquire_fail;
	}

	// Fast readers have the SATA lock
	sata_lock_held_s = 1;

	// We should not hold the SATA lock for too long if others need it
	lock_hold_limit_jiffies_s = jiffies + LOCK_HOLD_LIMIT_JIFFIES;

	// Must ensure read is working on an upto date filemap then acquire the
	// filemap read lock so disk reads can be sure of consistent filemap state
	lock_filemap(inode);

	if (unlikely(current_fill_progress_s->entry->disk_read_started)) {
//printk("defer_fill() entry %p fill has been started so call fill step\n", current_fill_progress_s->entry);
		// We can only reach here if the tasklet has called fill_block_step_first()
		// and found it needs to invoke fill_block_step_next() to do further
		// disk reads for the fill entry, but was unable to do so as the filemap
		// access lock was contented
		if (unlikely(!fill_block_step_next(current_fill_progress_s))) {
//printk("defer_fill() entry %p fill has NOT finished\n", current_fill_progress_s->entry);
			// Another fill from disk has been started as there is still more
			// filling to do for the current fill entry. When finished ISR and
			// then tasklet will be invoked again
		} else {
			// Fill is complete, so invoke tasklet to cleanup and chose next fill

			// Ensure changes to fill structures made without spinlock held will be
			// seen correctly by another CPU
			smp_wmb();

			// Complete processing for this entry and look for another
			tasklet_hi_schedule(&tasklet_s);
		}
	} else {
		int fill_start_result = fill_block_start(current_fill_progress_s);

		if (unlikely(fill_start_result)) {
			if (fill_start_result > 0) {
				// Fill hit only holes on disk, so no disk read required
//printk("defer_fill() Fill start for entry %p  found only holes\n", current_fill_progress_s->entry);
			} else if (fill_start_result < 0) {
				// Current implementation w/o net tx usage tracking will never not
				// be able to get a block to associate with a fill, in which case
				// fill_block_start() will never fail
//printk("defer_fill() Fill start for entry %p returned error %d\n", current_fill_progress_s->entry, fill_start_result);
				current_fill_progress_s->entry->aborted = 1;
			}

			// Ensure changes to fill structures made without spinlock held will be
			// seen correctly by another CPU
			smp_wmb();

			// Complete processing for this entry and look for another
			tasklet_hi_schedule(&tasklet_s);
		}
	}

	return;

acquire_fail:
printk(KERN_WARNING "deferred_fill_work() Failed to acquire SATA core\n");

	// Spinlock not held so ensure this CPU correctly sees any changes made to
	// fill structures by other CPUs
	smp_rmb();

	// The fill was not started as the SATA core could not be acquired
//printk("deferred_fill_work() Aborting entry %p due to SATA core acquire failure\n", current_fill_progress_s->entry); 
	current_fill_progress_s->entry->aborted = 1;

	// Ensure changes to fill structures made without spinlock held will be
	// seen correctly by another CPU
	smp_wmb();

	// The entry has been queued for fill, so readers may already be waiting on
	// its completion, so arrange for those readers to be woken even though
	// we're not going to actually fill it.
	//
	// NB We are invoking the tasklet without holding the SATA lock. In this
	// case the tasklet should cleanup the current fill, but not then schedule
	// another fill - instead it should leave the queues in such a state that
	// only another actual read request will cause resumption of filling
	//
//printk("deferred_fill_work() Poking tasklet to wake any waiters for entry %p\n", current_fill_progress_s->entry); 
	tasklet_hi_schedule(&tasklet_s);

	return;
}

static void defer(void)
{
	incoherent_sendfile_context_t *context = current_fill_progress_s->context;
	direct_access_context_t *sata_context = context->inode->filemap_info.direct_access_context;
	void (*release_fn)(int) = sata_context->release;

//printk("defer() Inode %p context %p deferring\n", context->inode, context);

	// Drop SATA lock so other have a chance to do some work
	(*release_fn)(1);

	// Fast readers do not have the SATA lock
	sata_lock_held_s = 0;

	// Deferred work does not hold the fill spinlock, so ensure the worker
	// thread will see all relevant changes
	smp_wmb();

	// Schedule work to compete for re-acquisition of the SATA lock
	schedule_work(&fast_reads_work);
}

/*
 * Used to setup next block fill without spending ages in ISR or incurring
 * context switch overhead as might be the case with a kernel thread
 */
static void tasklet_func(unsigned long arg)
{
	int fill_start_result;
	fill_que_entry_t *entry, *next_entry;
//	incoherent_sendfile_context_t *fill_context;	// For use of debug messages only
//printk("tasklet_func() Entered\n");

again:
//printk("tasklet_func() entry %p for context %p\n", current_fill_progress_s->entry, current_fill_progress_s->context);
	// Spinlock not held so ensure this CPU correctly sees any changes made to
	// fill structures by other CPUs
	smp_rmb();
	BUG_ON(!current_fill_progress_s);

	entry = current_fill_progress_s->entry;
//	fill_context = current_fill_progress_s->context;	// For use of debug messages only

	if (likely(sata_lock_held_s) && likely(entry->disk_read_started)) {
//printk("tasklet_func() entry %p fill has been started so call fill step\n", entry);
		// The first part of fill_block_step processing will not attempt to
		// read from disk and therefore does not need the filemap access lock
		if (unlikely(!fill_block_step_first(current_fill_progress_s))) {
			// Fill has not been completed, so must acquire filemap lock before
			// calling fill_block_step_next() that may need to be disk read
			if (!trylock_filemap(current_fill_progress_s->context->inode)) {
				// Could not immediately acquire the filemap read lock, or filemap
				// updates were pending
				defer();
				goto out;
			}

			if (unlikely(!fill_block_step_next(current_fill_progress_s))) {
//printk("tasklet_func() entry %p fill has NOT finished\n", entry);
				// Another fill from disk has been started as there is still more
				// filling to do for the current fill entry. When finished ISR and
				// then this tasklet will be invoked again
				goto out;
			}
		}
	}

//printk("tasklet_func() entry %p fill has finished\n", entry);
	entry->filled = 1;
	current_fill_progress_s->eof = 0;

//if (entry->remaining) {
//printk("tasklet_func() Entry %p had %d bytes of fill remaining\n", entry, entry->remaining);
//}

	if (unlikely(entry->aborted)) {
		incoherent_sendfile_context_t *context = current_fill_progress_s->context;

		// The fill was never completed and thus cache_add_block() was never
		// called. Return any blocks to the free list
		struct list_head *p, *n;
		list_for_each_safe(p, n, &entry->blocks) {
			incoherent_cache_block_t *block =
				list_entry(p, incoherent_cache_block_t, fill_head);
			list_del_init(p);
//printk("tasklet_func() Moving block %p of aborted fill entry %p on context %p to free list\n", block, entry, context);
			cache_add_free(context, block);
		}
	}

	// Return the fill entry to the tail of free list so it won't get used again
	// quickly, as waiters may be waking up when we return and removing
	// themselves from the entry's wait queue
	spin_lock(&fill_entries_lock_s);
	list_add_tail(&entry->list, &free_fill_entries_s);
	spin_unlock(&fill_entries_lock_s);

	// Hold fill lock while manipulating global and context-local fill queues
	spin_lock(&fill_lock_s);

	// We were called to cleanup after the current fill that has had to be
	// abandoned due to us not managing to acquire the SATA lock
	if (unlikely(!sata_lock_held_s)) {
printk("tasklet() Cleaning up after fill abandoned due to not getting SATA lock\n");
		current_fill_progress_s->in_progress = 0;
		current_fill_progress_s = 0;
		wake_up(&entry->wait_queue);
		spin_unlock(&fill_lock_s);
		goto out;
	}

	if (list_empty(&current_fill_progress_s->queue)) {
		// Are there any other contexts with fills queued?
		if (!list_empty(&pending_fills_s)) {
			// Mark current context as having no more pending fills against it
			// before looking for other contexts with pending fills
			current_fill_progress_s->in_progress = 0;
		} else {
			incoherent_sendfile_context_t *context = current_fill_progress_s->context;
			direct_access_context_t *sata_context = context->inode->filemap_info.direct_access_context;
			void (*release_fn)(int) = sata_context->release;

			// Mark current context as having no more pending fills against it
			// before we drop the fill lock so we're synchronised with task
			// level code trying to cleanup the context before closing inode
			current_fill_progress_s->in_progress = 0;

//printk("tasklet_func() Finished fill [1] of entry %p for context %p, releasing SATA lock and waking any waiting tasks\n", entry, current_fill_progress_s->context);
			// Record that there are no pending fills against any contexts
			current_fill_progress_s = 0;

			// No more filling to do so relinquish ownership of the SATA core.
//printk("Release SATA core\n");
			(*release_fn)(1);

			sata_lock_held_s = 0;

			// Wake any tasks waiting for completion of the block fill. Delayed
			// to this point so the in_progress flag is always zeroed before
			// wake_up is called, and both operations occur atomically under the
			// fill lock, so there can be no chance of task level waiting for
			// fill to complete because in_progress is set but never being
			// awoken because the wake_up has already been done
			wake_up(&entry->wait_queue);

			spin_unlock(&fill_lock_s);

			goto out;
		}
	} else {
		// Further fills are pending for the context - have we been continuously
		// serving fills for the current context for a long time?
		if (unlikely(time_after(jiffies, queue_schedule_limit_jiffies_s))) {
			// Yes - are there other contexts waiting for fills
			if (!list_empty(&pending_fills_s)) {
				// Yes - queue the context we're switching away from at the tail
				// of the list of contexts with queued fills so it will be the
				// last to get served again
				list_add_tail(&current_fill_progress_s->list, &pending_fills_s);

				// No fills currently in progress against the context we're
				// switching away from
				current_fill_progress_s->in_progress = 0;

				// Select the context that has been waiting the longest
//printk("tasklet_func() Forcing switch to filling another context\n");
				goto pick_new_context;
			}

			// Wait a little longer before checking for waiting context again
			queue_schedule_limit_jiffies_s = jiffies + QUEUE_SCHEDULE_LIMIT_INC_JIFFIES;
		}

		goto pick_next_fill;
	}

pick_new_context:
	current_fill_progress_s = list_entry(pending_fills_s.next, fill_progress_t, list);
	BUG_ON(!current_fill_progress_s);
	BUG_ON(list_empty(&current_fill_progress_s->queue));

	// Currently active context's fill progress does not remain on the list
	list_del_init(&current_fill_progress_s->list);

	// Start the timer used to detect when we should force a switch to filling
	// for another context
	queue_schedule_limit_jiffies_s = jiffies + QUEUE_SCHEDULE_LIMIT_JIFFIES;

pick_next_fill:
	// Select the oldest queued fill for the current context
	next_entry = list_entry(current_fill_progress_s->queue.next, fill_que_entry_t, list);
	BUG_ON(!next_entry);

	// Currently active fill does not remain on the list
	list_del_init(&next_entry->list);

	// Record which fill we're going to start for the context
	current_fill_progress_s->entry = next_entry;
//printk("tasklet_func() Context %p -> next_entry %p, pos %lld\n", current_fill_progress_s->context, next_entry, next_entry->pos);

	// Record which fill we're going to start for the context
	current_fill_progress_s->entry = next_entry;
	current_fill_progress_s->in_progress = 1;

	// Wake any tasks waiting for completion of the block fill. Delayed to this
	// point so the in_progress flag is always zeroed before wake_up is called,
	// and both operations occur atomically under the fill lock, so there can be
	// no chance of task level waiting for fill to complete because in_progress
	// is set but never being awoken because the wake_up has already been done
//printk("tasklet_func() Finished fill [2] of entry %p, waking any waiting tasks\n", entry);
	wake_up(&entry->wait_queue);

	if (time_after(jiffies, lock_hold_limit_jiffies_s)) {
		if (sata_core_has_waiters()) {
			defer();
			spin_unlock(&fill_lock_s);
			goto out;
		} else {
			// No one waiting, so keep core for a little longer before
			// checking again
			lock_hold_limit_jiffies_s = jiffies + LOCK_HOLD_INC_JIFFIES;
		}
	}

	if (!trylock_filemap(current_fill_progress_s->context->inode)) {
		// Could not immediately acquire the filemap read lock, or filemap
		// updates were pending
		defer();
		spin_unlock(&fill_lock_s);
		goto out;
	}

	// Finished manipulating global and context-local fill queues
	spin_unlock(&fill_lock_s);

//printk("tasklet_func() Fill start for entry %p, context %p\n", current_fill_progress_s->entry, current_fill_progress_s->context);
	fill_start_result = fill_block_start(current_fill_progress_s);
	if (unlikely(fill_start_result)) {
		if (fill_start_result > 0) {
			// Fill hit only holes on disk, so no disk read required
//printk("tasklet_func() Fill start for entry %p  found only holes\n", current_fill_progress_s->entry);
		} else if (fill_start_result < 0) {
			// Current implementation w/o net tx usage tracking will never not
			// be able to get a block to associate with a fill, in which case
			// fill_block_start() will never fail
//printk("tasklet_func() Fill start for entry %p returned error %d\n", current_fill_progress_s->entry, fill_start_result);
			current_fill_progress_s->entry->aborted = 1;
		}

		// Ensure changes to fill structures made without spinlock held will be
		// seen correctly by another CPU
		smp_wmb();

		// Complete processing for this entry and look for another
		goto again;
	}

out:
	return;
}

/* flag 0 - reads and flag 1 - writes */
int init_filemapinfo(
	struct inode *inode,
	int           flag)
{
	int                  retval = 0;

	return retval;
}

/*
 * Externally visible call to have the incoherent sendfile context for the
 * specified file recompute the filemap info, presumably due to a change brought
 * about by a file write - could be extending the file, or perhaps having
 * written to holes
 *
 * NB: May want to do lazy remapping so that we only remap the next time we have
 * a incoherent sendfile call - this may allow us to roll up several extending
 * writes into a single remap on the next read
 */
void incoherent_sendfile_remap_file(struct inode *inode)
{
	BUG_ON(!inode);
//printk(KERN_INFO "incoherent_sendfile_remap_file() Inode %p\n", inode);

	/* Remember that a filemap update is required before its next use */
	while (down_timeout(&inode->reader_sem, HZ)) {
		printk("incoherent_sendfile_remap_file() A second has elapsed while waiting, inode %p\n", inode);
	}

	inode->filemap_update_pending = 1;
	smp_wmb();

	up(&inode->reader_sem);
}

/*
 * Invalidate any cache blocks covering the passed file interval, both for
 * available blocks and for any in-progress fills from disk
 */
void incoherent_sendfile_invalidate_cache(
	struct inode *inode,
	loff_t        start,
	loff_t        length)
{
	loff_t end = start + length;
	struct list_head *p;
	int do_invalidation = 1;

	BUG_ON(!inode);

//printk(KERN_INFO "incoherent_sendfile_invalidate_cache() Inode %p over interval %lld, %lld\n", inode, start, end);

	while (down_timeout(&inode->reader_sem, HZ)) {
		printk("incoherent_sendfile_invalidate_cache() A second has elapsed while waiting, inode %p\n", inode);
	}

	/* Don't bother searching for contexts to invalidate if we haven't yet
	 * allocated a filemap, i.e. no fast readers yet active */
	smp_rmb();
	if (!inode->filemap_info.map) {
//printk(KERN_INFO "incoherent_sendfile_invalidate_cache() Inode %p, no map yet allocated so ignoring\n", inode);
		do_invalidation = 0;
	}

	up(&inode->reader_sem);

	if (!do_invalidation) {
		return;
	}

	read_lock(&inode->fast_files_lock);
	list_for_each(p, &inode->fast_files) {
		struct file *file = list_entry(p, struct file, fast_head);
		void *ptr;
		incoherent_sendfile_context_t *context;
		struct list_head *q, *n;

		/* Assignment of context should be done with a single pointer setting
		 * so should be OK to test here without locks
		 */
		ptr = file->fast_context;
		if (!ptr)
			continue;

		context = (incoherent_sendfile_context_t*)ptr;
//printk(KERN_INFO "incoherent_sendfile_invalidate_cache() Inode %p, file %p, context %p\n", inode, file, context);

		/* Serialize access to the context's block cache */
		spin_lock_bh(&context->lock);

		/*
		 * We would hope that most of the time writers and readers will not
		 * be stepping on each others toes so that the list searches below
		 * should normally not find anything, in which case it makes sense
		 * to use the non-prefetching versions of the list iteration macros
		 */

		/* Search the valid list for blocks intersecting the given address
		 * range and remove any blocks found from the valid list and mark
		 * those blocks as stale */
		list_for_each_safe(q, n, &context->cache->valid) {
			incoherent_cache_block_t *block =
				list_entry(q, incoherent_cache_block_t, head);

			loff_t b_s = block->start << SECTOR_SHIFT;
			loff_t b_e = b_s + INCOHERENT_ALLOC_BYTES;

			/* If the given range overlaps the block */
			if (((b_e > start) && (b_e <= end)) ||
				((b_s >= start) && (b_s < end))) {
//printk(KERN_INFO "incoherent_sendfile_invalidate_cache() Inode %p, file %p, context %p, marking valid block %p (%lld,%lld) as stale\n", inode, file, context, block, b_s, b_e);
				block->stale = 1;
				list_del_init(&block->head);
			}
		}

		/* Search filling list for blocks intersecting the given address
		 * range and mark any blocks found as stale */
		__list_for_each(q, &context->cache->filling) {
			incoherent_cache_block_t *block =
				list_entry(q, incoherent_cache_block_t, head);

			loff_t b_s = block->start << SECTOR_SHIFT;
			loff_t b_e = b_s + INCOHERENT_ALLOC_BYTES;

			/* If the given range overlaps the block */
			if (((b_e > start) && (b_e <= end)) ||
				((b_s >= start) && (b_s < end))) {
//printk(KERN_INFO "incoherent_sendfile_invalidate_cache() Inode %p, file %p, context %p, marking filling block %p (%lld,%lld) as stale\n", inode, file, context, block, b_s, b_e);
				block->stale = 1;
			}
		}

		spin_unlock_bh(&context->lock);
	}
	read_unlock(&inode->fast_files_lock);
}

static int alloc_block_cache(incoherent_cache_t *cache)
{
	int i;
	int retval = 0;
//printk(KERN_INFO "alloc_block_cache() Entered for cache %p\n", cache);

	for (i=0; i < CONFIG_OXNAS_FAST_READ_BLOCKS_PER_CACHE; i++) {
		incoherent_cache_block_t *block = &cache->blocks[i];
		struct page              *pages;
		int                       j;

//printk(KERN_INFO "alloc_block_cache() Allocating pages for block 0x%p\n", block);
		block->start = 0;
		block->count = 0;
		block->context = NULL;
		block->stale = 0;
		block->is_hole = 0;

		block->pages = alloc_pages(GFP_KERNEL, CONFIG_OXNAS_FAST_READ_ALLOC_ORDER);
		if (!block->pages) {
			retval = -ENOMEM;
			goto out;
		}

		pages = block->pages;
		for (j=0; j < INCOHERENT_ALLOC_PAGES; j++) {
			init_page_count(pages);
			__SetPageIncoherentSendfile(pages);
			pages->private = (unsigned long)block;
			block->ptbl[j] = pages++;
		}
		atomic_set(&block->users, 0);

		INIT_LIST_HEAD(&block->head);
		INIT_LIST_HEAD(&block->lru_head);
		INIT_LIST_HEAD(&block->fill_head);
		INIT_LIST_HEAD(&block->frags_head);
	}

out:
	return retval;
}

static incoherent_cache_t* acquire_block_cache(void)
{
	incoherent_cache_t *cache = 0;

	/* Try to get a cache from the list of free caches */
	spin_lock(&caches_lock_s);
	if (list_empty(&free_caches_s)) {
		spin_unlock(&caches_lock_s);
//printk(KERN_INFO "acquire_block_cache() No cache available\n");
	} else {
		int i;
		struct list_head *p, *n;

		// Get an unused cache
		cache = list_entry(free_caches_s.next, incoherent_cache_t, list);

		// Remove the cache from the free list
		list_del_init(&cache->list);
		spin_unlock(&caches_lock_s);

		// Empty all the block lists */
		list_for_each_safe(p, n, &cache->free) {
			list_del_init(p);
		}
		list_for_each_safe(p, n, &cache->lru) {
			list_del_init(p);
		}
		list_for_each_safe(p, n, &cache->valid) {
			list_del_init(p);
		}
		list_for_each_safe(p, n, &cache->filling) {
			list_del_init(p);
		}
//printk(KERN_INFO "acquire_block_cache() List free  %s %d\n",   list_empty(&cache->free)    ? "empty" : "not empty");
//printk(KERN_INFO "acquire_block_cache() List lru   %s %d\n",   list_empty(&cache->lru)     ? "empty" : "not empty");
//printk(KERN_INFO "acquire_block_cache() List valid %s %d\n",   list_empty(&cache->valid)   ? "empty" : "not empty");
//printk(KERN_INFO "acquire_block_cache() List filling %s %d\n", list_empty(&cache->filling) ? "empty" : "not empty");

		/* For each block in the cache */
		for (i=0; i < CONFIG_OXNAS_FAST_READ_BLOCKS_PER_CACHE; i++) {
			incoherent_cache_block_t *block = &cache->blocks[i];
			struct page              *pages = block->pages;
			int                       j;

			/* Re-initialise state of all pages in the block */
			for (j=0; j < INCOHERENT_ALLOC_PAGES; j++) {
				init_page_count(pages++);
			}

			/* Forget about any frags describing the block */
			empty_frags_list(block);

			/* Zeroise the block */
			block->stale = 0;
			block->is_hole = 0;
			block->start = 0;
			block->count = 0;
		}
	}

	return cache;
}

static void assign_block_cache(
	incoherent_sendfile_context_t *context,
	incoherent_cache_t            *cache)
{
	int i;
//printk(KERN_INFO "assign_block_cache() Assigning cache %p to context %p\n", cache, context);

	/* For each block in the cache */
	for (i=0; i < CONFIG_OXNAS_FAST_READ_BLOCKS_PER_CACHE; i++) {
		/* Point each block at the context */
		incoherent_cache_block_t *block = &cache->blocks[i];
		block->context = context;

		/* Add the block to the list of unused blocks */
		list_add(&block->head, &cache->free);
//printk(KERN_INFO "assign_block_cache() Cache %p, block %p assigned to context %p\n", cache, block, block->context);
	}

	context->cache = cache;
}

static void init_fill_entry(
	fill_que_entry_t *entry,
	loff_t            pos,
	size_t            count)
{
	int    num_blocks;
	loff_t requested_pos = pos;

	// Always fill from start of block
	pos >>= INCOHERENT_BLOCK_SHIFT;
	pos <<= INCOHERENT_BLOCK_SHIFT;
	entry->pos = pos;

	// Record the disk sector associated with the start of the cache block
	entry->sector = pos >> SECTOR_SHIFT;

	// Adjust amount to fill to account for repositioning to start of block
	count += (requested_pos - pos);

	// Round fill amount upto a whole number of blocks
	num_blocks = count >> INCOHERENT_BLOCK_SHIFT;
	if ((num_blocks << INCOHERENT_BLOCK_SHIFT) < count) {
		++num_blocks;
	}
	entry->remaining = num_blocks << INCOHERENT_BLOCK_SHIFT;

	// A fill can span several blocks
	INIT_LIST_HEAD(&entry->blocks);

	// A fill is not complete, and any waiters awakened, until all blocks have been filled
	entry->filled = 0;
	entry->aborted = 0;
	entry->stale = 0;
	entry->disk_read_started = 0;
	entry->failed = 0;
}

static void incoherent_sendfile_free_filemap(struct inode *inode)
{
	kfree(inode->filemap_info.map);
	inode->filemap_info.map = 0;

	if (inode->filemap_info.direct_access_context) {
		free_direct_sata_context(inode->filemap_info.direct_access_context);
		inode->filemap_info.direct_access_context = NULL;
	}
}

void incoherent_sendfile_check_and_free_filemap(struct inode * inode)
{
//printk("incoherent_sendfile_check_and_free_filemap() Entered inode %p\n", inode);
	if (!down_trylock(&inode->reader_sem)) {
//printk("incoherent_sendfile_check_and_free_filemap() Doing work for inode %p\n", inode);
		incoherent_sendfile_free_filemap(inode);
		up(&inode->reader_sem);
	}
//printk("incoherent_sendfile_check_and_free_filemap() Leaving inode %p\n", inode);
}

/* Called with inode.fast_lock held, will drop once safe against parallel
 * context allocation for same file
 */
static int alloc_context(
	struct inode *inode,
	struct file  *file)
{
	int retval = 0;

	if (unlikely(!file->fast_context)) {
		incoherent_cache_t *cache;

		/* Prevent parallel context allocation while fast lock not held. This
		 * never happen as Samba should not issue another operation on this file
		 * handle until the present one - the first sendfile - has finished
		 */
		file->fast_context = (void*)-1;
		spin_unlock(&inode->fast_lock);

//printk(KERN_INFO "alloc_context() Inode %p, file %p\n", inode, file);
		cache = acquire_block_cache();
		if (!cache) {
//printk("alloc_context() Inode %p, file %p failed to allocate cache\n", inode, file);
			file->fast_context = 0;
			retval = -EIO;
		} else {
			incoherent_sendfile_context_t *context =
				kzalloc(sizeof(incoherent_sendfile_context_t), GFP_KERNEL);
			if (!context) {
				printk(KERN_WARNING "alloc_context() Failed to allocate context for file %p\n", file);
				release_block_cache(cache);
				file->fast_context = 0;
				retval = -EIO;
			} else {
				spin_lock_init(&context->lock);
				INIT_LIST_HEAD(&context->fill_progress.list);
				INIT_LIST_HEAD(&context->fill_progress.queue);
				context->fill_progress.context = context;
//printk("alloc_context() Context %p, &fill_progress %p\n", context, &context->fill_progress);

				/* Initialise index of last cache block explicitly requested by reader */
				context->last_block_index = -2;

				/* Initialise cache of filemap indexing info */
				context->cur_map_entry = 1;
				context->cur_map_entry_offset = 0;
				context->next_offset = 0;

				/* Initialise record of first block beyond end of last readahead */
				context->last_ra_end_index = 0;

				/* Attach the block cache to the context */
				assign_block_cache(context, cache);

				context->inode = inode;
				file->fast_context = context;
			}
		}

		spin_lock(&inode->fast_lock);
	}

//printk(KERN_INFO "alloc_context() Inode %p, file %p, context %p, cache %p\n", inode, file, file->fast_context, cache);
	return retval;
}

/*
 * Called with inode.fast_lock held and returns in same state. May drop lock
 * during execution
 */
void do_fast_fallback(struct inode *inode)
{
	int fallback_count = 0;
#ifdef CONFIG_OXNAS_FAST_WRITES
	int once = 1;
#endif // CONFIG_OXNAS_FAST_WRITES

	// Drop fast lock while freeing context/filemap as those operations involve
	// taking other locks. Safe as the fallback_in_progress flag is already set
	// so noone else will attempt to invoke do_fast_fallback, open/close fast
	// files or start a new fast read operation via the sendfile syscall
	spin_unlock(&inode->fast_lock);

//printk("do_fast_fallback() Inode %p\n", inode);
	// Free all fast read resources for the inode
	while (1) {
		struct file *file;
		incoherent_sendfile_context_t *context;

		// Fallback operations may need to take the fast-files list access lock
		// so only hold it here for minimum time necessary
		write_lock(&inode->fast_files_lock);

		if (list_empty(&inode->fast_files)) {
			// All files forced back to non-fast access
			write_unlock(&inode->fast_files_lock);
			break;
		}

		// Get the first file on the fast-files list
		file = list_entry(inode->fast_files.next, struct file, fast_head);
//printk("do_fast_fallback() Inode %p file %p\n", inode, file);

		// Force file mode from fast to normal
		file->f_flags &= ~O_FAST;

		// Record that file no longer open for fast mode, but normal instead
		list_del(&file->fast_head);
		++fallback_count;

		// Removed the fast-file from the list so can drop the list access lock
		write_unlock(&inode->fast_files_lock);

		// Was a context allocated for this slot in the inode?
		context = file->fast_context;
		if (context) {
//printk("do_fast_fallback() Inode %p file %p, freeing context %p\n", inode, file, context);
			incoherent_sendfile_free_context(context);
			file->fast_context = NULL;
		}
#ifdef CONFIG_OXNAS_FAST_WRITES	
		if (once && file->fast_write_context) {
			once = 0;
			writer_reset_prealloc(file);
		}
		if (file->fast_write_context) {
//printk("do_fast_fallback() - cleaning write - Inode %p file %p, freeing context %p\n", inode, file, context);
			complete_fast_write(file);
			file->fast_write_context = NULL;
		}
#endif // CONFIG_OXNAS_FAST_WRITES
	}

//printk("do_fast_fallback() Inode %p, freeing filemap %p\n", inode, inode->filemap_info.map);
	incoherent_sendfile_free_filemap(inode);
#ifdef CONFIG_OXNAS_FAST_WRITES
//printk("do_fast_fallback() Inode %p, freeing filemap %p\n", inode, inode->filemap_info.map);
	fast_write_check_and_free_filemap(inode);
#endif // CONFIG_OXNAS_FAST_WRITES

	// Re-take lock before updating fallback_in_progress
	spin_lock(&inode->fast_lock);

	// All fast readers fallen back to normal
//printk("do_fast_fallback() Inode %p, fallback_count %d\n", inode, fallback_count);
	inode->fast_open_count = 0;
	inode->fallback_in_progress = 0;
	inode->normal_open_count += fallback_count;

//printk("do_fast_fallback() Inode %p, waking all waiters now that fallback is complete\n", inode);
	wake_up(&inode->fallback_wait_queue);
}

/*
 * Called with inode.fast_lock held and returns in same state. May drop lock
 * during execution
 */
void wait_fallback_complete(struct inode *inode)
{
	DEFINE_WAIT(wait);
	for (;;) {
		prepare_to_wait(&inode->fallback_wait_queue, &wait, TASK_UNINTERRUPTIBLE);
		if (!inode->fallback_in_progress) {
			break;
		}
		spin_unlock(&inode->fast_lock);
		if (!schedule_timeout(HZ)) {
			printk(KERN_INFO "wait_fallback_complete() A second has passed while waiting inode %p\n", inode);
		}
		spin_lock(&inode->fast_lock);
	}
	finish_wait(&inode->fallback_wait_queue, &wait);
}

/*
 * We can no longer support fast mode for this inode so we should cause any
 * active fast mode reads and writes to fallback to normal mode, free the 
 * filemap then force this new fast request to normal mode
 *
 * Called with inode.fast_lock held and returns in same state. May drop lock
 * during execution
 */
void fast_fallback(struct inode *inode)
{
//printk("fast_fallback() Inode %p\n", inode);
	if (unlikely(inode->fallback_in_progress)) {
		// Someone else is driving the fallback operation, so just wait for it
		// to complete
//printk("fast_fallback() Inode %p, fallback already in progress\n", inode);
		wait_fallback_complete(inode);
//printk("fast_fallback() Inode %p, Wait for fallback completion complete\n", inode);
	} else {
		// We have to initiate the fallback
		inode->fallback_in_progress = 1;

		if (inode->fast_reads_in_progress_count
#ifdef CONFIG_OXNAS_FAST_WRITES 
				|| inode->fast_writes_in_progress_count
#endif // CONFIG_OXNAS_FAST_WRITES
				) {
			// There are some fast reads or writes in progress, so must wait for these to
			// complete. Once the last in-progress reader/ writer finishes it will see
			// the fallback flag set and initiate the fallback
//printk("fast_fallback() Inode %p, flagging fallback required to in-progress readers\n", inode);
			wait_fallback_complete(inode);
//printk("fast_fallback() Inode %p, in-progress readers all finished and fallback completed\n", inode);
		} else {
			// As there are no fast read operations in progress, we can initiate
			// the fallback of all readers to normal mode
//printk("fast_fallback() Inode %p, beginning fallback\n", inode);
			do_fast_fallback(inode);
//printk("fast_fallback() Inode %p, fallback complete\n", inode);
		}
	}
}

/* Called with inode.fast lock held */
static int do_initial_file_prep(
	struct inode *inode,
	struct file  *file)
{
	int retval = 0;

	if (unlikely(!inode->filemap_info.map)) {
//printk("do initial read prep for inode %p\n", inode);
		spin_unlock(&inode->fast_lock);

		while (down_timeout(&inode->reader_sem, HZ)) {
			printk("do_initial_file_prep() A second has elapsed while waiting, inode %p\n", inode);
		}

		// Did another user initialise the filemap
		smp_rmb();
		if (inode->filemap_info.map) {
			// Yes, so nothing more to do
//printk("do_initial_file_prep() Inode %p, file %p someone else is initialising the filemap\n", inode, file);
			goto out;
		}

		// First fast read on inode so in case there is data hanging around
		// in the pagecache from previous normal writes we need to flush and
		// invalidate the pagecache for this file
		retval = vfs_fsync(file, file->f_dentry, 0);
		if (unlikely(retval)) {
			printk(KERN_WARNING "do_initial_file_prep() Inode %p, file %p fsync failed, error %d\n", inode, file, retval);
		} else {
			/* invalidate the page cache for the file */
			truncate_inode_pages(inode->i_mapping, 0);

			/* Find partition/disk info */
			retval = init_filemapinfo(inode, 0);
			if (unlikely(retval)) {
				printk(KERN_WARNING "do_initial_file_prep() Inode %p, file %p Failed to initialise filemapinfo, error %d\n", inode, file, retval);
			} else {
//printk("do_initial_file_prep() Inode %p, file %p filemap really null, allocating\n", inode, file);
				retval = alloc_filemap(inode, 0);
				if (unlikely(retval)) {
					printk(KERN_WARNING "do_fast_context_prep() Inode %p, file %p Failed to allocate filemap, error %d\n", inode, file, retval);
				}
//printk("do_initial_file_prep() Inode %p, file %p allocated filemap %p\n", inode, file, inode->filemap_info.map);
			}
		}

		// Completed file prep that must be performed from non-atomic context
		inode->filemap_update_pending = 0;
		smp_wmb();
out:
		up(&inode->reader_sem);

		spin_lock(&inode->fast_lock);
	}

	return retval;
}

/*
 * Should be called with fill_lock_s held and will release before returning
 * return non-zero on error
 */
int schedule_fill(
	incoherent_sendfile_context_t *context,
	fill_que_entry_t              *fill_entry)
{
	int retval = 0;

	if (context->fill_progress.in_progress) {
		// Queue set of fill requests such that initial fill is first to
		// be processed when next the tasklet runs to start a new block
		// fill
//printk("Fill in progress so add new file to list\n");
		list_add_tail(&fill_entry->list, &context->fill_progress.queue);
//printk("schedule_fill() Queued entry %p for context %p, fill_progress %p, currently in-progress\n", fill_entry, context, &context->fill_progress);
		spin_unlock_bh(&fill_lock_s);
	} else {
		// Sort out scheduling this fill via the list of pending fills for all
		// contexts
		if (!current_fill_progress_s) {
			direct_access_context_t *sata_context;
			int fill_finished = 0;

			// No fill in progress on this context and no other fills globally
			// in progress so can start this fill immediately
			context->fill_progress.entry = fill_entry;
			context->fill_progress.in_progress = 1;

			current_fill_progress_s = &context->fill_progress;

			// Start the timer used to detect when we should force a switch to
			// filling for another context
			queue_schedule_limit_jiffies_s = jiffies + QUEUE_SCHEDULE_LIMIT_JIFFIES;

//printk("schedule_fill() Immediate start entry %p for context %p, fill_progress %p\n", fill_entry, context, &context->fill_progress);
			spin_unlock_bh(&fill_lock_s);

			write_flush_filemap_if_required(context->inode);
//printk("generic_file_incoherent_sendfile() Any write flushing completed %p \n", inode);

			// Acquire the SATA core whilst in context where we can sleep
//printk("Acquire SATA core\n");
			sata_context = context->inode->filemap_info.direct_access_context;

			if ((*sata_context->acquire)(sata_context, SATA_ACQUIRE_TIMEOUT_JIFFIES, context, 1)) {
				printk(KERN_WARNING "generic_file_incoherent_sendfile() Failed to acquire SATA core\n");
				retval = -EBUSY;
				fill_finished = 1;
			}
//printk("SATA core acquired\n");

			if (!retval) {
				// Fast readers have the SATA lock
BUG_ON(sata_lock_held_s);
				sata_lock_held_s = 1;

				// We should not hold the SATA lock for too long if others need it
				lock_hold_limit_jiffies_s = jiffies + LOCK_HOLD_LIMIT_JIFFIES;

				// Must ensure read is working on an upto date filemap then acquire the
				// filemap read lock so disk reads can be sure of consistent filemap state
				lock_filemap(context->inode);

				// Start disk read as no fill is in progress and no fills
				// globally pending
//printk("No Fill in progress so start fill from task level\n");
//printk("F %lld\n", current_fill_progress_s->entry->pos);
				retval = fill_block_start(current_fill_progress_s);
				if (unlikely(retval)) {
					if (retval > 0) {
						// Fill hit only holes on disk, so no disk read required
//printk("schedule_fill() Fill start for entry %p found only holes\n", current_fill_progress_s->entry);
						retval = 0;
					} else if (retval < 0) {
//printk("schedule_fill() Fill start for entry %p returned error %d\n", current_fill_progress_s->entry, retval);
					}

					fill_finished = 1;
				}
			}

			if (retval) {
				// The fill was not started due to a real error and there may
				// not be a block associcated with the fill
//printk("schedule_fill() Aborting entry %p due to error %d\n", fill_entry, retval); 
				fill_entry->aborted = 1;
			}

			if (fill_finished) {
				// Ensure changes to fill structures made without spinlock held will be
				// seen correctly by another CPU
				smp_wmb();

				// The entry has been queued for fill, so readers may already be
				// waiting on its completion, so arrange for those readers to
				// be woken even though we're not going to actually fill it.
				// Could be due to either a fill request hitting only holes on
				// disk, i.e. still a good fill, or a real failure perhaps due
				// to no block being associated with the fill
//printk("schedule_fill() Poking tasklet to complete wake any waiters for entry %p (retval = %d)\n", fill_entry, retval); 
				tasklet_hi_schedule(&tasklet_s);
			}
		} else {
			// There are other fills globally pending against other contexts so
			// we cannot start this fill immediately, so queue it
			list_add_tail(&fill_entry->list, &context->fill_progress.queue);
//printk("schedule_fill() Queued entry %p for context %p, fill_progress %p, NOT currently in progress\n", fill_entry, context, &context->fill_progress);

			// Is this context's fill list already on the global pending fill queue?
			if (list_empty(&context->fill_progress.list)) {
				// No, so queue up this context's fill request on the global
				// list of all contexts with pending fills against them so that
				// the fill we've just queued with the context's fill queue
				// eventually gets processed
				list_add_tail(&context->fill_progress.list, &pending_fills_s);
//printk("schedule_fill() Queued context %p, fill_progress %p with global pending list %p\n", context, &context->fill_progress, &pending_fills_s);
//printk("Normal OK\n");
			}

			spin_unlock_bh(&fill_lock_s);
		}
	}

	return retval;
}

/**
 * Called with fill_lock held and drops it before returning
 */
static int readahead_opportunity(
	struct inode                  *inode,
	incoherent_sendfile_context_t *context,
	loff_t                         block_index,
	loff_t                         filemap_length,
	fill_que_entry_t              *fill_entry,
	int                           *error)
{
	int    do_readahead;
	loff_t last_ra_end_index;
	int    readahead_block_offset;
	loff_t readahead_byte_offset;
	size_t readahead_bytes;
	loff_t ra_start;
	loff_t ra_end;
	loff_t ra_limit;
	int    max_readahead_blocks;

	// Is there an opportunity to do readahead?
	do_readahead = 0;
	readahead_block_offset = 1;

	// If there is currently a fill in progress and thus we hold the SATA
	// lock then we should check whether we have been holding the lock for
	// a long time and if so then we should not do a readahead if anyone
	// else is waiting for the lock such that when the currently queued
	// fills complete we will drop the lock
	if (current_fill_progress_s) {
		if (time_after(jiffies, lock_hold_limit_jiffies_s)) {
			if (sata_core_has_waiters()) {
//printk("readahead_opportunity() Inode %p context %p found waiters\n", inode, context);
				goto no_readahead;
			} else {
//printk("readahead_opportunity() Inode %p context %p: Extend lock hold time\n", inode, context);
				// No one waiting, so keep core for a little longer before
				// checking again
				lock_hold_limit_jiffies_s = jiffies + LOCK_HOLD_INC_JIFFIES;
			}
		}
	}

	last_ra_end_index = context->last_ra_end_index;
//printk("RA %lld, %lld\n", filemap_length, last_ra_end_index);

	do {
		sector_t sector_offset;
		loff_t readahead_block_index = block_index + readahead_block_offset;

		if (readahead_block_index < last_ra_end_index) {
			// Can assume block is in cache or being filled
			continue;
		}

		readahead_byte_offset = readahead_block_index << INCOHERENT_BLOCK_SHIFT;
		if (readahead_byte_offset >= filemap_length) {
			// Don't search for blocks which cannot exist within the file
			break;
		}

		sector_offset = readahead_byte_offset >> SECTOR_SHIFT;

		if (!search_pending_fills_by_sector(&context->fill_progress, sector_offset) &&
			!cache_is_block_available(context, sector_offset)) {
			// Block is not in cache, nor in the process of being filled
			do_readahead = 1;
			break;
		}
	} while (++readahead_block_offset <= CONFIG_OXNAS_FAST_READ_MAX_READAHEAD_SEARCH);

	if (!do_readahead) {
//printk("readahead_opportunity() Inode %p context %p: No readahead opportunity\n", inode, context);
		goto no_readahead;
	}

	// Determine how many blocks to fill via readahead
	ra_start = block_index + readahead_block_offset;
	ra_end = ra_start + 1;	// We know the first readahead block is not available

	// If the currently requested block is near the start of where we would like
	// to begin readahead we should reduce the size of the readahead in order
	// that we're not still filling the readahead blocks when a read request for
	// a block within that readahead range arrives
	max_readahead_blocks = CONFIG_OXNAS_FAST_READ_MAX_READAHEAD_BLOCKS;
	if (readahead_block_offset <= 2) {
		max_readahead_blocks = 1;
	}
	if (readahead_block_offset == 3) {
		max_readahead_blocks = 2;
	}

	// NB max_readahead_blocks must be >=1 as otherwise the logic below fails
	ra_limit = ra_start + max_readahead_blocks;
//printk("RB %lld, %lld, %lld\n", ra_start, ra_end, ra_limit);

	while (ra_end < ra_limit) {
		loff_t   next_byte_offset;
		sector_t next_sector_offset;
		loff_t   test_block_index = ra_end;

		if (test_block_index < last_ra_end_index) {
			// Can assume block is in cache or being filled, so stop search for
			// end of region to be read-ahead
			break;
		}

		// Test the next block for availability
		next_byte_offset = test_block_index << INCOHERENT_BLOCK_SHIFT;	
		if (next_byte_offset >= filemap_length) {
			// Don't search for blocks which cannot exist within the file
			break;
		}

		next_sector_offset = next_byte_offset >> SECTOR_SHIFT;

		if (search_pending_fills_by_sector(&context->fill_progress, next_sector_offset) ||
			cache_is_block_available(context, next_sector_offset)) {
			// Block is in cache, or in the process of being
			// filled so stop readahead at previous block
			break;
		}

		// Include next block in readahead
		++ra_end;
	}

	// Remember the index of the block just beyond the last block read-ahead
	context->last_ra_end_index = ra_end;

	readahead_bytes = (ra_end - ra_start) << INCOHERENT_BLOCK_SHIFT;
	if (!readahead_bytes) {
		goto no_readahead;
	}

	// Initialise the fill entry
	init_fill_entry(fill_entry, readahead_byte_offset, readahead_bytes);
//printk("readahead_opportunity() Inode %p context %p: Readahead at %lld, length %d\n", inode, context, readahead_byte_offset, readahead_bytes);
//printk("R %lld, %d\n", readahead_byte_offset, readahead_bytes);

	// Sort out scheduling this fill via the list of pending fills for all
	// contexts
	*error = schedule_fill(context, fill_entry);
	return 1;

no_readahead:
	spin_unlock_bh(&fill_lock_s);
	return 0;
}

static fill_que_entry_t* alloc_fill_entry(void)
{
	fill_que_entry_t *fill_entry = NULL;
	struct list_head *p;

	spin_lock_bh(&fill_entries_lock_s);

	// Detect running out of free fill entries
	BUG_ON(unlikely(list_empty(&free_fill_entries_s)));

	// Search for a free fill entry that is not still being waited on
	list_for_each(p, &free_fill_entries_s) {
		fill_entry = list_entry(p, fill_que_entry_t, list);

		if (unlikely(!list_empty_careful(&fill_entry->wait_queue.task_list))) {
			printk("alloc_fill_entry() Found entry %p which still has waiters\n", fill_entry);
		} else {
			list_del_init(p);
			break;
		}

		fill_entry = NULL;
	}

	spin_unlock_bh(&fill_entries_lock_s);

	return fill_entry;
}

static void free_fill_entry(fill_que_entry_t *fill_entry)
{
	// Return the fill entry to the free list
	spin_lock_bh(&fill_entries_lock_s);
	list_add_tail(&fill_entry->list, &free_fill_entries_s);
	spin_unlock_bh(&fill_entries_lock_s);
}

ssize_t generic_file_incoherent_sendfile(
	struct file *in_file,
	loff_t      *ppos,
	size_t       count,
	read_actor_t actor,
	void        *target)
{
	read_descriptor_t              desc;
	struct inode                  *inode;
	incoherent_sendfile_context_t *context;
	oxnas_filemap_info_t          *filemap_info;
	fill_que_entry_t              *fill_entry = NULL;
	fill_que_entry_t              *readahead_fill_entry = NULL;
	loff_t                         filemap_length = (loff_t)-1;

	BUG_ON(!in_file->inode);
	inode = in_file->inode;
	filemap_info = &inode->filemap_info;

	BUG_ON(!in_file->fast_context);
	context = in_file->fast_context;

	// Have any writers with pending data write that data to disk and
	// trigger any filemap updates as required. Must do this before
	// acquiring the SATA lock as the writers will need this to do the flush
	// of any pending data to SATA while they hold the SATA lock
	// NB To ensure readahead and write accumulation are coherent, this call
	//    will disable future write accumulation on this inode, until it is
	//    closed and re-opened of course
	write_flush_pending(inode, 1);

//printk(KERN_INFO "generic_file_incoherent_sendfile() Entered file %s, inode %p, pos %lld, count %d target = %p\n", in_file->f_path.dentry->d_name.name, inode, *ppos, count, target);
	if (unlikely(!count)) {
		printk(KERN_WARNING "generic_file_incoherent_sendfile() File %p count is zero\n", in_file);
		return 0;
	}

	desc.written = 0;
	desc.count = count;
	desc.arg.data = target;
	desc.error = 0;

	while (desc.count) {
		incoherent_cache_block_t *block;
		ssize_t                   block_offset;
		ssize_t                   available_bytes;
		ssize_t                   network_send_bytes;
		loff_t                    block_index;
		int                       readahead_performed = 0;

		// Remember which cache block we're reading from so that we can later
		// decide whether to readahead
		block_index = *ppos >> INCOHERENT_BLOCK_SHIFT;

		// Only readahead if the reader has explicitly requested contiguous
		// reads, i.e. don't take previous readahead reads into account
		if ((block_index != context->last_block_index) &&
			(block_index != (context->last_block_index + 1))) {
//printk("readahead_opportunity() Inode %p context %p: No readahead as blocks not contiguous\n", inode, context);
			// Don't do any readahead for this block
			readahead_performed = 1;

			// Reset the record of the first block beyond the end of last
			// readahead, as once we've had a non-contiguous read request we
			// must check the cache and fill queue for any blocks we might be
			// considering for readahead
			context->last_ra_end_index = 0;
		} else {
			// We're going to attempt readahead so we need to know the file
			// length, but we want to take the hit of calculating this once
			// per call to generic_file_incoherent_sendfile()
			if (filemap_length == (loff_t)-1) {
				// Discover the current file length, safe against concurrent
				// filemap updates
				lock_filemap(inode);
				filemap_length = filemap_info->length;
//printk("readahead_opportunity() Inode %p context %p filemap_length %lld\n", inode, context, filemap_length);
				up(&inode->reader_sem);
			}
		}

		if (!readahead_performed) {
			// If we are going to try to readahead for this block then allocate
			// a fill entry before we obtain the fill lock so that we don't
			// later need to obtain the fill entries lock while holding the fill lock
			if (readahead_fill_entry == NULL) {
				readahead_fill_entry = alloc_fill_entry();
				BUG_ON(!readahead_fill_entry);
			}
		}

//printk(KERN_INFO "generic_file_incoherent_sendfile() Loop top ppos = %lld, desc.count %d\n", *ppos, desc.count);
		// Try to find an existing cache block containing at least some of the
		// required data. Need fill lock to protect against block completing
		// in-progress fill just after we find it's not in the block cache
again:
		// Allocate a fill entry before we obtain the fill lock so that we don't
		// later need to obtain the fill entries lock while holding the fill lock
		if (fill_entry == NULL) {
			fill_entry = alloc_fill_entry();
			BUG_ON(!fill_entry);
		}

		spin_lock_bh(&fill_lock_s);
		block = cache_get_block(context, *ppos, &available_bytes, &block_offset);
//if (block) printk("A %lld, %d\n", *ppos, desc.count);
//if (!block) printk("N %lld, %d\n", *ppos, desc.count);
		if (block) {
//printk("File %p Context %p: block available at %lld, offset %d, desc.count = %u\n", in_file, context, *ppos, block_offset, desc.count);
			spin_unlock_bh(&fill_lock_s);
		} else {
			// Look to see if there's a fill in progress and if so whether it's
			// filling the block we want and if so wait on that fill completing
			fill_que_entry_t *entry =
				search_pending_fills_by_sector(&context->fill_progress,
					*ppos >> SECTOR_SHIFT);

			if (entry) {
				unsigned long end;
				int timed_out;

				DEFINE_WAIT(wait);

				// If we haven't already started readahead based on this block
				// start one now when we would probably only be waiting for the
				// pending block fill to complete anyway
				if (!readahead_performed) {
					int error = 0;

					// Should always be a preallocated fill entry available
					// when we need to try readahead
					BUG_ON(readahead_fill_entry == NULL);

					readahead_performed = readahead_opportunity(inode, context,
						block_index, filemap_length, readahead_fill_entry, &error);

					if (readahead_performed) {
						// A readahead was started using the provided fill
						// entry so must allocate a new one next time one is
						// needed
						readahead_fill_entry = NULL;
					}

					if (unlikely(error)) {
						printk(KERN_WARNING "generic_file_incoherent_sendfile()"
							" Inode %p, Context %p: readahead [1] for request "
							"pos %lld failed\n", inode, context, *ppos);
					}

					// readahead_opportunity() will have dropped the fill_lock
					// so regain it before examining the block fill status
					spin_lock_bh(&fill_lock_s);
				}

//printk(KERN_INFO "incoherent_sendfile_free_context() Wait [1] for in-progress or queued fill to complete, entry %p\n", entry);
				end = jiffies + FILL_TIMEOUT_JIFFIES;
				timed_out = 0;
				for (;;) {
					prepare_to_wait(&entry->wait_queue, &wait, TASK_UNINTERRUPTIBLE);
					if (entry->filled || entry->aborted) {
						break;
					}
					if (time_after(jiffies, end)) {
						printk(KERN_WARNING "generic_file_incoherent_sendfile() Timed-out of wait for fill entry %p\n", entry);
						timed_out = 1;
						break;
					}
					spin_unlock_bh(&fill_lock_s);
					if (!schedule_timeout(FILL_CHECK_INTERVAL_JIFFIES)) {
						printk(KERN_INFO "generic_file_incoherent_sendfile() %d seconds have passed while waiting [1] context %p, entry %p\n", FILL_CHECK_INTERVAL_JIFFIES/HZ, context, entry);
					}
					spin_lock_bh(&fill_lock_s);
				}
				finish_wait(&entry->wait_queue, &wait);
//printk(KERN_INFO "incoherent_sendfile_free_context() Woken [1], entry filled = %d, aborted = %d\n", !!entry->filled, !!entry->aborted);

				spin_unlock_bh(&fill_lock_s);

				if (unlikely(entry->stale)) {
printk(KERN_INFO "generic_file_incoherent_sendfile() Entry %p stale, so try again [1]\n", entry);
					goto again;
				}

				// Fills are only aborted when file close has been invoked, at
				// which point there should not be any outstanding read requests
				if (unlikely(entry->aborted) || unlikely(timed_out)) {
					printk(KERN_ERR "generic_file_incoherent_sendfile() entry %p has %s [1]\n", entry, timed_out ? "timed-out" : "been aborted");
					desc.error = -EIO;
					break;
				}

				// The required block should now be in the cache
				block = cache_get_block(context, *ppos, &available_bytes, &block_offset);
				if (unlikely(!block)) {
					// Shouldn't happen that the completed fill didn't get us
					// the associated block, unless an invalidate has occured
					// in the window since we checked for stale without the fill
					// lock held
printk(KERN_INFO "generic_file_incoherent_sendfile() Entry %p, pos %lld not present in block cache [1]\n", entry, *ppos);
					goto again;
//					desc.error = -EIO;
//					break;
				}
//printk("File %p context %p: block after wait at %lld, offset %d, desc.count %u\n", in_file, context, *ppos, block_offset, desc.count);
//if (block) printk("B %lld, %d\n", *ppos, desc.count);
//if (!block) printk("M %lld, %d\n", *ppos, desc.count);
			}
		}

		if (!block) {
			unsigned long end;
			int timed_out;

			DEFINE_WAIT(wait);

//printk("No block, so kick off fill\n");
			// Should always be a preallocated fill entry available
			BUG_ON(fill_entry == NULL);

			// Initialise with details of fill
			init_fill_entry(fill_entry, *ppos, desc.count);

			// Add a fill entry to the wait queue of the initial fill so we're
			// woken when fill is finished
			desc.error = schedule_fill(context, fill_entry);
			if (unlikely(desc.error)) {
				fill_entry = NULL;
				break;
			}

			// schedule_fill() drops the fill_lock, but we must hold fill_lock
			// before trying readahead or testing the block fill for completion
			spin_lock_bh(&fill_lock_s);

			// If we haven't aleady started readahead based on this block
			// then use the wait time to start a readahead
			if (!readahead_performed) {
				int error = 0;

				// Should always be a preallocated fill entry available when we
				// need to try readahead
				BUG_ON(readahead_fill_entry == NULL);

				readahead_performed = readahead_opportunity(inode,
					context, block_index, filemap_length, readahead_fill_entry,
					&error);

				if (readahead_performed) {
					// A readahead was started using the provided fill
					// entry so no need to free it once we're finished with
					// this block
					readahead_fill_entry = NULL;
				}

				if (unlikely(error)) {
					printk(KERN_WARNING "generic_file_incoherent_sendfile() "
						"Inode %p, Context %p: readahead [2] for request pos "
						"%lld failed\n", inode, context, *ppos);
				}

				// readahead_opportunity() will have dropped the fill_lock so
				// regain it before examining the block fill status
				spin_lock_bh(&fill_lock_s);
			}

			end = jiffies + FILL_TIMEOUT_JIFFIES;
			timed_out = 0;
//printk(KERN_INFO "incoherent_sendfile_free_context() Wait [2] for in-progress or queued fill to complete, entry %p\n", fill_entry);
			for (;;) {
				prepare_to_wait(&fill_entry->wait_queue, &wait, TASK_UNINTERRUPTIBLE);
				if (fill_entry->filled || fill_entry->aborted) {
					break;
				}
				if (time_after(jiffies, end)) {
					printk(KERN_WARNING "generic_file_incoherent_sendfile() Timed-out of wait for fill entry %p\n", fill_entry);
					timed_out = 1;
					break;
				}
				spin_unlock_bh(&fill_lock_s);
				if (!schedule_timeout(FILL_CHECK_INTERVAL_JIFFIES)) {
					printk(KERN_INFO "generic_file_incoherent_sendfile() %d seconds have passed while waiting [2] context %p, entry %p\n", FILL_CHECK_INTERVAL_JIFFIES/HZ, context, fill_entry);
				}
				spin_lock_bh(&fill_lock_s);
			}
			finish_wait(&fill_entry->wait_queue, &wait);
//printk(KERN_INFO "incoherent_sendfile_free_context() Woken [2], entry filled = %d, aborted = %d\n", !!fill_entry->filled, !!fill_entry->aborted);

			spin_unlock_bh(&fill_lock_s);

			if (unlikely(fill_entry->stale)) {
printk(KERN_INFO "generic_file_incoherent_sendfile() Entry %p stale, so try again [2]\n", fill_entry);
				fill_entry = NULL;
				goto again;
			}

			// Fills are only aborted when file close has been invoked, at
			// which point there should not be any outstanding read requests
			if (unlikely(fill_entry->aborted) || unlikely(timed_out)) {
				printk(KERN_ERR "generic_file_incoherent_sendfile() entry %p has %s [2]\n", fill_entry, timed_out ? "timed-out" : "been aborted");
				fill_entry = NULL;
				desc.error = -EIO;
				break;
			}

//printk("W %lld\n", *ppos);
//printk("Woken after tasklet\n");
			// The required block should now be in the cache
			block = cache_get_block(context, *ppos, &available_bytes, &block_offset);
			if (unlikely(!block)) {
				// Shouldn't happen that the completed fill didn't get us
				// the associated block, unless an invalidate has occured
				// in the window since we checked for stale without the fill
				// lock held
printk(KERN_INFO "generic_file_incoherent_sendfile() Entry %p, pos %lld not present in block cache [2]\n", fill_entry, *ppos);
				fill_entry = NULL;
				goto again;
//				desc.error = -EIO;
//				break;
			}

			fill_entry = NULL;
//printk("C %lld, %d\n", *ppos, desc.count);
//printk("File %p context %p: block after fill at %lld, offset %d, desc.count %u\n", in_file, context, *ppos, block_offset, desc.count);
//printk("Context %p, filled block %p, block_offset %d, block->start %lld block->count %u, available_bytes = %d\n", context, block, block_offset, block->start, block->count, available_bytes);
		}
//printk(KERN_INFO "generic_file_incoherent_sendfile() Got filled block 0x%p\n", block);

		// How much should we transfer to the network stack?
		network_send_bytes = desc.count < available_bytes ? desc.count :
		                                                    available_bytes;

//printk("Context %p, block %p, network_send_bytes %d, *ppos %lld, block_offset %d (desc.count %d, available_bytes %d)\n", context, block, network_send_bytes, *ppos, block_offset, desc.count, available_bytes);
		if (list_empty(&block->frags_head)) {
//printk("Context %p, block %p Empty list, all %s\n", context, block, block->is_hole ? "hole" : "data");
//if (block->is_hole) printk("generic_file_incoherent_sendfile() Hole!\n");
			*ppos += actor(&desc,
				block->is_hole ? zero_block_s.ptbl : block->ptbl, block_offset,
				network_send_bytes);
		} else {
			struct list_head *p;
			int               send_started = 0;
			ssize_t           frag_start = 0;

//printk("generic_file_incoherent_sendfile() Frags!\n");
//printk("Inode %p, context %p, block %p has frags\n", inode, context, block);
			// Send as much data as was available from disk to the network
			list_for_each(p, &block->frags_head) {
				read_frag_t *frag = list_entry(p, read_frag_t, head);
				size_t       frag_available_length;
				int          written;

				if (!send_started) {
					// Does current frag include the block offset of the start of
					// the required data?
					if ((block_offset >= frag_start) &&
						(block_offset < (frag_start + frag->length))) {
						// Found frag in which required data begins
						send_started = 1;
					} else {
						// Move to the next frag to look for the start offset
						frag_start += frag->length;
						continue;
					}
				}

				// How much data is available from the current block offset to the
				// end of this frag?
				frag_available_length = frag_start + frag->length - block_offset;

//printk("Inode %p context %p before actor: ppos %lld, desc.count %d, offset %d "
//	   "in %s send %d (frag: s %d, l %d, a %d)\n", inode, context,
//	   *ppos, desc.count, block_offset, frag->is_hole ? "Hole" : "Non-hole",
//	   network_send_bytes, frag_start, frag->length, frag_available_length);

				written = actor(&desc,
					frag->is_hole ? zero_block_s.ptbl : block->ptbl,
					block_offset, frag_available_length);

				if (unlikely(desc.error)) {
//printk("Context %p, actor() returned error %d\n", context, desc.error);
					break;
				}

				*ppos              += written;	// Update file offset
				block_offset       += written;	// Update offset within this block
				network_send_bytes -= written;	// Update count of bytes still to send

//printk("Inode %p context %p  after actor: ppos %lld, desc.count %d, offset %d, send %d\n",
//	inode, context, *ppos, desc.count, block_offset, network_send_bytes);

				BUG_ON(network_send_bytes < 0);
				if (!network_send_bytes) {
					// Have transfered all the required data from this block
					break;
				}
				BUG_ON(written != frag_available_length);

				// Move to the next frag
				frag_start += frag->length;
			}
			BUG_ON(!send_started);
		}
//printk("Context %p, Completed send\n", context);

//printk("Context %p ,block %p, *ppos %lld, block_index %lld, desc.count %d\n", context, *ppos >> INCOHERENT_BLOCK_SHIFT, *ppos, block_index, desc.count);

		// If we have not already started readahead based on this block then do
		// so now
		if (!readahead_performed) {
			int error = 0;

			// readahead_opportunity() expects to be called with fill_lock held
			spin_lock_bh(&fill_lock_s);

			// Should always be a preallocated fill entry available when we need
			// to try readahead
			BUG_ON(readahead_fill_entry == NULL);

			readahead_performed = readahead_opportunity(inode, context,
				block_index, filemap_length, readahead_fill_entry, &error);

			if (readahead_performed) {
				// A readahead was started using the provided fill entry so no
				// need to free it once we're finished with this block
				readahead_fill_entry = NULL;
			}

			if (unlikely(error)) {
				printk(KERN_WARNING "generic_file_incoherent_sendfile() Inode "
					"%p, Context %p: readahead [3] for request pos %lld "
					"failed\n", inode, context, *ppos);
			}
		}

		// Remember which cache block was last explicitly requested by the reader
		context->last_block_index = block_index;
		if (unlikely(desc.error)) {
			printk(KERN_WARNING "generic_file_incoherent_sendfile() Context %p socket write error %d\n", context, -(desc.error));
			break;
		}
	}

	// If we have a preallocated fill entries that we didn't use then free them
	if (fill_entry) {
//printk(KERN_INFO "generic_file_incoherent_sendfile() Freeing entry %p prior to exit\n", fill_entry);
		free_fill_entry(fill_entry);
	}
	if (readahead_fill_entry) {
//printk(KERN_INFO "generic_file_incoherent_sendfile() Freeing readahead entry %p prior to exit\n", readahead_fill_entry);
		free_fill_entry(readahead_fill_entry);
	}

//printk(KERN_INFO "generic_file_incoherent_sendfile() written=%u, error=%d\n", desc.written, desc.error);

	if (desc.written)
		return desc.written;

	return desc.error;
}

int do_incoherent_sendfile(
	struct file *in_file,
	struct file *out_file,
	loff_t      *ppos,
	size_t       count)
{
	int do_fast = 1;
	int retval = 0;

	do {
		if (do_fast && in_file->f_op->incoherent_sendfile) {
			struct inode *inode = in_file->inode;

			if (!inode) {
				printk(KERN_WARNING "do_incoherent_sendfile() File %p, %s inode ptr NULL\n",
					in_file, in_file->f_path.dentry->d_name.name);
				do_fast = 0;
				retval = -EAGAIN;
				continue;
			}

//printk("Fast read\n");
			spin_lock(&inode->fast_lock);
			if (unlikely(inode->fallback_in_progress)) {
				wait_fallback_complete(inode);

//printk("Fallback completed\n");
//printk("do_incoherent_sendfile() Inode %p, file %p, name %s: Fallback completed\n", inode, in_file, in_file->f_path.dentry->d_name.name);
				// Retry with, presumably, normal rather than fast mode
				spin_unlock(&inode->fast_lock);
				retval = -EAGAIN;
				continue;
			} else if (!(in_file->f_flags & O_FAST)) {
				// Retry with normal mode
				spin_unlock(&inode->fast_lock);
				do_fast = 0;
				retval = -EAGAIN;
				continue;
//			} else if (unlikely(inode->writer_file_context)) {
//				// Force fallback if fast write has already happened on this inode
//printk("do_incoherent_sendfile() Fast read attempt where write has already happened on inode %p, fp %p, file %s - fallback\n", inode, in_file, in_file->f_path.dentry->d_name.name);
//				inode->fallback_in_progress = 1;
//
//				if (!inode->fast_reads_in_progress_count
//#ifdef CONFIG_OXNAS_FAST_WRITES
//					&& !inode->fast_writes_in_progress_count
//#endif // CONFIG_OXNAS_FAST_WRITES
//						){
//					// We are the only fast acccess in progress, so must do the
//					// fallback now
////printk("do_incoherent_sendfile() Inode %p, file %p, name %s: Fast write already happened so do_fast_fallback\n", inode, in_file, in_file->f_path.dentry->d_name.name);
//					do_fast_fallback(inode);
//				} else {
//					// Wait for one of the in-progress fast reads or writes to
//					// complete the fallback. Can't go around the while loop
//					// again to get the wait done as could be a race with O_FAST
//					// being reset when not holding fast lock
//					wait_fallback_complete(inode);
//				}
//
//				// Retry with normal rather than fast mode
//				spin_unlock(&inode->fast_lock);
//				retval = -EAGAIN;
//				continue;
			} else {
				// Increment in-progress reads count so that any fast fallback
				// request issued while we have dropped the lock to do context
				// allocation will not be able to proceed
				 ++inode->fast_reads_in_progress_count;

				// Fast_lock will be dropped while allocating context, but code
				// is safe against parallel attempts to alloc context for same
				// file
				retval = alloc_context(inode, in_file);

				if (likely(!retval)) {
					// Fast context prep was successful, so see if file prep has
					// been done yet for this inode. This operation can drop the
					// inode fast_lock, but is safe against parallel prep on the
					// same file
					retval = do_initial_file_prep(inode, in_file);
				}

				if (unlikely(inode->fallback_in_progress)) {
					if (!--inode->fast_reads_in_progress_count
#ifdef CONFIG_OXNAS_FAST_WRITES
						&& !inode->fast_writes_in_progress_count
#endif // CONFIG_OXNAS_FAST_WRITES
							){
//printk("do_incoherent_sendfile() Inode %p, file %p, name %s: Fallback in progress after doing file prep\n", inode, in_file, in_file->f_path.dentry->d_name.name);
						// We are the only fast read in progress, so must do the
						// fallback now
						do_fast_fallback(inode);
					} else {
						// Wait for one of the in-progress fast reads or writes to
						// complete the fallback. Can't go around the while loop
						// again to get the wait done as could be a race with O_FAST
						// being reset when not holding fast lock
						wait_fallback_complete(inode);
					}

					spin_unlock(&inode->fast_lock);
					retval = -EAGAIN;
					continue;
				} else if (unlikely(retval)) {
					// Context or file prep failed
					inode->fallback_in_progress = 1;
					if (!--inode->fast_reads_in_progress_count 
#ifdef CONFIG_OXNAS_FAST_WRITES
						&& !inode->fast_writes_in_progress_count
#endif // CONFIG_OXNAS_FAST_WRITES
							){
						// We are the only fast read in progress, so must do the
						// fallback now
//printk("do_incoherent_sendfile() Inode %p, file %p, name %s: Failed file prep so do_fast_fallback\n", inode, in_file, in_file->f_path.dentry->d_name.name);
						do_fast_fallback(inode);
					} else {
						// Wait for one of the in-progress fast reads or writes to
						// complete the fallback. Can't go around the while loop
						// again to get the wait done as could be a race with O_FAST
						// being reset when not holding fast lock
						wait_fallback_complete(inode);
					}

					spin_unlock(&inode->fast_lock);
					retval = -EAGAIN;
					continue;
				}

				spin_unlock(&inode->fast_lock);
			}

//printk("Doing read\n");
//printk("do_incoherent_sendfile() Inode %p, file %p, name %s: Fast read start\n", inode, in_file, in_file->f_path.dentry->d_name.name);
			retval = generic_file_incoherent_sendfile(in_file, ppos, count, file_send_actor, out_file);
//printk("do_incoherent_sendfile() Inode %p, file %p, name %s: Fast read complete, retval = %d\n", inode, in_file, in_file->f_path.dentry->d_name.name, retval);
//printk("Read complete\n");

			spin_lock(&inode->fast_lock);
			if (!--inode->fast_reads_in_progress_count
#ifdef CONFIG_OXNAS_FAST_WRITES
					&& !inode->fast_writes_in_progress_count
#endif // CONFIG_OXNAS_FAST_WRITES
					&& unlikely(inode->fallback_in_progress)) {
				// We've just completed the only active fast read and found
				// that fallback to normal mode is pending, so we must start
				// the fallback process here
//printk("do_incoherent_sendfile() Inode %p, file %p, name %s: Fallback in progress after read finished\n", inode, in_file, in_file->f_path.dentry->d_name.name);
				do_fast_fallback(inode);
			} else {
				// Wait for one of the in-progress fast reads or writes to
				// complete the fallback
				wait_fallback_complete(inode);
			}
			spin_unlock(&inode->fast_lock);
//printk("Count decrement complete, retval %d\n", retval);
		} else {
//printk("Normal read start\n");
//printk("do_incoherent_sendfile() File %p, name %s: Normal read start\n", in_file, in_file->f_path.dentry->d_name.name);
			retval = in_file->f_op->sendfile(in_file, ppos, count, file_send_actor, out_file);
//printk("do_incoherent_sendfile() File %p, name %s: Normal read complete, retval = %d\n", in_file, in_file->f_path.dentry->d_name.name, retval);
//printk("Normal read complete, retval %d\n", retval);
		}
	} while (retval == -EAGAIN);

	return retval;
}

static void incoherent_sendfile_exit(void)
{
	int i;
//printk("incoherent_sendfile_exit() Called\n");

	flush_scheduled_work();

	for (i=0; i<CONFIG_OXNAS_FAST_READ_NUM_BLOCK_CACHES; i++) {
//printk("incoherent_sendfile_init() Freeing cache %p\n", &caches_s[i]);
		free_block_cache(&caches_s[i]);
	}

	if (frag_cache_s)
		kmem_cache_destroy(frag_cache_s);

	__free_page(zero_block_s.zero_page);
}
module_exit(incoherent_sendfile_exit);

static int __init incoherent_sendfile_init( void )
{
	int i, j, retval;

//printk("incoherent_sendfile_init() Called\n");

	// Make caches/blocks safe against incoherent_sendfile_exit() being called
	// for cleanup due to failure later in this function
	for (i=0; i < CONFIG_OXNAS_FAST_READ_NUM_BLOCK_CACHES; i++) {
		for (j=0; j < CONFIG_OXNAS_FAST_READ_BLOCKS_PER_CACHE; j++) {
			incoherent_cache_block_t *block = &caches_s[i].blocks[j];
			block->pages = NULL;
			INIT_LIST_HEAD(&block->frags_head);
		}
	}

	// Make zero page safe against incoherent_sendfile_exit() being called
	// for cleanup due to failure later in this function
	zero_block_s.zero_page = NULL;

	INIT_LIST_HEAD(&pending_fills_s);

	tasklet_init(&tasklet_s, tasklet_func, 0);

	memset(&fill_entries_s, 0, sizeof(fill_que_entry_t) * CONFIG_OXNAS_FAST_READ_NUM_FILL_ENTRIES);
	INIT_LIST_HEAD(&free_fill_entries_s);
	for (i=0; i<CONFIG_OXNAS_FAST_READ_NUM_FILL_ENTRIES; i++) {
//printk("incoherent_sendfile_init() Initialising fill entry %p\n", &fill_entries_s[i]);
		init_waitqueue_head(&fill_entries_s[i].wait_queue);
		INIT_LIST_HEAD(&fill_entries_s[i].list);

		list_add(&fill_entries_s[i].list, &free_fill_entries_s);
	}

	INIT_LIST_HEAD(&free_caches_s);
	for (i=0; i<CONFIG_OXNAS_FAST_READ_NUM_BLOCK_CACHES; i++) {
//printk("incoherent_sendfile_init() Initialising cache %p\n", &caches_s[i]);
		INIT_LIST_HEAD(&caches_s[i].free);
		INIT_LIST_HEAD(&caches_s[i].lru);
		INIT_LIST_HEAD(&caches_s[i].valid);
		INIT_LIST_HEAD(&caches_s[i].filling);
		retval = alloc_block_cache(&caches_s[i]);
		if (retval)
			goto error;

		INIT_LIST_HEAD(&caches_s[i].list);
		list_add(&caches_s[i].list, &free_caches_s);
	}

	frag_cache_s = kmem_cache_create("fast_read_holes", sizeof(read_frag_t), 0, SLAB_HWCACHE_ALIGN|SLAB_PANIC, NULL);
	if (!frag_cache_s) {
		retval = -ENOMEM;
		goto error;
	}

	// Need a cache block's worth of zeros
	zero_block_s.zero_page = alloc_page(GFP_KERNEL);
	if (!zero_block_s.zero_page) {
		retval = -ENOMEM;
		goto error;
	} else {
		// Zero the single page of real memory to be used as the zero source
		memset(page_address(zero_block_s.zero_page), 0, PAGE_SIZE);
		init_page_count(zero_block_s.zero_page);

		// Point every page pointer entry in the cache block sized page pointer
		// array at the single zero page so that sendpages() can reference a
		// full block's worth of zeros
		for (i=0; i < INCOHERENT_ALLOC_PAGES; i++) {
			zero_block_s.ptbl[i] = zero_block_s.zero_page;
		}
	}

//printk("&tasklet_struct %p \n", &tasklet_s);
//printk("&current_fill_progress_s %p\n", &current_fill_progress_s);
//printk("&pending_fills_s %p\n", &pending_fills_s);
//printk("caches_s %p\n", caches_s);
//printk("&free_caches_s %p\n", &free_caches_s);
//printk("&fill_entries_s %p\n", &fill_entries_s);

    return 0;

error:
	incoherent_sendfile_exit();
	return retval;
}

module_init(incoherent_sendfile_init);
