/*
 * arch/arm/plat-oxnas/include/mach/incoherent_sendfile.h
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

#ifndef INCOHERENT_SENDFILE_H
#define INCOHERENT_SENDFILE_H
#include <linux/mm_types.h>
#include <mach/filemap_info.h>

#define INCOHERENT_ALLOC_PAGES (1 << CONFIG_OXNAS_FAST_READ_ALLOC_ORDER)

typedef struct incoherent_cache_block {
	struct list_head  head;		// For free, valid and filling lists
	struct list_head  lru_head;	// For least-recently-used list
	struct list_head  fill_head;	// For attaching to chain of blocks associated with a fill entry
	sector_t          start;
	size_t		  count;
	struct page      *pages;
	struct page      *ptbl[INCOHERENT_ALLOC_PAGES];
	struct incoherent_sendfile_context *context;
	int               stale:1,	// Block has been invalidated
	                  is_hole:1;	// Entire block is a hole
	struct list_head  frags_head;
	atomic_t          users;
} incoherent_cache_block_t;

typedef struct incoherent_cache {
	struct list_head list;
	struct list_head free;
	struct list_head lru;
	struct list_head valid;
	struct list_head filling;
	incoherent_cache_block_t blocks[CONFIG_OXNAS_FAST_READ_BLOCKS_PER_CACHE];
} incoherent_cache_t;

typedef struct fill_que_entry {
	struct list_head  list;
	loff_t            pos;			// File offset, updated as fill proceeds
	size_t            remaining;	// File read count, updated as fill proceeds
	sector_t          sector;		// The sector from which to start the fill
	wait_queue_head_t wait_queue;
	int               filled:1,		// Whether the block fill has finished
	                  aborted:1,	// Whether the fill was aborted due to file closure
			          stale:1,
			          disk_read_started:1,
                      failed:1;
	struct list_head  blocks;		// The blocks filled as a result of the disk read
} fill_que_entry_t;

typedef struct fill_progress {
	struct list_head          list;		// For attaching to global list of pending fills
	int                       in_progress:1,// Whether there is a block fill in progress
	                          eof:1,
	                          is_sg:1;
	struct list_head          queue;	// Queue of block fills
	fill_que_entry_t         *entry;	// The currently in progress fill
	size_t                    fill_len_sectors;// The total length of the in-progress fill
	incoherent_cache_block_t *block;	// The block currently being filled
	int                       block_offset;	// Offset into block's buffer for next read from disk
	size_t                    last_read_count;// How much we tried to read from disk last time
	struct incoherent_sendfile_context *context;
	struct odrb_dma_prd_list *prd_list;
} fill_progress_t;

typedef struct incoherent_sendfile_context {
	spinlock_t          lock;
	fill_progress_t     fill_progress;
	incoherent_cache_t *cache;
	loff_t              last_block_index;
	struct inode       *inode;
	int                 need_reindex;
	int                 cur_map_entry;
	long long           cur_map_entry_offset;// In sectors
	loff_t              next_offset;
	loff_t              last_ra_end_index;
	unsigned int        failure_count;
} incoherent_sendfile_context_t;

/*
 * The below filemap and cache invalidation functions may all sleep if there is
 * contention for the fast reads filemap access lock
 */
extern void incoherent_sendfile_remap_file(struct inode *inode);
extern void incoherent_sendfile_invalidate_cache(struct inode *inode, loff_t offset, loff_t length);

extern int init_filemapinfo(struct inode *inode, int flag);
extern int alloc_filemap(struct inode *inode, int flag);
extern void fast_fallback(struct inode *inode);
extern void do_fast_fallback(struct inode *inode);
extern void wait_fallback_complete(struct inode *inode);
extern void incoherent_sendfile_free_context(void* ptr);
extern void incoherent_sendfile_check_and_free_filemap(struct inode * inode);

static inline void fast_put(struct page *page)
{
	incoherent_cache_block_t *block = (incoherent_cache_block_t*)page->private;
	atomic_dec(&block->users);
}

static inline void fast_get(struct page *page)
{
	incoherent_cache_block_t *block = (incoherent_cache_block_t*)page->private;
	atomic_inc(&block->users);
}
#endif        //  #ifndef INCOHERENT_SENDFILE_H
