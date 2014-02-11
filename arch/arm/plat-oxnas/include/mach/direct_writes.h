/*
 * arch/arm/plat-oxnas/include/mach/direct_writes.h
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

#ifndef DIRECT_WRITES_H_
#define DIRECT_WRITES_H_
#include <linux/workqueue.h>

/* structure to hold the partial sector details */
typedef struct oxnas_partial_block {
	sector_t     lba;
	loff_t       start_offset_into_file;
	int          bytes_into_block; /* start byte offset */
	int          length; /* less than FSB */
	int          status;
	int          unwritten; /* GETBMAPX_OF_PREALLOC is used for an preallocated unwritten extent */
	char        *buffer;
	dma_addr_t  buffer_pa;
	struct file *fp;
} oxnas_partial_block_t;

/* structure needed for samba writes - stored as part of struct file */
typedef struct oxnas_file_context {
	oxnas_partial_block_t *partial_block;
	long long              write_start_offset;
	long long              write_end_offset;
	int                    write_extent_flag;
	int 				   last_extent_flag:1;
	struct delayed_work    write_completion_work;
	struct file           *fp;
	struct file 		  *acc_fp;
	int 				  disable_accumulation;
	sector_t 			  prev_partial_write_loc;
} oxnas_file_context_t;

extern int oxnas_do_direct_disk_write(struct socket *socket, struct file *fp, loff_t offset, loff_t count);

extern int oxnas_get_extent_status(getbmapx_t *map, int cur_map_entry);

extern void __write_flush_filemap(struct inode *inode);

static inline void write_flush_filemap(struct inode *inode)
{
	smp_rmb();
	__write_flush_filemap(inode);
}

/* Returns 1 if flush required */
static inline int is_writer_flush_filemap_required(struct inode *inode)
{
	oxnas_file_context_t *file_context;

	smp_rmb();

	file_context = (oxnas_file_context_t*)inode->writer_file_context;
	if (file_context && inode->writer_filemap_info.map) {
		struct file *file = file_context->fp;

		if (file && (file_context->partial_block || file_context->write_end_offset)) {
			return 1;
		}
	}

	return 0;
}

static inline void write_flush_filemap_if_required(struct inode *inode)
{
	if (is_writer_flush_filemap_required(inode)) {
		__write_flush_filemap(inode);
	}
}

extern void writer_reset_prealloc(struct file *file);
extern int complete_fast_write(struct file *fp);
extern void fast_write_check_and_free_filemap(struct inode *inode);
extern void write_flush_pending(struct inode *inode, int disable_accumulation);
extern void flush_writes(struct inode *inode);
extern void writer_remap_file(struct inode *inode);
extern loff_t i_tent_size_read(const struct inode *inode);
extern void i_tent_size_write(struct inode *inode, loff_t i_size);

#endif /* DIRECT_WRITES_H_ */
