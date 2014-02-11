/*
 * arch/arm/plat-oxnas/filter_misc.c
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
 
#include <linux/fs.h>
#include <linux/slab.h>
#include <mach/fast_open_filter.h>

#ifndef CONFIG_OXNAS_FAST_OPEN_FILTER

int fast_open_filter(struct file *file, struct inode *inode)
{
	while (down_timeout(&inode->writer_sem, HZ)) {
		printk("fast_open_filter() A second has elapsed while waiting, inode %p\n", inode);
	}

	++inode->normal_open_count;

	up(&inode->writer_sem);

	return 0;
}
 
void fast_close_filter(struct file* file)
{
	/* if space_reserved, need to do a space unreserve */
	struct dentry *dentry;
	struct inode	*inode;

	dentry = file->f_path.dentry;
	inode = dentry->d_inode;

	while (down_timeout(&inode->writer_sem, HZ)) {
		printk("fast_close_filter() A second has elapsed while waiting, inode %p\n", inode);
	}

	if (--inode->normal_open_count == 0) {
		inode->prealloc_initialised = 0;

		if (inode->space_reserve) {
			loff_t offset = 0;
			loff_t length = 0;

			inode->space_reserve = 0;
			inode->do_space_reserve = 0;

			/* Calculate unused preallocated length at end of file */
			offset = i_size_read(inode);
			length = inode->prealloc_size;
			length -= offset;

			if ((length > 0) && (offset >= 0)) {
				file->f_op->unpreallocate(file, offset, length);
			}
		}

		// Set preallocated size to be the real file size now that unused
		// preallocated space at end of file has been removed
		inode->prealloc_size = i_size_read(inode);
	}

	up(&inode->writer_sem);
}

#endif //CONFIG_OXNAS_FAST_OPEN_FILTER
