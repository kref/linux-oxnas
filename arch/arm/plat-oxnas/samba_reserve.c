/*
 * arch/arm/plat-oxnas/samba_reserve.c
 *
 * Copyright (C) 2008, 2009 Oxford Semiconductor Ltd
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
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <mach/prealloc_init.h>
#include <asm/uaccess.h>

#define PREALLOC_CHUNK 	(CONFIG_OXNAS_PREALLOC_CHUNK_MB * 1024 * 1024)

int do_preallocate(struct file *file, loff_t offset, loff_t len)
{
	struct inode *inode;
	int retval = 0;

	inode = file->f_path.dentry->d_inode;

	while (down_timeout(&inode->writer_sem, HZ)) {
		printk("do_preallocate() A second has elapsed while waiting, inode %p\n", inode);
	}

	// Initialise preallocation support if not already done
	do_prealloc_init(inode, file);

	if(inode->do_space_reserve) {
//		printk(KERN_INFO "do_preallocate() Call preallocate() %p, %lld, %lld\n", file, offset, len);
		BUG_ON(!file->f_op->preallocate);

		if(inode->prealloc_size < (offset + len)) {
			loff_t prealloc_chunk = PREALLOC_CHUNK;
			if((inode->prealloc_size + prealloc_chunk)  <  (offset + len)) {
				prealloc_chunk = (offset + len) - inode->prealloc_size;
			}
			retval = file->f_op->preallocate(file, inode->prealloc_size, prealloc_chunk);
			if(retval < 0) {
				/* try allocating actual size reqd */
				prealloc_chunk = (offset + len) - inode->prealloc_size;
				retval = file->f_op->preallocate(file, inode->prealloc_size, prealloc_chunk);
				if(retval < 0) {
					/* failed prealloc - return failure */
					retval = -ENOSPC;
				}
			}

			// If preallocation has been successful update the inode's record
			if (!retval) {
				inode->prealloc_size += prealloc_chunk;
				inode->space_reserve = 1;
			} else {
				printk(KERN_WARNING "do_preallocate() File %s failed prealloc at offset %lld for length %lld\n",
					file->f_path.dentry->d_name.name, offset, len);
			}
		}
	}

	up(&inode->writer_sem);

	return retval;
}

/* This functon has to ideally return -ENOSYS, but to retain
 * compatibility with the samba versions doing prealloc, this
 * returns 0
 */
SYSCALL_DEFINE3(samba_reserve, int, fd, loff_t __user*, start, loff_t __user*, length)
{
	return 0;
}
