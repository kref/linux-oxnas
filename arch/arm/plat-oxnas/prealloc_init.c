/*
 * arch/arm/plat-oxnas/prealloc_init.c
 *
 * Copyright (C) 2008, 2009, 2010 Oxford Semiconductor Ltd
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
#include <mach/prealloc_init.h>

/*
 * writer filemap semaphore should be held around calls to this function
 */
void do_prealloc_init(
	struct inode *inode,
	struct file  *file)
{
	if (unlikely(!inode->prealloc_initialised)) {
		inode->prealloc_initialised = 1;

		if ((file->f_flags & O_PREALLOC) && file->f_op->preallocate) {
			const char *fs_name = inode->i_sb->s_type->name;
//printk("do_prealloc_init() File %s, inode %p\n", file->f_path.dentry->d_name.name, inode);
			inode->prealloc_size = i_size_read(inode);
			inode->do_space_reserve = 1;
			if( (fs_name[0] == 'e') && (fs_name[1] == 'x') && (fs_name[2] == 't')
				&& (fs_name[3] == '4') && (fs_name[4] == '\0') ) {
//printk(KERN_INFO "do_prealloc_init: file system name - %s\n", inode->i_sb->s_type->name);
				inode->truncate_space_reset = 1; /* implies prealloc space gets reset on truncate */
			} else {
				inode->truncate_space_reset = 0;
			}
		} else {
//printk("do_prealloc_init() No Prealloc for File %s, inode %p\n", file->f_path.dentry->d_name.name, inode);
			inode->do_space_reserve = 0;
			inode->prealloc_size = 0;
			inode->truncate_space_reset = 0;
		}
	}
}
