/*
 * arch/arm/plat-oxnas/fast_open_filter.c
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
#include <linux/list.h>
#include <linux/net.h>
#include <linux/spinlock.h>
#include <mach/prealloc_init.h>
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
#include <mach/fast_open_filter.h>
#include <mach/incoherent_sendfile.h>
#include <linux/sched.h>
#include <linux/wait.h>
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
#ifdef CONFIG_OXNAS_FAST_WRITES
#include <mach/direct_writes.h>
#endif // CONFIG_OXNAS_FAST_WRITES


/*
 * Must be called while holding the fast_lock
 */
static void set_close_in_progress(struct inode *inode)
{
//printk("set_close_in_progress() Inode %p\n", inode);
	inode->close_in_progress = 1;
}

/* DEBUG ONLY */
//static void update_close_progress(struct inode *inode, int progress)
//{
//	spin_lock(&inode->fast_lock);
//	if (inode->close_in_progress) {
//		inode->close_in_progress = progress;
//	}
//	spin_unlock(&inode->fast_lock);
//}

/*
 * fast_lock will be taken by this call
*/
static void clear_close_in_progress(struct inode *inode)
{
	spin_lock(&inode->fast_lock);

//printk("clear_close_in_progress() Inode %p\n", inode);
	if (inode->close_in_progress) {
		inode->close_in_progress = 0;
		wake_up(&inode->close_wait_queue);
	}

	spin_unlock(&inode->fast_lock);
}

/*
 * If the final close in the inode is in progress wait for it to complete
 * Must be called while holding the fast_lock, may drop the lock and reacquire
 */
static void wait_for_close_in_progress(struct inode *inode)
{
	if (unlikely(inode->close_in_progress)) {
		DEFINE_WAIT(wait);
		for (;;) {
			prepare_to_wait(&inode->close_wait_queue, &wait, TASK_UNINTERRUPTIBLE);
			if (!inode->close_in_progress) {
				break;
			}
			spin_unlock(&inode->fast_lock);
//printk("wait_for_close_in_progress() Inode %p\n", inode);
			if (!schedule_timeout(HZ)) {
				printk(KERN_INFO "wait_for_close_in_progress() A second has passed while waiting inode %p, close progress %d\n", inode, inode->close_in_progress);
			}
			spin_lock(&inode->fast_lock);
		}
		finish_wait(&inode->close_wait_queue, &wait);
	}
}

static void lightweight_filter_open(
	struct file  *file,
	struct inode *inode)
{
	while (down_timeout(&inode->writer_sem, HZ)) {
		printk("lightweight_filter_open() A second has elapsed while waiting, inode %p, file %s\n", inode, file->f_path.dentry->d_name.name);
	}

	++inode->normal_open_count;
	file->inode = inode;

	up(&inode->writer_sem);
}

static void lightweight_filter_close(struct file *file)
{
	struct inode *inode = file->inode;

	while (down_timeout(&inode->writer_sem, HZ)) {
		printk("lightweight_filter_close() A second has elapsed while waiting, inode %p, file %s\n", inode, file->f_path.dentry->d_name.name);
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

	file->inode = NULL;

	up(&inode->writer_sem);
}

int fast_open_filter(
	struct file  *file,
	struct inode *inode)
{
	int ret = 0;
	int supports_fast_mode = 0;

//printk("fast_open_filter() File %p, inode %p\n", file, inode);
	BUG_ON(file->inode);

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
	supports_fast_mode = file->f_op->incoherent_sendfile &&
		inode->i_op->get_extents && inode->i_op->getbmapx;
#ifdef CONFIG_OXNAS_FAST_WRITES
	supports_fast_mode = supports_fast_mode && inode->i_fop->resetpreallocate;
#endif // CONFIG_OXNAS_FAST_WRITES
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

	if (!supports_fast_mode) {
//printk("fast_open_filter() File %p, inode %p FAST mode is not supported\n", file, inode);
		if (file->f_flags & O_FAST) {
			/* The filesystem on which the file resides does not support FAST
			 * mode so force to NORMAL mode
			 */
			file->f_flags &= ~O_FAST;
		}

		// Still need to track file open/close for prealloc/unprealloc support
		lightweight_filter_open(file, inode);
	} else {
		spin_lock(&inode->fast_lock);

		/* If fallback from fast to normal mode is in progress, wait for it to
		 * complete */
		if (unlikely(inode->fallback_in_progress)) {
//printk("fast_open_filter() File %p, inode %p, waiting for fallback to complete\n", file, inode);
			wait_fallback_complete(inode);
		}

		// If some close processing that involves the entire inode is in progress
		// then wait until it is complete before opening the file again
		wait_for_close_in_progress(inode);

#ifdef CONFIG_OXNAS_BACKUP
		if (file->f_flags & O_BKP) {
			/* Must not allow backup open if already open in normal or fast mode
			 * and must only allow single open in backup mode */
			if ((inode->backup_open_count > 0) ||
				(inode->fast_open_count > 0) ||
				(inode->normal_open_count > 0)) {
//printk("fast_open_filter() File %p, inode %p cannot open in BACKUP mode (backup %d, normal %d, fast %d)\n", file, inode, inode->backup_open_count, inode->normal_open_count, inode->fast_open_count);
				ret = -1;
			} else {
				++inode->backup_open_count;
			}
		} else {
			/* Must not allow normal or fast mode open if already open in
			 * backup mode */
			if (inode->backup_open_count > 0) {
				ret = -1;
			} else {
#endif // CONFIG_OXNAS_BACKUP
				if (file->f_flags & O_FAST) {
//printk("fast_open_filter() File %p, inode %p FAST open request (normal %d, fast %d)\n", file, inode, inode->normal_open_count, inode->fast_open_count);
					if (inode->normal_open_count > 0) {
//printk("fast_open_filter() File %p, inode %p already open for NORMAL read so force back to NORMAL (normal %d, fast %d)\n", file, inode, inode->normal_open_count, inode->fast_open_count);
						file->f_flags &= ~O_FAST;
						++inode->normal_open_count;
					} else {
						if (!inode->fast_open_count) {
							// Reset the record of the file size including any
							// not-yet-commited accumulated writes here rather
							// than on close in case there can be a possibility
							// of stat() not seeing the full size if it races
							// with close
							i_tent_size_write(inode, 0);

							// Clear inode error record on open, rather than on
							// close when could lose the information for the
							// cleanup operations
							clear_write_error(inode);
						}

						/* Remember that we haven't yet allocated any context for this
						 * new fast reader
						 */
						file->fast_context = NULL;
#ifdef CONFIG_OXNAS_FAST_WRITES
						file->fast_write_context = NULL;
#endif // CONFIG_OXNAS_FAST_WRITES

						/* Record this fast open file with the inode */
						INIT_LIST_HEAD(&file->fast_head);
						write_lock(&inode->fast_files_lock);
						list_add_tail(&file->fast_head, &inode->fast_files);
						write_unlock(&inode->fast_files_lock);
						++inode->fast_open_count;
//printk("fast_open_filter() File %s, fp %p, inode %p sucessfully opened for FAST read (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
					}
				} else {
//printk("fast_open_filter() File %p, inode %p NORMAL open request (normal %d, fast %d)\n", file, inode, inode->normal_open_count, inode->fast_open_count);
					if (inode->fast_open_count > 0) {
						/* Force the FAST mode users to fallback to normal mode so that
						 * we can then allow the new normal open to proceed
						 */
//printk("fast_open_filter() File %p, inode %p already open for FAST read, so denying NORMAL open (normal %d, fast %d)", file, inode, inode->normal_open_count, inode->fast_open_count);
						fast_fallback(inode);
					}
		
					++inode->normal_open_count;
//printk("fast_open_filter() File %p, inode %p sucessfully opened for NORMAL read (normal %d, fast %d)\n", file, inode, inode->normal_open_count, inode->fast_open_count);
				}
#ifdef CONFIG_OXNAS_BACKUP
			}
		}
#endif //  CONFIG_OXNAS_BACKUP

		spin_unlock(&inode->fast_lock);

		if (!ret) {
			/* Successfully opened a file on a filesystem capable of FAST mode */
			file->inode = inode;
		}
	}

	return ret;
}

void fast_close_filter(struct file* file)
{
	int supports_fast_mode = 0;
	struct inode *inode = file->inode;

//	WARN(!inode, "fast_close_filter() fp %p, inode %p, file %s\n", file, file->inode, file->f_path.dentry->d_name.name);
	if (!inode) {
		// Probably a shared-memory, semaphore or message queue object
		return;
	}

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
	supports_fast_mode = file->f_op->incoherent_sendfile &&
		inode->i_op->get_extents && inode->i_op->getbmapx;
#ifdef CONFIG_OXNAS_FAST_WRITES
	supports_fast_mode = supports_fast_mode && inode->i_fop->resetpreallocate;
#endif // CONFIG_OXNAS_FAST_WRITES
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

//printk("fast_close_filter() File %p, inode %p, f_count = %ld\n", file, file->inode, atomic_long_read(&file->f_count));
	if (!supports_fast_mode) {
		// For filesystems without fast access support still need to track file
		// open/close for prealloc/unprealloc support
		lightweight_filter_close(file);
	} else {
		int final_close = 0;
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		int free_filemap = 0;
		incoherent_sendfile_context_t *context = 0;
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
#ifdef CONFIG_OXNAS_FAST_WRITES
		int reset_prealloc = 0;
		void *write_context = 0;
#endif // CONFIG_OXNAS_FAST_WRITES

		spin_lock(&inode->fast_lock);

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		/* If fallback from fast to normal mode is in progress, wait for it to
		 * complete */
		if (unlikely(inode->fallback_in_progress)) {
//printk("fast_close_filter() File %p, inode %p, waiting for fallback to complete\n", file, inode);
			wait_fallback_complete(inode);
		}

		if (file->f_flags & O_FAST) {
//printk("fast_close_filter() File %s, fp %p, inode %p FAST close request (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);

			if (!(--inode->fast_open_count)) {
				final_close = 1;
				set_close_in_progress(inode);

				free_filemap = 1;
#ifdef CONFIG_OXNAS_FAST_WRITES
				reset_prealloc = 1;
#endif // CONFIG_OXNAS_FAST_WRITES
//printk("fast_close_filter() File %s, fp %p, inode %p FAST open count now zero (normal %d, fast %d)\n", file->f_path.dentry->d_name.name, file, inode, inode->normal_open_count, inode->fast_open_count);
			}

			/* Was a fast write context allocated for this file? */
			if (file->fast_write_context) {
				/*
				 * Cleaning up a file handle's fast write context involves
				 * the entire inode, so need to hold off subsequent opens on
				 * the inode
				 */
				write_context = file->fast_write_context;
				set_close_in_progress(inode);
			}

			/* Was a fast read context reserved for this file? */
			if (file->fast_context) {
				/*
				 * Want context deallocated once inode fast lock dropped
				 * No need to worry about subsequent open calls on the inode as
				 * the read context is local to the file handle being closed
				 * for the last time
				 */
				context = file->fast_context;
			}

			/* Remove the reference from the inode to this fast file */
			write_lock(&inode->fast_files_lock);
			list_del(&file->fast_head);
			write_unlock(&inode->fast_files_lock);
		} else {
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
#ifdef CONFIG_OXNAS_BACKUP
			if (file->f_flags & O_BKP) {
				//printk("fast_close_filter() File %p, inode %p BACKUP close request (backup %d, normal %d, fast %d)\n", file, inode, inode->backup_open_count, inode->normal_open_count, inode->fast_open_count);
				BUG_ON(--inode->backup_open_count < 0);
			} else {
#endif // CONFIG_OXNAS_BACKUP
//printk("fast_close_filter() File %p, inode %p NORMAL close request (normal %d, fast %d)\n", file, inode, inode->normal_open_count, inode->fast_open_count);
				BUG_ON(--inode->normal_open_count < 0);
				if (inode->normal_open_count == 0) {
					final_close = 1;
					set_close_in_progress(inode);
				}
#ifdef CONFIG_OXNAS_BACKUP
			}
#endif // CONFIG_OXNAS_BACKUP
#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		}
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

		spin_unlock(&inode->fast_lock);

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		if (context) {
//printk("fast_close_filter() File %s, fp %p, inode %p freeing context %p\n", file->f_path.dentry->d_name.name, file, inode, context);
//update_close_progress(inode, 2);
			incoherent_sendfile_free_context(context);
//update_close_progress(inode, 3);
		}

#ifdef CONFIG_OXNAS_FAST_WRITES
		if (write_context) {
//update_close_progress(inode, 4);
//printk("fast_close_filter() File %s, fp %p, inode %p complete_fast_write\n", file->f_path.dentry->d_name.name, file, inode);
			complete_fast_write(file);
//update_close_progress(inode, 5);
			if (reset_prealloc) {
				/* last file - reset prealloc */
//update_close_progress(inode, 6);
				writer_reset_prealloc(file);
//update_close_progress(inode, 7);
			}
		}
#endif // CONFIG_OXNAS_FAST_WRITES
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES

		// Need to clean up preallocation on the last file close on the inode
		if (final_close) {
//update_close_progress(inode, 8);
			while (down_timeout(&inode->writer_sem, HZ)) {
				printk("fast_close_filter() A second has elapsed while waiting, inode %p, file %s\n", inode, file->f_path.dentry->d_name.name);
			}

//printk("fast_close_filter() unprealloc for file %s, inode %p\n", file->f_path.dentry->d_name.name, inode);
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

//printk("fast_close_filter() File %s, fp %p, inode %p, prealloc_size %lld\n", file->f_path.dentry->d_name.name, file, inode, inode->prealloc_size);
			up(&inode->writer_sem);
//update_close_progress(inode, 9);
		}

#ifdef CONFIG_OXNAS_FAST_READS_AND_WRITES
		if (free_filemap) {
//printk("fast_close_filter() File %s, fp %p, inode %p freeing filemap\n", file->f_path.dentry->d_name.name, file, inode);
//update_close_progress(inode, 10);
			incoherent_sendfile_check_and_free_filemap(inode);
//update_close_progress(inode, 11);
#ifdef CONFIG_OXNAS_FAST_WRITES
			fast_write_check_and_free_filemap(inode);
//update_close_progress(inode, 12);
#endif // CONFIG_OXNAS_FAST_WRITES
		}

		file->inode = NULL;
		file->fast_context = NULL;
#ifdef CONFIG_OXNAS_FAST_WRITES
		file->fast_write_context = NULL;
#endif // CONFIG_OXNAS_FAST_WRITES

		// Wake any one waiting to open the inode due to close cleanup being
		// in-progress
		clear_close_in_progress(inode);
#endif // CONFIG_OXNAS_FAST_READS_AND_WRITES
	}
}
