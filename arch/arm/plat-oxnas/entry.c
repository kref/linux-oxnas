/*
 * arch/arm/plat-oxnas/entry.c
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
 
#include <linux/syscalls.h>
#include <linux/security.h>
#include <linux/file.h>
#include <linux/genhd.h>
#include <linux/pagemap.h>
#include <mach/oxnas_errors.h>

#ifdef CONFIG_OXNAS_BACKUP

#define MAX_RW_COUNT (INT_MAX & PAGE_CACHE_MASK)

#include <mach/backup.h>

SYSCALL_DEFINE1(init_backup, int, sock_in_fd)
{
	struct file	*in_file;
	int          fput_needed_in;
	int          retval;

	/*
	 * Get input socket, and verify that it is ok
	 */
	retval = -EBADF;
	in_file = fget_light(sock_in_fd, &fput_needed_in);
	if (!in_file) {
		printk(KERN_INFO "No infile returning\n");
		goto out;
	}
	if (!(in_file->f_mode & FMODE_READ)) {
		printk(KERN_INFO "Cant set mode of in file\n");
		goto fput_in;
	}
	retval = security_file_permission(in_file, MAY_READ);
	if (retval) {
		printk(KERN_INFO "File security permissions failed\n");
		goto fput_in;
	}

	retval = oxnas_init_backup(in_file);

fput_in:
	fput_light(in_file, fput_needed_in);
out:
	return retval;
}

SYSCALL_DEFINE1(exit_backup, int, sock_in_fd)
{
	struct file *in_file;
	int          fput_needed_in;
	int          retval;

	/*
	 * Get input socket, and verify that it is ok
	 */
	retval = -EBADF;
	in_file = fget_light(sock_in_fd, &fput_needed_in);
	if (!in_file) {
		printk(KERN_INFO "No infile returning\n");
		goto out;
	}

	retval = oxnas_exit_backup(in_file);

	fput_light(in_file, fput_needed_in);
out:
	return retval;

}

SYSCALL_DEFINE3(do_backup, int, sock_in_fd, int, file_out_fd, loff_t __user *, count)
{
	struct file  *in_file;
	struct inode *in_inode;
	int           fput_needed_in;
	struct file  *out_file = 0;
	struct inode *out_inode;
	int           fput_needed_out;
	loff_t        file_offset = 0;
	loff_t        total_count = 0;
	loff_t        remaining = 0;
	long          retval;

	if (unlikely(copy_from_user(&total_count, count, sizeof(loff_t)))) {
		retval = -OXERR_VALIDATION;
		printk(KERN_INFO "Copy from user failed on count \n");
		goto out;
	}

#ifdef DEBUG
	printk(KERN_INFO "sys_xfsbackup - count value - %lld\n", total_count);
#endif

	/*
	 * Get input socket, and verify that it is ok
	 */
	retval = -OXERR_VALIDATION;
	in_file = fget_light(sock_in_fd, &fput_needed_in);
	if (!in_file) {
		printk(KERN_INFO "No infile returning\n");
		goto out;
	}

	retval = -OXERR_VALIDATION;
	in_inode = in_file->f_dentry->d_inode;

	file_offset = in_file->f_pos;
	remaining = total_count;

	/* have to verify the area in bits as the max. that can be verified
	 * is limited
	 */
	do {
		int temp_count = remaining > MAX_RW_COUNT ? MAX_RW_COUNT : remaining;
		retval = rw_verify_area(READ, in_file, &file_offset, temp_count);
		if (retval < 0) {
			printk(KERN_INFO "read verify area returned less than 0\n");
			retval = OXERR_VALIDATION;
			goto fput_in;
		}

		file_offset += temp_count;
		remaining -= temp_count;
	} while (remaining);

	/*
	 * Get output file, and verify that it is ok
	 */
	retval = -OXERR_VALIDATION;
	out_file = fget_light(file_out_fd, &fput_needed_out);
	if (!out_file) {
		printk(KERN_INFO "Failed no output file\n");
		goto fput_in;
	}
	if (!(out_file->f_mode & FMODE_WRITE)) {
		printk(KERN_INFO "Output file mode not right - failed\n");
		goto fput_out;
	}

	retval = -OXERR_VALIDATION;
	out_inode = out_file->f_dentry->d_inode;
	if (!out_inode) {
		printk(KERN_INFO "No output file NODE - failed\n");
		goto fput_out;
	}
	if (out_file->f_pos) {
		/* Backup only supported on empty files */
		printk(KERN_INFO "Output file not empty - failed\n");
		goto fput_out;
	}

	/* have to verify the area in bits as the max. that can be verified
	 * is limited
	 */
	remaining = total_count;
	file_offset = 0;

	do {
		int temp_count = remaining > MAX_RW_COUNT ? MAX_RW_COUNT : remaining;
		retval = rw_verify_area(WRITE, out_file, &file_offset, temp_count);
		if (retval < 0) {
			printk(KERN_INFO "verify area failed on output file\n");
			retval = OXERR_VALIDATION;
			goto fput_out;
		}

		file_offset += temp_count;
		remaining -= temp_count;
	} while(remaining);

	retval = security_file_permission(out_file, MAY_WRITE);
	if (retval) {
		printk(KERN_INFO "Output file permissions not right\n");
		retval = OXERR_VALIDATION;
		goto fput_out;
	}

#ifdef DEBUG
	printk(KERN_INFO "Before calling do_xfsbackup\n");
#endif

	retval = do_xfsbackup(in_file, out_file, total_count);

#ifdef DEBUG
	printk(KERN_INFO "After do_xfsbackup\n");
#endif

fput_out:
	fput_light(out_file, fput_needed_out);
fput_in:
	fput_light(in_file, fput_needed_in);
out:
	return retval;
}

#else /* CONFIG_OXNAS_BACKUP */

SYSCALL_DEFINE1(init_backup, int, sock_in_fd)
{
	return -EPERM;
}

SYSCALL_DEFINE1(exit_backup, int, sock_in_fd)
{
	return -EPERM;
}

SYSCALL_DEFINE3(do_backup, int, sock_in_fd, int, file_out_fd, loff_t __user *, count)
{
	return -EPERM;
}

#endif /* CONFIG_OXNAS_BACKUP */

#ifdef CONFIG_OXNAS_FAST_WRITES

#include <mach/direct_writes.h>

SYSCALL_DEFINE4(direct_disk_write, int, sock_in_fd, int, file_out_fd, loff_t __user *,offset, size_t, count)
{
	struct file   *in_file;
	int            fput_needed_in;
	struct file   *out_file = 0;
	int            fput_needed_out;
	long           retval;
	struct socket *socket;
	loff_t         start_offset = 0;
	loff_t         file_offset = 0;

	/*
	 * Get input socket, and verify that it is ok
	 */
	retval = -EBADF;
	in_file = fget_light(sock_in_fd, &fput_needed_in);
	if (unlikely(!in_file)) {
		printk(KERN_INFO "sys_write_direct_disk - No infile returning\n");
		goto out;
	}

	/*
	 * Get output file, and verify that it is ok
	 */
	retval = -EBADF;
	out_file = fget_light(file_out_fd, &fput_needed_out);
	if (unlikely(!out_file)) {
		printk(KERN_INFO "sys_write_direct_disk - Failed no output file\n");
		goto fput_in;
	}

	if(!(out_file->f_flags & O_FAST)) {
//		printk(KERN_INFO "Fast write called without FAST Open\n");
		goto net_to_cache;
	}

	socket = in_file->private_data;

	if (unlikely(copy_from_user(&start_offset, offset, sizeof(loff_t)))) {
		retval = -EFAULT;
		printk(KERN_INFO "sys_write_direct_disk - Copy from user failed on offset \n");
		goto fput_out;
	}

	file_offset = in_file->f_pos;

	retval = rw_verify_area(READ, in_file, &file_offset, count);
	if (retval < 0) {
		printk(KERN_INFO "sys_write_direct_disk - read verify area returned less than 0\n");
		goto fput_in;
	}
	count = retval;

	file_offset = start_offset;
	retval = rw_verify_area(WRITE, out_file, &file_offset, count);
	if (retval < 0) {
		printk(KERN_INFO "sys_write_direct_disk - verify area failed on output file\n");
		goto fput_out;
	}
	count = retval;

//	printk(KERN_INFO "Values received from samba offset - %lld, size - %lld \n", start_offset, total_count);

	retval = oxnas_do_direct_disk_write(socket, out_file, start_offset, count);

	if (retval == OXNAS_FALLBACK) {
//		printk(KERN_INFO "Fallback - calling net to cache writes\n");
net_to_cache:
		retval = sys_direct_netrx_write(sock_in_fd, file_out_fd, offset, count);
	}

fput_out:
	fput_light(out_file, fput_needed_out);
fput_in:
	fput_light(in_file, fput_needed_in);
out:
	return retval;

}

#else /* CONFIG_OXNAS_FAST_WRITES */

SYSCALL_DEFINE4(direct_disk_write, int, sock_in_fd, int, file_out_fd, loff_t __user *,offset, size_t, count)
{
	return -EPERM;
}
 
#endif /* CONFIG_OXNAS_FAST_WRITES */

/* End of File */
