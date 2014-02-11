/*
 * Trustees ACL Project
 *
 * Copyright (c) 1999-2000 Vyacheslav Zavadsky
 * Copyright (c) 2004 Andrew Ruder (aeruder@ksu.edu)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The security module (LSM API) component of the trustees system
 *
 * One quick note: generally security modules with the LSM are supposed
 * to be solely restrictive modules.  Unless the trustees module were to
 * require that people set all files rwx by all, it could not function
 * as it is meant to function as a solely restrictive module.
 *
 * To compensate, every process is given the capability CAP_DAC_OVERRIDE.
 * In other words, every process is first given full rights to the filesystem.
 * This is the only non-restricting portion of this module, since it -does-
 * in fact give additional permissions.  However, in the inode_permission hook,
 * any rights the user should not have are taken away.
 *
 * Side effects: Posix ACLs or other filesystem-specific permissions are not
 * honored.  Trustees ACLs can (and do) take into account the standard unix
 * permissions, but any permissions further than that are difficult, to say
 * the least, to take into account.  I, personally, do not find this to
 * be a problem since if you are using Trustees ACLs, why also require the use
 * of another ACL system?
 */

#include <linux/security.h>
#include <linux/capability.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/nsproxy.h>
#include <linux/cred.h>
#include <linux/mnt_namespace.h>

#include "internal.h"

static int trustees_capable(struct task_struct *tsk, const struct cred *cred,
			    int cap,
			    int audit);
static int trustees_inode_permission(struct inode *inode, int mask);

/* Checks if user has access to the inode due to root status
 */
static inline int has_root_perm(struct inode *inode, int mask)
{
	umode_t mode = inode->i_mode;

	if (!(mask & MAY_EXEC) || (mode & S_IXUGO) || S_ISDIR(mode))
		if (current_fsuid() == 0)
			return 0;

	return -EACCES;
}

/* The logic for this was mostly stolen from vfs_permission.  The security API
 * doesn't give a good way to use the actual vfs_permission for this since our
 * CAP_DAC_OVERRIDE causes it to always return 0.  But if we didn't return
 * CAP_DAC_OVERRIDE, we'd never get to handle permissions!  Since we don't need
 * to handle capabilities and dealing with ACLs with trustees loaded isn't an
 * issue for me, the function ends up being pretty simple.
 */

static inline int has_unix_perm(struct inode *inode, int mask)
{
	umode_t mode = inode->i_mode;
	mask &= ~MAY_APPEND;

	if (current_fsuid() == inode->i_uid)
		mode >>= 6;
	else if (in_group_p(inode->i_gid))
		mode >>= 3;

	if (((mode & mask & (MAY_READ | MAY_WRITE | MAY_EXEC)) == mask))
		return 0;

	return -EACCES;
}

/* Find a vfsmount given an inode */
static inline struct vfsmount *find_inode_mnt(struct inode *inode)
{
	struct mnt_namespace *ns = NULL;
	struct vfsmount *mnt = NULL;

	if (!inode->i_sb) {
		TS_ERR_MSG("Inode without a i_sb entry?");
		return NULL;
	}

	/* Okay, we need to find the vfsmount by looking
	 * at the namespace now.
	 */
	task_lock(current);
	if (current->nsproxy) {
		ns = current->nsproxy->mnt_ns;
		if (ns)
			get_mnt_ns(ns);
	}
	task_unlock(current);

	if (!ns)
		return NULL;

	list_for_each_entry(mnt, &ns->list, mnt_list) {
		if (mnt->mnt_sb == inode->i_sb && mnt->mnt_devname) {
			mntget(mnt);
			goto out;
		}
	}

  out:
	put_mnt_ns(ns);

	return mnt;
}

/* Find a dentry given an inode */
static inline struct dentry *find_inode_dentry(struct inode *inode)
{
	struct dentry *dentry;

	dentry = d_find_alias(inode);

	return dentry;
}

/* Find a path (dentry/vfsmount pair) given an inode
 */
static inline int find_inode_path(struct inode *inode, struct path *path)
{
	int ret = 0;

	path->dentry = NULL;
	path->mnt = NULL;

	path->mnt = find_inode_mnt(inode);
	if (unlikely(!path->mnt)) {
		TS_ERR_MSG("inode does not have a mnt!\n");
		goto error_out;
	}

	path->dentry = find_inode_dentry(inode);
	if (unlikely(!path->dentry)) {
		/* Most of the time when this happens, it is the /
		 * If it is not, we need to dump as much information
		 * as possible on it and dump it to logs, because
		 * I'm really not sure how it happens.
		 */
		if (path->mnt->mnt_root && inode == path->mnt->mnt_root->d_inode) {
			path->dentry = dget(path->mnt->mnt_root);
		} else {
			/* I have seen this happen once but I did not have any
			 * way to see what caused it.  I am gonna dump_stack
			 * until I have that happen again to see if the cause
			 * is something that I need to worry about.
			 */
			dump_stack();	/* DEBUG FIXME */
			TS_ERR_MSG("Inode number: %ld\n", inode->i_ino);
			TS_ERR_MSG("dentry does not exist!\n");
			goto error_mnt_out;
		}
	}

	goto out;

error_mnt_out:
	mntput(path->mnt);
	path->mnt = NULL;

error_out:
	ret = 1;

out:
	return ret;
}

/*
 * Return 1 if they are under the same set of trustees
 * otherwise return 0.  In the case that we are handling
 * a directory, we also check to see if there are subdirectories
 * with trustees.
 */
static inline int have_same_trustees_rename(struct path *path1, struct path *path2)
{
	char *filename1, *filename2;
	int depth1, depth2;
	struct trustee_hash_element *deep1, *deep2;
	int is_dir;
	int ret = 0;

	filename1 = trustees_filename_for_dentry(path1->dentry, &depth1, 1);
	if (!filename1) {
		TS_ERR_MSG("Couldn't allocate filename\n");
		goto out;
	}

	filename2 = trustees_filename_for_dentry(path2->dentry, &depth2, 1);
	if (!filename2) {
		TS_ERR_MSG("Couldn't allocate filename\n");
		goto out_file_name;
	}

	is_dir = S_ISDIR(path1->dentry->d_inode->i_mode);

	read_lock(&trustee_hash_lock);
	trustee_perm(path1, filename1, ret, depth1, is_dir, &deep1);
	trustee_perm(path2, filename2, ret, depth2, is_dir, &deep2);
	if (deep1 == deep2) {
		ret = 1;
		if (is_dir) {
			if (trustee_has_child(path1->mnt, filename1) ||
				trustee_has_child(path2->mnt, filename2)) ret = 0;
		}
	}
	read_unlock(&trustee_hash_lock);

	kfree(filename2);
  out_file_name:
	kfree(filename1);
  out:

	return ret;
}

static int trustees_inode_rename(struct inode *old_dir,
				 struct dentry *old_dentry,
				 struct inode *new_dir,
				 struct dentry *new_dentry);
static int trustees_inode_link(struct dentry *old_dentry,
			       struct inode *dir,
			       struct dentry *new_dentry);

/* Structure where we fill in the various hooks we are implementing in this module
 */
struct security_operations trustees_security_ops = {
	.capable = trustees_capable,
	.inode_permission = trustees_inode_permission,
	.inode_link = trustees_inode_link,
	.inode_rename = trustees_inode_rename
};

#define ALL_MAYS (MAY_WRITE | MAY_EXEC | MAY_READ)
/* Converts a trustee_mask to a normal unix mask
 */
static int inline trustee_mask_to_normal_mask(int mask, int isdir)
{
	int r = 0;
	if ((mask & TRUSTEE_READ_MASK) && !isdir)
		r |= MAY_READ;
	if ((mask & TRUSTEE_READ_DIR_MASK) && isdir)
		r |= MAY_READ;
	if (mask & TRUSTEE_WRITE_MASK)
		r |= MAY_WRITE;
	if ((mask & TRUSTEE_BROWSE_MASK) && isdir)
		r |= MAY_EXEC;
	if ((mask & TRUSTEE_EXECUTE_MASK) && !isdir)
		r |= MAY_EXEC;
	return r;
}

/* This is the meat of the permissions checking.  First it checks for root,
 * otherwise it first checks for any errors finding the dentry/vfsmount for
 * the inode, and then it looks up the dentry in the trustees hash.
 */
static int trustees_inode_permission(struct inode *inode, int mask)
{
	struct path path;
	char *file_name;
	int is_dir;
	int ret;
	int depth;
	int amask;
	int dmask;
	umode_t mode = inode->i_mode;

	if (has_root_perm(inode, mask) == 0)
		return 0;

	ret = has_unix_perm(inode, mask);

	if (find_inode_path(inode, &path)) {
		return -EACCES;
	}

	file_name = trustees_filename_for_dentry(path.dentry, &depth, 1);
	if (!file_name) {
		TS_ERR_MSG("Couldn't allocate filename\n");
		ret = -EACCES;
		goto out_path;
	}

	is_dir = S_ISDIR(inode->i_mode);

	read_lock(&trustee_hash_lock);
	amask = trustee_perm(&path, file_name, ret, depth, is_dir,
			     (struct trustee_hash_element **)NULL);
	read_unlock(&trustee_hash_lock);
	dmask = amask >> TRUSTEE_NUM_ACL_BITS;

	/* no permission if denied */
	if (trustee_mask_to_normal_mask(dmask, is_dir) & mask & ALL_MAYS) {
		ret = -EACCES;
		goto out;
	}
	/* use unix perms */
	if (!(dmask & TRUSTEE_USE_UNIX_MASK) &&
	    (amask & TRUSTEE_USE_UNIX_MASK) && (!ret))
		goto out;

	/* if the file isn't executable, then the trustees shouldn't
	 * make it executable
	 */
	if ((mask & MAY_EXEC) && !(mode & S_IXOTH) &&
	    !((mode >> 3) & S_IXOTH) & !((mode >> 6) & S_IXOTH) &&
	    (!is_dir)) {
		ret = -EACCES;
		goto out;
	}
	/* Check trustees for permission
	 */
	if ((trustee_mask_to_normal_mask(amask, is_dir) & mask & ALL_MAYS)
	    == mask) {
		ret = 0;
		goto out;
	} else
		ret = -EACCES;

out:
	kfree(file_name);
out_path:
	path_put(&path);

	return ret;
}

/* We should only allow hard links under one of two conditions:
 *   1. Its in the same trustee
 *        - if the two dentries are covered by the same trustee, there shouldn't
 *          be much of a problem with allowing the hardlink to occur.
 *   2. fsuid = 0
 */
static int trustees_inode_link(struct dentry *old_dentry,
			       struct inode *dir,
			       struct dentry *new_dentry)
{
	int ret = -EXDEV;
	struct path path1;
	struct path path2;

	if (current_fsuid() == 0)
		return 0;

	path1.dentry = dget(old_dentry);
	path1.mnt = find_inode_mnt(old_dentry->d_inode);
	path2.dentry = dget(new_dentry);
	path2.mnt = mntget(path1.mnt);

	if (have_same_trustees_rename(&path1, &path2))
		ret = 0;

	path_put(&path1);
	path_put(&path2);

	return ret;
}

/* We have a few renames to protect against:
 *   1. Any file or directory that is affected by different trustees at its
 *      old location than at its new location.
 *   2. In the case of a directory, we should protect against moving a directory
 *      that has trustees set inside of it.
 *
 * In any case above, we return -EXDEV which signifies to the calling program that
 * the files are on different devices, and assuming the program is written correctly
 * it should then handle the situation by copying the files and removing the originals
 * ( which will then use the trustees permissions as they are meant to be used )
 */
static int trustees_inode_rename(struct inode *old_dir,
				 struct dentry *old_dentry,
				 struct inode *new_dir,
				 struct dentry *new_dentry)
{
	return trustees_inode_link(old_dentry, new_dir, new_dentry);
}

/* Return CAP_DAC_OVERRIDE on everything.  We want to handle our own
 * permissions (overriding those normally allowed by unix permissions)
 */
static int trustees_capable(struct task_struct *tsk, const struct cred *cred,
			    int cap,
			    int audit)
{
	if (cap == CAP_DAC_OVERRIDE)
		return 0;

	return cap_capable(tsk, cred, cap, audit);
}

/* Register the security module
 */
int trustees_init_security(void)
{
	if (register_security(&trustees_security_ops)) {
		TS_ERR_MSG("Could not register security component\n");
		return -EINVAL;
	}

	return 0;
}
