/*
 * arch/arm/plat-oxnas/include/mach/oxnas_direct_disk.h
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
#ifndef __OXNAS_DIRECT_DISK_H__
#define __OXNAS_DIRECT_DISK_H__

#include <linux/fs.h>
#include <linux/workqueue.h>
#include <mach/oxnas_net.h>
#include <mach/filemap_info.h>
#include <mach/dma.h>

#define NUM_SATA_LISTS 2 

typedef struct oxnas_filemap_offset {
	int       cur_map_entry;
	long long cur_map_entry_offset;	/* in sectors */
} oxnas_filemap_offset_t;

typedef struct oxnas_direct_disk_context {
	oxnas_net_rx_context_t  net_rx_context;
	char                    frag_cache_name[24];
	getbmapx_t             *map;
	char                   *buffer;
	dma_addr_t              buffer_pa;
	int                     fs_blocklog;
	int                     port;
	int 					list_idx;
	int                     prealloc_write:1,
							sata_need_to_wait:1;
	atomic_t 				free_sg;
	atomic_t 				cur_transfer_idx;
	atomic_t 				cur_sg_status[NUM_SATA_LISTS];
	struct inode           *inode;
	struct semaphore        sata_active_sem;
	odrb_prd_list_t        *prd_list[NUM_SATA_LISTS];
	struct file            *file;
} oxnas_direct_disk_context_t;

extern int oxnas_get_fs_blocksize(struct file * file);
extern int oxnas_set_filesize(struct file * file, loff_t size);
extern int oxnas_reset_extent_preallocate_flag(struct file *file, loff_t offset, size_t length, int extent_flag, int disable_accumulation);

static inline void set_need_to_wait(oxnas_direct_disk_context_t* context)
{
	context->sata_need_to_wait = 1;
}

static inline void wait_sata_complete(oxnas_direct_disk_context_t* context)
{
	smp_rmb();
	if (context->sata_need_to_wait) {
		while (down_timeout(&context->sata_active_sem, HZ << 2)) {
			printk("wait_sata_complete() Still waiting context %p\n", context);
		}
//printk("wait_sata_complete() Down for context %p\n", context);
		context->sata_need_to_wait = 0;
		smp_wmb();
	}
}

extern void fast_writes_isr(int error, void *arg);

#endif        //  #ifndef __OXNAS_DIRECT_DISK_H__
