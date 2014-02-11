/*
 * drivers/md/hwraid.h
 *
 * Copyright (C) 2010 PLX Technology 
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#ifndef _HWRAID_H
#define _HWRAID_H

struct mddev_s;
struct raid0_private_data;
struct r1_private_data_s;
struct raidset_s;
struct mdk_rdev_s;

extern int hwraid_stop_new_commands(void);
extern void hwraid_start_new_commands(void);

/**
 * RAID-0
 */
extern int hwraid0_compatible(struct mddev_s*, struct raid0_private_data* );
extern int hwraid0_assemble(struct mddev_s*, struct raid0_private_data* );
extern void hwraid0_stop(struct mddev_s*, struct raid0_private_data* );

/**
 * RAID-1
 */
extern int  hwraid1_run(struct mddev_s*, struct r1_private_data_s*); 
extern int  hwraid1_add_disk(struct mddev_s*, struct mdk_rdev_s*);
extern int  hwraid1_remove_disk(struct mddev_s*, int, struct mdk_rdev_s*);
extern void hwraid1_reshape_begin(struct mddev_s*);
extern void hwraid1_reshape_end(struct mddev_s*);
extern void hwraid1_stop(struct mddev_s* );
extern void hwraid1_error(struct mddev_s*, struct mdk_rdev_s*);
extern sector_t hwraid1_sync_request(mddev_t*, sector_t, int*, int);

#endif        /*  #ifndef _HWRAID_H */
