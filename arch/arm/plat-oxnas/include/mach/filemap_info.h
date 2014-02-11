/*
 * arch/arm/plat-oxnas/include/mach/filemap_info.h
 *
 * Copyright (C) 2009 Oxford Semiconductor Ltd
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

#ifndef FILEMAP_INFO_H
#define FILEMAP_INFO_H

#include <linux/semaphore.h>
#include <mach/direct-storage.h>

typedef struct getbmap {
	long long bmv_offset;	/* file offset of segment in blocks */
	long long bmv_block;	/* starting block (64-bit daddr_t)  */
	long long bmv_length;	/* length of segment, blocks	    */
	long	  bmv_count;	/* # of entries in array incl. 1st  */
	long	  bmv_entries;	/* # of entries filled in (output). */
} getbmap_t;
#define HAVE_GETBMAP

#define GETBMAPX_OF_PREALLOC	0x1
#define GETBMAPX_OF_DELALLOC	0x2
#define GETBMAPX_BLOCK_HOLE		(-1LL)

typedef struct getbmapx {
	long long bmv_offset;	/* file offset of segment in blocks */
	long long bmv_block;	/* starting block (64-bit daddr_t)  */
	long long bmv_length;	/* length of segment, blocks	    */
	long	  bmv_count;	/* # of entries in array incl. 1st  */
	long	  bmv_entries;	/* # of entries filled in (output). */
	long	  bmv_iflags;	/* input flags (1st structure)	    */
	long	  bmv_oflags;	/* output flags (after 1st structure)*/
	long	  bmv_unused1;	/* future use			    */
	long	  bmv_unused2;	/* future use			    */
} getbmapx_t;
#define HAVE_GETBMAPX

typedef struct oxnas_filemap_info {
	direct_access_context_t *direct_access_context;
	getbmapx_t *map;
	int         used_extents;
	int         alloced_extents;
	loff_t		length;
} oxnas_filemap_info_t;

#endif        //  #ifndef FILEMAP_INFO_H
