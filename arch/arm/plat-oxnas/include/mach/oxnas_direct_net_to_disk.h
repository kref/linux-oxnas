/*
 * arch/arm/plat-oxnas/include/mach/oxnas_direct_net_to_disk.h
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
#ifndef __OXNAS_DIRECT_NET_TO_DISK_H__
#define __OXNAS_DIRECT_NET_TO_DISK_H__

#include <mach/oxnas_net.h>

extern struct kmem_cache *direct_net_to_disk_frag_cache;

typedef struct direct_net_to_disk_context {
	oxnas_net_rx_context_t  netrx_context;
	struct iovec		   *iov;
	size_t					iov_size;
} direct_net_to_disk_context_t;

extern void free_direct_net_to_disk_context(direct_net_to_disk_context_t *context);

#endif // __OXNAS_DIRECT_NET_TO_DISK_H__
