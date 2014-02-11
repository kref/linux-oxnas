/*
 * arch/arm/plat-oxnas/oxnas_direct_net_to_disk.c
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
#include <mach/oxnas_direct_net_to_disk.h>

static char cache_name[15];
struct kmem_cache *direct_net_to_disk_frag_cache;

void free_direct_net_to_disk_context(direct_net_to_disk_context_t *context)
{
	kfree(context->iov);
	kfree(context);
}

static int __init direct_net_to_disk_init(void)
{
	int ret = 0;

	snprintf(cache_name, sizeof(cache_name), "DNTD-FragCache");

printk("Create fragment cache\n");
	direct_net_to_disk_frag_cache = kmem_cache_create(cache_name,
		  sizeof(frag_list_entry_t), 0, 0, frag_list_entry_ctor);

	if (!direct_net_to_disk_frag_cache) {
printk("Fragment cache creation failed\n");
		ret = -ENOMEM;
	}

	return ret;
}

static void __exit direct_net_to_disk_exit(void)
{
	if (direct_net_to_disk_frag_cache) {
printk("Destroy fragment cache\n");
		kmem_cache_destroy(direct_net_to_disk_frag_cache);
	}
}
 
module_init(direct_net_to_disk_init);
module_exit(direct_net_to_disk_exit);
