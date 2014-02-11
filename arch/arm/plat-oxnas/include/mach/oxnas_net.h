/*
 * include/asm-arm/plat-oxnas/oxnas_net.h
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
#if !defined(__OXNAS_NET_H__)
#define __OXNAS_NET_H__

#include <linux/bio.h>
#include <linux/skbuff.h>
#include <linux/tcp.h>

#define NUM_NET_RX_FRAG_LISTS 2

typedef struct frag_list_entry {
	struct list_head head;
	struct bio_vec   bio_vec;
} frag_list_entry_t;

static inline void frag_list_entry_ctor(void *object)
{
	frag_list_entry_t *entry = object;
	INIT_LIST_HEAD(&entry->head);
}

typedef struct oxnas_net_data_ref {
	loff_t start_offset;
	loff_t length;
}oxnas_net_data_ref_t;

typedef struct oxnas_net_rx_context {
	struct list_head   page_info[NUM_NET_RX_FRAG_LISTS];
	int                page_cnt[NUM_NET_RX_FRAG_LISTS];
	int                frag_cnt[NUM_NET_RX_FRAG_LISTS];
	struct kmem_cache *frag_cache;
	int                fill_frag_list_idx;
	int                release_frag_list_idx;
	int                max_frag_cnt;
	oxnas_net_data_ref_t data_ref[NUM_NET_RX_FRAG_LISTS];
} oxnas_net_rx_context_t;

extern int oxnas_net_rx_actor(
	read_descriptor_t *desc,
	struct sk_buff    *skb,
	u32                offset);

extern void release_netdma_net_frags(oxnas_net_rx_context_t *context);

extern void release_netdma_net_frags_by_index(oxnas_net_rx_context_t *context, int release_idx);

static inline int is_dodgy_packet(struct sk_buff *skb)
{
	return (skb_shinfo(skb)->nr_frags == 0);
}

static inline void print_dodgy_packet(struct sk_buff *skb)
{
	unsigned char *ptr;
	unsigned int len;
	int i, j;
	struct skb_shared_info *shinfo = skb_shinfo(skb);

	printk(KERN_INFO "print_dodgy_packet() skb %p %d fragments, len %d, "
		"data_len %d, header_len %d, mac_len %d, ip_header_len %d, tcp->d_offx4 %d\n",
		skb, shinfo->nr_frags, skb->len, skb->data_len, skb->data - skb_mac_header(skb),
		skb->mac_len, skb->ip_header_len, tcp_hdr(skb)->doff * 4);

	for (i=0; i<4; ++i) {
		printk(KERN_INFO "  frag[%d]: page=0x%p, page_offset=%d, size=%d\n", i,
			shinfo->frags[i].page, shinfo->frags[i].page_offset,
			shinfo->frags[i].size);
	}

	printk(KERN_WARNING "Packet contents from MAC header:\n");
	ptr = skb_mac_header(skb);
	len = skb->len + (skb->data - skb_mac_header(skb));
	i = 0;
	while (i < len) {
		printk("%p: ", (void*)i);
		for (j=0; j<16 && i<len; ++i, ++j, ++ptr) {
			printk("%02x ", *ptr);
		}
		printk("\n");
	}
	printk(KERN_WARNING "Packet contents end\n");
}

#endif        //  #if !defined(__GMAC_H__)
