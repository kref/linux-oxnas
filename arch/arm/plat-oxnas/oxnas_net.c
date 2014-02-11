/*
 * arch/arm/plat-oxnas/oxnas_net.c
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
#include <net/tcp.h>
#include <mach/oxnas_net.h>
#include <linux/tcp.h>

/**
 * Satisfy as much of the requirement as possible from the passed skb and its
 * fragments (not including any fraglist)
 *
 * We cannot maintain a reference to the skb itself, because of the way
 * the stack unconditionally frees the skb in sk_eat_sbk(), but we can assume
 * that we're always using receive-into-pages so that the headlen data will
 * also exist at the start of the first fragment buffer.
 *
 * @param skb Skb from which to use data
 * @param offset Offset into skb from which to begin using data
 * @param required Amount of data still outstanding to satisfy user request
 * @return Amount of data used from skb
 */
static int get_pages_from_skb(
	struct sk_buff         *skb,
	u32                    *offset,
	int                     required,
	oxnas_net_rx_context_t *context)
{
	int                     i;
	struct skb_shared_info *shinfo;
	int                     header_len;
	skb_frag_t             *frag;
	struct list_head       *page_list;
	frag_list_entry_t      *entry;
	u32                     data_in_fragment;
	int                     used = 0;
	struct page            *prev_page = 0;
	u32                     prev_offset = 0;
	frag_list_entry_t      *prev_entry = 0;
	int                     hlen;
	int                     desc_len;
	int                     pulled = 0;
	int                     length_in_skb;

	/* Check contract */
	BUG_ON(!skb);
	BUG_ON(!offset);
	BUG_ON(required == 0);
	BUG_ON(!context);

	if (unlikely(is_dodgy_packet(skb))) {
		print_dodgy_packet(skb);

		/* Not much we can do if a dodgy packet has made it this far */
		BUG();
	}

	shinfo = skb_shinfo(skb);

	/* Length of Eth+IP+TCP headers */
	header_len = skb->mac_len + skb->ip_header_len + (tcp_hdr(skb)->doff * 4);

	/* Start of packet data in first fragment buffer */
	frag = &shinfo->frags[0];

	/* skb->len will at this point have been adjusted so as not to include the
	   Eth+IP+TCP headers lengths. Cope with the stack having pulled data
	   from the fragments into the skb which could result in the length in
	   the skb being greater than the amount copied in by the GMAC driver */
	length_in_skb = header_len + skb->len - skb->data_len;
	if (length_in_skb > CONFIG_OXNAS_GMAC_HLEN) {
		hlen = CONFIG_OXNAS_GMAC_HLEN;
		pulled = length_in_skb - CONFIG_OXNAS_GMAC_HLEN;
//printk(KERN_INFO "get_pages_from_skb() skb %p, hlen=%d != expected %d, pulled = %d\n", skb, hlen, CONFIG_OXNAS_GMAC_HLEN, pulled);
	} else {
		hlen = length_in_skb;
	}

	desc_len = hlen + frag->size + pulled; 
	data_in_fragment = desc_len - header_len;

	/* List of all pages of Rx payload accumulated so far */
	page_list = &context->page_info[context->fill_frag_list_idx];

//printk("get_pages_from_skb() skb %p: hlen %d, desc_len %d, data_in_fragment %d *offset %d\n", skb, hlen, desc_len, data_in_fragment, *offset);

	/* Get refs to all the other fragments */
	if (*offset >= data_in_fragment) {
		/* Adjust offset relative to start of next fragment */
		*offset -= data_in_fragment;
//printk(KERN_INFO "get_pages_from_skb() skb 0x%p offset %u is beyound first fragment\n", skb, *offset);
	} else {
		unsigned len_from_frag = min((u32)required, data_in_fragment - *offset);

		if (context->frag_cache) {
			entry = kmem_cache_alloc(context->frag_cache, GFP_KERNEL);
		} else {
			entry = kmalloc(sizeof(frag_list_entry_t), GFP_KERNEL);
			INIT_LIST_HEAD(&entry->head);
		}

		entry->bio_vec.bv_page   = frag->page;
		entry->bio_vec.bv_offset = frag->page_offset - pulled - hlen + header_len + *offset;
		entry->bio_vec.bv_len    = len_from_frag;

		prev_entry  = entry;
		prev_page   = frag->page;
		prev_offset = entry->bio_vec.bv_offset + len_from_frag;

		used	 += len_from_frag;
		required -= len_from_frag;

		/* Make the page stay around while we're using it */
		get_page(frag->page);

		list_add_tail(&entry->head, page_list);
		++context->page_cnt[context->fill_frag_list_idx];

		/* Made use of offset to get to the start of the available data */
		*offset = 0;

		++context->frag_cnt[context->fill_frag_list_idx];
//		printk("get_pages_from_skb() skb %p: used %d from first fragment - frag count - %d\n", skb, len_from_frag, context->frag_cnt[context->fill_frag_list_idx]);
	}

	for (i=1; required && (i < shinfo->nr_frags) &&
			(context->frag_cnt[context->fill_frag_list_idx] <
				context->max_frag_cnt); i++) {
//printk(KERN_INFO "get_pages_from_skb() skb 0x%p looking for data in fragments not the first\n", skb);
		frag = &shinfo->frags[i];
		data_in_fragment = frag->size;

		if (*offset >= data_in_fragment) {
			/* Adjust offset relative to start of next fragment */
			*offset -= data_in_fragment;
		} else {
			u32		 next_offset   = frag->page_offset + *offset;
			unsigned len_from_frag = min((u32)required, data_in_fragment - *offset);

			if (prev_entry && (frag->page == prev_page) &&
				(next_offset == prev_offset)) {
				entry = prev_entry;
				entry->bio_vec.bv_len += len_from_frag;
			} else {
				if (context->frag_cache) {
					entry = kmem_cache_alloc(context->frag_cache, GFP_KERNEL);
				} else {
					entry = kmalloc(sizeof(frag_list_entry_t), GFP_KERNEL);
					INIT_LIST_HEAD(&entry->head);
				}

				entry->bio_vec.bv_page   = frag->page;
				entry->bio_vec.bv_offset = next_offset;
				entry->bio_vec.bv_len    = len_from_frag;

				/* Make the page stay around while we're using it */
				get_page(frag->page);

				list_add_tail(&entry->head, page_list);
				++context->page_cnt[context->fill_frag_list_idx];
			}

			prev_entry	= entry;
			prev_page	= entry->bio_vec.bv_page;
			prev_offset = entry->bio_vec.bv_offset + entry->bio_vec.bv_len;

			used	 += len_from_frag;
			required -= len_from_frag;

			/* Made use of offset to get to the start of the available data */
			*offset = 0;
			++context->frag_cnt[context->fill_frag_list_idx];
			
//			printk("get_pages_from_skb() skb %p: used %d from fragment %d frag counted - %d\n", skb, len_from_frag, i, context->frag_cnt[context->fill_frag_list_idx]);
		}
	}

	return used;
}

/**
 * Get references to as much data as we need from the skb, its frags and any
 * fraglist - returns non zero if max frags reached
 */
int oxnas_net_rx_actor(
	read_descriptor_t *desc,
	struct sk_buff    *skb,
	u32                offset)
{
	oxnas_net_rx_context_t *context;
	u32	                    local_offset = offset;
	int 					retval = 0;

	/* Check contract */
	BUG_ON(!desc);
	BUG_ON(!skb);

	context = (oxnas_net_rx_context_t*)desc->arg.data;

	/* Use data from the skb and its fragments */
	desc->count -= get_pages_from_skb(skb, &local_offset, desc->count, context);

	/* Satisfy any remaining requirement from the fraglist */
	for (skb = skb_shinfo(skb)->frag_list; skb &&
			desc->count && (context->frag_cnt[context->fill_frag_list_idx] <
				context->max_frag_cnt); skb = skb->next) {
//		printk(KERN_INFO "oxnas_net - max descs - %d, cur desc count - %d\n", context->max_frag_cnt, context->frag_cnt[context->fill_frag_list_idx]); 
		desc->count -= get_pages_from_skb(skb, &local_offset, desc->count, context);
	}
	
	retval = (context->frag_cnt[context->fill_frag_list_idx] == context->max_frag_cnt) ? 1 : 0; 
	
	return retval;
}
EXPORT_SYMBOL(oxnas_net_rx_actor);

void release_netdma_net_frags(oxnas_net_rx_context_t *context)
{
	int release_idx = context->release_frag_list_idx;
	if (release_idx >= 0) {
		struct list_head *list_head = &context->page_info[release_idx];

		while (!list_empty(list_head)) {
			struct frag_list_entry *frag =
				list_entry(list_head->next, struct frag_list_entry, head);
			struct page *page = frag->bio_vec.bv_page;

			if (context->frag_cache) {
				/* Remove the fragment from the list */
				list_del_init(&frag->head);

				/* Return the fragment descriptor to the pool */
				kmem_cache_free(context->frag_cache, frag);
			} else {
				list_del(&frag->head);
				kfree(frag);
			}

			--context->page_cnt[release_idx];

			/* Drop reference to the page described by the fragment */
			put_page(page);
		}
		context->frag_cnt[release_idx] = 0;
	}
}
EXPORT_SYMBOL(release_netdma_net_frags);

void release_netdma_net_frags_by_index(
	oxnas_net_rx_context_t *context,
	int                     release_idx)
{
	if (release_idx >= 0) {
		struct list_head *list_head = &context->page_info[release_idx];

		while (!list_empty(list_head)) {
			struct frag_list_entry *frag =
				list_entry(list_head->next, struct frag_list_entry, head);
			struct page *page = frag->bio_vec.bv_page;

			if (context->frag_cache) {
				/* Remove the fragment from the list */
				list_del_init(&frag->head);

				/* Return the fragment descriptor to the pool */
				kmem_cache_free(context->frag_cache, frag);
			} else {
				list_del(&frag->head);
				kfree(frag);
			}

			--context->page_cnt[release_idx];

			/* Drop reference to the page described by the fragment */
			put_page(page);
		}
		context->frag_cnt[release_idx] = 0;
	}
}
EXPORT_SYMBOL(release_netdma_net_frags_by_index);
