/*
 * arch/arm/mach-ox820/include/mach/ipi.h
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
#ifndef _IPI_H
#define _IPI_H

/*
 * data structures for communicating cache coherency operations between
 * processors using FIQ
 */

/* max number of SGDMA entries that can be communicated in one go */
#define IPI_SGDMA_ELEMENTS 256

struct smp_dma_cache_range_s {
	const void *start;
	const void *end;
};

struct cache_coherency_communication_s {
	/* same as the type parameter in dma_map_single() */
	int type; 
	struct smp_dma_cache_range_s range[IPI_SGDMA_ELEMENTS];
};

struct tlb_args {
	struct vm_area_struct *ta_vma;
	unsigned long ta_start;
	unsigned long ta_end;
};

struct tlb_communication {
	void *tlb_arg;
	void (*tlb_op)(void *arg);
};

enum fiq_message_type {
	CACHE_COHERENCY = 0,
	TLB = 1
};

struct fiq_coherency_communication_s {
	enum fiq_message_type type;
	int nents;
	union {        
		struct cache_coherency_communication_s cache_coherency;
		struct tlb_communication tlb;
	} message;        
};

/* When working with the two CPUs, the initiating CPU number is used to index
 * into this per-cpu variable */
DECLARE_PER_CPU(struct fiq_coherency_communication_s, fiq_coherency_communication);

#endif        //  #ifndef _IPI_H
