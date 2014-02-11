/*
 * arch/arm/mach-ox820/include/mach/lazy-flush.h
 *
 * Copyright (C) 2009 PLX Technology
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

#ifndef __LAZY_FLUSH_H
#define __LAZY_FLUSH_H

#ifdef CONFIG_SMP_LAZY_DCACHE_FLUSH

#if (CONFIG_NR_CPUS > 2)
#error Too many cpus for lazy flush_dcache_page
#endif

/**
 * These define a mask for recording the CPU that dirtied the page for lazy
 * flush_dcache_page in SMP systems
 */
#define PG_dcache_cpu_shift (PG_dcache_cpu)
#define PG_dcache_cpu_mask  1

/**
 * Returns the identity of the cpu that may have dirty cache-lines containing 
 * data from the page.
 */
static inline int dcache_dirty_cpu(struct page* page) {
	return (((page)->flags >> PG_dcache_cpu_shift) & PG_dcache_cpu_mask);
}


/**
 * This will atomically clear the clean bit and set the cpu dirty bit
 * for this cpu in the page->flags
 */
static inline void clear_dcache_clean_cpu(struct page* page, int this_cpu) {
	unsigned long flags;
	unsigned long set_bits;
	unsigned long clear_bits;

	clear_bits = (1UL << PG_dcache_clean);
	set_bits = (1UL << (this_cpu + PG_dcache_cpu_shift));
    
	local_irq_save(flags);
	asm volatile(
		"1: ldrex r5, [%2]    \n"	/* get page->flags for exclusive access */
		"   bic   r5, r5, %1  \n"	/* mask out clear bits - the clean bit */
		"   orr   r5, r5, %0  \n"	/* add set bits - the cpu dirty bit */
		"   strex r6, r5, [%2]\n"	/* try and write back to mem */ 
		"   teq	  r6, #0      \n"	/* did we get it in? if not try again */
		"   bne	1b            \n"
		: /* no outputs */
		: "r" (set_bits), "r" (clear_bits), "r" (&page->flags)
		: "memory", "cc", "r5", "r6");
	local_irq_restore(flags);
}

/** 
 * This will atomically set the clean bit in page->flags and return the
 * cpu dirty mask prior to the clean
 */
static inline unsigned set_dcache_clean(struct page* page) {
	unsigned long flags;
    unsigned long page_flags;
    
	unsigned long set_bits;
	unsigned long clear_bits;

	set_bits = (1UL << PG_dcache_clean);
	clear_bits = ((1UL << NR_CPUS) - 1 ) <<  PG_dcache_cpu_shift;

	local_irq_save(flags);


	asm volatile(
		"1: ldrex r5, [%3]    \n"	/* get page->flags for exclusive access */
		"   bic   r4, r5, %2  \n"	/* mask out all cpu dirty bits */
		"   orr   r4, r4, %1  \n"	/* set the dcache clean bit */
		"   strex r6, r4, [%3]\n"	/* try and write back to mem */ 
		"   teq	  r6, #0      \n"	/* did we get it in? if not try again */
		"   bne	1b            \n"
        "   str   r5, [%0]    \n"   /* save cpu dirty flags prior to reset */
		:  /* outputs */
		: "r" (&page_flags) ,"r" (set_bits), "r" (clear_bits), "r" (&page->flags)
		: "memory", "cc", "r4", "r5", "r6");


    local_irq_restore(flags);

    return (page_flags &  (((1UL << NR_CPUS) - 1 ) <<  PG_dcache_cpu_shift)) >>  PG_dcache_cpu_shift;
}

void remote_flush_dcache_page(void *info);

#endif // CONFIG_SMP_LAZY_DCACHE_FLUSH
#endif
