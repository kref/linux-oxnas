/*
 *  linux/arch/arm/mm/flush.c
 *
 *  Copyright (C) 1995-2002 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/pagemap.h>

#include <asm/cacheflush.h>
#include <asm/cachetype.h>
#include <asm/system.h>
#include <asm/tlbflush.h>
#ifdef CONFIG_SMP_LAZY_DCACHE_FLUSH
#include <mach/lazy-flush.h>
#endif // CONFIG_SMP_LAZY_DCACHE_FLUSH

#include "mm.h"

#ifdef CONFIG_ARM_ERRATA_411920
extern void v6_icache_inval_all(void);
#endif

#ifdef CONFIG_CPU_CACHE_VIPT

#define ALIAS_FLUSH_START	0xffff4000

static void flush_pfn_alias(unsigned long pfn, unsigned long vaddr)
{
	unsigned long to = ALIAS_FLUSH_START + (CACHE_COLOUR(vaddr) << PAGE_SHIFT);
	const int zero = 0;

	set_pte_ext(TOP_PTE(to), pfn_pte(pfn, PAGE_KERNEL), 0);
	flush_tlb_kernel_page(to);

	asm(	"mcrr	p15, 0, %1, %0, c14\n"
	"	mcr	p15, 0, %2, c7, c10, 4\n"
#ifndef CONFIG_ARM_ERRATA_411920
	"	mcr	p15, 0, %2, c7, c5, 0\n"
#endif
	    :
	    : "r" (to), "r" (to + PAGE_SIZE - L1_CACHE_BYTES), "r" (zero)
	    : "cc");
#ifdef CONFIG_ARM_ERRATA_411920
	v6_icache_inval_all();
#endif
}

void flush_cache_mm(struct mm_struct *mm)
{
	if (cache_is_vivt()) {
		if (cpu_isset(smp_processor_id(), mm->cpu_vm_mask))
			__cpuc_flush_user_all();
		return;
	}

	if (cache_is_vipt_aliasing()) {
		asm(	"mcr	p15, 0, %0, c7, c14, 0\n"
		"	mcr	p15, 0, %0, c7, c10, 4\n"
#ifndef CONFIG_ARM_ERRATA_411920
		"	mcr	p15, 0, %0, c7, c5, 0\n"
#endif
		    :
		    : "r" (0)
		    : "cc");
#ifdef CONFIG_ARM_ERRATA_411920
		v6_icache_inval_all();
#endif
	}
}

void flush_cache_range(struct vm_area_struct *vma, unsigned long start, unsigned long end)
{
	if (cache_is_vivt()) {
		if (cpu_isset(smp_processor_id(), vma->vm_mm->cpu_vm_mask))
			__cpuc_flush_user_range(start & PAGE_MASK, PAGE_ALIGN(end),
						vma->vm_flags);
		return;
	}

	if (cache_is_vipt_aliasing()) {
		asm(	"mcr	p15, 0, %0, c7, c14, 0\n"
		"	mcr	p15, 0, %0, c7, c10, 4\n"
#ifndef CONFIG_ARM_ERRATA_411920
		"	mcr	p15, 0, %0, c7, c5, 0\n"
#endif
		    :
		    : "r" (0)
		    : "cc");
#ifdef CONFIG_ARM_ERRATA_411920
		v6_icache_inval_all();
#endif
	}
	if (vma->vm_flags & VM_EXEC)
		__flush_icache_all();
}

void flush_cache_page(struct vm_area_struct *vma, unsigned long user_addr, unsigned long pfn)
{
	if (cache_is_vivt()) {
		if (cpu_isset(smp_processor_id(), vma->vm_mm->cpu_vm_mask)) {
			unsigned long addr = user_addr & PAGE_MASK;
			__cpuc_flush_user_range(addr, addr + PAGE_SIZE, vma->vm_flags);
		}
		return;
	}

	if (cache_is_vipt_aliasing())
		flush_pfn_alias(pfn, user_addr);
}

#ifdef CONFIG_SMP
static void flush_ptrace_access_other(void *args)
{
        __flush_icache_all();
}
#endif

void flush_ptrace_access(struct vm_area_struct *vma, struct page *page,
			 unsigned long uaddr, void *kaddr,
			 unsigned long len, int write)
{
	if (cache_is_vivt()) {
		if (cpu_isset(smp_processor_id(), vma->vm_mm->cpu_vm_mask)) {
			unsigned long addr = (unsigned long)kaddr;
			__cpuc_coherent_kern_range(addr, addr + len);
		}
		return;
	}

	if (cache_is_vipt_aliasing()) {
		flush_pfn_alias(page_to_pfn(page), uaddr);
		__flush_icache_all();
		return;
	}

	/* VIPT non-aliasing cache */
	if (vma->vm_flags & VM_EXEC) {
		/* only flushing the kernel mapping on non-aliasing VIPT */
		__cpuc_flush_dcache_area(kaddr, len);
        __flush_icache_all();
#ifdef CONFIG_SMP
		smp_call_function(flush_ptrace_access_other,
				  NULL, 1);
#endif

	}
}
#else
#define flush_pfn_alias(pfn,vaddr)	do { } while (0)
#endif

void __flush_dcache_page(struct address_space *mapping, struct page *page)
{
	/*
	 * Writeback any data associated with the kernel mapping of this
	 * page.  This ensures that data in the physical page is mutually
	 * coherent with the kernels mapping.
	 */
	__cpuc_flush_dcache_area(page_address(page), PAGE_SIZE);

	/*
	 * If this is a page cache page, and we have an aliasing VIPT cache,
	 * we only need to do one flush - which would be at the relevant
	 * userspace colour, which is congruent with page->index.
	 */
	if (mapping && cache_is_vipt_aliasing())
		flush_pfn_alias(page_to_pfn(page),
				page->index << PAGE_CACHE_SHIFT);
}

static void __flush_dcache_aliases(struct address_space *mapping, struct page *page)
{
	struct mm_struct *mm = current->active_mm;
	struct vm_area_struct *mpnt;
	struct prio_tree_iter iter;
	pgoff_t pgoff;

	/*
	 * There are possible user space mappings of this page:
	 * - VIVT cache: we need to also write back and invalidate all user
	 *   data in the current VM view associated with this page.
	 * - aliasing VIPT: we only need to find one mapping of this page.
	 */
	pgoff = page->index << (PAGE_CACHE_SHIFT - PAGE_SHIFT);

	flush_dcache_mmap_lock(mapping);
	vma_prio_tree_foreach(mpnt, &iter, &mapping->i_mmap, pgoff, pgoff) {
		unsigned long offset;

		/*
		 * If this VMA is not in our MM, we can ignore it.
		 */
		if (mpnt->vm_mm != mm)
			continue;
		if (!(mpnt->vm_flags & VM_MAYSHARE))
			continue;
		offset = (pgoff - mpnt->vm_pgoff) << PAGE_SHIFT;
		flush_cache_page(mpnt, mpnt->vm_start + offset, page_to_pfn(page));
	}
	flush_dcache_mmap_unlock(mapping);
}

#ifdef CONFIG_SMP_LAZY_DCACHE_FLUSH
/*
 * Ensure cache coherency between kernel mapping and userspace mapping
 * of this page.
 *
 * Same as the non-SMP case below, but this one remembers the identity of the 
 * CPU with the dirty caches in an atomic-friendly way.
 * 
 */
void flush_dcache_page(struct page *page)
{
	struct address_space *mapping = page_mapping(page);

	if (!PageHighMem(page) && mapping && !mapping_mapped(mapping)) {
		int this_cpu = get_cpu();
   		/* mark page for flush by our cpu */
		clear_dcache_clean_cpu(page, this_cpu);
		put_cpu();
	} else {
		__flush_dcache_page(mapping, page);
		if (mapping && cache_is_vivt())
			__flush_dcache_aliases(mapping, page);
		else if (mapping)
			__flush_icache_all();
	}

}

#ifdef CONFIG_SMP
void __sync_icache_dcache(pte_t pteval)
{
	unsigned long pfn = pte_pfn(pteval);

    struct page *page;
    
     if (!pfn_valid(pfn))
        return;
    
    page = pfn_to_page(pfn);
    
    if (pte_present_exec_user(pteval)) {
        
		int cpu_needs_flush_mask = set_dcache_clean(page);
        
        if (cpu_needs_flush_mask) {
            unsigned i;
            
            int this_cpu = get_cpu();
            // Iterate over all CPUs calling the appropriate flush function.
            // It's either the local one or it needs to be called remotely.
            for ( i = 0; i < NR_CPUS; ++i)
            {
                int cpu_needs_flush = ((1UL << i) & cpu_needs_flush_mask);
            
                if (cpu_needs_flush) {
                    if (i == this_cpu) {
                        /* flush locally */
                        __flush_dcache_page(NULL, page);
                    } else {
                        /* flush on other processor, (waiting for it to finish)*/
                        if (cpu_online(i)) {
                            smp_call_function_single(i,
                                                     remote_flush_dcache_page, (void*)page, 1);
                        }
                    }
                }
            }
            put_cpu();
        }
        
        __flush_icache_all();
    }
}
#endif

#else

#ifdef CONFIG_SMP
void __sync_icache_dcache(pte_t pteval)
{
	unsigned long pfn = pte_pfn(pteval);

	if (pfn_valid(pfn) && pte_present_exec_user(pteval)) {
		struct page *page = pfn_to_page(pfn);

		if (!test_and_set_bit(PG_dcache_clean, &page->flags))
			__flush_dcache_page(NULL, page);
		__flush_icache_all();
	}
}
#endif

/*
 * Ensure cache coherency between kernel mapping and userspace mapping
 * of this page.
 *
 * We have three cases to consider:
 *  - VIPT non-aliasing cache: fully coherent so nothing required.
 *  - VIVT: fully aliasing, so we need to handle every alias in our
 *          current VM view.
 *  - VIPT aliasing: need to handle one alias in our current VM view.
 *
 * If we need to handle aliasing:
 *  If the page only exists in the page cache and there are no user
 *  space mappings, we can be lazy and remember that we may have dirty
 *  kernel cache lines for later.  Otherwise, we assume we have
 *  aliasing mappings.
 *
 * Note that we disable the lazy flush for SMP.
 */
void flush_dcache_page(struct page *page)
{
	struct address_space *mapping = page_mapping(page);

#ifndef CONFIG_SMP
	if (!PageHighMem(page) && mapping && !mapping_mapped(mapping))
		clear_bit(PG_dcache_clean, &page->flags);
	else
#endif
	{
		__flush_dcache_page(mapping, page);
		if (mapping && cache_is_vivt())
			__flush_dcache_aliases(mapping, page);
		else if (mapping)
			__flush_icache_all();
		set_bit(PG_dcache_clean, &page->flags);
	}
}
#endif // CONFIG_SMP_LAZY_DCACHE_FLUSH

EXPORT_SYMBOL(flush_dcache_page);

/*
 * Flush an anonymous page so that users of get_user_pages()
 * can safely access the data.  The expected sequence is:
 *
 *  get_user_pages()
 *    -> flush_anon_page
 *  memcpy() to/from page
 *  if written to page, flush_dcache_page()
 */
void __flush_anon_page(struct vm_area_struct *vma, struct page *page, unsigned long vmaddr)
{
	unsigned long pfn;

	/* VIPT non-aliasing caches need do nothing */
	if (cache_is_vipt_nonaliasing())
		return;

	/*
	 * Write back and invalidate userspace mapping.
	 */
	pfn = page_to_pfn(page);
	if (cache_is_vivt()) {
		flush_cache_page(vma, vmaddr, pfn);
	} else {
		/*
		 * For aliasing VIPT, we can flush an alias of the
		 * userspace address only.
		 */
		flush_pfn_alias(pfn, vmaddr);
	}

	/*
	 * Invalidate kernel mapping.  No data should be contained
	 * in this mapping of the page.  FIXME: this is overkill
	 * since we actually ask for a write-back and invalidate.
	 */
	__cpuc_flush_dcache_area(page_address(page), PAGE_SIZE);
}
