/*
 *  linux/include/asm-arm/arch-oxnas/memory.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* Max. size of each memory node */
#if defined (CONFIG_ARCH_OX820)
#define NODE_MAX_MEM_SHIFT (29) /* 512MB */
#endif

#if defined (CONFIG_ARCH_OX820)
#define MEM_MAP_ALIAS_SHIFT 30
#endif

#if defined (CONFIG_ARCH_OX820)
#define SDRAM_PA    (0x60000000)
#endif

#define SDRAM_SIZE  (1UL << (NODE_MAX_MEM_SHIFT))

#if defined (CONFIG_ARCH_OX820)
#define SRAM_PA	    (0x50000000)
#endif

/* Only a portion of the SRAM may be available for the use of Linux */
#define SRAM_SIZE   (CONFIG_SRAM_NUM_PAGES * PAGE_SIZE)

#define SDRAM_END   (SDRAM_PA + SDRAM_SIZE - 1)
#define SRAM_END    (SRAM_PA  + SRAM_SIZE  - 1)

#define PHYS_OFFSET SDRAM_PA

#define __virt_to_phys(x)   ((x) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt(x)   ((x) - PHYS_OFFSET + PAGE_OFFSET)

#define __virt_to_bus(x) __virt_to_phys(x)
#define __bus_to_virt(x) __phys_to_virt(x)

#endif // __ASM_ARCH_MEMORY_H
