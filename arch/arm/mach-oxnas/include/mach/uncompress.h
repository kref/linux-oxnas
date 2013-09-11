/* linux/include/asm-arm/arch-oxnas0/uncompress.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <mach/iomap.h>

static inline void putc(int c)
{
#ifdef CONFIG_OXNAS_UART1
    static volatile unsigned char* uart = (volatile unsigned char*)OXNAS_UART1_BASE;
#elif defined(CONFIG_OXNAS_UART2)
    static volatile unsigned char* uart = (volatile unsigned char*)OXNAS_UART2_BASE;
#elif defined(CONFIG_OXNAS_UART3)
    static volatile unsigned char* uart = (volatile unsigned char*)OXNAS_UART3_BASE;
#elif defined(CONFIG_OXNAS_UART4)
    static volatile unsigned char* uart = (volatile unsigned char*)OXNAS_UART4_BASE;
#else
#define NO_UART
#endif

#ifndef NO_UART
    while (!(uart[5] & 0x20)) { /* LSR reg THR empty bit */
        barrier();
    }
    uart[0] = c;                /* THR register */
#endif // NO_UART
}

static inline void flush(void)
{
}

#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif // __ASM_ARCH_UNCOMPRESS_H
