#include <mach/iomap.h>

#define OXNAS_UART1_BASE_VA	0xF4200000

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =OXNAS_UART1_BASE
	ldr	\rv, =OXNAS_UART1_BASE_VA
	.endm

#define UART_SHIFT	0
#include <asm/hardware/debug-8250.S>
