#include <mach/iomap.h>

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =OXNAS_UART1_BASE
	ldr	\rv, =OXNAS_UART1_BASE_VA
	.endm

#define UART_SHIFT	0
#include <asm/hardware/debug-8250.S>
