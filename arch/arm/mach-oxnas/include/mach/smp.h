/*
 * smp.h
 *
 *  Created on: Sep 24, 2013
 *      Author: mahaijun
 */

#ifndef _NAS782X_SMP_H_
#define _NAS782X_SMP_H_

#include <mach/hardware.h>

extern void ox820_secondary_startup(void);
extern void ox820_cpu_die(unsigned int cpu);

static inline void write_pen_release(int val)
{
	writel(val, HOLDINGPEN_CPU);
}

static inline int read_pen_release(void)
{
	return readl(HOLDINGPEN_CPU);
}

#endif /* _NAS782X_SMP_H_ */
