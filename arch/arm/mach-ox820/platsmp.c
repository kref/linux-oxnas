/*
 *  arch/arm/mach-ox820/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <asm/localtimer.h>
#include <asm/smp_scu.h>
#include <mach/rps-irq.h>
#include <mach/hardware.h>
#include <asm/tlbflush.h>
#include <asm/cputype.h>
#include <mach/rps-timer.h>
#include <linux/irqchip/arm-gic.h>
#include <mach/smp.h>
#include <mach/ipi.h>

static DEFINE_SPINLOCK(boot_lock);

extern void ox820_secondary_startup(void);

static void __iomem *scu_base = __io_address(OX820_ARM11MP_SCU_BASE);

//number of physical core count
static inline unsigned int get_core_count(void)
{
	return scu_get_core_count(scu_base);
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init oxnas_init_cpus(void)
{
/*
 * Setup the set of possible CPUs (via set_cpu_possible)
 */
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

/*
 * Initialize cpu_possible map, and enable coherency
 */
void __init oxnas_prepare_cpus(unsigned int max_cpus)
{
	/* Initialise the SCU */
	scu_enable(scu_base);

}

static void __cpuinit write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}


int __cpuinit oxnas_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);
	/*
	 * Don't know why we need this when realview and omap2 appear not to, but
	 * the secondary CPU doesn't start without it.
	 */
	flush_cache_all();
	/*
	 * Enable gic interrupts on the secondary CPU so the interrupt that wakes
	 * it from WFI can be received
	 */

	writel(1, __io_address(OX820_GIC_CPUN_BASE_ADDR(cpu) + GIC_CPU_CTRL));
	/* Write the address that we want the cpu to start at. */
	writel(virt_to_phys(ox820_secondary_startup), HOLDINGPEN_LOCATION);
	wmb();
	writel(cpu, HOLDINGPEN_CPU);
	wmb();
	write_pen_release(cpu);
	/* Wake the CPU from WFI */
	arch_send_wakeup_ipi_mask(get_cpu_mask(cpu));
	/* Give the secondary CPU time to get going */
	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout))
	{
		smp_rmb();
		if (pen_release == -1)
			break;
	}
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}


/*
 * Perform platform specific initialisation of (ON) the specified CPU.
 */
void __cpuinit oxnas_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();

	/*
	 * If any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);
	write_pen_release(-1);
	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static DECLARE_COMPLETION(cpu_killed);

static inline void cpu_enter_lowpower(void)
{
	unsigned int v;

	flush_cache_all();
	asm volatile(
	"	mcr	p15, 0, %1, c7, c5, 0\n"
	"	mcr	p15, 0, %1, c7, c10, 4\n"
	/*
	 * Turn off coherency
	 */
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	bic	%0, %0, #0x20\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	bic	%0, %0, #0x04\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (v)
	  : "r" (0)
	  : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile(	"mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, #0x04\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, #0x20\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  :
	  : "cc");
}

static inline void platform_do_lowpower(unsigned int cpu)
{
	/* Copied from Realview where they use a pen_release variable to tell if
	   the CPU has been intentionally woken from interrupt. They don't do
	   anything if the wake is unintentional, so may as well ignore for now */
	asm(".word	0xe320f003\n"
		:
		:
		: "memory", "cc");
}

int oxnas_cpu_kill(unsigned int cpu)
{
	return wait_for_completion_timeout(&cpu_killed, 5000);
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void oxnas_cpu_die(unsigned int cpu)
{
#ifdef DEBUG
	unsigned int this_cpu = hard_smp_processor_id();

	if (cpu != this_cpu) {
		printk(KERN_CRIT "Eek! platform_cpu_die running on %u, should be %u\n",
			   this_cpu, cpu);
		BUG();
	}
#endif

	printk(KERN_NOTICE "CPU%u: shutdown\n", cpu);
	complete(&cpu_killed);

	/*
	 * we're ready for shutdown now, so do it
	 */
	cpu_enter_lowpower();
	platform_do_lowpower(cpu);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();
}

int oxnas_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}

struct smp_operations oxnas_smp_ops __initdata = {
	.smp_init_cpus		= oxnas_init_cpus,
	.smp_prepare_cpus	= oxnas_prepare_cpus,
	.smp_secondary_init	= oxnas_secondary_init,
	.smp_boot_secondary	= oxnas_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill		= oxnas_cpu_kill,
	.cpu_die		= oxnas_cpu_die,
	.cpu_disable		= oxnas_cpu_disable,
#endif
};
