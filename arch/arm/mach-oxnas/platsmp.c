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
#include <asm/tlbflush.h>
#include <asm/cputype.h>
#include <asm/delay.h>
#include <linux/irqchip/arm-gic.h>
#include <mach/iomap.h>
#include <mach/smp.h>
#include <mach/hardware.h>

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit ox820_secondary_init(unsigned int cpu)
{
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit ox820_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * This is really belt and braces; we hold unintended secondary
	 * CPUs in the holding pen until we're ready for them.  However,
	 * since we haven't sent them a soft interrupt, they shouldn't
	 * be there.
	 */
	write_pen_release(cpu);

	writel(1, IOMEM(OXNAS_GICN_BASE_VA(cpu) + GIC_CPU_CTRL));

	/*
	 * Send the secondary CPU a soft interrupt, thereby causing
	 * the boot monitor to read the system wide flags register,
	 * and branch to the address found there.
	 */

	arch_send_wakeup_ipi_mask(cpumask_of(cpu));
	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (read_pen_release() == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return read_pen_release() != -1 ? -ENOSYS : 0;
}

void *scu_base_addr(void)
{
	return  IOMEM(OXNAS_SCU_BASE_VA);
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init ox820_smp_init_cpus(void)
{
	void __iomem *scu_base = scu_base_addr();
	unsigned int i, ncores;

	ncores = scu_base ? scu_get_core_count(scu_base) : 1;

	/* sanity check */
	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

static void __init ox820_smp_prepare_cpus(unsigned int max_cpus)
{

	scu_enable(scu_base_addr());

	/*
	 * Write the address of secondary startup into the
	 * system-wide flags register. The BootMonitor waits
	 * until it receives a soft interrupt, and then the
	 * secondary CPU branches to this address.
	 */
	writel(virt_to_phys(ox820_secondary_startup),
					HOLDINGPEN_LOCATION);
}

struct smp_operations ox820_smp_ops __initdata = {
	.smp_init_cpus		= ox820_smp_init_cpus,
	.smp_prepare_cpus	= ox820_smp_prepare_cpus,
	.smp_secondary_init	= ox820_secondary_init,
	.smp_boot_secondary	= ox820_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= ox820_cpu_die,
#endif
};
