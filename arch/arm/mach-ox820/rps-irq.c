/*
 * arch/arm/mach-ox820/rps-irq.c
 *
 * Copyright (C) 2009 Oxford Semiconductor Ltd
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
#include <linux/init.h>
#include <linux/list.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/rps-irq.h>

/* change the system interrupt number into an RPS interrupt number */
static inline unsigned int OX820_RPS_irq(unsigned int irq) {
	struct OX820_RPS_chip_data* chip_data = get_irq_chip_data(irq);
	return irq - chip_data->irq_offset;
}

static void OX820_RPS_mask_irq(unsigned int irq)
{
    *((volatile unsigned long*)RPSA_IRQ_DISABLE) = (1UL << OX820_RPS_irq(irq));
}

static void OX820_RPS_unmask_irq(unsigned int irq)
{
    *((volatile unsigned long*)RPSA_IRQ_ENABLE) = (1UL << OX820_RPS_irq(irq));
}

static struct irq_chip OX820_RPS_chip = {
	.name	= "OX820_RPS",
    .ack	= OX820_RPS_mask_irq,
    .mask	= OX820_RPS_mask_irq,
    .unmask = OX820_RPS_unmask_irq,
};

static struct OX820_RPS_chip_data OX820_RPS_irq_data;

static void OX820_RPS_handle_cascade_irq(unsigned int irq, struct irq_desc *desc) {
	struct OX820_RPS_chip_data* chip_data = get_irq_data(irq);
	struct irq_chip *chip = get_irq_chip(irq);
	unsigned int cascade_irq, rps_irq;
	unsigned long status;

	/* primary controller ack'ing */
	chip->ack(irq);

    /* read the IRQ number from the RPS core */
	status = *((volatile unsigned long*)RPSA_IRQ_STATUS);

    /* convert the RPS interrupt number into a system interrupt number */
	rps_irq = find_first_bit(&status, 31);
	cascade_irq = rps_irq + chip_data->irq_offset;
	if (unlikely(cascade_irq >= NR_IRQS))
		do_bad_IRQ(cascade_irq, desc);
	else
		generic_handle_irq(cascade_irq);

	/* primary controller unmasking */
	chip->unmask(irq);
}

/*
 * Tell the kernel that "irq" is gnerated by our interrupt chip and needs
 * to run the OX820_RPS_handle_cascade_irq to get the actual source of the
 * interrupt.
 */
void __init OX820_RPS_cascade_irq(unsigned int irq) {
    /* change the irq chip data to point to the RPS chip data */
	if (set_irq_data(irq, (void*)&OX820_RPS_irq_data) != 0)
		BUG();
    
    /* setup the handler */
	set_irq_chained_handler(irq, OX820_RPS_handle_cascade_irq);
}

void __init OX820_RPS_init_irq(unsigned int start, unsigned int end) {
    unsigned irq;

    printk("OX820_RPS_init_irq: interrupts %d to %d\n",start,end);
    /* Disable all IRQs */
    *((volatile unsigned long*)RPSA_IRQ_DISABLE) = ~0UL;

    /* store the offset information */
    OX820_RPS_irq_data.irq_offset = start;
    
    /* Initialise IRQ tracking structures */
    for (irq = start; irq < end; irq++)
    {
        set_irq_chip(irq, &OX820_RPS_chip);
        set_irq_chip_data(irq, (void*)&OX820_RPS_irq_data);
        set_irq_handler(irq, handle_level_irq);
        set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
    }
}

/** 
 * induces a FIQ in the cpu number passed as a parameter
 */
void OX820_RPS_trigger_fiq(unsigned int cpu) {
    void* base[2] = {
        (void* )RPSA_BASE,
        (void* )RPSC_BASE
    };
    
    *((volatile unsigned long*)(base[cpu] + RPS_FIQ_IRQ_TO_FIQ)) = 1;
    wmb();
    *((volatile unsigned long*)(base[cpu] + RPS_FIQ_ENABLE)) = 1;
    wmb();
    *((volatile unsigned long*)(base[cpu] + RPS_IRQ_SOFT)) = 2;
    wmb();
}

/* clears FIQ mode for the interrupt specified, those interrupts will now 
be forwarded on the IRQ output line */
void OX820_RPS_clear_fiq(unsigned int cpu) {
    void* base[2] = {
        (void* )RPSA_BASE,
        (void* )RPSC_BASE
    };
    
    *((volatile unsigned long*)(base[cpu] + RPS_IRQ_SOFT)) = 0;
    wmb();
    *((volatile unsigned long*)(base[cpu] + RPS_FIQ_DISABLE)) = 1;
    wmb();
}
