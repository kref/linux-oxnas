/*
 * arch/arm/mach-ox820/rps-time.c
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

enum {
	TIMER_LOAD = 0,
	TIMER_CURR = 4,
	TIMER_CTRL = 8,
	TIMER_CLRINT = 0xC,

	TIMER_BITS = 24,

	TIMER_MAX_VAL = (1 << TIMER_BITS) - 1,

	TIMER_PERIODIC = (1 << 6),
	TIMER_ENABLE = (1 << 7),

	TIMER_DIV1  = (0 << 2),
	TIMER_DIV16  = (1 << 2),
	TIMER_DIV256  = (2 << 2),

	TIMER1_OFFSET = 0,
	TIMER2_OFFSET = 0x20,

};

struct rps_clock_evt_device {
	struct clock_event_device evt;
	void __iomem *base;
};

struct rps_clock_evt_device *to_rps_evt_device(struct clock_event_device *evt)
{
	return container_of(evt, struct rps_clock_evt_device, evt);
}

static int rps_set_next_event(unsigned long cycles,
				struct clock_event_device *evt)
{
	struct rps_clock_evt_device *rps_timer = to_rps_evt_device(evt);
	u32 ctrl;

	if (cycles == 0)
		return -ETIME;

	/* Stop timers before programming */
	ctrl = ioread32(rps_timer->base + TIMER_CTRL);
	iowrite32(ctrl & ~TIMER_ENABLE, rps_timer->base + TIMER_CTRL);

    /* Setup timer 1 load value */
	iowrite32(cycles, rps_timer->base + TIMER_LOAD);

    /* Setup timer 1 prescaler, periodic operation and start it */
	iowrite32(TIMER_ENABLE | TIMER_DIV16, rps_timer->base + TIMER_CTRL);

	return 0;
}

static inline void rps_set_mode(enum clock_event_mode mode,
					 struct clock_event_device *evt)
{
	struct rps_clock_evt_device *rps_timer = to_rps_evt_device(evt);
	u32 ctrl;
	unsigned long cycles_per_jiffy;

	/* Stop timers before programming */
	 ctrl = ioread32(rps_timer->base + TIMER_CTRL);
	iowrite32(ctrl & ~TIMER_ENABLE, rps_timer->base + TIMER_CTRL);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		cycles_per_jiffy =
				(((unsigned long long) NSEC_PER_SEC / HZ * evt->mult) >> evt->shift);

		/* Setup timer 1 load value */
		iowrite32(cycles_per_jiffy, rps_timer->base + TIMER_LOAD);
		/* Setup timer 1 prescaler, periodic operation and start it */
		iowrite32(ctrl | TIMER_ENABLE | TIMER_PERIODIC,
				rps_timer->base + TIMER_CTRL);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static irqreturn_t rps_clock_event_isr(int irq, void *dev_id)
{
	struct rps_clock_evt_device *rps_timer = dev_id;
	 /* Clear the timer interrupt - any write will do */
	iowrite32(0, rps_timer->base + TIMER_CLRINT);
	rps_timer->evt.event_handler(&rps_timer->evt);

	return IRQ_HANDLED;
}

struct rps_clock_evt_device rps_clk_evt = {
		.evt = {
				.name		= "rps_event_timer",
				.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
				.rating		= 200,
				.set_next_event	= rps_set_next_event,
				.set_mode	= rps_set_mode,
		},
};

static struct irqaction rps_clock_event_irq = {
	.name		= "rps_time_irq",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= rps_clock_event_isr,
	.dev_id		= &rps_clk_evt,
};

static void __init rps_clockevent_init(void __iomem *base, ulong ref_rate, int irq)
{
	ulong clock_rate;
	rps_clk_evt.base = base;

	/* use prescale 16 */
	clock_rate = ref_rate / 16;
	iowrite32(TIMER_DIV16, base + TIMER_CTRL);

	rps_clk_evt.evt.cpumask = cpumask_of(0);

	clockevents_config_and_register(&rps_clk_evt.evt,
			clock_rate, 1, TIMER_MAX_VAL);

	setup_irq(irq, &rps_clock_event_irq);
}

static void __init rps_clocksource_init(void __iomem *base, ulong ref_rate)
{
	int ret;
	ulong clock_rate;
	/* use prescale 16 */
	clock_rate = ref_rate / 16;

	iowrite32(TIMER_MAX_VAL, base + TIMER_LOAD);
	iowrite32(TIMER_PERIODIC | TIMER_ENABLE | TIMER_DIV16, base + TIMER_CTRL);

	ret = clocksource_mmio_init(base + TIMER_CURR, "rps_clocksource_timer",
					clock_rate, 250, TIMER_BITS,
					clocksource_mmio_readl_down);
	if (ret)
		panic("can't register clocksource\n");
}

static void __init rps_timer_init(struct device_node *np)
{
	struct clk *refclk;
	unsigned long ref_rate;
	int irq;
	void __iomem *base;

	refclk = of_clk_get(np, 0);

	if(IS_ERR(refclk) || clk_prepare_enable(refclk))
		panic("rps_timer_init: failed to get refclk\n");
	ref_rate = clk_get_rate(refclk);

	irq = irq_of_parse_and_map(np, 0);
	if(!irq)
		panic("rps_timer_init: failed to map irq\n");

	base = of_iomap(np, 0);
	if(!base)
		panic("rps_timer_init: failed to map io\n");

	rps_clockevent_init(base + TIMER1_OFFSET, ref_rate, irq);
	rps_clocksource_init(base + TIMER2_OFFSET, ref_rate);
}

CLOCKSOURCE_OF_DECLARE(nas782x, "plxtech,nas782x-rps-timer", rps_timer_init);
