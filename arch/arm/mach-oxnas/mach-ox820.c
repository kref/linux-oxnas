#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/of_platform.h>
#include <linux/clocksource.h>
#include <linux/clk-provider.h>
#include <linux/clk/oxnas.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/page.h>
#include <mach/iomap.h>

extern struct smp_operations ox820_smp_ops;

static struct map_desc ox820_io_desc[] __initdata = {
	{
		.virtual = (unsigned long)OXNAS_PERCPU_BASE_VA,
		.pfn = __phys_to_pfn(OXNAS_PERCPU_BASE),
		.length = OXNAS_PERCPU_SIZE,
		.type = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)OXNAS_SYSCRTL_BASE_VA,
		.pfn = __phys_to_pfn(OXNAS_SYSCRTL_BASE),
		.length = OXNAS_SYSCRTL_SIZE,
		.type = MT_DEVICE,
	},
};

void __init ox820_map_common_io(void)
{
	debug_ll_io_init();
	iotable_init(ox820_io_desc, ARRAY_SIZE(ox820_io_desc));
}


static void __init ox820_dt_init(void)
{
        int ret;

        oxnas_init_clocks();

        ret = of_platform_populate(NULL, of_default_bus_match_table, NULL,
                                   NULL);
        if (ret) {
                pr_err("of_platform_populate failed: %d\n", ret);
                BUG();
        }
}

static void __init ox820_timer_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
	return;
}

void ox820_init_early(void)
{

}

static const char * const ox820_dt_board_compat[] = {
	"plxtech,nas7820",
	"plxtech,nas7821",
	"plxtech,nas7825",
	NULL
};

DT_MACHINE_START(OX820_DT, "PLXTECH NAS782X SoC (Flattened Device Tree)")
	.map_io		= ox820_map_common_io,
	.smp		= smp_ops(ox820_smp_ops),
	.init_early	= ox820_init_early,
	//.init_irq	= ox820_dt_init_irq,
	.init_time	= ox820_timer_init,
	.init_machine	= ox820_dt_init,
	//.init_late	= ox820_dt_init_late,
	//.restart	= ox820_assert_system_reset,
	.dt_compat	= ox820_dt_board_compat,
MACHINE_END
