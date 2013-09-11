/*
 * Copyright (C) 2010 Broadcom
 * Copyright (C) 2012 Stephen Warren
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

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of.h>

static DEFINE_SPINLOCK(lock);

static struct clk *clk_table[] = {

};
static void __iomem *reg_base;

#ifdef CONFIG_OF
static struct clk_onecell_data clk_data;
#endif


void __init oxnas_init_clocks(struct device_node *np)
{
/*
	struct clk *clk;
	int ret;

	clk = clk_register_fixed_rate(NULL, "uart0_pclk", NULL, CLK_IS_ROOT,
					3000000);
	if (IS_ERR(clk))
		pr_err("uart0_pclk not registered\n");
	ret = clk_register_clkdev(clk, NULL, "20201000.uart");
	if (ret)
		pr_err("uart0_pclk alias not registered\n");


#ifdef CONFIG_OF
	clk_data.clks = clk_table;
	clk_data.clk_num = nr_clks;
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
#endif
*/

}

CLK_OF_DECLARE(oxnas_clk, "plxtech,nas7820-clock", oxnas_init_clocks);
