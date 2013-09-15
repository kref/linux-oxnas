/*
 * drivers/usb/host/ehci-oxnas.c
 *
 * Tzachi Perelstein <tzachi@marvell.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <mach/clock.h>

#include "ehci.h"

#define DRIVER_DESC "Oxnas On-Chip EHCI Host Controller"

static struct hc_driver __read_mostly oxnas_hc_driver;

static int start_oxnas_usb_ehci(struct platform_device *dev)
{
	u32 input_polarity = 0;
	u32 output_polarity = 0;
	u32 power_polarity_default = readl(SYS_CTRL_USBHSMPH_CTRL);
	u32 usb_hs_ifg;
	u32 reg;

	power_polarity_default = readl(SYS_CTRL_USBHSMPH_CTRL);

	power_polarity_default &= ~0xf;
	usb_hs_ifg = (power_polarity_default >> 25) & 0x3f;
	usb_hs_ifg += 12;
	power_polarity_default &= ~(0x3f << 25);
	power_polarity_default |= (usb_hs_ifg << 25);
	power_polarity_default |= (input_polarity & 0x3);
	power_polarity_default |= (output_polarity & ( 0x3 <<2));

	writel(power_polarity_default, SYS_CTRL_USBHSMPH_CTRL);

	// turn on internal 12MHz clock for OX820 architecture USB
	enable_clock(SYS_CTRL_CLK_REF600);
	block_reset(SYS_CTRL_RST_PLLB, 0);

#ifdef CONFIG_USB_OX820_FROM_PLLB
	writel((readl(SYS_CTRL_SECONDARY_SEL) | (1UL<<9)), SYS_CTRL_SECONDARY_SEL); /* enable monitor output  mf_a9 */
	writel( (1 << PLLB_ENSAT) | (1 << PLLB_OUTDIV) | (2<<PLLB_REFDIV), SEC_CTRL_PLLB_CTRL0); /*  */
	writel( (50 << USB_REF_600_DIVIDER), SEC_CTRL_PLLB_DIV_CTRL);  // 600MHz pllb divider for 12MHz
#endif
	writel(25 << USB_REF_300_DIVIDER, SYS_CTRL_REF300_DIV); // ref 300 divider for 12MHz

	//block_reset(SYS_CTRL_RST_USBDEV, 1);

	// Ensure the USB block is properly reset
	block_reset(SYS_CTRL_RST_USBHS, 1);
	wmb();
	block_reset(SYS_CTRL_RST_USBHS, 0);

	block_reset(SYS_CTRL_RST_USBHSPHYA, 1);
	wmb();
	block_reset(SYS_CTRL_RST_USBHSPHYA, 0);

	block_reset(SYS_CTRL_RST_USBHSPHYB, 1);
	wmb();
	block_reset(SYS_CTRL_RST_USBHSPHYB, 0);

	// Force the high speed clock to be generated all the time, via serial
	// programming of the USB HS PHY
	writel((2UL << USBHSPHY_TEST_ADD) |
		   (0xe0UL << USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

	writel((1UL << USBHSPHY_TEST_CLK) |
		   (2UL << USBHSPHY_TEST_ADD) |
		   (0xe0UL << USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

	writel((0xfUL << USBHSPHY_TEST_ADD) |
		   (0xaaUL << USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

	writel((1UL << USBHSPHY_TEST_CLK) |
		   (0xfUL << USBHSPHY_TEST_ADD) |
		   (0xaaUL << USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

	/* select the correct clock now out of reset */
#ifdef CONFIG_USB_OX820_FROM_PLLB
	writel(USB_CLK_INTERNAL | USB_INT_CLK_PLLB, SYS_CTRL_USB_CTRL); // use pllb clock
#else
	writel(USB_CLK_INTERNAL | USB_INT_CLK_REF300, SYS_CTRL_USB_CTRL); // use ref300 derived clock
#endif
/*
	// Configure USB PHYA as a host
	reg = readl(SYS_CTRL_USB_CTRL);
	reg &= ~USBAMUX_DEVICE;
	writel(reg, SYS_CTRL_USB_CTRL);
*/
	// Enable the clock to the USB block
	enable_clock(SYS_CTRL_CLK_USBHS);
	// Ensure reset and clock operations are complete
	wmb();

	return 0;
}

static void stop_oxnas_usb_ehci(void)
{
	block_reset(SYS_CTRL_RST_USBHS, 1);
	disable_clock(SYS_CTRL_CLK_USBHS);
}

static int ehci_oxnas_reset(struct usb_hcd *hcd)
{
	#define  txttfill_tuning	reserved2[0]

	struct ehci_hcd	*ehci;
	u32 tmp;
	int retval = ehci_setup(hcd);
	if (retval)
		return retval;

	ehci = hcd_to_ehci(hcd);
	tmp = ehci_readl(ehci, &ehci->regs->txfill_tuning);
	tmp &= ~0x00ff0000;
	tmp |= 0x003f0000; /* set burst pre load count to 0x40 (63 * 4 bytes)  */
	tmp |= 0x16; /* set sheduler overhead to 22 * 1.267us (HS) or 22 * 6.33us (FS/LS)*/
	ehci_writel(ehci, tmp,  &ehci->regs->txfill_tuning);

	tmp = ehci_readl(ehci, &ehci->regs->txttfill_tuning);
	tmp |= 0x2; /* set sheduler overhead to 2 * 6.333us */
	ehci_writel(ehci, tmp,  &ehci->regs->txttfill_tuning);

	return retval;
}

static int ehci_oxnas_drv_probe(struct platform_device *ofdev)
{
	struct device_node *np = ofdev->dev.of_node;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct resource res;
	int irq, err;

	if (usb_disabled())
		return -ENODEV;

	if (!ofdev->dev.dma_mask)
		ofdev->dev.dma_mask = &ofdev->dev.coherent_dma_mask;
	if (!ofdev->dev.coherent_dma_mask)
		ofdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	err = of_address_to_resource(np, 0, &res);
	if (err)
		return err;

	hcd = usb_create_hcd(&oxnas_hc_driver,	&ofdev->dev,
					dev_name(&ofdev->dev));
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = res.start;
	hcd->rsrc_len = resource_size(&res);

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		dev_err(&ofdev->dev, "irq_of_parse_and_map failed\n");
		err = -EBUSY;
		goto err_irq;
	}

	hcd->regs = devm_ioremap_resource(&ofdev->dev, &res);
	if (IS_ERR(hcd->regs)) {
		dev_err(&ofdev->dev, "devm_ioremap_resource failed\n");
		err = PTR_ERR(hcd->regs);
		goto err_irq;
	}

	hcd->has_tt = 1;
	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;

	start_oxnas_usb_ehci(ofdev);

	err = usb_add_hcd(hcd, irq, IRQF_SHARED | IRQF_DISABLED);
	if (err)
		goto err_add_hcd;

	return 0;

err_add_hcd:
	stop_oxnas_usb_ehci();
err_irq:
	usb_put_hcd(hcd);
	return err;
}

static int ehci_oxnas_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	return 0;
}

static const struct of_device_id oxnas_ehci_dt_ids[] = {
	{ .compatible = "plxtch,nas782x-ehci" },
	{ /* sentinel */ }
};

static struct platform_driver ehci_oxnas_driver = {
	.probe		= ehci_oxnas_drv_probe,
	.remove		= ehci_oxnas_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver.name	= "oxnas-ehci",
	.driver.of_match_table	= oxnas_ehci_dt_ids,
};

static const struct ehci_driver_overrides oxnas_overrides __initdata = {
	.reset = ehci_oxnas_reset,
};

static int __init ehci_oxnas_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ehci_init_driver(&oxnas_hc_driver, &oxnas_overrides);
	return platform_driver_register(&ehci_oxnas_driver);
}
module_init(ehci_oxnas_init);

static void __exit ehci_oxnas_cleanup(void)
{
	platform_driver_unregister(&ehci_oxnas_driver);
}
module_exit(ehci_oxnas_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:oxnas-ehci");
MODULE_LICENSE("GPL");
