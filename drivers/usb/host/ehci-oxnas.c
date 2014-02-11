/*
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 2005 John Larkworthy <john.larkworthy@oxsemi.com>
 * Copyright 2009 Oxford Semiconductor Ltd.
 * 
 * OXNAS Bus Glue
 *
 * Written by John Larkworthy 
 *
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>
#include <mach/hardware.h>

extern spinlock_t oxnas_gpio_spinlock;

/* called during probe() after chip reset completes */
static int ehci_oxnas_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int temp;
	int retval;

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset\n");
	dbg_hcc_params(ehci, "reset\n");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
			return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	if (ehci_is_TDI(ehci))
		ehci_reset(ehci);

	/* at least the Genesys GL880S needs fixup here */
	temp = HCS_N_CC(ehci->hcs_params) * HCS_N_PCC(ehci->hcs_params);
	temp &= 0x0f;
	if (temp && HCS_N_PORTS(ehci->hcs_params) > temp) {
		ehci_dbg(ehci, "bogus port configuration: "
			"cc=%d x pcc=%d < ports=%d\n",
			HCS_N_CC(ehci->hcs_params),
			HCS_N_PCC(ehci->hcs_params),
			HCS_N_PORTS(ehci->hcs_params));
	}

	ehci_port_power(ehci, 0);

	return retval;
}

static const struct hc_driver ehci_oxnas_driver = {
	.description =		hcd_name,
	.product_desc =		"OXNAS EHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =   ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset =   ehci_oxnas_setup,
	.start =   ehci_run,
#ifdef	CONFIG_PM
	.suspend = ehci_suspend,
	.resume =  ehci_resume,
#endif
	.stop =	   ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_hub_control,
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
};

extern void start_usb_clocks(void);
extern void stop_usb_clocks(void);

static int start_oxnas_usb_ehci(struct platform_device *dev)
{
    unsigned long input_polarity = 0;
    unsigned long output_polarity = 0;
	unsigned long power_polarity_default=readl(SYS_CTRL_USBHSMPH_CTRL);
	unsigned usb_hs_ifg;
	u32 reg;

	if (usb_disabled())
		return -ENODEV;

	pr_debug(KERN_INFO "starting usb for 820\n");
	pr_debug("%s: block sizes: qh %Zd qtd %Zd itd %Zd sitd %Zd\n",
		hcd_name,
		sizeof (struct ehci_qh), sizeof (struct ehci_qtd),
		sizeof (struct ehci_itd), sizeof (struct ehci_sitd));

	pr_debug(KERN_INFO "initialise for OX820 series USB\n");
#ifdef CONFIG_OXNAS_USB_OVERCURRENT_POLARITY_NEGATIVE
    input_polarity = ((1UL << SYS_CTRL_USBHSMPH_IP_POL_A_BIT) |
                      (1UL << SYS_CTRL_USBHSMPH_IP_POL_B_BIT));
#endif // CONFIG_OXNAS_USB_OVERCURRENT_POLARITY_NEGATIVE

#ifdef CONFIG_OXNAS_USB_POWER_SWITCH_POLARITY_NEGATIVE
    output_polarity = ((1UL << SYS_CTRL_USBHSMPH_OP_POL_A_BIT) |
                       (1UL << SYS_CTRL_USBHSMPH_OP_POL_B_BIT));
#endif // CONFIG_OXNAS_USB_POWER_SWITCH_POLARITY_NEGATIVE

	power_polarity_default &= ~0xf;
	usb_hs_ifg = (power_polarity_default >> 25) & 0x3f;
	usb_hs_ifg += 12;
	power_polarity_default &= ~(0x3f << 25);
	power_polarity_default |= (usb_hs_ifg << 25);
	power_polarity_default |= (input_polarity & 0x3);
	power_polarity_default |= (output_polarity & ( 0x3 <<2));

	writel(power_polarity_default, SYS_CTRL_USBHSMPH_CTRL);
	pr_debug(KERN_INFO "usb hsmph ctrl set to:%#lx\n", power_polarity_default);

#ifdef CONFIG_OXNAS_USB_PORTA_POWER_CONTROL

#ifdef CONFIG_USB_PORTA_POWO_SECONDARY
	// Select USBA power output from secondary MFP function
	mask = 1UL << USBA_POWO_SEC_MFP;
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_SECONDARY_SEL)   |  mask, SYS_CTRL_SECONDARY_SEL);
    writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~mask, SYS_CTRL_TERTIARY_SEL);
    writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~mask, SYS_CTRL_QUATERNARY_SEL);
    writel(readl(SYS_CTRL_DEBUG_SEL)       & ~mask, SYS_CTRL_DEBUG_SEL);
    writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~mask, SYS_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable output onto USBA power output secondary function
    writel(mask, GPIO_A_OUTPUT_ENABLE_SET);
#endif // CONFIG_USB_PORTA_POWO_SECONDARY

#ifdef CONFIG_USB_PORTA_POWO_TERTIARY
	// Select USBA power output from tertiary MFP function
	mask = 1UL << (USBA_POWO_TER_MFP - SYS_CTRL_NUM_PINS);
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SEC_CTRL_SECONDARY_SEL)   & ~mask, SEC_CTRL_SECONDARY_SEL);
    writel(readl(SEC_CTRL_TERTIARY_SEL)    |  mask, SEC_CTRL_TERTIARY_SEL);
    writel(readl(SEC_CTRL_QUATERNARY_SEL)  & ~mask, SEC_CTRL_QUATERNARY_SEL);
    writel(readl(SEC_CTRL_DEBUG_SEL)       & ~mask, SEC_CTRL_DEBUG_SEL);
    writel(readl(SEC_CTRL_ALTERNATIVE_SEL) & ~mask, SEC_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable output onto USBA power output tertiary function
    writel(mask, GPIO_B_OUTPUT_ENABLE_SET);
#endif // CONFIG_USB_PORTA_POWO_TERTIARY

#ifdef CONFIG_USB_PORTA_OVERI_SECONDARY
	// Select USBA overcurrent from secondary MFP function
	mask = 1UL << USBA_OVERI_SEC_MFP;
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_SECONDARY_SEL)   |  mask, SYS_CTRL_SECONDARY_SEL);
    writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~mask, SYS_CTRL_TERTIARY_SEL);
    writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~mask, SYS_CTRL_QUATERNARY_SEL);
    writel(readl(SYS_CTRL_DEBUG_SEL)       & ~mask, SYS_CTRL_DEBUG_SEL);
    writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~mask, SYS_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable input from USBA secondary overcurrent function
    writel(mask, GPIO_A_OUTPUT_ENABLE_CLEAR);
#endif // CONFIG_USB_PORTA_OVERI_SECONDARY

#ifdef CONFIG_USB_PORTA_OVERI_TERTIARY
	// Select USBA overcurrent from tertiary MFP function
	mask = 1UL << (USBA_OVERI_TER_MFP - SYS_CTRL_NUM_PINS);
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SEC_CTRL_SECONDARY_SEL)   & ~mask, SEC_CTRL_SECONDARY_SEL);
    writel(readl(SEC_CTRL_TERTIARY_SEL)    |  mask, SEC_CTRL_TERTIARY_SEL);
    writel(readl(SEC_CTRL_QUATERNARY_SEL)  & ~mask, SEC_CTRL_QUATERNARY_SEL);
    writel(readl(SEC_CTRL_DEBUG_SEL)       & ~mask, SEC_CTRL_DEBUG_SEL);
    writel(readl(SEC_CTRL_ALTERNATIVE_SEL) & ~mask, SEC_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable input from USBA tertiary overcurrent function
    writel(mask, GPIO_B_OUTPUT_ENABLE_CLEAR);
#endif // CONFIG_USB_PORTA_OVERI_TERTIARY

#endif // CONFIG_OXNAS_USB_PORTA_POWER_CONTROL

#ifdef CONFIG_OXNAS_USB_PORTB_POWER_CONTROL

#ifdef CONFIG_USB_PORTB_POWO_SECONDARY
	// Select USBB power output from secondary MFP function
	mask = 1UL << USBB_POWO_SEC_MFP;
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_SECONDARY_SEL)   |  mask, SYS_CTRL_SECONDARY_SEL);
    writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~mask, SYS_CTRL_TERTIARY_SEL);
    writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~mask, SYS_CTRL_QUATERNARY_SEL);
    writel(readl(SYS_CTRL_DEBUG_SEL)       & ~mask, SYS_CTRL_DEBUG_SEL);
    writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~mask, SYS_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable output onto USBB power output secondary function
    writel(mask, GPIO_A_OUTPUT_ENABLE_SET);
#endif // CONFIG_USB_PORTB_POWO_SECONDARY

#ifdef CONFIG_USB_PORTB_POWO_TERTIARY
	// Select USBB power output from tertiary MFP function
	mask = 1UL << USBB_POWO_TER_MFP;
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_SECONDARY_SEL)   & ~mask, SYS_CTRL_SECONDARY_SEL);
    writel(readl(SYS_CTRL_TERTIARY_SEL)    |  mask, SYS_CTRL_TERTIARY_SEL);
    writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~mask, SYS_CTRL_QUATERNARY_SEL);
    writel(readl(SYS_CTRL_DEBUG_SEL)       & ~mask, SYS_CTRL_DEBUG_SEL);
    writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~mask, SYS_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable output onto USBB power output tertiary function
    writel(mask, GPIO_A_OUTPUT_ENABLE_SET);
#endif // CONFIG_USB_PORTB_POWO_TERTIARY

#ifdef CONFIG_USB_PORTB_OVERI_SECONDARY
	// Select USBB overcurrent from secondary MFP function
	mask = 1UL << USBB_OVERI_SEC_MFP;
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_SECONDARY_SEL)   |  mask, SYS_CTRL_SECONDARY_SEL);
    writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~mask, SYS_CTRL_TERTIARY_SEL);
    writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~mask, SYS_CTRL_QUATERNARY_SEL);
    writel(readl(SYS_CTRL_DEBUG_SEL)       & ~mask, SYS_CTRL_DEBUG_SEL);
    writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~mask, SYS_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable input from USBB secondary overcurrent function
    writel(mask, GPIO_A_OUTPUT_ENABLE_CLEAR);
#endif // CONFIG_USB_PORTB_OVERI_SECONDARY

#ifdef CONFIG_USB_PORTB_OVERI_TERTIARY
	// Select USBB overcurrent from tertiary MFP function
	mask = 1UL << USBB_OVERI_TER_MFP;
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_SECONDARY_SEL)   & ~mask, SYS_CTRL_SECONDARY_SEL);
    writel(readl(SYS_CTRL_TERTIARY_SEL)    |  mask, SYS_CTRL_TERTIARY_SEL);
    writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~mask, SYS_CTRL_QUATERNARY_SEL);
    writel(readl(SYS_CTRL_DEBUG_SEL)       & ~mask, SYS_CTRL_DEBUG_SEL);
    writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~mask, SYS_CTRL_ALTERNATIVE_SEL);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable input from USBB tertiary overcurrent function
    writel(mask, GPIO_A_OUTPUT_ENABLE_CLEAR);
#endif // CONFIG_USB_PORTB_OVERI_TERTIARY

#endif // CONFIG_OXNAS_USB_PORTB_POWER_CONTROL

	// USB host and device must coordinate clock control
	start_usb_clocks();

	// Configure USB PHYA as a host
	reg = readl(SYS_CTRL_USB_CTRL);
	reg &= ~(1UL << SYS_CTRL_USB_CTRL_USBAMUX_DEVICE_BIT);
	writel(reg, SYS_CTRL_USB_CTRL); 

    // Enable the clock to the USB block
    writel(1UL << SYS_CTRL_CKEN_USBHS_BIT, SYS_CTRL_CKEN_SET_CTRL);
    wmb();

    pr_debug(KERN_INFO "registers:\n");
    pr_debug(KERN_INFO "SYS_CTRL_CKEN_CTRL 0x%08x: 0x%08x", SYS_CTRL_CKEN_CTRL, readl(SYS_CTRL_CKEN_SET_CTRL));
    pr_debug(KERN_INFO "SYS_CTRL_RSTEN_CTRL 0x%08x: 0x%08x", SYS_CTRL_RSTEN_CTRL, readl(SYS_CTRL_RSTEN_CLR_CTRL));
    pr_debug(KERN_INFO "SYS_CTRL_USBHSPHY_CTRL 0x%08x: 0x%08x", SYS_CTRL_USBHSPHY_CTRL, readl(SYS_CTRL_USBHSPHY_CTRL));
    pr_debug(KERN_INFO "SYS_CTRL_TERTIARY_SEL 0x%08x: 0x%08x", SYS_CTRL_TERTIARY_SEL, readl(SYS_CTRL_TERTIARY_SEL));
    pr_debug(KERN_INFO "SYS_CTRL_SECONDARY_SEL 0x%08x: 0x%08x", SYS_CTRL_SECONDARY_SEL, readl(SYS_CTRL_SECONDARY_SEL));
    pr_debug(KERN_INFO "SYS_CTRL_REF300_DIV 0x%08x: 0x%08x", SYS_CTRL_REF300_DIV, readl(SYS_CTRL_REF300_DIV));
    pr_debug(KERN_INFO "SYS_CTRL_USB_CTRL 0x%08x: 0x%08x", SYS_CTRL_USB_CTRL, readl(SYS_CTRL_USB_CTRL));

	return 0;
}

static void stop_oxnas_usb_ehci(struct platform_device *dev)
{
	// put usb core into reset
    writel(1UL << SYS_CTRL_RSTEN_USBHS_BIT, SYS_CTRL_RSTEN_SET_CTRL);

    // Disable the clock to the USB block
    writel(1UL << SYS_CTRL_CKEN_USBHS_BIT, SYS_CTRL_CKEN_CLR_CTRL);

	// USB host and device must coordinate clock control
	stop_usb_clocks();
}

/**
 * usb_hcd_oxnas_probe - initialize OXNAS-based HCD
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
static int usb_hcd_oxnas_probe(const struct hc_driver *driver, struct platform_device *dev)
{
	int retval;
	unsigned long ehci_id;
	struct usb_hcd *hcd = 0;
	struct ehci_hcd *ehci;

	if (dev->num_resources != 2) {
		pr_debug("wrong number of resources %d, expected %d", dev->num_resources, 2);
	}
	pr_debug(KERN_INFO "probe for usb\n");

	start_oxnas_usb_ehci(dev);

	if (((ehci_id = readl(USBHOST_BASE)) & 0x2f) != 0x05) {
		pr_debug("wrong chip ID found %lx", ehci_id);
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, &dev->dev, "usb");
	if (!hcd) {
		pr_debug("usb_create_hcd() failed");
		retval = -ENOMEM;
	}
	hcd->regs = (void *)(USBHOST_BASE + 0x100); /* adjust to point at cap length register */

	pr_debug(KERN_INFO "@%p Device ID register %lx\n", (void *)USBHOST_BASE, *(unsigned long *)USBHOST_BASE);

	/* OXNAS device has a transaction translator */
	hcd->has_tt = 1;
	ehci = hcd_to_ehci(hcd);
	ehci->is_tdi_rh_tt = 1;

	/* Finished initialisation and register */
	if ((retval = usb_add_hcd(hcd, dev->resource[1].start, 0))) {
		pr_debug("usb_add_hcd() failed");
		stop_oxnas_usb_ehci(dev);
		usb_put_hcd(hcd);
		return retval;
	}
	return 0;
}

/**
 * usb_hcd_oxnas_remove - shutdown processing for OXNAS-based HCD
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_oxnas_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_oxnas_remove(struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	stop_oxnas_usb_ehci(dev);
}

static int ehci_hcd_oxnas_drv_probe(struct platform_device *dev)
{
	if (usb_disabled())
		return -ENODEV;

	return usb_hcd_oxnas_probe(&ehci_oxnas_driver, dev);
}

static int ehci_hcd_oxnas_drv_remove(struct platform_device *dev)
{
	usb_hcd_oxnas_remove(platform_get_drvdata(dev), dev);
	return 0;
}

MODULE_ALIAS("oxnas-ehci");

static struct platform_driver ehci_hcd_oxnas_driver = {
	.probe = ehci_hcd_oxnas_drv_probe,
	.remove = ehci_hcd_oxnas_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "oxnas-ehci",
	},
};
