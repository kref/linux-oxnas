/*
 * arch/arm/mach-0x820/include/mach/hardware.h
 *
 * Copyright (C) 2009 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/io.h>
#include <mach/iomap.h>

/*
 * Location of flags and vectors in SRAM for controlling the booting of the
 * secondary ARM11 processors.
 */

#define OXNAS_SCU_BASE_VA			OXNAS_PERCPU_BASE_VA
#define OXNAS_GICN_BASE_VA(n)		(OXNAS_PERCPU_BASE_VA + 0x200 + n * 0x100)


#define HOLDINGPEN_CPU          IOMEM(OXNAS_SYSCRTL_BASE_VA + 0xc8)
#define HOLDINGPEN_LOCATION     IOMEM(OXNAS_SYSCRTL_BASE_VA + 0xc4)


/**
 * System block reset and clock control
 */
#define SYS_CTRL_PCI_STAT             IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x20)
#define SYS_CTRL_CLK_SET_CTRL        IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x2C)
#define SYS_CTRL_CLK_CLR_CTRL        IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x30)
#define SYS_CTRL_RST_SET_CTRL       IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x34)
#define SYS_CTRL_RST_CLR_CTRL       IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x38)

#define SYS_CTRL_PLLSYS_CTRL          IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x48)
#define SYS_CTRL_PLLSYS_KEY_CTRL      IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x6C)
#define SYS_CTRL_GMAC_CTRL            IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x78)


/* Scratch registers */
#define SYS_CTRL_SCRATCHWORD0         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0xc4)
#define SYS_CTRL_SCRATCHWORD1         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0xc8)
#define SYS_CTRL_SCRATCHWORD2         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0xcc)
#define SYS_CTRL_SCRATCHWORD3         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0xd0)

#define SYS_CTRL_PLLA_CTRL0         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x1F0)
#define SYS_CTRL_PLLA_CTRL1         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x1F4)
#define SYS_CTRL_PLLA_CTRL2         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x1F8)
#define SYS_CTRL_PLLA_CTRL3         IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x1FC)

#define SYS_CTRL_USBHSMPH_CTRL          IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x40)
#define SYS_CTRL_USBHSMPH_STAT          IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x44)
#define SYS_CTRL_REF300_DIV             IOMEM(OXNAS_SYSCRTL_BASE_VA + 0xF8)
#define SYS_CTRL_USBHSPHY_CTRL          IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x84)
#define SYS_CTRL_USB_CTRL               IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x90)

#define USB_REF_300_DIVIDER     8
#define USBHSPHY_SUSPENDM_MANUAL_ENABLE    16
#define USBHSPHY_SUSPENDM_MANUAL_STATE     15
#define USBHSPHY_ATE_ESET                  14
#define USBHSPHY_TEST_DIN                   6
#define USBHSPHY_TEST_ADD                   2
#define USBHSPHY_TEST_DOUT_SEL              1
#define USBHSPHY_TEST_CLK                   0

#define USB_CTRL_USBAPHY_CKSEL_SHIFT	5
#define USB_CLK_XTAL0_XTAL1		(0 << USB_CTRL_USBAPHY_CKSEL_SHIFT)
#define USB_CLK_XTAL0			(1 << USB_CTRL_USBAPHY_CKSEL_SHIFT)
#define USB_CLK_INTERNAL		(2 << USB_CTRL_USBAPHY_CKSEL_SHIFT)

#define USBAMUX_DEVICE			BIT(4)

#define USBPHY_REFCLKDIV_SHIFT	2
#define USB_PHY_REF_12MHZ	(0 << USBPHY_REFCLKDIV_SHIFT)
#define USB_PHY_REF_24MHZ	(1 << USBPHY_REFCLKDIV_SHIFT)
#define USB_PHY_REF_48MHZ	(2 << USBPHY_REFCLKDIV_SHIFT)

#define USB_CTRL_USB_CKO_SEL_BIT	0

#define USB_INT_CLK_XTAL 	0
#define USB_INT_CLK_REF300	2
#define USB_INT_CLK_PLLB	3

#define SYS_CTRL_GMAC_AUTOSPEED     3
#define SYS_CTRL_GMAC_RGMII         2
#define SYS_CTRL_GMAC_SIMPLE_MUX    1
#define SYS_CTRL_GMAC_CKEN_GTX      0

#define SYS_CTRL_CKCTRL_CTRL_ADDR     IOMEM(OXNAS_SYSCRTL_BASE_VA + 0x64)

#define SYS_CTRL_CKCTRL_PCI_DIV_BIT 0
#define SYS_CTRL_CKCTRL_SLOW_BIT    8

#define SYS_CTRL_UART2_DEQ_EN       0
#define SYS_CTRL_UART3_DEQ_EN       1
#define SYS_CTRL_UART3_IQ_EN        2
#define SYS_CTRL_UART4_IQ_EN        3
#define SYS_CTRL_UART4_NOT_PCI_MODE 4

#define SYS_CTRL_PCI_CTRL1_PCI_STATIC_RQ_BIT 11


#endif
