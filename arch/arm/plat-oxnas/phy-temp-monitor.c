/*
 *  arch/arm/mach-oxnas/oxnas-ahb-ahb_monitor.c
 *
 *  Copyright (C) 2006 Oxford Semiconductor Ltd
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
 *
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/smp_lock.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/capability.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/vmalloc.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <mach/hardware.h>


/* usb test masks and offsets */
#define TEST_MASK    0xF
#define TEST_OFFSET  16

#define MODULE_VERS "0.2"
#define MODULE_NAME "oxnas-test"
MODULE_AUTHOR(		"John Larkworthy"					);
MODULE_DESCRIPTION(	"Driver to report temperature parameters for PCI and SATA phys."	);
MODULE_LICENSE(		"GPL"							);

#define SATA_PHY_ASIC_STAT (SATA_PHY_BASE + 0)
#define SATA_PHY_ASIC_DATA (SATA_PHY_BASE + 4)

#define PCI_PHY_ASIC_STAT (PCIE_PHY + 0)
#define PCI_PHY_ASIC_DATA (PCIE_PHY + 4)

/* PHY registers for temperature measurement. */
#define MPLL_TST  0x0017
#define RTUNE_CTL 0x0009
#define DAC_CTL   0x0008
#define ADC_OUT   0x000A


#define MEAS_IV_SHIFT    2
#define MEAS_IV_MASK    0x7FF
#define ADC_MODE_SHIFT       0
#define ADC_MODE_MASK       0x3
#define DAC_MODE_SHIFT   12
#define DAC_MODE_MASK    0x7
#define SEL_ATBP_SHIFT   4
#define SEL_ATBP_MASK    1
#define VALUE_SHIFT      0
#define VALUE_MASK       0x3FF 

/* scaling shift to allow precision calculation using scaled integers */
#define ARITH_SCALING  12


enum {
SATA_TYPE = 0,
PCI_TYPE =1 };

static struct dentry *phy_temp_dir, *sata_temp_read, *pci_temp_read;

static u32 phy_data, phy_stat;

static u32 readings[174];

/*********sata access routines ********************/
#define CR_READ_ENABLE  (1<<16)
#define CR_WRITE_ENABLE (1<<17)
#define CR_CAP_DATA     (1<<18)

static void wait_cr_ack(void){
	while ((readl(phy_stat) >> 16) & 0x1f)
		/* wait for an ack bit to be set */ ;
}

static u16 read_cr(u16 address) {
	writel(address, phy_stat);
	wait_cr_ack();
	writel(CR_READ_ENABLE, phy_data);
	wait_cr_ack();
	return readl(phy_stat);
}

static void write_cr(u16 data, u16 address) {
	writel(address, phy_stat);
	wait_cr_ack();
	writel((data | CR_CAP_DATA), phy_data);
	wait_cr_ack();
	writel(CR_WRITE_ENABLE, phy_data);
	wait_cr_ack();
	return ;
}


struct debug_buffer {
	ssize_t (*fill_func)(struct debug_buffer *);	/* fill method */
	u32  type;
	struct mutex mutex;	/* protect filling of buffer */
	size_t count;		/* number of characters filled into buffer */
	char *output_buf;
	size_t alloc_size;
};
static u16 read_adc(void) {
	u32 temp;
	do {
		temp  = read_cr(ADC_OUT);
	} while (0 == (temp & (1<<10)));
	
	return (temp >> VALUE_SHIFT) & VALUE_MASK;
}

static ssize_t fill_temp_func(struct debug_buffer *buf)
{
	u32 st1,st2,st3,st4;

	u32 m1, m2;
	s32 a;
	s32 theta;

	unsigned		temp, size, i;
	char			*next;

	phy_stat = buf->type == SATA_TYPE ? SATA_PHY_ASIC_STAT : PCI_PHY_ASIC_STAT;
	phy_data = buf->type == SATA_TYPE ? SATA_PHY_ASIC_DATA : PCI_PHY_ASIC_DATA;

	next = buf->output_buf;
	size = buf->alloc_size;

	st1 = (read_cr(MPLL_TST) >> MEAS_IV_SHIFT) & MEAS_IV_MASK;
	st2 = (read_cr(RTUNE_CTL) >> ADC_MODE_SHIFT) & ADC_MODE_MASK;
	st3 = (read_cr(DAC_CTL) >> DAC_MODE_SHIFT) & DAC_MODE_MASK;
	st4 = (read_cr(RTUNE_CTL) >> SEL_ATBP_SHIFT) & SEL_ATBP_MASK;
	
	write_cr ((read_cr(MPLL_TST)  & ~(MEAS_IV_MASK << MEAS_IV_SHIFT)) |  (512 << MEAS_IV_SHIFT), MPLL_TST);
	write_cr ((read_cr(RTUNE_CTL)  & ~(ADC_MODE_MASK << ADC_MODE_SHIFT)) | (1 << ADC_MODE_SHIFT),  RTUNE_CTL);
	write_cr ((read_cr(DAC_CTL) & ~(DAC_MODE_MASK << DAC_MODE_SHIFT)) | (4<<DAC_MODE_SHIFT),DAC_CTL);
	write_cr ((read_cr(RTUNE_CTL) & ~(SEL_ATBP_MASK<< SEL_ATBP_SHIFT)) | (0 <<SEL_ATBP_SHIFT),RTUNE_CTL);
	
//	printk(KERN_INFO "register dump:MPLL_TST:%#x RTUNE_CTL:%#x DAC_CTL:%x\n", read_cr(MPLL_TST),read_cr(RTUNE_CTL),read_cr(DAC_CTL));

	/* read m1 from average of last 80 readings */
	m1 = read_adc();
	readings[0]=m1;
	m1 = read_adc();
	readings[1]=m1;
	m1 = read_adc();
	readings[2]=m1;
	for(i=0;i <80; i++){
		readings[3+i]=read_adc();
		m1 += readings[3+i];
	}

	write_cr ((read_cr(RTUNE_CTL) & ~(SEL_ATBP_MASK<< SEL_ATBP_SHIFT)) | (1 <<SEL_ATBP_SHIFT),RTUNE_CTL);
	/* read m2 from average of last 80 readings */
	m2 = read_adc();
	readings[83]=m2;
	m2 = read_adc();
	readings[84]=m2;
	m2 = read_adc();
	for(i=0;i <80; i ++){
		readings[85+i]=read_adc();
		m2 += readings[85+i];
	}

	/* restore parameters to starting conditions */
	write_cr ((read_cr(MPLL_TST)  & ~(MEAS_IV_MASK << MEAS_IV_SHIFT)) | (st1 << MEAS_IV_SHIFT), MPLL_TST);
	write_cr ((read_cr(RTUNE_CTL)  & ~(ADC_MODE_MASK <<ADC_MODE_SHIFT)) | (st2 << ADC_MODE_SHIFT),  RTUNE_CTL);
	write_cr ((read_cr(DAC_CTL) & ~(DAC_MODE_MASK << DAC_MODE_SHIFT)) | (st3<<DAC_MODE_SHIFT),DAC_CTL);
	write_cr ((read_cr(RTUNE_CTL) & ~(SEL_ATBP_MASK<< SEL_ATBP_SHIFT)) | (st4 <<SEL_ATBP_SHIFT),RTUNE_CTL);

	if (m2) {
		/* calculate temperature from returned values */
		a = (m2-m1);
		a <<=ARITH_SCALING;
		a /=m2;
	
		/* use a value with polynomial to get temperature. */
		theta=-90514;
		theta *= a;
		theta /= 1<<ARITH_SCALING;
		theta += 777061;
		theta *=a;
		theta /= 1<<ARITH_SCALING;
		theta += -286410;
	
		/* print data */
		temp = scnprintf (next, size, "m1:%d m2:%d a=%d temperature %d.%01d\n", m1,m2, a, theta/1000, (theta%1000)/100);
	} else {
	temp = scnprintf (next,size, "no result m2 = 0\n");
	}
	size -= temp;
	next += temp;
//	for (i=0; i < 164; i++){
//		temp = scnprintf (next, size, "reading[%d]:%d\n", i, readings[i]);
//		size -= temp;
//		next += temp;
//	}

	return buf->alloc_size - size;
}

static int fill_buffer(struct debug_buffer *buf)
{
	int ret = 0;

	if (!buf->output_buf)
		buf->output_buf = (char *)vmalloc(buf->alloc_size);

	if (!buf->output_buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = buf->fill_func(buf);

	if (ret >= 0) {
		buf->count = ret;
		ret = 0;
	}

out:
	return ret;
}

static ssize_t  temperature_read (struct file *file, char __user *user_buf,
			    size_t len, loff_t *offset)
{
	struct debug_buffer *buf = file->private_data;
	int ret = 0;

	mutex_lock(&buf->mutex);
	if (buf->count == 0) {
		ret = fill_buffer(buf);
		if (ret != 0) {
			mutex_unlock(&buf->mutex);
			goto out;
		}
	}

	ret = simple_read_from_buffer(user_buf, len, offset,
				      buf->output_buf, buf->count);
	mutex_unlock(&buf->mutex);

out:
	return ret;

}


static struct debug_buffer *alloc_buffer(u32 data, ssize_t (*fill_func)(struct debug_buffer *))
{
	struct debug_buffer *buf;

	buf = kzalloc(sizeof(struct debug_buffer), GFP_KERNEL);

	if (buf) {
		buf->type = data;
		buf->fill_func = fill_temp_func;
		mutex_init(&buf->mutex);
		buf->alloc_size = PAGE_SIZE;
	}

	return buf;
}

static int temperature_open (struct inode *inode, struct file *file)
{
	file->private_data = alloc_buffer((u32)inode->i_private,
					  fill_buffer);

	return file->private_data ? 0 : -ENOMEM;
}

static int temperature_close(struct inode *inode, struct file *file)
{
	struct debug_buffer *buf = file->private_data;

	if (buf) {
		if (buf->output_buf)
			vfree(buf->output_buf);
		kfree(buf);
	}

	return 0;
}


static struct file_operations temperature_fops = {
	.owner		= THIS_MODULE,
	.open = temperature_open,
	.read = temperature_read,
	.release	= temperature_close,
};


static int __init oxnas_temperature_init(void)
{
	int rv=0;
	
	phy_temp_dir = debugfs_create_dir(MODULE_NAME, NULL);
	if (phy_temp_dir == NULL) {
		printk(KERN_ERR "%s: unable to register debugfs directory\n", MODULE_NAME);
		rv= -ENOMEM;
		goto out;
	}
	
	pci_temp_read = debugfs_create_file("pci", 0444, phy_temp_dir, (void *)PCI_TYPE, &temperature_fops);
	if (pci_temp_read == NULL) {
		printk(KERN_ERR "%s: unable to register pci\n", MODULE_NAME);
		rv = -ENOMEM;
		goto no_pci_read;
	}

	sata_temp_read = debugfs_create_file("sata", 0444, phy_temp_dir, SATA_TYPE, &temperature_fops);
	if (sata_temp_read == NULL) {
		printk(KERN_ERR "%s: unable to register sata\n", MODULE_NAME);
		rv = -ENOMEM;
		goto no_sata_read;
	}



	printk(KERN_INFO "%s %s initialised\n", MODULE_NAME, MODULE_VERS);

	return 0;

	no_sata_read:
		debugfs_remove(pci_temp_read);
	no_pci_read:
		debugfs_remove(phy_temp_dir);
	out:
		return rv;
}


static void __exit oxnas_temperature_exit(void)
{
	
	debugfs_remove(sata_temp_read);
	debugfs_remove(pci_temp_read);
	debugfs_remove(phy_temp_dir);

	printk(KERN_INFO "%s %s removed\n", MODULE_NAME, MODULE_VERS);
	
}


module_init(oxnas_temperature_init);
module_exit(oxnas_temperature_exit);
