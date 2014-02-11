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
#include <linux/proc_fs.h>
#include <linux/capability.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <asm/hardware.h>


/* usb test masks and offsets */
#define TEST_MASK    0xF
#define TEST_OFFSET  16

#define MODULE_VERS "0.2"
#define MODULE_NAME "oxnas-test"
MODULE_AUTHOR(		"John Larkworthy"					);
MODULE_DESCRIPTION(	"Driver to access the test hardware in oxnas units"	);
MODULE_LICENSE(		"GPL"							);


static struct proc_dir_entry *proc_dir_usb_test_read, *oxnas_test_dir;


static struct {
	void * address;
	char * name;
	long unsigned low_add;
	long unsigned high_add;
	unsigned burst;
	unsigned burst_mask;
	unsigned hprot;
	unsigned hprot_mask;
	unsigned mode;
	unsigned hwrite;
	unsigned trigger;
} ahb_monitor[] =
{
	{ (void *) 0x00000, "ARM0_ETM", 0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0x10000, "ARM1_ETM", 0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0x60000, "CoPro",    0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0x70000, "GMAC1",    0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0x80000, "GMAC2",    0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0x90000, "SATA",     0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0xB0000, "DMA-SG",   0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0xC0000, "PCIEA",    0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0xD0000, "PCIEB",    0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0xE0000, "USBMPH",   0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
	{ (void *) 0xF0000, "USBDEV",   0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0 },
};		
static struct {
	void * address;
	char * name;
	long unsigned low_add;
	long unsigned high_add;
	unsigned burst;
	unsigned burst_mask;
	unsigned hprot;
	unsigned hprot_mask;
	unsigned mode;
	unsigned hwrite;
	unsigned trigger;
	unsigned reg_id;
	unsigned reg_id_mask;
	unsigned reg_len;
	unsigned reg_len_mask;
	unsigned reg_size;
	unsigned reg_size_mask;
	unsigned reg_cache;
	unsigned reg_cache_mask;
	unsigned reg_lock;
	unsigned reg_lock_mask;
	
} axi_monitor[] =
{
	{ (void *) 0x20000, "ARMRA", 0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ (void *) 0x30000, "ARMRB", 0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ (void *) 0x40000, "ARMWA", 0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ (void *) 0x50000, "ARMWB", 0, 0xFFFFFFFCl, 0, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

#define NO_AHB_MONITORS (sizeof(ahb_monitor)/sizeof(ahb_monitor[0]))
#define NO_AXI_MONITORS (sizeof(axi_monitor)/sizeof(axi_monitor[0]))

/* create proc filing system entries to accept configuration data */
static int usb_test_write_entries(const char *name, write_proc_t *w, read_proc_t *r, int data)
{
	struct proc_dir_entry * entry = create_proc_entry(name, 0222, oxnas_test_dir);
	if (entry) {
		entry->write_proc = w;
		entry->read_proc =r;
		entry->data = (void *)data;
		entry->owner = THIS_MODULE;
		return 0;
	}
	else
	{
		return -ENOMEM;
	}
}

static int
oxnas_test_read(char *buf, char **start, off_t offset,
		  int count, int *eof, void *unused)
{
	int i;
	int len = 0;
	long unsigned *rd_monitor;
	
	for (i=0; i < NO_AXI_MONITORS; i++)
	{
    	rd_monitor = (long unsigned *) (AHB_MON_BASE + axi_monitor[i].address);
    	
		len += sprintf(buf+len, "%-8s CLKS:%10lu TRANX:%10lu DATA:%10lu WAIT:%10lu\n", 
			axi_monitor[i].name, 
			rd_monitor[0], 
			rd_monitor[3],
			rd_monitor[1],
			rd_monitor[2]);
	}	

	for (i=0; i < 2; i++) // ETM Monitors
	{
    	rd_monitor = (long unsigned *) (AHB_MON_BASE + ahb_monitor[i].address);
	
	    if (*rd_monitor == 0xDEADDEAD) // Avoid CPU-1 in single CPU FPGA builds
	        continue;
	        
		len += sprintf(buf+len, "%-8s CLKS:%10lu HIT  :%10lu MISS:%10lu\n", 
			ahb_monitor[i].name, 
			rd_monitor[3], 
			rd_monitor[4],
			rd_monitor[5]);
	}

	for (i=2; i < NO_AHB_MONITORS; i++)
	{
    	rd_monitor = (long unsigned *) (AHB_MON_BASE + ahb_monitor[i].address);
	
		len += sprintf(buf+len, "%-8s CLKS:%10lu DATA :%10lu WAIT:%10lu\n", 
			ahb_monitor[i].name, 
			*rd_monitor, 
			*(rd_monitor+1),
			*(rd_monitor+2));
	}

	*eof=1;
	return len;
}
/*
 * function to clear and start all the timers mainly together.
 */
#define MAX_CMD 5
static int
oxnas_test_control(struct file *file, const char *buf, unsigned long count, void * data)
{
	int len;
	int i;
	long unsigned *rd_monitor;
	unsigned cmd = 0;
	
	char local[MAX_CMD];
	int result;

	if (count > MAX_CMD-1)
		len= MAX_CMD-1;
	else 
		len=count;
		
	if (copy_from_user(&local, buf, len))
		return -EFAULT;
	
	result=sscanf(local, "%d", &cmd);
	
	switch (cmd) 
	{
		case 0: 
			printk(KERN_INFO "oxnas-test: stop command\n");
			break;
		case 1:
			printk(KERN_INFO "oxnas-test: run command\n");
			break;
		case 2:
			printk(KERN_INFO "oxnas-test: reset command\n");
			break;
		default:
			printk(KERN_INFO "oxnas-test: ignored command\n");
			return len;
			break;
	}
	
	for (i=0; i < NO_AHB_MONITORS; i++)
	{
		rd_monitor = (long unsigned *) (AHB_MON_BASE + ahb_monitor[i].address);
		*rd_monitor = (long unsigned) cmd;
	}

	for (i=0; i < NO_AXI_MONITORS; i++)
	{
		rd_monitor = (long unsigned *) (AHB_MON_BASE + axi_monitor[i].address);
		*rd_monitor = (long unsigned) cmd;
	}

	return len;
}


/*
 * The write function accepts a line as below:
 * 
 * ETM Command line format (9 parameters - Only 3 used)
 * low,high,mode,ignored,ignored,ignored,ignored,ignored,ignored
 *
 * AHB Command line format (9 parameters)
 * low,high,mode,burst,burst_mask,hprot,hprot_mask,write,trigger
 *
 * AXI Command line format (18 parameters)
 * low,high,mode,burst,burst_mask,hprot,hprot_mask,trigger,id,id_mask,len,len_mask,size,size_mask,cache,cache_mask,lock,lock_mask
*/ 
 
#define MAX_STRING 128
static int
oxnas_test_config_write(struct file *file, const char *buf, unsigned long count, void * data)
{
	
	int len;	
	int i = (int) data;
	char local[MAX_STRING];
	int result;
	unsigned long * mon_ptr;

	if (count > MAX_STRING-1)
		len= MAX_STRING-1;
	else 
		len=count;
		
	if (copy_from_user(&local, buf, len))
		return -EFAULT;

	if(i < NO_AHB_MONITORS) {
		/* extract value from buffer and store */
		
		result = sscanf(local, "%li,%li,%i,%i,%i,%i,%i,%i,%i", 
			&ahb_monitor[i].low_add, 
			&ahb_monitor[i].high_add, 
			&ahb_monitor[i].mode, 
			&ahb_monitor[i].burst, 
			&ahb_monitor[i].burst_mask, 
			&ahb_monitor[i].hprot,
			&ahb_monitor[i].hprot_mask,
			&ahb_monitor[i].hwrite,
			&ahb_monitor[i].trigger
		);
		if (result != 9) {
			printk(KERN_WARNING "decode failed after %d parameters\n", result);
			return -EINVAL;
		}
			
		/* load values on hardware */
		
		mon_ptr=(unsigned long *) (AHB_MON_BASE + ahb_monitor[i].address);

		*(mon_ptr + 0) = ahb_monitor[i].mode & 0x3;   //00
		*(mon_ptr + 1) = ahb_monitor[i].hwrite & 0x3;   //04
		*(mon_ptr + 2) = ahb_monitor[i].low_add; 	  //08
		*(mon_ptr + 3) = ahb_monitor[i].high_add;     //0c
		*(mon_ptr + 4) = ((ahb_monitor[i].burst & 0x7) << 4 | (ahb_monitor[i].burst_mask & 0x7));  //10
		*(mon_ptr + 5) = ((ahb_monitor[i].hprot & 0xf) << 4 | (ahb_monitor[i].hprot_mask &0xf));  //14
		*(mon_ptr + 6) = ahb_monitor[i].trigger;  //18
	}
	else {
		i -= NO_AHB_MONITORS;
		/* extract value from buffer and store */
		
		result = sscanf(local, "%li,%li,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i", 
			&axi_monitor[i].low_add, 
			&axi_monitor[i].high_add, 
			&axi_monitor[i].mode, 
			&axi_monitor[i].burst, 
			&axi_monitor[i].burst_mask, 
			&axi_monitor[i].hprot,
			&axi_monitor[i].hprot_mask,
			&axi_monitor[i].trigger,
			&axi_monitor[i].reg_id,
			&axi_monitor[i].reg_id_mask,
			&axi_monitor[i].reg_len,
			&axi_monitor[i].reg_len_mask,
			&axi_monitor[i].reg_size,
			&axi_monitor[i].reg_size_mask,
			&axi_monitor[i].reg_cache,
			&axi_monitor[i].reg_cache_mask,
			&axi_monitor[i].reg_lock,
			&axi_monitor[i].reg_lock_mask
		);
		if (result != 18) {
			printk(KERN_WARNING "decode failed after %d parameters\n", result);
			return -EINVAL;
		}
			
		/* load values on hardware */
		
		mon_ptr=(unsigned long *) (AHB_MON_BASE + axi_monitor[i].address);

		mon_ptr[ 0] = axi_monitor[i].mode & 0x3; //0
		mon_ptr[ 2] = axi_monitor[i].low_add;    //8
		mon_ptr[ 3] = axi_monitor[i].high_add;   //C
		mon_ptr[ 4] = ((axi_monitor[i].burst & 0x7) << 4 | (axi_monitor[i].burst_mask & 0x7)); //10
		mon_ptr[ 5] = ((axi_monitor[i].hprot & 0xf) << 4 | (axi_monitor[i].hprot_mask &0xf));  //14
		mon_ptr[ 6] = axi_monitor[i].trigger;   //18
		mon_ptr[ 7] = ((axi_monitor[i].reg_id & 0xf) << 4 | (axi_monitor[i].reg_id_mask & 0xf)); //1C
		mon_ptr[ 8] = ((axi_monitor[i].reg_len & 0xf) << 4 | (axi_monitor[i].reg_len_mask & 0xf)); //20
		mon_ptr[ 9] = ((axi_monitor[i].reg_size & 0x7) << 4 | (axi_monitor[i].reg_size_mask & 0x7)); //24
		mon_ptr[10] = ((axi_monitor[i].reg_cache & 0xf) << 4 | (axi_monitor[i].reg_cache_mask & 0xf)); //28
		mon_ptr[11] = ((axi_monitor[i].reg_lock & 0x3) << 4 | (axi_monitor[i].reg_lock_mask & 0x3)); //2C
		
	}
	return len;
}

static int
oxnas_test_config_read(char *buf, char **start, off_t offset,
		  int count, int *eof, void *data)
{
	
	int len = 0;	
	int i = (int) data;
	if (i < NO_AHB_MONITORS) {
	
		len += sprintf(buf+len, "name     low        high       mode burst   hprot   trigger\n"); 
		
		len += sprintf(buf+len, "%-8s 0x%08lx 0x%08lx %4d 0x%x/0x%x 0x%x/0x%x 0x%x\n", 
			ahb_monitor[i].name, 
			ahb_monitor[i].low_add, 
			ahb_monitor[i].high_add, 
			ahb_monitor[i].mode, 
			ahb_monitor[i].burst, 
			ahb_monitor[i].burst_mask, 
			ahb_monitor[i].hprot,
			ahb_monitor[i].hprot_mask,
			ahb_monitor[i].trigger);
		
	}
	else {
		i -= NO_AHB_MONITORS;
		
		len += sprintf(buf+len, "name     low        high       mode burst   hprot   trigger id      len     size    cache   lock \n"); 

		len += sprintf(buf+len, "%-8s 0x%08lx 0x%08lx %4d 0x%x/0x%x 0x%x/0x%x 0x%x     0x%x/0x%x 0x%x/0x%x 0x%x/0x%x 0x%x/0x%x 0x%x/0x%x \n", 
			axi_monitor[i].name, 
			axi_monitor[i].low_add, 
			axi_monitor[i].high_add, 
			axi_monitor[i].mode, 
			axi_monitor[i].burst, 
			axi_monitor[i].burst_mask, 
			axi_monitor[i].hprot,
			axi_monitor[i].hprot_mask,
			axi_monitor[i].trigger,
			axi_monitor[i].reg_id,
			axi_monitor[i].reg_id_mask,
			axi_monitor[i].reg_len,
			axi_monitor[i].reg_len_mask,
			axi_monitor[i].reg_size,
			axi_monitor[i].reg_size_mask,
			axi_monitor[i].reg_cache,
			axi_monitor[i].reg_cache_mask,
			axi_monitor[i].reg_lock,
			axi_monitor[i].reg_lock_mask
			);
	}
		
	*eof=1;
	return len;
}

static int __init oxnas_test_init(void)
{
	int rv=0;
	int i,n;
	
	oxnas_test_dir = proc_mkdir(MODULE_NAME, NULL);
	if (oxnas_test_dir == NULL) {
		printk(KERN_ERR "oxnas-test: unable to register /proc/oxnas-test\n");
		rv= -ENOMEM;
		goto out;
	}
	
	oxnas_test_dir->owner= THIS_MODULE;
	
	proc_dir_usb_test_read = create_proc_entry("read", 0444, oxnas_test_dir);
	if (proc_dir_usb_test_read) {
		proc_dir_usb_test_read->read_proc = oxnas_test_read;
	} else {
		printk(KERN_ERR "oxnas-test: unable to register /proc/oxnas-test/read\n");
		rv = -ENOMEM;
		goto no_read;
	}

	/* create port write file entries */
	for (i=0;i<NO_AHB_MONITORS;i++) 
	{
		rv = usb_test_write_entries(ahb_monitor[i].name, &oxnas_test_config_write, &oxnas_test_config_read, i);
		if (rv < 0)
		{
			while (i != 0)
			{
				i--;
				/* remove any allocated entries */
				remove_proc_entry (ahb_monitor[i].name, oxnas_test_dir);
			} 
			goto no_write;
		}
	}
	for (n=0;n<NO_AXI_MONITORS;n++,i++) 
	{
		rv = usb_test_write_entries(axi_monitor[n].name, &oxnas_test_config_write, &oxnas_test_config_read, i);
		if (rv < 0)
		{
			while (n != 0)
			{
				n--;
				/* remove any allocated entries */
				remove_proc_entry (axi_monitor[n].name, oxnas_test_dir);
			} 
			goto no_axi_monitors;
		}
	}

	{
		struct proc_dir_entry * entry = create_proc_entry("control", 0666, oxnas_test_dir);
		if (entry) {
			entry->write_proc = oxnas_test_control;
			entry->owner = THIS_MODULE;
			return 0;
		}
		else
		{
			goto no_control;
		}
	}


	printk(KERN_INFO "%s %s initialised\n", MODULE_NAME, MODULE_VERS);

	return 0;

	no_control:
		for (i = NO_AXI_MONITORS; i != 0; )
		{
			i--;
			/* remove any allocated entries */
			remove_proc_entry (axi_monitor[i].name, oxnas_test_dir);
		} 
		
	no_axi_monitors:
		for (i = NO_AHB_MONITORS; i != 0; )
		{
			i--;
			/* remove any allocated entries */
			remove_proc_entry (ahb_monitor[i].name, oxnas_test_dir);
		} 
	
	no_write:
		remove_proc_entry("read", oxnas_test_dir);
	no_read:
		remove_proc_entry(MODULE_NAME, NULL);
	out:
		return rv;
}


static void __exit oxnas_test_exit(void)
{
	int i;
	
	remove_proc_entry("control", oxnas_test_dir);
	
	for (i = 0; i < NO_AHB_MONITORS; i++)
	{
		remove_proc_entry(ahb_monitor[i].name, oxnas_test_dir);
	}

	for (i = 0; i < NO_AXI_MONITORS; i++)
	{
		remove_proc_entry(axi_monitor[i].name, oxnas_test_dir);
	}
	
	remove_proc_entry("read", oxnas_test_dir);
	remove_proc_entry(MODULE_NAME, NULL);

	printk(KERN_INFO "%s %s removed\n", MODULE_NAME, MODULE_VERS);
	
}


module_init(oxnas_test_init);
module_exit(oxnas_test_exit);






















