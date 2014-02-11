/*
 * linux/arch/arm/mach-oxnas/watchdog.c
 * A module that sends a heartbeat message to a copro-implemented
 * system watchdog. If the copro fails to receive a regular heartbeat
 * it will reset the system.
 *
 * Copyright (C) 2010 PLX Technology Ltd
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
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/leon.h>
#include <mach/leon-watchdog-prog.h>

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andy Parker (PLX Technology Ltd)");
MODULE_DESCRIPTION("GMAC Network Driver");
MODULE_VERSION("v0.1");

#define DEFAULT_HEART_RATE  10	/* In beats per second */
#define DEFAULT_DEATH_COUNT 10	/* In missed beats */

static int heart_rate = DEFAULT_HEART_RATE; /* the heart beat rate - beats per second */
static int death_count = DEFAULT_DEATH_COUNT; /* the number of missed heartbeats after which the system is deemed to be dead */

module_param(heart_rate, int, S_IRUGO|S_IWUSR);
module_param(death_count, int, S_IRUGO|S_IWUSR);

#define HEARTBEAT_JIFFIES (HZ / heart_rate)
    
static struct timer_list timer;

#define COPRO_SEM_INT_HEARTBEAT      0


/*
 * Device driver object
 */
typedef struct watchdog_driver_s {
	/** sysfs dir tree root for power button driver */
	struct kset    *kset;
	struct kobject  kobject;
} watchdog_driver_t;

static watchdog_driver_t watchdog_driver;

static void work_handler(struct work_struct * not_used) {
	kobject_uevent(&watchdog_driver.kobject, KOBJ_OFFLINE);
}

DECLARE_WORK(watchdog_hotplug_work, work_handler);

static void timer_handler(unsigned long data)
{
    /* Restart the heartbeat timer with a suitable heartbeat timeout */
    timer.expires =  jiffies + HEARTBEAT_JIFFIES;
    add_timer(&timer);

    // Interrupt the copro with the heartbeat
	writel(1UL << COPRO_SEM_INT_HEARTBEAT, SYS_CTRL_SEMA_SET_CTRL);
}

static struct kobj_type ktype_watchdog = {
	.release = 0,
	.sysfs_ops = 0,
	.default_attrs = 0,
};

static int watchdog_hotplug_filter(struct kset* kset, struct kobject* kobj) {
	return get_ktype(kobj) == &ktype_watchdog;
}

static const char* watchdog_hotplug_name(struct kset* kset, struct kobject* kobj) {
	return "oxnas_watchdog";
}

static struct kset_uevent_ops watchdog_uevent_ops = {
	.filter = watchdog_hotplug_filter,
	.name   = watchdog_hotplug_name,
	.uevent = NULL,
};

static int __init watchdog_init(void)
{
	int err = 0;
    int watchdogTimeOutMS;
    
	/* Prepare the sysfs interface for use */
	watchdog_driver.kset = kset_create_and_add("watchdog", &watchdog_uevent_ops, kernel_kobj);
	if (!watchdog_driver.kset) {
		printk(KERN_ERR "watchdog_init() Failed to create kset\n");
		return -ENOMEM;
	}

	watchdog_driver.kobject.kset = watchdog_driver.kset;
	err = kobject_init_and_add(&watchdog_driver.kobject,
		&ktype_watchdog, NULL, "%d", 0);
	if (err) {
		printk(KERN_ERR "watchdog_init() Failed to add kobject\n");
		kset_unregister(watchdog_driver.kset);
		kobject_put(&watchdog_driver.kobject);
		return -EINVAL;
	}

	/* Setup the timer that will time how long the user holds down the power
	   button */
	init_timer(&timer);
	timer.data = 0;
	timer.function = timer_handler;
    timer.expires = jiffies + HEARTBEAT_JIFFIES;
    add_timer(&timer);
    
    // Load CoPro program and start it running
    watchdogTimeOutMS = (death_count * 1000) / heart_rate;
    
    init_copro(leon_srec, watchdogTimeOutMS);

	printk(KERN_INFO "Watchdog driver registered\n");
	return 0;
}

static void __exit watchdog_exit(void)
{

	kobject_put(&watchdog_driver.kobject);
	kset_unregister(watchdog_driver.kset);

	/* Deactive the timer */
	del_timer_sync(&timer);

}

/** 
 * macros to register initialisation and exit functions with kernel
 */
module_init(watchdog_init);
module_exit(watchdog_exit);
