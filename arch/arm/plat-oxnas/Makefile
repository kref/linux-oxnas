#
# Makefile for the linux kernel.
#

obj-y :=
obj-m :=
obj-n :=
obj-  :=

obj-y += dma.o
obj-y += leon.o
obj-y += samba_reserve.o
obj-y += filter_misc.o
obj-y += prealloc_init.o

obj-$(CONFIG_OXNAS_FAST_OPEN_FILTER) += fast_open_filter.o

obj-$(CONFIG_OXNAS_ZERO_COPY_RX_SUPPORT) += oxnas_net.o

obj-$(CONFIG_OXNAS_FAST_READS_AND_WRITES) += incoherent_sendfile.o

obj-$(CONFIG_OXNAS_FAST_WRITES_SUPPORT) += entry.o
obj-$(CONFIG_OXNAS_FAST_WRITES_SUPPORT) += oxnas_direct_net_to_disk.o
obj-$(CONFIG_OXNAS_FAST_WRITES_SUPPORT) += oxnas_direct_disk.o

obj-$(CONFIG_OXNAS_FAST_WRITES) += direct_writes.o

obj-$(CONFIG_OXNAS_BACKUP) += backup.o

obj-$(CONFIG_TACHO_THERM_AND_FAN)		+= thermAndFan.o
obj-$(CONFIG_OXNAS_AHB_MONITOR_MODULE)		+= oxnas-ahb-monitor.o
obj-$(CONFIG_OXNAS_PHY_TEMPERATURE_MODULE)	+= phy-temp-monitor.o
obj-$(CONFIG_I2S)				+= i2s.o
obj-$(CONFIG_OXNAS_USB_TEST_MODES)		+= usb-test-mode.o
obj-$(CONFIG_LEON_POWER_BUTTON_MONITOR)		+= power_button.o
obj-$(CONFIG_USER_RECOVERY_BUTTON_MONITOR)	+= user_recovery_button.o
obj-$(CONFIG_LEON_WATCHDOG_MODULE)		+= watchdog.o
