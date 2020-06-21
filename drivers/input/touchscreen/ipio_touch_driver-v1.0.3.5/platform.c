/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include "common.h"
#include "core/config.h"
#include "core/i2c.h"
#include "core/spi.h"
#include "core/firmware.h"
#include "core/finger_report.h"
#include "core/flash.h"
#include "core/protocol.h"
#include "platform.h"
#include "core/mp_test.h"
#include "core/gesture.h"
#include <linux/wakelock.h>

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"

#define DTS_OF_NAME		"tchip,ilitek"


#define DEVICE_ID	"ILITEK_TDDI"

#ifdef USE_KTHREAD
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#endif

/*Huaqin add for fw update fail by liufurong at 20181112 start*/
#ifdef BOOT_FW_UPGRADE
extern int tp_fw_update_flag;
struct wake_lock ili_tp_update_wakelock;
#endif
/*Huaqin add for fw update fail by liufurong at 20181112 end*/

/* Debug level */
uint32_t ipio_debug_level = 0;//DEBUG_ALL;
EXPORT_SYMBOL(ipio_debug_level);
struct ilitek_platform_data *ipd = NULL;
void ilitek_platform_disable_irq(void)
{
	unsigned long nIrqFlag;

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			disable_irq_nosync(ipd->isr_gpio);
			ipd->isEnableIRQ = false;
			ipio_debug(DEBUG_IRQ, "Disable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already disabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_disable_irq);

void ilitek_platform_enable_irq(void)
{
	unsigned long nIrqFlag;

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (!ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			enable_irq(ipd->isr_gpio);
			ipd->isEnableIRQ = true;
			ipio_debug(DEBUG_IRQ, "Enable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already enabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_enable_irq);

int ilitek_platform_tp_hw_reset(bool isEnable)
{
	int ret = 0;
	ipio_info("HW Reset: %d\n", isEnable);

	ilitek_platform_disable_irq();

	if (isEnable) {
		gpio_direction_output(ipd->reset_gpio, 1);
		mdelay(ipd->delay_time_high);
		gpio_set_value(ipd->reset_gpio, 0);
		mdelay(ipd->delay_time_low);
		gpio_set_value(ipd->reset_gpio, 1);
		mdelay(ipd->edge_delay);
	} else {
		gpio_set_value(ipd->reset_gpio, 0);
	}
	mdelay(10);
	ilitek_platform_enable_irq();
	return ret;
}
EXPORT_SYMBOL(ilitek_platform_tp_hw_reset);

#ifdef REGULATOR_POWER_ON
void ilitek_regulator_power_on(bool status)
{
	int res = 0;

	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (ipd->vdd) {
			res = regulator_enable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_enable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	} else {
		if (ipd->vdd) {
			res = regulator_disable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_disable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	}
	core_config->icemodeenable = false;
	mdelay(5);
}
EXPORT_SYMBOL(ilitek_regulator_power_on);
#endif /* REGULATOR_POWER_ON */

#ifdef BATTERY_CHECK
static void read_power_status(uint8_t *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s\n", POWER_STATUS_PATH);
		return;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ipio_debug(DEBUG_BATTERY, "Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
}

static void ilitek_platform_vpower_notify(struct work_struct *pWork)
{
	uint8_t charge_status[20] = { 0 };

	ipio_debug(DEBUG_BATTERY, "isEnableCheckPower = %d\n", ipd->isEnablePollCheckPower);
	read_power_status(charge_status);
	ipio_debug(DEBUG_BATTERY, "Batter Status: %s\n", charge_status);

/* Huaqin modify for ZQL1830-1463 by liufurong at 10181030 start */
	if (strstr(charge_status, "Charging") != NULL || strstr(charge_status, "Full") != NULL
	    || strstr(charge_status, "Fully charged") != NULL) {
			ipio_debug(DEBUG_BATTERY, "Charging mode\n");
			core_config_plug_ctrl(false);
	} else {
			ipio_debug(DEBUG_BATTERY, "Not charging mode\n");
			core_config_plug_ctrl(true);
	}
/* Huaqin modify for ZQL1830-1463 by liufurong at 10181030 end */

	if (ipd->isEnablePollCheckPower)
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
}
#endif

#ifdef ESD_CHECK
static void ilitek_platform_esd_recovery(struct work_struct *work)
{
	int ret = 0;

	mutex_lock(&ipd->plat_mutex);
	ret = ilitek_platform_tp_hw_reset(true);
	if(ret < 0)
		ipio_err("host download failed!\n");
	mutex_unlock(&ipd->plat_mutex);
}

static void ilitek_platform_esd_check(struct work_struct *pWork)
{
	int ret;
	uint8_t tx_data = 0x82, rx_data = 0;


}
#endif /* ESD_CHECK */
#if defined CONFIG_FB
/* Huaqin modify Bright screen speed of 1244451 for ZQL1830 by liufurong at 2018/10/09 start*/
int fb_ilitek_resume(void *data)
{
	core_config_ic_resume();
	return 0;
}
/* Huaqin modify Bright screen speed of 1244451 for ZQL1830 by liufurong at 2018/10/09 end*/
static int ilitek_platform_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;

	ipio_info("Notifier's event = %ld\n", event);

	/*
	 *  FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *  FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	 */
	if (evdata && evdata->data && (event == FB_EARLY_EVENT_BLANK)) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN){
			ipio_info("TP early Suspend\n");

			if (!core_firmware->isUpgrading) {
/* Huaqin modify for ili suspend by qimaokang at 2018/08/22 start*/
				core_config_ic_early_suspend();
/* Huaqin modify for ili suspend by qimaokang at 2018/08/22 end*/
			}
		}
	}
	else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			ipio_info("TP Resuem\n");

			if (!core_firmware->isUpgrading) {
			    /* Huaqin modify Bright screen speed of 1244451 for ZQL1830 by liufurong at 2018/10/09 start*/
			    kthread_run(fb_ilitek_resume,&ipd->client->dev,"tp_ilitek_resume");
			    /* Huaqin modify Bright screen speed of 1244451 for ZQL1830 by liufurong at 2018/10/09 end*/
			}
/* Huaqin modify for ili suspend by qimaokang at 2018/08/22 start*/
		} else if (*blank == FB_BLANK_POWERDOWN) {
			ipio_info("TP Suspend\n");

			if (!core_firmware->isUpgrading) {
				core_config_ic_suspend();
			}
		}
/* Huaqin modify for ili suspend by qimaokang at 2018/08/22 end*/
	}

	return NOTIFY_OK;
}
#else /* CONFIG_HAS_EARLYSUSPEND */
static void ilitek_platform_early_suspend(struct early_suspend *h)
{
	ipio_info("TP Suspend\n");

	/* TODO: there is doing nothing if an upgrade firmware's processing. */

	core_fr_touch_release(0, 0, 0);

	input_sync(core_fr->input_device);

	core_fr->isEnableFR = false;

	core_config_ic_suspend();
}

static void ilitek_platform_late_resume(struct early_suspend *h)
{
	ipio_info("TP Resuem\n");

	core_fr->isEnableFR = true;
	core_config_ic_resume();
}
#endif

/**
 * reg_power_check - register a thread to inquery status at certain time.
 */
static int ilitek_platform_reg_power_check(void)
{
	int res = 0;

#ifdef BATTERY_CHECK
	INIT_DELAYED_WORK(&ipd->check_power_status_work, ilitek_platform_vpower_notify);
	ipd->check_power_status_queue = create_workqueue("ili_power_check");
	ipd->work_delay = msecs_to_jiffies(CHECK_BATTERY_TIME);
	ipd->isEnablePollCheckPower = true;
	if (!ipd->check_power_status_queue) {
		ipio_err("Failed to create a work thread to check power status\n");
		ipd->vpower_reg_nb = false;
		res = -1;
	} else {
		ipio_info("Created a work thread to check power status at every %u jiffies\n",
			 (unsigned)ipd->work_delay);

		if (ipd->isEnablePollCheckPower) {
			queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work,
					   ipd->work_delay);
			ipd->vpower_reg_nb = true;
		}
	}
#endif /* BATTERY_CHECK */

	return res;
}

static int ilitek_platform_reg_esd_check(void)
{
	int res = 0;

#ifdef ESD_CHECK
	INIT_DELAYED_WORK(&ipd->check_esd_status_work, ilitek_platform_esd_check);
	ipd->check_esd_status_queue = create_workqueue("ili_esd_check");
	ipd->esd_check_time = msecs_to_jiffies(CHECK_ESD_TIME);
	ipd->isEnablePollCheckEsd = true;
	if (!ipd->check_esd_status_queue) {
		ipio_err("Failed to create a work thread to check power status\n");
		ipd->vesd_reg_nb = false;
		res = -1;
	} else {
		ipio_info("Created a work thread to check power status at every %u jiffies\n",
			 (unsigned)ipd->esd_check_time);

		INIT_WORK(&ipd->esd_recovery, ilitek_platform_esd_recovery);

		if (ipd->isEnablePollCheckEsd) {
			queue_delayed_work(ipd->check_esd_status_queue, &ipd->check_esd_status_work,
					   ipd->esd_check_time);
			ipd->vesd_reg_nb = true;
		}
	}
#endif /* ESD_CHECK */

	return res;
}
/**
 * Register a callback function when the event of suspend and resume occurs.
 *
 * The default used to wake up the cb function comes from notifier block mechnaism.
 * If you'd rather liek to use early suspend, CONFIG_HAS_EARLYSUSPEND in kernel config
 * must be enabled.
 */
static int ilitek_platform_reg_suspend(void)
{
	int res = 0;

	ipio_info("Register suspend/resume callback function\n");
#ifdef CONFIG_FB
	ipd->notifier_fb.notifier_call = ilitek_platform_notifier_fb;
	res = fb_register_client(&ipd->notifier_fb);
#else
	ipd->early_suspend->suspend = ilitek_platform_early_suspend;
	ipd->early_suspend->esume = ilitek_platform_late_resume;
	ipd->early_suspend->level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	res = register_early_suspend(ipd->early_suspend);
#endif /* CONFIG_FB */

	return res;
}

#ifndef USE_KTHREAD
static void ilitek_platform_work_queue(struct work_struct *work)
{
	ipio_debug(DEBUG_IRQ, "work_queue: IRQ = %d\n", ipd->isEnableIRQ);

	core_fr_handler();

	if (!ipd->isEnableIRQ)
		ilitek_platform_enable_irq();
}
#endif /* USE_KTHREAD */

static irqreturn_t ilitek_platform_irq_handler(int irq, void *dev_id)
{
	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);

	if (ipd->isEnableIRQ) {
		ilitek_platform_disable_irq();
#ifdef USE_KTHREAD
		ipd->irq_trigger = true;
		wake_up_interruptible(&waiter);
#else
		schedule_work(&ipd->report_work_queue);
#endif /* USE_KTHREAD */
	}

	return IRQ_HANDLED;
}

static int ilitek_platform_input_init(void)
{
	int res = 0;

	ipd->input_device = input_allocate_device();

	if (ERR_ALLOC_MEM(ipd->input_device)) {
		ipio_err("Failed to allocate touch input device\n");
		res = -ENOMEM;
		goto fail_alloc;
	}

	ipd->input_device->name = ipd->client->name;
	ipd->input_device->phys = "I2C";
	ipd->input_device->dev.parent = &ipd->client->dev;
	ipd->input_device->id.bustype = BUS_I2C;

	core_fr_input_set_param(ipd->input_device);
	/* register the input device to input sub-system */
	res = input_register_device(ipd->input_device);
	if (res < 0) {
		ipio_err("Failed to register touch input device, res = %d\n", res);
		goto out;
	}

	return res;

fail_alloc:
	input_free_device(core_fr->input_device);
	return res;

out:
	input_unregister_device(ipd->input_device);
	input_free_device(core_fr->input_device);
	return res;
}

#if defined (USE_KTHREAD) || defined (BOOT_FW_UPGRADE)
static int kthread_handler(void *arg)
{
	int res = 0;
	char *str = (char *)arg;

	if (strcmp(str, "boot_fw") == 0) {
		/* FW Upgrade event */
		core_firmware->isboot = true;

		ilitek_platform_disable_irq();

#ifdef BOOT_FW_UPGRADE
/*Huaqin add for fw update fail by liufurong at 20181112 start*/
		wake_lock(&ili_tp_update_wakelock);
		tp_fw_update_flag = 1;
		res = core_firmware_boot_upgrade();
		if (res < 0)
			ipio_err("Failed to upgrade FW at boot stage\n");
		tp_fw_update_flag = 0;
		wake_unlock(&ili_tp_update_wakelock);
/*Huaqin add for fw update fail by liufurong at 20181112 end*/
#endif
/* Huaqin modify for ZQL1830-600 by liufurong at 20180828 start */
		//ilitek_platform_input_init();
		ilitek_platform_enable_irq();
/* Huaqin modify for ZQL1830-600 by liufurong at 20180828 end */
		core_firmware->isboot = false;
	} else if (strcmp(str, "irq") == 0) {
#ifdef USE_KTHREAD
		/* IRQ event */
		struct sched_param param = {.sched_priority = 4 };

		sched_setscheduler(current, SCHED_RR, &param);

		while (!kthread_should_stop() && !ipd->free_irq_thread) {
			set_current_state(TASK_INTERRUPTIBLE);
			wait_event_interruptible(waiter, ipd->irq_trigger);
			ipd->irq_trigger = false;
			set_current_state(TASK_RUNNING);
			core_fr_handler();
			ilitek_platform_enable_irq();
		}
#endif
	} else {
		ipio_err("Unknown EVENT\n");
	}

	return res;
}
#endif

static int ilitek_platform_isr_register(void)
{
	int res = 0;

#ifdef USE_KTHREAD
	ipd->irq_thread = kthread_run(kthread_handler, "irq", "ili_irq_thread");
	if (ipd->irq_thread == (struct task_struct *)ERR_PTR) {
		ipd->irq_thread = NULL;
		ipio_err("Failed to create kthread\n");
		res = -ENOMEM;
		goto out;
	}
	ipd->irq_trigger = false;
	ipd->free_irq_thread = false;
#else
	INIT_WORK(&ipd->report_work_queue, ilitek_platform_work_queue);
#endif /* USE_KTHREAD */

	ipd->isr_gpio = gpio_to_irq(ipd->int_gpio);

	ipio_info("ipd->isr_gpio = %d\n", ipd->isr_gpio);

	res = request_threaded_irq(ipd->isr_gpio,
				   NULL,
				   ilitek_platform_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", NULL);

	if (res != 0) {
		ipio_err("Failed to register irq handler, irq = %d, res = %d\n", ipd->isr_gpio, res);
		goto out;
	}
/* Huaqin modify for ZQL1830-600 by liufurong at 20180828 start */
       ipd->isEnableIRQ = true;
       ilitek_platform_disable_irq();
/* Huaqin modify for ZQL1830-600 by liufurong at 20180828 end */

out:
	return res;
}

static int ilitek_platform_gpio(void)
{
	int res = 0;

#ifdef CONFIG_OF
	struct device_node *dev_node = ipd->client->dev.of_node;
	uint32_t flag;

	ipd->int_gpio = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	ipd->reset_gpio = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);
#endif /* CONFIG_OF */

	ipio_info("GPIO INT: %d\n", ipd->int_gpio);
	ipio_info("GPIO RESET: %d\n", ipd->reset_gpio);

	if (!gpio_is_valid(ipd->int_gpio)) {
		ipio_err("Invalid INT gpio: %d\n", ipd->int_gpio);
		return -EBADR;
	}

	if (!gpio_is_valid(ipd->reset_gpio)) {
		ipio_err("Invalid RESET gpio: %d\n", ipd->reset_gpio);
		return -EBADR;
	}

	res = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
	if (res < 0) {
		ipio_err("Request IRQ GPIO failed, res = %d\n", res);
		gpio_free(ipd->int_gpio);
		res = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
		if (res < 0) {
			ipio_err("Retrying request INT GPIO still failed , res = %d\n", res);
			goto out;
		}
	}

	res = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
	if (res < 0) {
		ipio_err("Request RESET GPIO failed, res = %d\n", res);
		gpio_free(ipd->reset_gpio);
		res = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
		if (res < 0) {
			ipio_err("Retrying request RESET GPIO still failed , res = %d\n", res);
			goto out;
		}
	}

	gpio_direction_input(ipd->int_gpio);

out:
	return res;
}

int ilitek_platform_read_tp_info(void)
{
/* huaqin modify for ZQL1830-1529 by liufurong at 20181101 start */
	int retry_cnt = 0;
	while(retry_cnt < 4){
		if (core_config_get_chip_id() < 0) {
			ipio_err("Failed to get chip id retry_cnt = %d \n",retry_cnt);
			retry_cnt++;
			if(retry_cnt == 3){
				return -1;
			}
		} else {
			break;
		}
	}
/* huaqin modify for ZQL1830-1529 by liufurong at 20181101 end */
/* huaqin modify for probe fail by liufurong at 20180731 start */
	if (core_config_get_protocol_ver() < 0) {
		ipio_err("Failed to get protocol version\n");
		//return -1;
	}
	udelay(50);
	if (core_config_get_fw_ver() < 0) {
		ipio_err("Failed to get firmware version\n");
		//return -1;
	}
	udelay(50);
	if (core_config_get_core_ver() < 0) {
		ipio_err("Failed to get core version\n");
		//return -1;
	}
	udelay(50);
	if (core_config_get_tp_info() < 0) {
		ipio_err("Failed to get TP information\n");
		//return -1;
	}
	udelay(50);
	if (core_config_get_key_info() < 0) {
		ipio_err("Failed to get key information\n");
		//return -1;
	}
/* huaqin modify for probe fail by liufurong at 20180731 end */
	return 0;
}
EXPORT_SYMBOL(ilitek_platform_read_tp_info);

/**
 * The function is to initialise all necessary structurs in those core APIs,
 * they must be called before the i2c dev probes up successfully.
 */
static int ilitek_platform_core_init(void)
{
	ipio_info("Initialise core's components\n");

	if (core_config_init() < 0 || core_protocol_init() < 0 ||
		core_firmware_init() < 0 || core_fr_init() < 0 ||
		core_gesture_init () < 0) {
		ipio_err("Failed to initialise core components\n");
		return -EINVAL;
	}

	if (core_i2c_init(ipd->client) < 0) {
		ipio_err("Failed to initialise interface\n");
		return -EINVAL;
	}
	return 0;
}

static int ilitek_platform_remove(struct i2c_client *client)
{
	ipio_info("Remove platform components\n");

	if (ipd->isEnableIRQ) {
		disable_irq_nosync(ipd->isr_gpio);
	}

	if (ipd->isr_gpio != 0 && ipd->int_gpio != 0 && ipd->reset_gpio != 0) {
		free_irq(ipd->isr_gpio, (void *)ipd->i2c_id);
		gpio_free(ipd->int_gpio);
		gpio_free(ipd->reset_gpio);
	}
#ifdef CONFIG_FB
	fb_unregister_client(&ipd->notifier_fb);
#else
	unregister_early_suspend(&ipd->early_suspend);
#endif /* CONFIG_FB */

#ifdef USE_KTHREAD
	if (ipd->irq_thread != NULL) {
		ipd->irq_trigger = true;
		ipd->free_irq_thread = true;
		wake_up_interruptible(&waiter);
		kthread_stop(ipd->irq_thread);
		ipd->irq_thread = NULL;
	}
#endif /* USE_KTHREAD */

	if (ipd->input_device != NULL) {
		input_unregister_device(ipd->input_device);
		input_free_device(ipd->input_device);
	}

	if (ipd->vpower_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		destroy_workqueue(ipd->check_power_status_queue);
	}

	if (ipd->vesd_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_esd_status_work);
		destroy_workqueue(ipd->check_esd_status_queue);
	}

	ilitek_proc_remove();
	return 0;
}

/* huaqin add for ito tset by liufurong at 20180725 start */
/**********add ito test mode function  *******************/
#define HWINFO_NAME		"tp_wake_switch"
extern int32_t ilitek_ito_selftest_open(void);
int ilitek_TestResultLen=0;
static struct platform_device hwinfo_device= {
	.name = HWINFO_NAME,
	.id = -1,
};

static ssize_t ito_test_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	int count;
	/* huaqin modify for ZQL1830-1529 by liufurong at 20181101 start */
	if (ipd->isEnablePollCheckPower)
		cancel_delayed_work(&ipd->check_power_status_work);
	ilitek_ito_selftest_open();
	count = sprintf(buf, "%d\n", ilitek_TestResultLen);
	ilitek_TestResultLen = 0;
	if (ipd->isEnablePollCheckPower)
		queue_delayed_work(ipd->check_power_status_queue,&ipd->check_power_status_work, ipd->work_delay);
	/* huaqin modify for ZQL1830-1529 by liufurong at 20181101 end */
	return count;
}

static ssize_t ito_test_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(factory_check, 0644, ito_test_show, ito_test_store);

static struct attribute *ito_test_attributes[] ={

	&dev_attr_factory_check.attr,
	NULL
};
static struct attribute_group ito_test_attribute_group = {
	.attrs = ito_test_attributes
};
int ilitek_test_node_init(struct platform_device *tpinfo_device)
{
	int err=0;
    err = sysfs_create_group(&tpinfo_device->dev.kobj, &ito_test_attribute_group);
    if (0 != err)
    {
        printk( "[ILITEK-ito] %s() - ERROR: sysfs_create_group() failed.",  __func__);
        sysfs_remove_group(&tpinfo_device->dev.kobj, &ito_test_attribute_group);
        return -EIO;
    }
    else
    {
        printk("[ILITEK-ito] %s() - sysfs_create_group() succeeded.", __func__);
    }
    return err;
}
/*************************************************/
/* huaqin add for ito tset by liufurong at 20180725 end */

/* huaqin add for gesture by liufurong at 20180807 start */
#define ILITEK_GESTURE_MODE "tpd_gesture"

static long gesture_mode = 0;

static ssize_t ilitek_gesture_mode_get_proc(struct file *file,
                        char __user *buffer, size_t size, loff_t *ppos)
{
	char ptr[64] = {0};
	unsigned int len = 0;
	unsigned int ret = 0;

	if (gesture_mode == 0) {
		len = sprintf(ptr, "0\n");
	} else {
		len = sprintf(ptr, "1\n");
	}
	ret = simple_read_from_buffer(buffer, size, ppos, ptr, (size_t)len);
	return ret;
}

static ssize_t ilitek_gesture_mode_set_proc(struct file *filp,
                        const char __user *buffer, size_t count, loff_t *off)
{
	char msg[20] = {0};
	int ret = 0;

	ret = copy_from_user(msg, buffer, count);
	ipio_err("msg = %s\n", msg);
	if (ret) {
		return -EFAULT;
	}

	ret = kstrtol(msg, 0, &gesture_mode);
	if (!ret) {
		if (gesture_mode == 0) {
			gesture_mode = 0;
			core_config->isEnableGesture = false;
		} else {
			gesture_mode = 1;
			core_config->isEnableGesture = true;
		}
	}
	else {
		ipio_err("set gesture mode failed\n");
	}
	ipio_err("gesture_mode = %d \n", core_config->isEnableGesture);

	return count;
}

static struct proc_dir_entry *ilitek_gesture_mode_proc = NULL;
static const struct file_operations gesture_mode_proc_ops = {
	.owner = THIS_MODULE,
	.read = ilitek_gesture_mode_get_proc,
	.write = ilitek_gesture_mode_set_proc,
};
/* huaqin add for gesture by liufurong at 20180807 end */

/**
 * The probe func would be called after an i2c device was detected by kernel.
 *
 * It will still return zero even if it couldn't get a touch ic info.
 * The reason for why we allow it passing the process is because users/developers
 * might want to have access to ICE mode to upgrade a firwmare forcelly.
 */
 /* huaqin add for Shutdown time by zhanghao at 20180906 start */
static void ilitek_platform_shutdown(struct i2c_client *client)
{
	printk("HQ add for ilitek tp shut down\n");
	ilitek_lcm_power_source_ctrl(0);
}
/* huaqin add for Shutdown time by zhanghao at 20180906 end */
static int ilitek_platform_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef REGULATOR_POWER_ON
	const char *vdd_name = "vdd";
	const char *vcc_i2c_name = "vcc_i2c";
#endif /* REGULATOR_POWER_ON */

	if (client == NULL) {
		ipio_err("i2c client is NULL\n");
		return -ENODEV;
	}

	/* Set i2c slave addr if it's not configured */
	ipio_info("I2C Slave address = 0x%x\n", client->addr);
	if (client->addr != ILI9881_SLAVE_ADDR) {
		client->addr = ILI9881_SLAVE_ADDR;
		ipio_err("I2C Slave addr doesn't be set up, use default : 0x%x\n", client->addr);
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ipio_err("I2C not supported\n");
		return -ENODEV;
	}

	ipd = devm_kzalloc(&client->dev, sizeof(*ipd), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ipd)) {
		ipio_err("Failed to allocate ipd memory, %ld\n", PTR_ERR(ipd));
		return -ENOMEM;
	}

	ipd->client = client;
	ipd->i2c_id = id;
	ipd->dev = &client->dev;
	ipd->chip_id = TP_TOUCH_IC;
	ipd->isEnableIRQ = false;
	ipd->isEnablePollCheckPower = false;
	ipd->isEnablePollCheckEsd = false;
	ipd->vpower_reg_nb = false;
	ipd->vesd_reg_nb = false;

	ipio_info("Driver Version : %s\n", DRIVER_VERSION);
	ipio_info("Driver for Touch IC :  %x\n", TP_TOUCH_IC);
	ipio_info("Driver on platform :  %x\n", TP_PLATFORM);
	ipio_info("Driver interface :  %s\n", (INTERFACE == I2C_INTERFACE) ? "I2C" : "SPI");

	/*
	 * Different ICs may require different delay time for the reset.
	 * They may also depend on what your platform need to.
	 */
	 if (ipd->chip_id == CHIP_TYPE_ILI9881) {
		 ipd->delay_time_high = 10;
		 ipd->delay_time_low = 5;

		 ipd->edge_delay = 100;

	}

	mutex_init(&ipd->plat_mutex);
	spin_lock_init(&ipd->plat_spinlock);

	/* Init members for debug */
	mutex_init(&ipd->ilitek_debug_mutex);
	mutex_init(&ipd->ilitek_debug_read_mutex);
	init_waitqueue_head(&(ipd->inq));
	ipd->debug_data_frame = 0;
	ipd->debug_node_open = false;

#ifdef REGULATOR_POWER_ON
	ipd->vdd = regulator_get(&ipd->client->dev, vdd_name);
	if (ERR_ALLOC_MEM(ipd->vdd)) {
		ipio_err("regulator_get vdd fail\n");
		ipd->vdd = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
			ipio_err("Failed to set vdd %d.\n", VDD_VOLTAGE);
	}

	ipd->vdd_i2c = regulator_get(&ipd->client->dev, vcc_i2c_name);
	if (ERR_ALLOC_MEM(ipd->vdd_i2c)) {
		ipio_err("regulator_get vdd_i2c fail.\n");
		ipd->vdd_i2c = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd_i2c, VDD_I2C_VOLTAGE, VDD_I2C_VOLTAGE) < 0)
			ipio_err("Failed to set vdd_i2c %d\n", VDD_I2C_VOLTAGE);
	}
	ilitek_regulator_power_on(true);
#endif /* REGULATOR_POWER_ON */
	atomic_set(&(ipd->lcm_lab_power), 0);
	atomic_set(&(ipd->lcm_ibb_power), 0);
	if (ilitek_lcm_bias_power_init()) {
		pr_err("power resource init error!\n");
		goto out_power_reinit;
	}
	ilitek_lcm_power_source_ctrl(1);

	if (ilitek_platform_gpio() < 0){
		ipio_err("Failed to request gpios\n ");
		goto out_request_gpio;
		}

	/* If kernel failes to allocate memory to the core components, driver will be unloaded. */
	if (ilitek_platform_core_init() < 0) {
		ipio_err("Failed to allocate cores' mem\n");
		goto out_core_init_fail;
	}

	ilitek_platform_tp_hw_reset(true);

	/* get our tp ic information */
	if(ilitek_platform_read_tp_info() < 0){
		ipio_err("Failed to get tp info\n");
		goto out_read_tp_info_fail;
	}

	/* If it defines boot upgrade, input register will be done inside boot function. */
/* Huaqin modify for ZQL1830-1478 by liufurong at 20181023 start */
//#ifndef BOOT_FW_UPGRADE
	if (ilitek_platform_input_init() < 0)
		ipio_err("Failed to init input device in kernel\n");
//#endif /* BOOT_FW_UPGRADE */
/* Huaqin modify for ZQL1830-1478 by liufurong at 20181023 end */

	if (ilitek_platform_isr_register() < 0)
		ipio_err("Failed to register ISR\n");

	if (ilitek_platform_reg_suspend() < 0)
		ipio_err("Failed to register suspend/resume function\n");

	if (ilitek_platform_reg_power_check() < 0)
		ipio_err("Failed to register power check function\n");

	if (ilitek_platform_reg_esd_check() < 0)
		ipio_err("Failed to register esd check function\n");
	/* Create nodes for users */
	ilitek_proc_init();
/* huaqin add for ito tset by liufurong at 20180725 start */
	platform_device_register(&hwinfo_device);
	ilitek_test_node_init(&hwinfo_device);
/* huaqin add for ito tset by liufurong at 20180725 end */
/* huaqin add for gesture by liufurong at 20180807 start */
	ilitek_gesture_mode_proc = proc_create(ILITEK_GESTURE_MODE, 0666, NULL,
				&gesture_mode_proc_ops);
	if (!ilitek_gesture_mode_proc) {
		ipio_err("create proc tpd_gesture failed\n");
	}
/* huaqin add for gesture by liufurong at 20180807 end */
#ifdef BOOT_FW_UPGRADE
	/*Huaqin add for fw update fail by liufurong at 20181112 start*/
	wake_lock_init(&ili_tp_update_wakelock, WAKE_LOCK_SUSPEND, "ili_tp-update");
	/*Huaqin add for fw update fail by liufurong at 20181112 end*/
	ipd->update_thread = kthread_run(kthread_handler, "boot_fw", "ili_fw_boot");
	if (ipd->update_thread == (struct task_struct *)ERR_PTR) {
		ipd->update_thread = NULL;
		ipio_err("Failed to create fw upgrade thread\n");
	}
#endif /* BOOT_FW_UPGRADE */

	return 0;
out_read_tp_info_fail:
	gpio_free(ipd->reset_gpio);
	gpio_free(ipd->int_gpio);
out_core_init_fail:
out_request_gpio:
	ilitek_lcm_power_source_ctrl(0);
	ilitek_lcm_bias_power_deinit();
out_power_reinit:
	mutex_destroy(&ipd->ilitek_debug_read_mutex);
	mutex_destroy(&ipd->ilitek_debug_mutex);
	mutex_destroy(&ipd->plat_mutex);
	/* huaqin modify for ZQL1830-1223 by liufurong at 20180930 start */
	devm_kfree(&client->dev , ipd);
	i2c_set_clientdata(client, NULL);
	return -ENODEV;
	/* huaqin modify for ZQL1830-1223 by liufurong at 20180930 end */
}

static const struct i2c_device_id tp_device_id[] = {
	{DEVICE_ID, 0},
	{},			/* should not omitted */
};

MODULE_DEVICE_TABLE(i2c, tp_device_id);

/*
 * The name in the table must match the definiation
 * in a dts file.
 *
 */
static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};



static struct i2c_driver tp_i2c_driver = {
	.driver = {
		   .name = DEVICE_ID,
		   .owner = THIS_MODULE,
		   .of_match_table = tp_match_table,
		   },
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
	.id_table = tp_device_id,
/* huaqin add for Shutdown time by zhanghao at 20180906 start */
	.shutdown = ilitek_platform_shutdown,
/* huaqin add for Shutdown time by zhanghao at 20180906 end */
};

static int __init ilitek_platform_init(void)
{
	printk("HQ add for ili tp init\n");
	ipio_info("TP driver init\n");
	return i2c_add_driver(&tp_i2c_driver);
}

static void __exit ilitek_platform_exit(void)
{
	ipio_info("I2C driver has been removed\n");

	i2c_del_driver(&tp_i2c_driver);
}

module_init(ilitek_platform_init);
module_exit(ilitek_platform_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
