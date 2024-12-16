/*
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file "COPYING" in the main directory of this archive for
 *  more details.
 *
 *  Copyright (C) 2020 Loongson Technology Corp.
 *  Author: Chao Li, lichao@loongson.cn
 *  Author: Yanbing Lv, lvyanbing@loongson.cn
 */

#include <linux/io.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <asm/bootinfo.h>
#include <linux/module.h>
#include <ec_it8528.h>
#include <linux/dmi.h>

#define EC_SCI_DEV  "3nod_ec"

#define OEM_STRING  "3Nod-laptop"

/* sci gpio irq number */
#define SCI_IRQ_NUM    123

static struct platform_device *platform_device;

struct quirk_entry {
	int sci_irq_num;
	int is_laptop;
	int is_allinone;
};

static struct quirk_entry quirk_default = {
	.sci_irq_num = SCI_IRQ_NUM,
	.is_laptop = 0,
	.is_allinone = 0,
};

static struct quirk_entry quirk_3nod_ls7a_laptop = {
	.sci_irq_num = SCI_IRQ_NUM,
	.is_laptop = 1,
	.is_allinone = 0,
};

static struct quirk_entry *quirks = &quirk_default;

static int dmi_check_cb(const struct dmi_system_id *dmi)
{
	pr_info("Identified 3nod device model '%s'\n", dmi->ident);

	quirks = dmi->driver_data;

	return 1;
}

static const struct dmi_system_id loongson_device_table[] = {
	{
		.ident = "3nod laptop",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Loongson"),
			DMI_MATCH(DMI_BOARD_NAME, "3Nod-LS3A4000-7A1000-laptop"),
			DMI_MATCH(DMI_CHASSIS_TYPE, "9"),
		},
		.callback = dmi_check_cb,
		.driver_data = &quirk_3nod_ls7a_laptop,
	},
	{}
};
MODULE_DEVICE_TABLE(dmi, loongson_device_table);

static int dmi_checked;

/* Power supply */
#define BIT_BAT_POWER_ACIN		(1 << 0)
enum
{
	APM_AC_OFFLINE =	0,
	APM_AC_ONLINE,
	APM_AC_BACKUP,
	APM_AC_UNKNOWN =	0xff
};
enum
{
	APM_BAT_STATUS_HIGH =		0,
	APM_BAT_STATUS_LOW,
	APM_BAT_STATUS_CRITICAL,
	APM_BAT_STATUS_CHARGING,
	APM_BAT_STATUS_NOT_PRESENT,
	APM_BAT_STATUS_UNKNOWN =	0xff
};

/* bat_reg_flag */
enum
{
	BAT_REG_TEMP_FLAG = 1,
	BAT_REG_VOLTAGE_FLAG,
	BAT_REG_CURRENT_FLAG,
	BAT_REG_AC_FLAG,
	BAT_REG_RC_FLAG,
	BAT_REG_FCC_FLAG,
	BAT_REG_ATTE_FLAG,
	BAT_REG_ATTF_FLAG,
	BAT_REG_RSOC_FLAG,
	BAT_REG_CYCLCNT_FLAG
};

/* Power info cached timeout */
#define POWER_INFO_CACHED_TIMEOUT	100	/* jiffies */

/* Backlight */
#define MAX_BRIGHTNESS	100

/* Power information structure */
struct loongson_power_info
{
	/* AC insert or not */
	unsigned int ac_in;
	/* Battery insert or not */
	unsigned int bat_in;
	unsigned int health;

	/* Battery designed capacity */
	unsigned int design_capacity;
	/* Battery designed voltage */
	unsigned int design_voltage;
	/* Battery capacity after full charged */
	unsigned int full_charged_capacity;
	/* Battery Manufacture Date */
	unsigned char manufacture_date[11];
	/* Battery Serial number */
	unsigned char serial_number[8];
	/* Battery Manufacturer Name, max 11 + 1(length) bytes */
	unsigned char manufacturer_name[13];
	/* Battery Device Name, max 7 + 1(length) bytes */
	unsigned char device_name[11];
	/* Battery Technology */
	unsigned int technology;
	/* Battery cell count */
	unsigned char cell_count;

	/* Battery dynamic charge/discharge voltage */
	unsigned int voltage_now;
	/* Battery dynamic charge/discharge average current */
	int current_now;
	int current_sign;
	int current_average;
	/* Battery current remaining capacity */
	unsigned int remain_capacity;
	/* Battery current remaining capacity percent */
	unsigned int remain_capacity_percent;
	/* Battery current temperature */
	unsigned int temperature;
	/* Battery current remaining time (AverageTimeToEmpty) */
	unsigned int remain_time;
	/* Battery current full charging time (averageTimeToFull) */
	unsigned int fullchg_time;
	/* Battery Status */
	unsigned int charge_status;
	/* Battery current cycle count (CycleCount) */
	unsigned int cycle_count;
};

/* SCI device structure */
struct sci_device
{
	/* The sci number get from ec */
	unsigned char number;
	/* Sci count */
	unsigned char parameter;
	/* Irq relative */
	unsigned char irq;
	unsigned char irq_data;
	/* Device name */
	unsigned char name[10];
};

extern void prom_printf(char *fmt, ...);
extern int it8528_query_get_event_num(void);

/* SCI device object */
static struct sci_device *loongson_sci_device = NULL;
static int loongson_sci_event_probe(struct platform_device *);
static ssize_t version_show(struct device_driver *, char *);
/* >>>Power management operation */
/* Update battery information handle function. */
static void loongson_power_battery_info_update(unsigned char bat_reg_flag);
/* Clear battery static information. */
static void loongson_power_info_battery_static_clear(void);
/* Get battery static information. */
static void loongson_power_info_battery_static_update(void);
/* Update power_status value */
static void loongson_power_info_power_status_update(void);
/* Power supply Battery get property handler */
static int loongson_bat_get_property(struct power_supply * pws,
			enum power_supply_property psp, union power_supply_propval * val);
/* Power supply AC get property handler */
static int loongson_ac_get_property(struct power_supply * pws,
			enum power_supply_property psp, union power_supply_propval * val);
/* SCI device AC event handler */
static int loongson_ac_handler(int status);
/* SCI device Battery event handler */
static int loongson_bat_handler(int status);

/* SCI device LID event handler */
static int loongson_lid_handler(int status);

/* Platform device suspend handler */
static int loongson_laptop_suspend(struct platform_device * pdev, pm_message_t state);
/* Platform device resume handler */
static int loongson_laptop_resume(struct platform_device * pdev);

/* SCI device event structure */
struct sci_event
{
	int index;
	sci_handler handler;
};

static struct input_dev *loongson_hotkey_dev = NULL;
static const struct sci_event se[] = {
	[SCI_EVENT_NUM_AC]             = {0, loongson_ac_handler},
	[SCI_EVENT_NUM_BAT]            = {INDEX_POWER_STATUS, loongson_bat_handler},

	[SCI_EVENT_NUM_LID]            = {INDEX_POWER_STATUS, loongson_lid_handler},
	[SCI_EVENT_NUM_WLAN]           = {INDEX_DEVICE_STATUS, NULL},
	[SCI_EVENT_NUM_TP]             = {0, NULL},
	[SCI_EVENT_NUM_BRIGHTNESS_OFF] = {0, NULL},
	[SCI_EVENT_NUM_BRIGHTNESS_DN]  = {0, NULL},
	[SCI_EVENT_NUM_BRIGHTNESS_UP]  = {0, NULL},
	[SCI_EVENT_NUM_DISPLAY_TOGGLE] = {0, NULL},
	[SCI_EVENT_NUM_MIC_TOGGLE]     = {0, NULL},
	[SCI_EVENT_NUM_SLEEP]          = {0, NULL},
	[SCI_EVENT_NUM_POWERBTN]       = {0, NULL},
};

static const struct key_entry loongson_keymap[] = {
	/* LID event */
	{KE_SW,  SCI_EVENT_NUM_LID, {SW_LID}},
	/* wifi  on or off */
	{KE_KEY, SCI_EVENT_NUM_WLAN, {KEY_WLAN}},
	/* TP  on or off */
	{KE_KEY, SCI_EVENT_NUM_TP, {KEY_TOUCHPAD_TOGGLE}},
	/* Fn+F6, Brightness off */
	{KE_KEY, SCI_EVENT_NUM_BRIGHTNESS_OFF, {KEY_DISPLAYTOGGLE} },
	/* Fn+F11, Brightness down */
	{KE_KEY, SCI_EVENT_NUM_BRIGHTNESS_DN, {KEY_BRIGHTNESSDOWN}},
	/* Fn+F12, Brightness up */
	{KE_KEY, SCI_EVENT_NUM_BRIGHTNESS_UP, {KEY_BRIGHTNESSUP}},
	/*Fn+F3, Dispaly switch */
	{KE_KEY, SCI_EVENT_NUM_DISPLAY_TOGGLE, {KEY_SWITCHVIDEOMODE}},
	/* Mute / unmute the microphone */
	{KE_KEY, SCI_EVENT_NUM_MIC_TOGGLE, {KEY_F20}},
	/* Fn+F1, Sleep */
	{KE_KEY, SCI_EVENT_NUM_SLEEP, {KEY_SLEEP}},
	/* Power button */
	{KE_KEY, SCI_EVENT_NUM_POWERBTN, {KEY_POWER}},
	{KE_END, 0}
};

static struct platform_driver loongson_3nod_pdriver = {
	.probe = loongson_sci_event_probe,
	.driver = {
		.name = "3nod_pm_hotkey",
		.owner = THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend = loongson_laptop_suspend,
	.resume  = loongson_laptop_resume,
#endif /* CONFIG_PM */
};

/* Power info object */
static struct loongson_power_info * power_info = NULL;
/* Power supply Battery property object */
static enum power_supply_property loongson_bat_props[] =
{
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	/* uAh */
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	/* percents */
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	/* Properties of type `const char *' */
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

/* Power supply Battery device object */
static struct power_supply_desc loongson_bat =
{
	.name = "loongson-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = loongson_bat_props,
	.num_properties = ARRAY_SIZE(loongson_bat_props),
	.get_property = loongson_bat_get_property,
};
static struct power_supply *three_nod_bat;

/* Power supply AC property object */
static enum power_supply_property loongson_ac_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};
/* Power supply AC device object */
static struct power_supply_desc loongson_ac =
{
	.name = "loongson-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = loongson_ac_props,
	.num_properties = ARRAY_SIZE(loongson_ac_props),
	.get_property = loongson_ac_get_property,
};
static struct power_supply *three_nod_ac;

static DRIVER_ATTR_RO(version);

static ssize_t version_show(struct device_driver *driver, char *buf)
{
	return sprintf(buf, "%s\n", "1.0");
}

/* SCI device event handler */
void loongson_3nod_sci_hotkey_handler(int event)
{
	int status = 0;
	struct key_entry * ke = NULL;
	struct sci_event * sep = NULL;

	sep = (struct sci_event*)&(se[event]);
	if (0 != sep->index)
		status = it8528_read(sep->index);
	if (NULL != sep->handler)
		status = sep->handler(status);

	ke = sparse_keymap_entry_from_scancode(loongson_hotkey_dev, event);
	if (ke) {
		if (SW_LID == ke->keycode) {
			/* report LID event. */
			input_report_switch(loongson_hotkey_dev, SW_LID, status);
			input_sync(loongson_hotkey_dev);
		} else {
			sparse_keymap_report_entry(loongson_hotkey_dev, ke, 1, true);
			input_sync(loongson_hotkey_dev);
		}
	}
}

static irqreturn_t loongson_sci_int_routine(int irq, void *dev_id)
{
	int event;
	if (loongson_sci_device->irq != irq)
		return IRQ_NONE;

	event = it8528_query_get_event_num();
	if ((SCI_EVENT_NUM_AC > event) || (SCI_EVENT_NUM_POWERBTN < event))
		goto exit_event_action;
	loongson_3nod_sci_hotkey_handler(event);
	return IRQ_HANDLED;

exit_event_action:
	return IRQ_HANDLED;
}

/* SCI driver pci driver init */
static int sci_pci_init(void)
{
	struct irq_fwspec fwspec;
	int ret = -EIO;

	printk(KERN_INFO "Loongson-3nod: SCI PCI init.\n");

	loongson_sci_device = kmalloc(sizeof(struct sci_device), GFP_KERNEL);
	if (!loongson_sci_device) {
		printk(KERN_ERR "Loongson-3nod: Malloc mem space for sci_dvice failed.\n");
		return -ENOMEM;
	}
	loongson_sci_device->irq = quirks->sci_irq_num;

	printk(KERN_INFO "Loongson-3nod: SCI irq_num:%d.\n", loongson_sci_device->irq);
	loongson_sci_device->irq_data = 0x00;
	loongson_sci_device->number = 0x00;
	loongson_sci_device->parameter = 0x00;
	strcpy(loongson_sci_device->name, EC_SCI_DEV);

	fwspec.fwnode = NULL;
	fwspec.param[0] = loongson_sci_device->irq;
	fwspec.param_count = 1;
	ret = irq_create_fwspec_mapping(&fwspec);
	if (ret >= 0) {
		loongson_sci_device->irq = ret;
		irq_set_irq_type(ret, IRQ_TYPE_LEVEL_LOW);
	}
	/* Regist pci irq */
	ret = request_irq(loongson_sci_device->irq, loongson_sci_int_routine,
				IRQF_SHARED, loongson_sci_device->name, loongson_sci_device);
	if (ret) {
		printk(KERN_ERR "Loongson-3nod: Request irq fail.\n");
		ret = -EFAULT;
		goto out_irq;
	}

	ret = 0;
	printk(KERN_INFO "Loongson-3nod: SCI PCI init successful.\n");
	return ret;

out_irq:
	kfree(loongson_sci_device);
	return ret;
}

/* SCI drvier pci driver init handler */
static int sci_pci_driver_init(void)
{
	int ret;
	ret = sci_pci_init();
	if (ret) {
		printk(KERN_ERR "Loongson-3nod: SCI Regist pci driver fail.\n");
		return ret;
	}
	printk(KERN_INFO "Loongson-3nod: SCI regist pci driver done.\n");
	return ret;
}

int is_3nod_laptop(void)
{
	if (!dmi_checked) {
		dmi_check_system(loongson_device_table);
		dmi_checked = 1;
	}

	return quirks->is_laptop;
}

/* Update battery information handle function. */
static void loongson_power_battery_info_update(unsigned char bat_reg_flag)
{
	short bat_info_value = 0;

	switch (bat_reg_flag) {
		/* Update power_info->temperature value */
		case BAT_REG_TEMP_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (it8528_read(INDEX_BATTERY_TEMP_HIGH) << 8) | it8528_read(INDEX_BATTERY_TEMP_LOW);
			power_info->temperature = (power_info->bat_in) ? (bat_info_value / 10 - 273) : 0;
			break;
		/* Update power_info->voltage value */
		case BAT_REG_VOLTAGE_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (it8528_read(INDEX_BATTERY_VOL_HIGH) << 8) | it8528_read(INDEX_BATTERY_VOL_LOW);
			power_info->voltage_now = (power_info->bat_in) ? bat_info_value : 0;
			break;
		/* Update power_info->current_now value */
		case BAT_REG_CURRENT_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (it8528_read(INDEX_BATTERY_CURRENT_HIGH) << 8) | it8528_read(INDEX_BATTERY_CURRENT_LOW);
			power_info->current_now = (power_info->bat_in) ? bat_info_value : 0;
			break;
		/* Update power_info->current_avg value */
		case BAT_REG_AC_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (it8528_read(INDEX_BATTERY_AC_HIGH) << 8) | it8528_read(INDEX_BATTERY_AC_LOW);
			power_info->current_average = (power_info->bat_in) ? bat_info_value : 0;
			break;
		/* Update power_info->remain_capacity value */
		case BAT_REG_RC_FLAG:
			power_info->remain_capacity = (it8528_read(INDEX_BATTERY_RC_HIGH) << 8) | it8528_read(INDEX_BATTERY_RC_LOW);
			break;
		/* Update power_info->full_charged_capacity value */
		case BAT_REG_FCC_FLAG:
			power_info->full_charged_capacity = (it8528_read(INDEX_BATTERY_FCC_HIGH) << 8) | it8528_read(INDEX_BATTERY_FCC_LOW);
			break;
		/* Update power_info->remain_time value */
		case BAT_REG_ATTE_FLAG:
			power_info->remain_time = (it8528_read(INDEX_BATTERY_ATTE_HIGH) << 8) | it8528_read(INDEX_BATTERY_ATTE_LOW);
			break;
		/* Update power_info->fullchg_time value */
		case BAT_REG_ATTF_FLAG:
			power_info->fullchg_time = (it8528_read(INDEX_BATTERY_ATTF_HIGH) << 8) | it8528_read(INDEX_BATTERY_ATTF_LOW);
			break;
		/* Update power_info->curr_cap value */
		case BAT_REG_RSOC_FLAG:
			power_info->remain_capacity_percent = it8528_read(INDEX_BATTERY_CAPACITY);
			break;
		/* Update power_info->cycle_count value */
		case BAT_REG_CYCLCNT_FLAG:
			power_info->cycle_count = (it8528_read(INDEX_BATTERY_CYCLECNT_HIGH) << 8) | it8528_read(INDEX_BATTERY_CYCLECNT_LOW);
			break;

		default:
			break;
	}
}

/* Clear battery static information. */
static void loongson_power_info_battery_static_clear(void)
{
	strcpy(power_info->manufacturer_name, "Unknown");
	strcpy(power_info->device_name, "Unknown");
	power_info->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	strcpy(power_info->serial_number, "Unknown");
	strcpy(power_info->manufacture_date, "Unknown");
	power_info->cell_count = 0;
	power_info->design_capacity = 0;
	power_info->design_voltage = 0;
}

/* Get battery static information. */
static void loongson_power_info_battery_static_update(void)
{
	unsigned char strbuf[16] = {0};
	unsigned int  bat_serial_number;
	unsigned int  count;
	unsigned char reg;

	/* Update manufacturer_name  */
	reg = 0xd0;
	memset(strbuf, 0, sizeof(strbuf)/sizeof(unsigned char));
	for(count=0; count< 12 ;count++){
		strbuf[count]=it8528_read(reg+count);
	}
	strcpy(power_info->manufacturer_name, strbuf);
	/* Update device_name  */
	reg = 0xc0;
	memset(strbuf, 0, sizeof(strbuf)/sizeof(unsigned char));
	for(count=0; count< 10;count++){
		strbuf[count]=it8528_read(reg+count);
	}
	strcpy(power_info->device_name, strbuf);

	power_info->technology = POWER_SUPPLY_TECHNOLOGY_LION;

	bat_serial_number = (it8528_read(INDEX_BATTERY_SN_HIGH) << 8) | it8528_read(INDEX_BATTERY_SN_LOW);
	snprintf(power_info->serial_number, 8, "%x", bat_serial_number);

	power_info->cell_count = ((it8528_read(INDEX_BATTERY_CV_HIGH) << 8) | it8528_read(INDEX_BATTERY_CV_LOW)) / 4200;

	power_info->design_capacity = (it8528_read(INDEX_BATTERY_DC_HIGH) << 8) | it8528_read(INDEX_BATTERY_DC_LOW);
	power_info->design_voltage = (it8528_read(INDEX_BATTERY_DV_HIGH) << 8) | it8528_read(INDEX_BATTERY_DV_LOW);
	power_info->full_charged_capacity = (it8528_read(INDEX_BATTERY_FCC_HIGH) << 8) | it8528_read(INDEX_BATTERY_FCC_LOW);
	printk(KERN_INFO "DesignCapacity: %dmAh, DesignVoltage: %dmV, FullChargeCapacity: %dmAh\n",
			power_info->design_capacity, power_info->design_voltage, power_info->full_charged_capacity);
}

/* Update power_status value */
static void loongson_power_info_power_status_update(void)
{
	unsigned int power_status = 0;

	power_status = it8528_read(INDEX_POWER_STATUS);

	power_info->ac_in = (power_status & MASK(BIT_POWER_ACPRES)) ?
					APM_AC_ONLINE : APM_AC_OFFLINE;

	power_info->bat_in = (power_status & MASK(BIT_POWER_BATPRES)) ? 1 : 0;
	if (power_info->bat_in && ((it8528_read(INDEX_BATTERY_DC_LOW) | (it8528_read(INDEX_BATTERY_DC_HIGH) << 8)) == 0))
		power_info->bat_in = 0;

	power_info->health = (power_info->bat_in) ?	POWER_SUPPLY_HEALTH_GOOD :
							POWER_SUPPLY_HEALTH_UNKNOWN;
	if (!power_info->bat_in) {
		power_info->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	else {
		if (power_status & MASK(BIT_POWER_BATFCHG)) {
			power_info->charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		else if (power_status & MASK(BIT_POWER_BATCHG)) {
			power_info->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else {
			power_info->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
	}
}

/* Power supply Battery get property handler */
static int loongson_bat_get_property(struct power_supply * pws,
			enum power_supply_property psp, union power_supply_propval * val)
{
	switch (psp) {
		/* Get battery static information. */
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			/* mV -> uV */
			val->intval = power_info->design_voltage * 1000;
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			/* mAh -> uAh */
			val->intval = power_info->design_capacity * 1000;
			break;
		case POWER_SUPPLY_PROP_MODEL_NAME:
			val->strval = power_info->device_name;
			break;
		case POWER_SUPPLY_PROP_MANUFACTURER:
			val->strval = power_info->manufacturer_name;
			break;
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			val->strval = power_info->serial_number;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = power_info->technology;
			break;
		/* Get battery dynamic information. */
		case POWER_SUPPLY_PROP_STATUS:
			loongson_power_info_power_status_update();
			val->intval = power_info->charge_status;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			loongson_power_info_power_status_update();
			val->intval = power_info->bat_in;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			loongson_power_info_power_status_update();
			val->intval = power_info->health;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			loongson_power_battery_info_update(BAT_REG_CURRENT_FLAG);
			/* mA -> uA */
			val->intval = power_info->current_now * 1000;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			loongson_power_battery_info_update(BAT_REG_AC_FLAG);
			/* mA -> uA */
			val->intval = power_info->current_average * 1000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			loongson_power_battery_info_update(BAT_REG_VOLTAGE_FLAG);
			/* mV -> uV */
			val->intval =  power_info->voltage_now * 1000;
			break;
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			loongson_power_battery_info_update(BAT_REG_RC_FLAG);
			/* mAh -> uAh */
			val->intval = power_info->remain_capacity * 1000;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			loongson_power_battery_info_update(BAT_REG_RSOC_FLAG);
			/* Percentage */
			val->intval = power_info->remain_capacity_percent;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			loongson_power_battery_info_update(BAT_REG_TEMP_FLAG);
			/* Celcius */
			val->intval = power_info->temperature;
			break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			loongson_power_battery_info_update(BAT_REG_ATTE_FLAG);
			if (power_info->remain_time == 0xFFFF) {
				power_info->remain_time = 0;
			}
			/* seconds */
			val->intval = power_info->remain_time * 60;
			break;
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
			loongson_power_battery_info_update(BAT_REG_ATTF_FLAG);
			if (power_info->fullchg_time == 0xFFFF) {
				power_info->fullchg_time = 0;
			}
			/* seconds */
			val->intval = power_info->fullchg_time * 60;
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			loongson_power_battery_info_update(BAT_REG_FCC_FLAG);
			/* mAh -> uAh */
			val->intval = power_info->full_charged_capacity * 1000;
			break;
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			loongson_power_battery_info_update(BAT_REG_CYCLCNT_FLAG);
			val->intval = power_info->cycle_count;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

/* Power supply AC get property handler */
static int loongson_ac_get_property(struct power_supply * pws,
			enum power_supply_property psp, union power_supply_propval * val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			loongson_power_info_power_status_update();
			val->intval = power_info->ac_in;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

/* SCI device AC event handler */
static int loongson_ac_handler(int status)
{
	loongson_power_info_power_status_update();
	/* Report status changed */
	power_supply_changed(three_nod_ac);

	return 0;
}

/* SCI device Battery event handler */
static int loongson_bat_handler(int status)
{
	/* Battery insert/pull-out to handle battery static information. */
	if (status & MASK(BIT_POWER_BATPRES)) {
		/* If battery is insert, get battery static information. */
		loongson_power_info_battery_static_update();
	}
	else {
		/* Else if battery is pull-out, clear battery static information. */
		loongson_power_info_battery_static_clear();
	}
	/* Report status changed */
	power_supply_changed(three_nod_bat);

	return 0;
}

/* SCI device LID event handler */
static int loongson_lid_handler(int status)
{
	if (status & BIT(BIT_LIDSTS)) {
		return 0;
	}

	return 1;
}

/* Hotkey device init */
static int loongson_hotkey_init(void)
{
	int ret;

	loongson_hotkey_dev = input_allocate_device();
	if (!loongson_hotkey_dev)
		return -ENOMEM;

	loongson_hotkey_dev->name = "Loongson-3nod PM Hotkey Hotkeys";
	loongson_hotkey_dev->phys = "button/input0";
	loongson_hotkey_dev->id.bustype = BUS_HOST;
	loongson_hotkey_dev->dev.parent = NULL;

	ret = sparse_keymap_setup(loongson_hotkey_dev, loongson_keymap, NULL);
	if (ret) {
		printk(KERN_ERR "Loongson-3nod PM Hotkey Platform Driver: Fail to setup input device keymap\n");
		input_free_device(loongson_hotkey_dev);

		return ret;
	}

	ret = input_register_device(loongson_hotkey_dev);
	if (ret) {
		input_free_device(loongson_hotkey_dev);

		return ret;
	}
	return 0;
}

/* Hotkey device exit */
static void loongson_hotkey_exit(void)
{
	if (loongson_hotkey_dev) {
		input_unregister_device(loongson_hotkey_dev);
		loongson_hotkey_dev = NULL;
	}
}
#ifdef CONFIG_PM
/* Platform device suspend handler */
static int loongson_laptop_suspend(struct platform_device * pdev, pm_message_t state)
{
	return 0;
}

/* Platform device resume handler */
static int loongson_laptop_resume(struct platform_device * pdev)
{
	/* Process LID event */
	loongson_3nod_sci_hotkey_handler(SCI_EVENT_NUM_LID);

	/* Update the power statu when resume */
	if(is_3nod_laptop())
		power_supply_changed(three_nod_ac);

	return 0;
}
#else
static int loongson_laptop_suspend(struct platform_device * pdev, pm_message_t state)
{
	return 0;
}

static int loongson_laptop_resume(struct platform_device * pdev)
{
	return 0;
}
#endif /* CONFIG_PM */

static int loongson_sci_event_probe(struct platform_device *dev)
{
	int ret = 0;

	printk(KERN_INFO "Loongson-3nod: in probe!\n");
	if (loongson_hotkey_init()) {
		printk(KERN_ERR "Loongson-3nod Platform Driver: Hotkey init fail.\n");
		goto fail_hotkey_init;
	}

	if (is_3nod_laptop()) {
		/* Register power supply START */
		power_info = kzalloc(sizeof(struct loongson_power_info), GFP_KERNEL);
		if (!power_info) {
			printk(KERN_ERR "Loongson-3nod Platform Driver: Alloc memory for power_info failed!\n");
			ret = -ENOMEM;
			goto fail_power_info_alloc;
		}

		loongson_power_info_power_status_update();
		if (power_info->bat_in) {
			/* Get battery static information. */
			loongson_power_info_battery_static_update();
		}
		else {
			printk(KERN_ERR "Loongson-3nod Platform Driver: The battery does not exist!!\n");
		}
		three_nod_bat = power_supply_register(NULL, &loongson_bat, NULL);
		if (IS_ERR(three_nod_bat)) {
			ret = -ENOMEM;
			goto fail_bat_power_supply_register;
		}

		three_nod_ac = power_supply_register(NULL, &loongson_ac, NULL);
		if (IS_ERR(three_nod_ac)) {
			ret = -ENOMEM;
			goto fail_ac_power_supply_register;
		}
		/* Register power supply END */
	}

	/* SCI PCI Driver Init Start */
	ret = sci_pci_driver_init();
	if (ret) {
		printk(KERN_ERR "Loongson-3nod: sci pci driver init fail.\n");
		goto fail_sci_pci_driver_init;
	}
	return 0;

fail_hotkey_init:
fail_sci_pci_driver_init:
	loongson_hotkey_exit();
fail_ac_power_supply_register:
	if(is_3nod_laptop())
		power_supply_unregister(three_nod_bat);
fail_bat_power_supply_register:
	if(is_3nod_laptop())
		kfree(power_info);
fail_power_info_alloc:
	platform_driver_unregister(&loongson_3nod_pdriver);
	return -1;
}

/* Loongson-3nod notebook platfrom init entry */
static int __init loongson_3nod_laptop_init(void)
{
	int ret;

	if (!dmi_check_system(loongson_device_table)) {
		const struct  dmi_device *oem_string = dmi_find_device(DMI_DEV_TYPE_OEM_STRING, NULL, NULL);
		printk(KERN_INFO "Loongson-3nod Laptop not match from device_table.\n");
		if(oem_string != NULL){
			if(strncmp(OEM_STRING, oem_string->name,strlen(OEM_STRING))){
				printk(KERN_ERR "Loongson-3nod Laptop not dmimatch devices!\n");
				return -ENODEV;
			}
			pr_info("Identified 3nod device model '%s'\n", OEM_STRING);
			quirks = &quirk_3nod_ls7a_laptop;
		}else{
			printk(KERN_ERR "Loongson-3nod Laptop not dmimatch devices!\n");
			return -ENODEV;
		}
	}

	dmi_checked = 1;

	printk(KERN_INFO "Loongson-3nod PM Hotkey Platform Driver:regist loongson platform driver begain.\n");

	ret = platform_driver_register(&loongson_3nod_pdriver);
	if (ret) {
		printk(KERN_ERR "Loongson-3nod PM Hotkey Platform Driver: Fail regist loongson platform driver.\n");
		return ret;
	}

	ret = driver_create_file(&loongson_3nod_pdriver.driver, &driver_attr_version);
	if (ret) {
		printk(KERN_ERR "Loongson-3nod creat file fail!!\n");
		return ret;
	}

	platform_device = platform_device_alloc("3nod_pm_hotkey", -1);
	if (!platform_device) {
		ret = -ENOMEM;
		goto fail_platform_device1;
	}

	ret = platform_device_add(platform_device);
	if (ret) {
		goto fail_platform_device2;
	}

	return ret;

fail_platform_device2:
	platform_device_put(platform_device);
fail_platform_device1:
	platform_driver_unregister(&loongson_3nod_pdriver);

	return ret;

}

static void __exit loongson_3nod_laptop_exit(void)
{
	platform_driver_unregister(&loongson_3nod_pdriver);
}

module_init(loongson_3nod_laptop_init);
module_exit(loongson_3nod_laptop_exit);

MODULE_AUTHOR("lichao@loongson.cn; lvyanbing@loongson.cn");
MODULE_DESCRIPTION("Loongson-3nod PM Hotkey Driver");
MODULE_LICENSE("GPL");
