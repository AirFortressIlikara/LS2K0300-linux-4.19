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
#include <ec_npce985x.h>
#include <linux/dmi.h>

#define EC_SCI_DEV	"sci"

static struct platform_device *platform_device;


struct quirk_entry {
	int sci_irq_num;
};

static struct quirk_entry *quirks;

static struct quirk_entry quirk_czc_rs780e = {
	.sci_irq_num = 3,
};

static struct quirk_entry quirk_czc_ls7a = {
	.sci_irq_num = 123,
};

static int dmi_check_cb(const struct dmi_system_id *dmi)
{
	pr_info("Identified laptop model '%s'\n", dmi->ident);

	quirks = dmi->driver_data;

	return 1;
}

static const struct dmi_system_id loongson_device_table[] __initconst = {
	{
		.ident = "RS78OE laptop",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Loongson Technology Co.,Ltd."),
			DMI_MATCH(DMI_PRODUCT_NAME, "CZC-LS3A3000-RS780E-laptop"),
			DMI_MATCH(DMI_CHASSIS_TYPE, "9"),
		},
		.callback = dmi_check_cb,
		.driver_data = &quirk_czc_rs780e,
	},
	{
		.ident = "LS7A laptop",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Loongson"),
			DMI_MATCH(DMI_PRODUCT_NAME, "CZC-LS3A3000-LS7A-laptop"),
			DMI_MATCH(DMI_CHASSIS_TYPE, "9"),
		},
		.callback = dmi_check_cb,
		.driver_data = &quirk_czc_ls7a,
	},
	{}
};
MODULE_DEVICE_TABLE(dmi, loongson_device_table);

extern struct board_devices *eboard;

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

enum /* bat_reg_flag */
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
#define MAX_BRIGHTNESS	9

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
	unsigned char manufacturer_name[12];
	/* Battery Device Name, max 7 + 1(length) bytes */
	unsigned char device_name[8];
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

const static char *version = "1.11";

extern void prom_printf(char *fmt, ...);
extern int ec985x_query_get_event_num(void);
/* SCI device object */
static struct sci_device *loongson_sci_device = NULL;
static int loongson_sci_event_probe(struct platform_device *);
static ssize_t version_show(struct device_driver *, char *);
static int loongson_get_brightness(struct backlight_device *);
static int loongson_set_brightness(struct backlight_device *);
/* >>>Power management operation */
/* Update battery information handle function. */
static void loongson_power_battery_info_update(unsigned char bat_reg_flag);
/* Clear battery static information. */
static void loongson_power_info_battery_static_clear(void);
/* Get battery static information. */
static void loongson_power_info_battery_static_update(void);
/* Update power_status value */
static void loongson_power_info_power_status_update(void);
static void loongson_bat_get_string(unsigned char index, unsigned char *bat_string);
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

/* <<<End power management operation */

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
	[SCI_EVENT_NUM_LID] =			{INDEX_POWER_STATUS, loongson_lid_handler},
	[SCI_EVENT_NUM_BRIGHTNESS_DN] = {0, NULL},
	[SCI_EVENT_NUM_BRIGHTNESS_UP] = {0, NULL},
	[SCI_EVENT_NUM_DISPLAY_TOGGLE] ={0, NULL},
	[SCI_EVENT_NUM_SLEEP] =			{0, NULL},
	[SCI_EVENT_NUM_WLAN] =			{0, NULL},
	[SCI_EVENT_NUM_TP] =			{0, NULL},
	[SCI_EVENT_NUM_POWERBTN] =		{0, NULL},
	[SCI_EVENT_NUM_AC] =			{0, loongson_ac_handler},
	[SCI_EVENT_NUM_BAT] =			{INDEX_POWER_STATUS, loongson_bat_handler},
};
static const struct key_entry loongson_keymap[] = {
	/* LID event */
	{KE_SW,  SCI_EVENT_NUM_LID, {SW_LID}},
	/* Fn+F4, Sleep */
	{KE_KEY, SCI_EVENT_NUM_SLEEP, {KEY_SLEEP}},
	/* Fn+F5, toutchpad on/off */
	{KE_KEY, SCI_EVENT_NUM_TP, {KEY_TOUCHPAD_TOGGLE}},
	/* Fn+F3, Brightness off */
	{KE_KEY, SCI_EVENT_NUM_BRIGHTNESS_OFF, {KEY_DISPLAYTOGGLE} },
	/* Fn+F1, Brightness down */
	{KE_KEY, SCI_EVENT_NUM_BRIGHTNESS_DN, {KEY_BRIGHTNESSDOWN}},
	/* Fn+F2, Brightness up */
	{KE_KEY, SCI_EVENT_NUM_BRIGHTNESS_UP, {KEY_BRIGHTNESSUP}},
	/*Fn+F7, Dispaly switch */
	{KE_KEY, SCI_EVENT_NUM_DISPLAY_TOGGLE, {KEY_SWITCHVIDEOMODE}},
	/*Fn+F6, WLAN on/off */
	{KE_KEY, SCI_EVENT_NUM_WLAN, {KEY_WLAN}},
	/* Power button */
	{KE_KEY, SCI_EVENT_NUM_POWERBTN, {KEY_POWER}},
	{KE_END, 0}
};

static struct platform_driver loongson_czc_pdriver = {
	.probe = loongson_sci_event_probe,
	.driver = {
		.name = "czc_pm_hotkey",
		.owner = THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend = loongson_laptop_suspend,
	.resume  = loongson_laptop_resume,
#endif /* CONFIG_PM */
};

/* Backlight device object */
static struct backlight_device * loongson_backlight_dev = NULL;
/* Backlight device operations table object */
static struct backlight_ops loongson_backlight_ops =
{
	.get_brightness = loongson_get_brightness,
	.update_status = loongson_set_brightness,
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
	POWER_SUPPLY_PROP_CHARGE_FULL, /* in uAh */
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY, /* in percents! */
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
static struct power_supply *czc_bat;

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
static struct power_supply *czc_ac;

static DRIVER_ATTR_RO(version);

static ssize_t version_show(struct device_driver *driver, char *buf)
{
	return sprintf(buf, "%s\n", version);
}

/* SCI device event handler */
void loongson_sci_hotkey_handler(int event)
{
	int status = 0;
	struct key_entry * ke = NULL;
	struct sci_event * sep = NULL;

	sep = (struct sci_event*)&(se[event]);
	if(0 != sep->index)
	{
		status = ec985x_read(sep->index);
	}
	if(NULL != sep->handler)
	{
		status = sep->handler(status);
	}

	ke = sparse_keymap_entry_from_scancode(loongson_hotkey_dev, event);
	if(ke)
	{
		if(SW_LID == ke->keycode)
		{
			/* report LID event. */
			input_report_switch(loongson_hotkey_dev, SW_LID, status);
			input_sync(loongson_hotkey_dev);
		}
		else
		{
			sparse_keymap_report_entry(loongson_hotkey_dev, ke, 1, true);
		}
	}
}


static irqreturn_t loongson_sci_int_routine(int irq, void *dev_id)
{
	int event;
	if (loongson_sci_device->irq != irq)
		return IRQ_NONE;

	/* Clean sci irq */
	clean_ec985x_event_status();

	event = ec985x_query_get_event_num();
	if ((SCI_EVENT_NUM_AC > event) || (SCI_EVENT_NUM_POWERBTN < event))
		goto exit_event_action;
	loongson_sci_hotkey_handler(event);
	return IRQ_HANDLED;

exit_event_action:
	clean_ec985x_event_status();
	return IRQ_HANDLED;
}

/* SCI driver pci driver init */
static int sci_pci_init(void)
{
	struct irq_fwspec fwspec;
	int ret = -EIO;
	struct pci_dev *pdev;

	printk(KERN_INFO "Loongson-czc: SCI PCI init.\n");

	loongson_sci_device = kmalloc(sizeof(struct sci_device), GFP_KERNEL);
	if (!loongson_sci_device) {
		printk(KERN_ERR "Loongson-czc: Malloc mem space for sci_dvice failed.\n");
		return -ENOMEM;
	}

	if (strstr(eboard->name, "B20-"))
		loongson_sci_device->irq = 3;
	else {
		loongson_sci_device->irq = quirks->sci_irq_num;
		fwspec.fwnode = NULL;
		fwspec.param[0] = loongson_sci_device->irq;
		fwspec.param_count = 1;
		ret = irq_create_fwspec_mapping(&fwspec);
		if (ret >= 0) {
			loongson_sci_device->irq = ret;
			irq_set_irq_type(ret, IRQ_TYPE_LEVEL_LOW);
		}
	}
	printk(KERN_INFO "Loongson-czc: SCI irq_num:%d.\n", loongson_sci_device->irq);
	loongson_sci_device->irq_data = 0x00;
	loongson_sci_device->number = 0x00;
	loongson_sci_device->parameter = 0x00;
	strcpy(loongson_sci_device->name, EC_SCI_DEV);

	/* Enable pci device and get the GPIO resources. */
	if (strstr(eboard->name, "B20-")) {
		pdev = pci_get_device(PCI_VENDOR_ID_ATI, PCI_DEVICE_ID_ATI_SBX00_SMBUS, NULL);
		if (ret = pci_enable_device(pdev)) {
			printk(KERN_ERR "Loongson-czc: Enable pci device fail.\n");
			ret = -ENODEV;
			goto out_pdev;
		}
	}

	clean_ec985x_event_status();

	/* Regist pci irq */
	ret = request_irq(loongson_sci_device->irq, loongson_sci_int_routine,
				IRQF_SHARED, loongson_sci_device->name, loongson_sci_device);
	if (ret) {
		printk(KERN_ERR "Loongson-czc: Request irq fail.\n");
		ret = -EFAULT;
		goto out_irq;
	}

	ret = 0;
	printk(KERN_INFO "Loongson-czc: SCI PCI init successful.\n");
	return ret;

out_irq:
	pci_disable_device(pdev);
out_pdev:
	kfree(loongson_sci_device);
	return ret;
}

/* SCI drvier pci driver init handler */
static int sci_pci_driver_init(void)
{
	int ret;
	ret = sci_pci_init();
	if (ret) {
		printk(KERN_ERR "Loongson-czc: SCI Regist pci driver fail.\n");
		return ret;
	}
	printk(KERN_INFO "Loongson-czc: SCI regist pci driver done.\n");
	return ret;
}

/* Backlight device set brightness handler */
static int loongson_set_brightness(struct backlight_device * pdev)
{
	unsigned int level = 0;

	level = ((FB_BLANK_UNBLANK==pdev->props.fb_blank) &&
				(FB_BLANK_UNBLANK==pdev->props.power)) ?
					pdev->props.brightness : 0;

	if(MAX_BRIGHTNESS < level)
	{
		level = MAX_BRIGHTNESS;
	}
	else if(level < 0)
	{
		level = 0;
	}

	ec985x_write(INDEX_DISPLAY_BRIGHTNESS, level);

	return 0;
}

/* Backlight device get brightness handler */
static int loongson_get_brightness(struct backlight_device * pdev)
{
	/* Read level from ec */
	return ec985x_read(INDEX_DISPLAY_BRIGHTNESS);
}

/* Update battery information handle function. */
static void loongson_power_battery_info_update(unsigned char bat_reg_flag)
{
	short bat_info_value = 0;

	switch (bat_reg_flag) {
		/* Update power_info->temperature value */
		case BAT_REG_TEMP_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (ec985x_read(INDEX_BATTERY_TEMP_HIGH) << 8) | ec985x_read(INDEX_BATTERY_TEMP_LOW);
			power_info->temperature = (power_info->bat_in) ? (bat_info_value / 10 - 273) : 0;
			break;
		/* Update power_info->voltage value */
		case BAT_REG_VOLTAGE_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (ec985x_read(INDEX_BATTERY_VOL_HIGH) << 8) | ec985x_read(INDEX_BATTERY_VOL_LOW);
			power_info->voltage_now = (power_info->bat_in) ? bat_info_value : 0;
			break;
		/* Update power_info->current_now value */
		case BAT_REG_CURRENT_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (ec985x_read(INDEX_BATTERY_CURRENT_HIGH) << 8) | ec985x_read(INDEX_BATTERY_CURRENT_LOW);
			power_info->current_now = (power_info->bat_in) ? bat_info_value : 0;
			break;
		/* Update power_info->current_avg value */
		case BAT_REG_AC_FLAG:
			loongson_power_info_power_status_update();
			bat_info_value = (ec985x_read(INDEX_BATTERY_AC_HIGH) << 8) | ec985x_read(INDEX_BATTERY_AC_LOW);
			power_info->current_average = (power_info->bat_in) ? bat_info_value : 0;
			break;
		/* Update power_info->remain_capacity value */
		case BAT_REG_RC_FLAG:
			power_info->remain_capacity = (ec985x_read(INDEX_BATTERY_RC_HIGH) << 8) | ec985x_read(INDEX_BATTERY_RC_LOW);
			break;
		/* Update power_info->full_charged_capacity value */
		case BAT_REG_FCC_FLAG:
			power_info->full_charged_capacity = (ec985x_read(INDEX_BATTERY_FCC_HIGH) << 8) | ec985x_read(INDEX_BATTERY_FCC_LOW);
			break;
		/* Update power_info->remain_time value */
		case BAT_REG_ATTE_FLAG:
			power_info->remain_time = (ec985x_read(INDEX_BATTERY_ATTE_HIGH) << 8) | ec985x_read(INDEX_BATTERY_ATTE_LOW);
			break;
		/* Update power_info->fullchg_time value */
		case BAT_REG_ATTF_FLAG:
			power_info->fullchg_time = (ec985x_read(INDEX_BATTERY_ATTF_HIGH) << 8) | ec985x_read(INDEX_BATTERY_ATTF_LOW);
			break;
		/* Update power_info->curr_cap value */
		case BAT_REG_RSOC_FLAG:
			power_info->remain_capacity_percent = ec985x_read(INDEX_BATTERY_CAPACITY);
			break;
		/* Update power_info->cycle_count value */
		case BAT_REG_CYCLCNT_FLAG:
			power_info->cycle_count = (ec985x_read(INDEX_BATTERY_CYCLECNT_HIGH) << 8) | ec985x_read(INDEX_BATTERY_CYCLECNT_LOW);
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
	unsigned int manufacture_date, bat_serial_number;
	char device_chemistry[5];

	if ((device_chemistry[2] == 'o') || (device_chemistry[2] == 'O')) {
		power_info->technology = POWER_SUPPLY_TECHNOLOGY_LION;
	}
	else if (((device_chemistry[1] = 'h') && (device_chemistry[2] == 'm')) ||
			((device_chemistry[1] = 'H') && (device_chemistry[2] == 'M'))) {
		power_info->technology = POWER_SUPPLY_TECHNOLOGY_NiMH;
	}
	else if ((device_chemistry[2] == 'p') || (device_chemistry[2] == 'P')) {
		power_info->technology = POWER_SUPPLY_TECHNOLOGY_LIPO;
	}
	else if ((device_chemistry[2] == 'f') || (device_chemistry[2] == 'F')) {
		power_info->technology = POWER_SUPPLY_TECHNOLOGY_LiFe;
	}
	else if ((device_chemistry[2] == 'c') || (device_chemistry[2] == 'C')) {
		power_info->technology = POWER_SUPPLY_TECHNOLOGY_NiCd;
	}
	else if (((device_chemistry[1] = 'n') && (device_chemistry[2] == 'm')) ||
			((device_chemistry[1] = 'N') && (device_chemistry[2] == 'M'))) {
		power_info->technology = POWER_SUPPLY_TECHNOLOGY_LiMn;
	}
	else {
		power_info->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	power_info->technology = POWER_SUPPLY_TECHNOLOGY_LION;

	bat_serial_number = (ec985x_read(INDEX_BATTERY_SN_HIGH) << 8) | ec985x_read(INDEX_BATTERY_SN_LOW);
	snprintf(power_info->serial_number, 8, "%x", bat_serial_number);

	power_info->cell_count = ((ec985x_read(INDEX_BATTERY_CV_HIGH) << 8) | ec985x_read(INDEX_BATTERY_CV_LOW)) / 4200;

	power_info->design_capacity = (ec985x_read(INDEX_BATTERY_DC_HIGH) << 8) | ec985x_read(INDEX_BATTERY_DC_LOW);
	power_info->design_voltage = (ec985x_read(INDEX_BATTERY_DV_HIGH) << 8) | ec985x_read(INDEX_BATTERY_DV_LOW);
	power_info->full_charged_capacity = (ec985x_read(INDEX_BATTERY_FCC_HIGH) << 8) | ec985x_read(INDEX_BATTERY_FCC_LOW);
	printk(KERN_INFO "DesignCapacity: %dmAh, DesignVoltage: %dmV, FullChargeCapacity: %dmAh\n",
		power_info->design_capacity, power_info->design_voltage, power_info->full_charged_capacity);
}

/* Update power_status value */
static void loongson_power_info_power_status_update(void)
{
	unsigned int power_status = 0;

	power_status = ec985x_read(INDEX_POWER_STATUS);

	power_info->ac_in = (power_status & MASK(BIT_POWER_ACPRES)) ?
					APM_AC_ONLINE : APM_AC_OFFLINE;

	power_info->bat_in = (power_status & MASK(BIT_POWER_BATPRES)) ? 1 : 0;
	if( power_info->bat_in && ((ec985x_read(INDEX_BATTERY_DC_LOW) | (ec985x_read(INDEX_BATTERY_DC_HIGH) << 8)) == 0) )
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

/* Get battery static information string */
static void loongson_bat_get_string(unsigned char index, unsigned char *bat_string)
{
	unsigned char length, i;

	length = ec985x_read(index);
	for (i = 0; i < length; i++) {
		*bat_string++ = ec985x_read(++index);
	}
	*bat_string = '\0';
}

/* Power supply Battery get property handler */
static int loongson_bat_get_property(struct power_supply * pws,
			enum power_supply_property psp, union power_supply_propval * val)
{
	switch (psp) {
		/* Get battery static information. */
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = power_info->design_voltage * 1000; /* mV -> uV */
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = power_info->design_capacity * 1000; /* mAh -> uAh */
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
			val->intval = power_info->current_now * 1000; /* mA -> uA */
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			loongson_power_battery_info_update(BAT_REG_AC_FLAG);
			val->intval = power_info->current_average * 1000; /* mA -> uA */
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			loongson_power_battery_info_update(BAT_REG_VOLTAGE_FLAG);
			val->intval =  power_info->voltage_now * 1000; /* mV -> uV */
			break;
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			loongson_power_battery_info_update(BAT_REG_RC_FLAG);
			val->intval = power_info->remain_capacity * 1000; /* mAh -> uAh */
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			loongson_power_battery_info_update(BAT_REG_RSOC_FLAG);
			val->intval = power_info->remain_capacity_percent;	/* Percentage */
			break;
		case POWER_SUPPLY_PROP_TEMP:
			loongson_power_battery_info_update(BAT_REG_TEMP_FLAG);
			val->intval = power_info->temperature;	 /* Celcius */
			break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			loongson_power_battery_info_update(BAT_REG_ATTE_FLAG);
			if (power_info->remain_time == 0xFFFF) {
				power_info->remain_time = 0;
			}
			val->intval = power_info->remain_time * 60;  /* seconds */
			break;
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
			loongson_power_battery_info_update(BAT_REG_ATTF_FLAG);
			if (power_info->fullchg_time == 0xFFFF) {
				power_info->fullchg_time = 0;
			}
			val->intval = power_info->fullchg_time * 60;  /* seconds */
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			loongson_power_battery_info_update(BAT_REG_FCC_FLAG);
			val->intval = power_info->full_charged_capacity * 1000;/* mAh -> uAh */
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
	/* Report status changed */
	power_supply_changed(czc_ac);

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
	power_supply_changed(czc_bat);

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
	if(!loongson_hotkey_dev)
		return -ENOMEM;

	loongson_hotkey_dev->name = "Loongson-czc Laptop Hotkeys";
	loongson_hotkey_dev->phys = "button/input0";
	loongson_hotkey_dev->id.bustype = BUS_HOST;
	loongson_hotkey_dev->dev.parent = NULL;

	ret = sparse_keymap_setup(loongson_hotkey_dev, loongson_keymap, NULL);
	if(ret)
	{
		printk(KERN_ERR "Loongson-czc Laptop Platform Driver: Fail to setup input device keymap\n");
		input_free_device(loongson_hotkey_dev);

		return ret;
	}

	ret = input_register_device(loongson_hotkey_dev);
	if(ret)
	{
		//sparse_keymap_free(loongson_hotkey_dev);
		input_free_device(loongson_hotkey_dev);

		return ret;
	}
	return 0;
}

/* Hotkey device exit */
static void loongson_hotkey_exit(void)
{
	if(loongson_hotkey_dev) {
		//sparse_keymap_free(loongson_hotkey_dev);
		input_unregister_device(loongson_hotkey_dev);
		loongson_hotkey_dev = NULL;
	}
}
#ifdef CONFIG_PM
/* Platform device suspend handler */
static int loongson_laptop_suspend(struct platform_device * pdev, pm_message_t state)
{
	struct pci_dev *dev;

	if (strstr(eboard->name, "B20-")) {
		dev = pci_get_device(PCI_VENDOR_ID_ATI, PCI_DEVICE_ID_ATI_SBX00_SMBUS, NULL);
		pci_disable_device(dev);
	}
	return 0;
}

/* Platform device resume handler */
static int loongson_laptop_resume(struct platform_device * pdev)
{
	struct pci_dev *dev;

	if (strstr(eboard->name, "B20-")) {
		dev = pci_get_device(PCI_VENDOR_ID_ATI, PCI_DEVICE_ID_ATI_SBX00_SMBUS, NULL);
		pci_enable_device(dev);
	}
	/* Process LID event */
	loongson_sci_hotkey_handler(SCI_EVENT_NUM_LID);

	/*
	 * Clear sci status: GPM9Status field in bit14 of
	 * EVENT_STATUS register for SB710, write 1 to clear
	 *
	 * Clear all SCI events when suspend
	 */
	clean_ec985x_event_status();

	/* Update the power statu when resume */
	power_supply_changed(czc_ac);

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
	printk(KERN_INFO "Loongson-czc: in probe!\n");

	/* Regist backlight */
	loongson_backlight_dev = backlight_device_register("loongson", NULL, NULL,
			&loongson_backlight_ops, NULL);
	if (IS_ERR(loongson_backlight_dev)) {
		ret = PTR_ERR(loongson_backlight_dev);
		goto fail_backlight_device_register;
	}
	loongson_backlight_dev->props.max_brightness = 10;
	loongson_backlight_dev->props.brightness = ec985x_read(INDEX_DISPLAY_BRIGHTNESS);
	backlight_update_status(loongson_backlight_dev);

	if (loongson_hotkey_init()) {
		printk(KERN_ERR "Loongson-czc Platform Driver: Hotkey init fail.\n");
		goto fail_hotkey_init;
	}

	/* Register power supply START */
	power_info = kzalloc(sizeof(struct loongson_power_info), GFP_KERNEL);
	if (!power_info) {
		printk(KERN_ERR "Loongson-czc Platform Driver: Alloc memory for power_info failed!\n");
		ret = -ENOMEM;
		goto fail_power_info_alloc;
	}

	loongson_power_info_power_status_update();
	if (power_info->bat_in) {
		/* Get battery static information. */
		loongson_power_info_battery_static_update();
	}
	else {
		printk(KERN_ERR "Loongson-czc Platform Driver: The battery does not exist!!\n");
	}
	czc_bat = power_supply_register(NULL, &loongson_bat, NULL);
	if (IS_ERR(czc_bat)) {
		ret = -ENOMEM;
		goto fail_bat_power_supply_register;
	}

	czc_ac = power_supply_register(NULL, &loongson_ac, NULL);
	if (IS_ERR(czc_ac)) {
		ret = -ENOMEM;
		goto fail_ac_power_supply_register;
	}
	/* Register power supply END */

	/* SCI PCI Driver Init Start */
	ret = sci_pci_driver_init();
	if (ret) {
		printk(KERN_ERR "Loongosn-czc: sci pci driver init fail.\n");
		goto fail_sci_pci_driver_init;
	}
	return 0;

fail_hotkey_init:
fail_sci_pci_driver_init:
	loongson_hotkey_exit();
fail_ac_power_supply_register:
	power_supply_unregister(czc_bat);
fail_bat_power_supply_register:
	kfree(power_info);
fail_power_info_alloc:
	backlight_device_unregister(loongson_backlight_dev);
fail_backlight_device_register:
	platform_driver_unregister(&loongson_czc_pdriver);
	return -ENXIO;
}

/* Loongson-czc notebook platfrom init entry */
static int __init loongson_czc_laptop_init(void)
{
	int ret;

	if (!strstr(eboard->name, "B20-")) {
		if ((!dmi_check_system(loongson_device_table))) {
			printk(KERN_ERR "Loongson-czc Laptop not dmimatch devices! \n");
			return -ENODEV;
		}
	}

	printk(KERN_INFO "Loongson-czc Laptop dmimatch devices success! \n");
	printk(KERN_INFO "Loongson-czc Laptop Platform Driver:regist loongson platform driver begain.\n");
	ret = platform_driver_register(&loongson_czc_pdriver);
	if (ret) {
		printk(KERN_ERR "Loongson-czc Laptop Platform Driver: Fail regist loongson platform driver.\n");
		return ret;
	}

	ret = driver_create_file(&loongson_czc_pdriver.driver, &driver_attr_version);
	if (ret)
		printk(KERN_ERR "Loongson-czc creat file fail!!\n");
	printk(KERN_INFO "Loongson-czc: ret = %d\n", ret);

	platform_device = platform_device_alloc("czc_pm_hotkey", -1);
	if (!platform_device) {
		ret = -ENOMEM;
		goto fail_platform_device1;
	}
	ret = platform_device_add(platform_device);
	if (ret)
		goto fail_platform_device2;

	return ret;

fail_platform_device2:
	platform_device_put(platform_device);
fail_platform_device1:
	platform_driver_unregister(&loongson_czc_pdriver);

	return ret;
}

static void __exit loongson_czc_laptop_exit(void)
{
	platform_device_unregister(platform_device);
	platform_driver_unregister(&loongson_czc_pdriver);
}

module_init(loongson_czc_laptop_init);
module_exit(loongson_czc_laptop_exit);

MODULE_AUTHOR("LI Chao<lichao@loongosn.cn>; Wang Yulong<wangyulong@loongson.cn>");
MODULE_DESCRIPTION("Loongson-czc Laptop Driver");
MODULE_LICENSE("GPL");
