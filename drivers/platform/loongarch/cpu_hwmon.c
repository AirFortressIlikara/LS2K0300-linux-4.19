// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/err.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/acpi.h>

#include <loongson.h>
#include <boot_param.h>
#include <larchintrin.h>

/*
 * Loongson-3 series cpu has two sensors inside,
 * each of them from 0 to 255,
 * if more than 127, that is dangerous.
 * here only provide sensor1 data, because it always hot than sensor0
 */
#if (!CONFIG_LS2K300_PLATFORM)
int loongson3_cpu_temp(int cpu)
{
	int cputemp;
	u32 reg;

	reg = LOONGSON_CHIPTEMP(cpu);
	reg = (reg & 0xffff)*731/0x4000 - 273;
	cputemp = (int)reg * 1000;

	return cputemp;
}
EXPORT_SYMBOL(loongson3_cpu_temp);
#else
int ls2k300_cpu_temp(int cpu)
{
	int cputemp;
	u32 reg;

	/* temp = value * 0.57 - 394.7 */
	reg = readl(TO_UNCAC(0x16001514));
	reg = (reg & 0x7ff) * 100;
	reg = reg * 57 / 100 - 39470;
	cputemp = (int)reg / 100;

	return cputemp;
}
#endif

static int nr_packages;
static struct device *cpu_hwmon_dev;

static ssize_t get_hwmon_name(struct device *dev,
			struct device_attribute *attr, char *buf);
static SENSOR_DEVICE_ATTR(name, S_IRUGO, get_hwmon_name, NULL, 0);

static struct attribute *cpu_hwmon_attributes[] = {
	&sensor_dev_attr_name.dev_attr.attr,
	NULL
};

/* Hwmon device attribute group */
static struct attribute_group cpu_hwmon_attribute_group = {
	.attrs = cpu_hwmon_attributes,
};

/* Hwmon device get name */
static ssize_t get_hwmon_name(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "cpu-hwmon\n");
}

static ssize_t get_cpu_temp(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t cpu_temp_label(struct device *dev,
			struct device_attribute *attr, char *buf);

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, get_cpu_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, cpu_temp_label, NULL, 1);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, get_cpu_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, cpu_temp_label, NULL, 2);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, get_cpu_temp, NULL, 3);
static SENSOR_DEVICE_ATTR(temp3_label, S_IRUGO, cpu_temp_label, NULL, 3);
static SENSOR_DEVICE_ATTR(temp4_input, S_IRUGO, get_cpu_temp, NULL, 4);
static SENSOR_DEVICE_ATTR(temp4_label, S_IRUGO, cpu_temp_label, NULL, 4);
static SENSOR_DEVICE_ATTR(temp5_input, S_IRUGO, get_cpu_temp, NULL, 5);
static SENSOR_DEVICE_ATTR(temp5_label, S_IRUGO, cpu_temp_label, NULL, 5);
static SENSOR_DEVICE_ATTR(temp6_input, S_IRUGO, get_cpu_temp, NULL, 6);
static SENSOR_DEVICE_ATTR(temp6_label, S_IRUGO, cpu_temp_label, NULL, 6);
static SENSOR_DEVICE_ATTR(temp7_input, S_IRUGO, get_cpu_temp, NULL, 7);
static SENSOR_DEVICE_ATTR(temp7_label, S_IRUGO, cpu_temp_label, NULL, 7);
static SENSOR_DEVICE_ATTR(temp8_input, S_IRUGO, get_cpu_temp, NULL, 8);
static SENSOR_DEVICE_ATTR(temp8_label, S_IRUGO, cpu_temp_label, NULL, 8);
static SENSOR_DEVICE_ATTR(temp9_input, S_IRUGO, get_cpu_temp, NULL, 9);
static SENSOR_DEVICE_ATTR(temp9_label, S_IRUGO, cpu_temp_label, NULL, 9);
static SENSOR_DEVICE_ATTR(temp10_input, S_IRUGO, get_cpu_temp, NULL, 10);
static SENSOR_DEVICE_ATTR(temp10_label, S_IRUGO, cpu_temp_label, NULL, 10);
static SENSOR_DEVICE_ATTR(temp11_input, S_IRUGO, get_cpu_temp, NULL, 11);
static SENSOR_DEVICE_ATTR(temp11_label, S_IRUGO, cpu_temp_label, NULL, 11);
static SENSOR_DEVICE_ATTR(temp12_input, S_IRUGO, get_cpu_temp, NULL, 12);
static SENSOR_DEVICE_ATTR(temp12_label, S_IRUGO, cpu_temp_label, NULL, 12);
static SENSOR_DEVICE_ATTR(temp13_input, S_IRUGO, get_cpu_temp, NULL, 13);
static SENSOR_DEVICE_ATTR(temp13_label, S_IRUGO, cpu_temp_label, NULL, 13);
static SENSOR_DEVICE_ATTR(temp14_input, S_IRUGO, get_cpu_temp, NULL, 14);
static SENSOR_DEVICE_ATTR(temp14_label, S_IRUGO, cpu_temp_label, NULL, 14);
static SENSOR_DEVICE_ATTR(temp15_input, S_IRUGO, get_cpu_temp, NULL, 15);
static SENSOR_DEVICE_ATTR(temp15_label, S_IRUGO, cpu_temp_label, NULL, 15);
static SENSOR_DEVICE_ATTR(temp16_input, S_IRUGO, get_cpu_temp, NULL, 16);
static SENSOR_DEVICE_ATTR(temp16_label, S_IRUGO, cpu_temp_label, NULL, 16);

static const struct attribute *hwmon_cputemp[16][3] = {
	{
		&sensor_dev_attr_temp1_input.dev_attr.attr,
		&sensor_dev_attr_temp1_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp2_input.dev_attr.attr,
		&sensor_dev_attr_temp2_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp3_input.dev_attr.attr,
		&sensor_dev_attr_temp3_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp4_input.dev_attr.attr,
		&sensor_dev_attr_temp4_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp5_input.dev_attr.attr,
		&sensor_dev_attr_temp5_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp6_input.dev_attr.attr,
		&sensor_dev_attr_temp6_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp7_input.dev_attr.attr,
		&sensor_dev_attr_temp7_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp8_input.dev_attr.attr,
		&sensor_dev_attr_temp8_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp9_input.dev_attr.attr,
		&sensor_dev_attr_temp9_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp10_input.dev_attr.attr,
		&sensor_dev_attr_temp10_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp11_input.dev_attr.attr,
		&sensor_dev_attr_temp11_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp12_input.dev_attr.attr,
		&sensor_dev_attr_temp12_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp13_input.dev_attr.attr,
		&sensor_dev_attr_temp13_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp14_input.dev_attr.attr,
		&sensor_dev_attr_temp14_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp15_input.dev_attr.attr,
		&sensor_dev_attr_temp15_label.dev_attr.attr,
		NULL
	},
	{
		&sensor_dev_attr_temp16_input.dev_attr.attr,
		&sensor_dev_attr_temp16_label.dev_attr.attr,
		NULL
	}
};

static ssize_t cpu_temp_label(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int id = (to_sensor_dev_attr(attr))->index - 1;
	return sprintf(buf, "CPU %d Temperature\n", id);
}

static ssize_t get_cpu_temp(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int id = (to_sensor_dev_attr(attr))->index - 1;
#if (!CONFIG_LS2K300_PLATFORM)
	int value = loongson3_cpu_temp(id);
#else
	int value = ls2k300_cpu_temp(id);
#endif
	return sprintf(buf, "%d\n", value);
}

static int create_sysfs_cputemp_files(struct kobject *kobj)
{
	int i, ret = 0;

	for (i=0; i<nr_packages; i++)
		ret = sysfs_create_files(kobj, hwmon_cputemp[i]);

	return ret;
}

static void remove_sysfs_cputemp_files(struct kobject *kobj)
{
	int i;

	for (i=0; i<nr_packages; i++)
		sysfs_remove_files(kobj, hwmon_cputemp[i]);
}

static int cpu_initial_threshold = 64000;
#ifndef CONFIG_FIX_HIGH_TEMP
static int cpu_thermal_threshold = 96000;
#else
static int cpu_thermal_threshold = 128000;
#endif
module_param(cpu_thermal_threshold, int, 0644);
MODULE_PARM_DESC(cpu_thermal_threshold, "cpu thermal threshold (96000 (default))");

static struct delayed_work thermal_work;

static void do_thermal_timer(struct work_struct *work)
{
	int i, value, temp_max = 0;

	for (i=0; i<nr_packages; i++) {
#if (!CONFIG_LS2K300_PLATFORM)
		value = loongson3_cpu_temp(i);
#else
		value = ls2k300_cpu_temp(i);
#endif
		if (value > temp_max)
			temp_max = value;
	}

	if (temp_max <= cpu_thermal_threshold)
		schedule_delayed_work(&thermal_work, msecs_to_jiffies(5000));
	else
		orderly_poweroff(true);
}

#ifdef CONFIG_ACPI
static int find_thermal_dev(struct device *dev, void *data)
{
	struct acpi_device *device = to_acpi_device(dev);
	const char *hid = acpi_device_hid(device);

	return !strcmp(hid, ACPI_THERMAL_HID);
}
#endif

static int __init loongson_hwmon_init(void)
{
	int i, ret, value, temp_max = 0;

#ifdef CONFIG_ACPI
	acpi_status tmp_status, crt_status;
	struct device *thermal_dev;
	struct acpi_device *adev;
	unsigned long long tmp = 0;

	if (!acpi_disabled) {
		thermal_dev = bus_find_device(&acpi_bus_type, NULL, NULL,
						find_thermal_dev);
		if (thermal_dev) {
			adev = to_acpi_device(thermal_dev);
			tmp_status = acpi_evaluate_integer(adev->handle, "_TMP", NULL, &tmp);
			crt_status = acpi_evaluate_integer(adev->handle, "_CRT", NULL, &tmp);
			put_device(thermal_dev);
			if ((tmp_status == 0) && (crt_status == 0)) {
				return 0;
			}
		}
	}
#endif

#if (!CONFIG_LS2K300_PLATFORM)
	if ((iocsr_read32(LOONGARCH_IOCSR_FEATURES) & IOCSRF_TEMP) == 0)
		return -1;
#else
	/* Configure the CPU temperature register */
	writel(0xff03, TO_UNCAC(0x16001518));
#endif

	pr_info("Loongson Hwmon Enter...\n");

	cpu_hwmon_dev = hwmon_device_register(NULL);
	if (IS_ERR(cpu_hwmon_dev)) {
		ret = -ENOMEM;
		pr_err("hwmon_device_register fail!\n");
		goto fail_hwmon_device_register;
	}

	nr_packages = loongson_sysconf.nr_cpus /
		loongson_sysconf.cores_per_package;

	ret = sysfs_create_group(&cpu_hwmon_dev->kobj,
				&cpu_hwmon_attribute_group);
	if (ret) {
		pr_err("fail to create loongson hwmon!\n");
		goto fail_sysfs_create_group_hwmon;
	}

	ret = create_sysfs_cputemp_files(&cpu_hwmon_dev->kobj);
	if (ret) {
		pr_err("fail to create cpu temperature interface!\n");
		goto fail_create_sysfs_cputemp_files;
	}

	for (i=0; i<nr_packages; i++) {
#if (!CONFIG_LS2K300_PLATFORM)
		value = loongson3_cpu_temp(i);
#else
		value = ls2k300_cpu_temp(i);
#endif
		if (value > temp_max)
			temp_max = value;
	}

	pr_info("Initial CPU temperature is %d (highest).\n", temp_max);
#ifndef CONFIG_FIX_HIGH_TEMP
	if (temp_max > cpu_initial_threshold)
		cpu_thermal_threshold += temp_max - cpu_initial_threshold;
#endif

	INIT_DEFERRABLE_WORK(&thermal_work, do_thermal_timer);
	schedule_delayed_work(&thermal_work, msecs_to_jiffies(20000));

	return ret;

fail_create_sysfs_cputemp_files:
	sysfs_remove_group(&cpu_hwmon_dev->kobj,
				&cpu_hwmon_attribute_group);

fail_sysfs_create_group_hwmon:
	hwmon_device_unregister(cpu_hwmon_dev);

fail_hwmon_device_register:
	return ret;
}

static void __exit loongson_hwmon_exit(void)
{
	cancel_delayed_work_sync(&thermal_work);
	remove_sysfs_cputemp_files(&cpu_hwmon_dev->kobj);
	sysfs_remove_group(&cpu_hwmon_dev->kobj,
				&cpu_hwmon_attribute_group);
	hwmon_device_unregister(cpu_hwmon_dev);
}

module_init(loongson_hwmon_init);
module_exit(loongson_hwmon_exit);

MODULE_AUTHOR("Yu Xiang <xiangy@lemote.com>");
MODULE_AUTHOR("Huacai Chen <chenhc@lemote.com>");
MODULE_DESCRIPTION("Loongson CPU Hwmon driver");
MODULE_LICENSE("GPL");
