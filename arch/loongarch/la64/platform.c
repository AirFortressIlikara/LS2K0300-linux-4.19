// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/smp.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/acpi.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <asm/bootinfo.h>
#include <boot_param.h>
#include <loongson.h>
#include <loongson-pch.h>

#include <linux/err.h>
#include <linux/smp.h>

static struct platform_device loongson2k_cpufreq_device = {
	.name = "loongson2k_cpufreq",
	.id = -1,
};

static int __init loongson2k_cpufreq_init(void)
{
	return platform_device_register(&loongson2k_cpufreq_device);

	return -ENODEV;
}
arch_initcall(loongson2k_cpufreq_init);

extern int loongson_acpi_init(void);
static struct platform_device loongson3_cpufreq_device = {
	.name = "loongson3_cpufreq",
	.id = -1,
};

static int __init loongson_cpufreq_init(void)
{
	return platform_device_register(&loongson3_cpufreq_device);
}

arch_initcall(loongson_cpufreq_init);

#define SE_DRIVER_MAX_IRQ 	2
static struct resource se_resources[] = {
	{
		.name	= "se-irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "se-lirq",
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device loongson3_crypto_device = {
	.name = "loongson3_crypto",
	.id = -1,
	.num_resources	= SE_DRIVER_MAX_IRQ,
	.resource	= se_resources,
};

static int __init loongson_crypto_init(void)
{
	struct irq_fwspec fwspec;
	struct device *dev;

	if (cpu_has_hypervisor)
		return 0;

	dev = bus_find_device_by_name(&platform_bus_type, NULL, "LOON0003:00");
	if (dev != NULL) {
		dev_info(dev, "device is added by ACPI.\n");
		return 0;
	}

	fwspec.fwnode = liointc_get_fwnode();
	if (!fwspec.fwnode)
		return -ENODEV;
	fwspec.param[0] = LOONGSON_CPU_HT0_VEC + 4;
	fwspec.param_count = 1;

	se_resources[0].start = irq_create_fwspec_mapping(&fwspec);
	if (se_resources[0].start)
		pr_info("SE-IRQ=%d\n", (int)se_resources[0].start);
	else {
		pr_warn("SE alloc IRQ failed\n");
		return -EINVAL;
	}

	fwspec.param[0] = LOONGSON_CPU_HT0_VEC + 1;
	se_resources[1].start = irq_create_fwspec_mapping(&fwspec);
	if (se_resources[1].start)
		pr_info("SE-IRQ-low=%d\n", (int)se_resources[1].start);
	else {
		pr_warn("SE alloc Low-IRQ failed\n");
		return -EINVAL;
	}
	return platform_device_register(&loongson3_crypto_device);
}

device_initcall(loongson_crypto_init);

static int __init loongson3_acpi_suspend_init(void)
{
#ifdef CONFIG_ACPI
	acpi_status status;
	unsigned long long suspend_addr = 0;

	if (acpi_disabled || acpi_gbl_reduced_hardware)
		return 0;

	acpi_write_bit_register(ACPI_BITREG_SCI_ENABLE, 1);

	status = acpi_evaluate_integer(NULL, "\\SADR", NULL, &suspend_addr);
	if (ACPI_FAILURE(status) || !suspend_addr) {
		pr_err("ACPI S3 is not support!\n");
		return -1;
	}
	loongson_sysconf.suspend_addr = (u64)phys_to_virt(TO_PHYS(suspend_addr));
#endif
	return 0;
}

device_initcall(loongson3_acpi_suspend_init);
