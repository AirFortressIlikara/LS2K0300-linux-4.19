/*
 * Copyright (C) 2021 Loongson Inc.
 * Author: Ming Wang, wangming01@Loongson.cn
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/err.h>
#include <linux/smp.h>
#include <linux/platform_device.h>

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
