/*
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *         Xiang Yu, xiangy@lemote.com
 *         Chen Huacai, chenhc@lemote.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <asm/bootinfo.h>
#include <boot_param.h>
#include <loongson.h>
#include <loongson-pch.h>
#include <loongson_hwmon.h>
#include <workarounds.h>
#include <linux/acpi.h>

#define LS_IOMMU_BAR 0xFDFE000300
/*
 * Kernel helper policy
 *
 * Fan is controlled by EC in laptop pruducts, but EC can not get the current
 * cpu temperature which used for adjusting the current fan speed.
 *
 * So, kernel read the CPU temperature and notify it to EC per second,
 * that's all!
 */
struct loongson_fan_policy kernel_helper_policy = {
	.type = KERNEL_HELPER_POLICY,
	.adjust_period = 1,
	.depend_temp = loongson3_cpu_temp,
};

/*
 * Policy at step mode
 *
 * up_step array    |   down_step array
 *                  |
 * [min, 50),  50%  |   (min, 45),  50%
 * [50,  60),  60%  |   [45,  55),  60%
 * [60,  70),  70%  |   [55,  65),  70%
 * [70,  80),  80%  |   [65,  75),  80%
 * [80,  max), 100% |   [75,  max), 100%
 *
 */
struct loongson_fan_policy step_speed_policy = {
	.type = STEP_SPEED_POLICY,
	.adjust_period = 1,
	.depend_temp = loongson3_cpu_temp,
	.up_step_num = 5,
	.down_step_num = 5,
	.up_step = {
			{MIN_TEMP, 50,    50},
			{   50,    60,    60},
			{   60,    70,    70},
			{   70,    80,    80},
			{   80, MAX_TEMP, 100},
		   },
	.down_step = {
			{MIN_TEMP, 45,    50},
			{   45,    55,    60},
			{   55,    65,    70},
			{   65,    75,    80},
			{   75, MAX_TEMP, 100},
		     },
};

/*
 * Constant speed policy
 *
 */
struct loongson_fan_policy constant_speed_policy = {
	.type = CONSTANT_SPEED_POLICY,
};

static struct resource ls_iommu_resources[] = {
	[0] = {
		.start = LS_IOMMU_BAR,
		.end   = LS_IOMMU_BAR + 0x40,
		.flags = IORESOURCE_REG,
	},
};

static struct platform_device ls_iommu_device = {
	.name		= "loongson-iommu",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(ls_iommu_resources),
	.resource	= ls_iommu_resources,
};
extern u32 cpu_guestmode;
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
	fwspec.fwnode = liointc_handle;
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


#define GPIO_LCD_CNTL		5
#define GPIO_BACKLIGHIT_CNTL	7

static struct platform_device loongson_laptop_device = {
	.name			= "loongson-laptop",
	.id   			= 4,
};

static struct platform_device loongson_thermal_device = {
	.name			= "loongson3_thermal",
	.id   			= 0,
};

static int __init loongson3_platform_init(void)
{
	int i;
	struct platform_device *pdev;

	loongson_pch->pch_arch_initcall();

	if (loongson_sysconf.ecname[0] != '\0')
		platform_device_register_simple(loongson_sysconf.ecname, -1, NULL, 0);

	for (i = 0; i < loongson_sysconf.nr_sensors; i++) {
		if (loongson_sysconf.sensors[i].type > SENSOR_FAN)
			continue;

		pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
		if (!pdev)
			return -ENOMEM;

		pdev->name = loongson_sysconf.sensors[i].name;
		pdev->id = loongson_sysconf.sensors[i].id;
		pdev->dev.platform_data = &loongson_sysconf.sensors[i];
		platform_device_register(pdev);
	}

	if (loongson_sysconf.workarounds & WORKAROUND_LVDS_GPIO) {
		gpio_request(GPIO_LCD_CNTL,  "gpio_lcd_cntl");
		gpio_request(GPIO_BACKLIGHIT_CNTL, "gpio_bl_cntl");
	}

	platform_device_register(&loongson_laptop_device);
	platform_device_register(&loongson_thermal_device);
	platform_device_register(&ls_iommu_device);
	if (!cpu_guestmode) {
		loongson_crypto_init();
	}
	return 0;
}

extern void power_button_init(void);
static int __init loongson3_device_init(void)
{
	loongson_pch->pch_device_initcall();

	if (acpi_disabled)
		power_button_init();
	return 0;
}

arch_initcall(loongson3_platform_init);
device_initcall(loongson3_device_init);
