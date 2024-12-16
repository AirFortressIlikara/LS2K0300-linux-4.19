/*
 * reset.c - Add reboot support for loongson-2K.
 *
 * Copyright (C) 2020, Loongson Technology Corporation Limited, Inc.
 *
 * Authors Ming Wang <wangming01@loongson.cn>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <acpi/reboot.h>
#include <linux/acpi.h>
#include <asm/bootinfo.h>
#include <asm/idle.h>
#include <asm/reboot.h>

#include <linux/efi.h>
#include <loongson-2k.h>
#include <boot_param.h>

#include <asm/delay.h>
#include <linux/sched/debug.h>

extern void ls2k_pm(enum ACPI_Sx sx);

static inline void loongson_reboot(void)
{
	unsigned long base;
	base = CKSEG1ADDR(APB_BASE) + ACPI_OFF;

	writel(1, (void *)(base + RST_CTR));

	while (1) {
		;
	}
}

static void loongson_restart(char *command)
{
	loongson_reboot();
}

static inline void ls2k_poweroff(void)
{
	unsigned int acpi_ctrl;

	acpi_ctrl = acpi_readl(PM1_STS);
	acpi_ctrl &= 0xffffffff;
	acpi_writel(acpi_ctrl, PM1_STS);

	acpi_ctrl = ((0x07 << 10) | (1 << 13));
	acpi_writel(acpi_ctrl, PM1_CTR);
}

static void loongson_poweroff(void)
{
#ifdef CONFIG_PM_SLEEP
	ls2k_pm(ACPI_S5);
#else
	ls2k_poweroff();
#endif
	while (1) {
		;
	}
}

static void loongson_halt(void)
{
	pr_notice("\n\n** You can safely turn off the power now **\n\n");
	while (1) {
		if (cpu_wait)
			cpu_wait();
	}
}

static int __init mips_reboot_setup(void)
{
	_machine_restart = loongson_restart;
	_machine_halt = loongson_halt;
	pm_power_off = loongson_poweroff;

	return 0;
}

arch_initcall(mips_reboot_setup);
