// SPDX-License-Identifier: GPL-2.0
/*
 * Author: Huacai Chen, chenhuacai@loongson.cn
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/acpi.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/efi.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <acpi/reboot.h>
#include <asm/bootinfo.h>
#include <asm/delay.h>
#include <asm/idle.h>
#include <asm/page.h>
#include <asm/reboot.h>

static void loongson_poweroff(void)
{
#ifdef CONFIG_EFI
	efi.reset_system(EFI_RESET_SHUTDOWN, EFI_SUCCESS, 0, NULL);
#endif
	while (1) {
		cpu_wait();
	}
}

static void loongson_restart(void)
{
	if (efi_capsule_pending(NULL)) {
		pr_info("EFI capsule is pending, forcing EFI reboot.\n");
		efi_reboot(REBOOT_WARM, NULL);
	}

	if (!acpi_disabled)
		acpi_reboot();

	efi_reboot(REBOOT_COLD, NULL);

	while (1) {
		cpu_wait();
	}
}

static int __init loongarch_reboot_setup(void)
{
	pm_restart = loongson_restart;
	if (loongson_fdt_blob != NULL)
		pm_power_off = NULL;
	else
		pm_power_off = loongson_poweroff;

	return 0;
}

arch_initcall(loongarch_reboot_setup);
