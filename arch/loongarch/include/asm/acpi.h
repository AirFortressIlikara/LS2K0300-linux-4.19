/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 * Author: Jianmin Lv <lvjianmin@loongson.cn>
 *         Huacai Chen <chenhuacai@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/suspend.h>
#include <linux/memblock.h>
#ifndef _ASM_LOONGARCH_ACPI_H
extern unsigned long max_low_pfn_mapped;
#define _ASM_LOONGARCH_ACPI_H

#ifdef CONFIG_ACPI
extern int acpi_strict;
extern int acpi_disabled;
extern int acpi_pci_disabled;
extern int acpi_noirq;

static inline void __iomem *acpi_os_ioremap(acpi_physical_address phys,
					    acpi_size size)
{
	if (memblock_is_memory(phys))
		return ioremap_cache(phys, size);

	return ioremap(phys, size);
}
#define acpi_os_ioremap acpi_os_ioremap

static inline void disable_acpi(void)
{
	acpi_disabled = 1;
	acpi_pci_disabled = 1;
	acpi_noirq = 1;
}

static inline bool acpi_has_cpu_in_madt(void)
{
	return true;
}

extern struct list_head acpi_wakeup_device_list;

#endif /* !CONFIG_ACPI */

#define ACPI_TABLE_UPGRADE_MAX_PHYS (max_low_pfn << PAGE_SHIFT)

extern int loongarch_acpi_suspend(void);
extern int (*acpi_suspend_lowlevel)(void);

#define acpi_wakeup_address loongarch_wakeup_start
#endif /* _ASM_LOONGARCH_ACPI_H */
