/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_DMI_H__
#define _ASM_DMI_H__
#include <linux/efi.h>
#include <linux/slab.h>

#define dmi_early_remap early_ioremap
#define dmi_early_unmap	early_iounmap
#define dmi_alloc(l)	alloc_bootmem(l)

void __init __iomem *dmi_remap(u64 phys_addr, unsigned long size)
{
	return ((void *)TO_CAC(phys_addr));
}
void dmi_unmap(volatile void __iomem *addr)
{
}

#endif /* _ASM_DMI_H */
