/*
 * init.c - Add Loongson-2k prom_init and plat_early_init function.
 *
 * Copyright (C) 2020, Loongson Technology Corporation Limited, Inc.
 *
 * Authors Ming Wang <wangming01@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <asm/bootinfo.h>
#include <asm/prom.h>

#ifdef CONFIG_SMP
#include <asm/smp.h>
extern struct plat_smp_ops loongson_smp_ops;
#endif

extern void __init prom_init_cmdline(void);

void *loongson_fdt_blob;
u32 cpu_guestmode;
EXPORT_SYMBOL_GPL(cpu_guestmode);

void __init prom_init(void)
{
	struct boot_param_header *fdtp;

	/* Loongson2k not support guestmode so here assigned a value of 0 */
	cpu_guestmode = 0;

	/* firmware arguments are initialized in head.S */
	fdtp = (struct boot_param_header *)fw_arg2;

	if (!fdtp)
		fdtp = (struct boot_param_header *)&__dtb_start;

	pr_info("FDT point@%px\n", fdtp);

	loongson_fdt_blob = fdtp;

#if defined(CONFIG_SMP)
	register_smp_ops(&loongson_smp_ops);
#endif

}

void __init plat_early_init(void)
{
	prom_init_cmdline();
}

void __init prom_free_prom_memory(void)
{
}
