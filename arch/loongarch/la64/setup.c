// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007 Lemote Inc. & Institute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/export.h>
#include <linux/init.h>
#include <linux/libfdt.h>
#include <linux/of_fdt.h>

#ifdef CONFIG_VT
#include <linux/console.h>
#include <linux/screen_info.h>
#include <linux/platform_device.h>
#endif

#include <asm/bootinfo.h>

#include <loongson.h>

const char *get_system_type(void)
{
	return "generic-loongson-machine";
}

void __init plat_mem_setup(void)
{
}

static int __init register_gop_device(void)
{
	void *pd;
	if (screen_info.orig_video_isVGA != VIDEO_TYPE_EFI)
		return 0;
	pd = platform_device_register_data(NULL, "efi-framebuffer", 0,
			&screen_info, sizeof(screen_info));
	return PTR_ERR_OR_ZERO(pd);
}
subsys_initcall(register_gop_device);

#define NR_CELLS 6

void __init device_tree_init(void)
{
	if (!initial_boot_params)
		return;

	if (early_init_dt_verify(initial_boot_params))
		unflatten_and_copy_device_tree();
}
