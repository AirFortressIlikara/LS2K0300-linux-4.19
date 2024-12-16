/*
 * machtype.c - Add machine type for loongson-2K.
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

#include <asm/bootinfo.h>
#include <linux/init.h>
#include <machine.h>

static const char *system_types[] = {
	[MACH_LOONGSON_UNKNOWN]	= "unknown loongson machine",
	[MACH_LOONGSON_2K] = "loongson2k-machine",
	[MACH_LOONGSON_END]	= NULL,
};

const char *get_system_type(void)
{
	return system_types[mips_machtype];
}

void __init prom_init_machtype(void)
{
	mips_machtype = LOONGSON_MACHTYPE;
}
