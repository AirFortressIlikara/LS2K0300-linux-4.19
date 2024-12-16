/*
 * cmdline.c - Add Loongson-2k prom_init_cmdline function.
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
#include <asm/bootinfo.h>
#include <linux/io.h>
#include <linux/init.h>
#include <boot_param.h>

extern void __init prom_init_machtype(void);

/*
 * For firmware compatibility UEFI may fill cmdline with 32bit physical address.
 * If x is physical address then convert to virtual address.
 */
#define CMDLINE_TO_CAC(x) ({	\
	long ret;			\
	if (((x) & 0x80000000) != 0x80000000) {	\
		ret = TO_CAC((unsigned int)(x));	\
	} else {				\
		ret = (long)(x);	\
	}					\
	ret;				\
})

void __init prom_init_cmdline(void)
{
	int prom_argc;
	/* pmon passes arguments in 32bit pointers */
	int *_prom_argv;
	int i;
	long l;

	/* firmware arguments are initialized in head.S */
	prom_argc = fw_arg0;
	_prom_argv = (int *)fw_arg1;

	/* arg[0] is "g", the rest is boot parameters */
	arcs_cmdline[0] = '\0';
	for (i = 1; i < prom_argc; i++) {
		l = CMDLINE_TO_CAC(_prom_argv[i]);
		if (strlen(arcs_cmdline) + strlen(((char *)l) + 1)
		    >= sizeof(arcs_cmdline))
			break;
		strcat(arcs_cmdline, ((char *)l));
		strcat(arcs_cmdline, " ");
	}

	prom_init_machtype();
}
