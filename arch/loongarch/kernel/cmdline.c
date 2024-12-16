// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <asm/addrspace.h>
#include <asm/fw.h>

long *_fw_argv;

void __init fw_init_cmdline(void)
{
	int i;

	if (fw_arg0 < 2)
		return;

	_fw_argv = (long *)TO_CAC(fw_arg1);

	arcs_cmdline[0] = '\0';
	for (i = 1; i < fw_arg0; i++) {
		strlcat(arcs_cmdline, fw_argv(i), COMMAND_LINE_SIZE);
		if (i < (fw_arg0 - 1))
			strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
	}
}

