/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2009 Lemote, Inc.
 * Author: Wu Zhangjin <wuzhangjin@gmail.com>
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_MACH_LOONGSON64_LOONGSON_H
#define __ASM_MACH_LOONGSON64_LOONGSON_H

#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <asm/addrspace.h>
#include <boot_param.h>

extern const struct plat_smp_ops loongson3_smp_ops;

/* loongson-specific command line, env and memory initialization */
extern void __init fw_init_env(void);
extern void __init fw_init_memory(void);
extern void __init fw_init_numa_memory(void);

#define MAX_PACKAGES		16

#define LOONGSON_LIO_BASE	0x18000000
#define LOONGSON_REG_BASE	0x1fe00000

#define LOONGSON_CHIP_TEMP_BASE	(LOONGSON_REG_BASE + 0x19c)
#define LOONGSON_FREQ_CTRL_BASE	(LOONGSON_REG_BASE + 0x1d0)
#define LOONGSON_LIOINTC_BASE	(LOONGSON_REG_BASE + 0x1400)

#define LOONGSON3_NODE_BASE(x)	(TO_UNCAC(((unsigned long)x & 0xf) << 44))

/* Chip Temperature registor of each physical cpu package, PRid >= Loongson-3A */
#define LOONGSON_CHIPTEMP(id) \
	(*(volatile u32 *)(LOONGSON3_NODE_BASE(id) + LOONGSON_CHIP_TEMP_BASE))

/* Freq Control register of each physical cpu package, PRid >= Loongson-3B */
#define LOONGSON_FREQCTRL(id) \
	(*(volatile u32 *)(LOONGSON3_NODE_BASE(id) + LOONGSON_FREQ_CTRL_BASE))

#ifdef CONFIG_CPU_SUPPORTS_CPUFREQ
#include <linux/cpufreq.h>
extern struct cpufreq_frequency_table loongson2_clockmod_table[];
extern struct cpufreq_frequency_table loongson3_clockmod_table[];
extern struct cpufreq_frequency_table *loongson3a4000_clockmod_table;
extern struct cpufreq_frequency_table ls3a4000_normal_table[];
extern struct cpufreq_frequency_table ls3a4000_boost_table[];
extern void ls3a4000_freq_table_switch(struct cpufreq_frequency_table *table);
extern int ls3a4000_set_boost(int mode, int freq_level);
extern int ls3a4000_freq_scale(struct cpufreq_policy* policy, unsigned long rate);

#define BOOST_FREQ_MAX	2000000000

#define CPU_ID_FIELD	0xf
#define NODE_FIELD	0xf0
#define FREQ_FIELD	0xf00
#define VOLTAGE_FIELD	0xf000
#define VOLTAGE_CHANGE_FIELD	0xc0000

#define BOOST_NORMAL_FIELD	0xc0000

#define COMMAND_FIELD	0x7f000000
#define COMPLETE_STATUS	0x80000000
#define VOLTAGE_COMMAND	0x21

#define DVFS_INFO	0x22
#define DVFS_INFO_BOOST_LEVEL	0x23
#define DVFS_INFO_MIN_FREQ	0xf
#define DVFS_INFO_MAX_FREQ	0xf0
#define DVFS_INFO_BOOST_CORE_FREQ	0xff00
#define DVFS_INFO_NORMAL_CORE_UPPER_LIMIT	0xf0000
#define DVFS_INFO_BOOST_CORES	0xf00000

#define BOOST_MODE	0x80000
#define NORMAL_MODE	0x40000

#endif
#endif /* __ASM_MACH_LOONGSON64_LOONGSON_H */
