/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 */
#ifndef _ASM_LBT_H
#define _ASM_LBT_H

#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/mipsregs.h>
#include <asm/cpu-features.h>

#ifdef CONFIG_CPU_HAS_LBT

#define STR(x)  __STR(x)
#define __STR(x)  #x

static inline void save_lbt_registers(struct task_struct *prev)
{
	unsigned long tmp = 0;

	__asm__ __volatile__ (
	"parse_r __reg, %[tmp]					\n"
	".word 0x70007ff4 | (__reg << 16)		\n"
	"sw %[tmp], " STR(THREAD_EFLAGS) "(%[prev])	\n"
	:
	: [prev] "r" (prev), [tmp] "r" (tmp)
	: "memory"
	);
}

static inline void restore_lbt_registers(struct task_struct *next)
{
	unsigned long tmp = 0;

	__asm__ __volatile__ (
	"parse_r __reg, %[tmp]					\n"
	"lw %[tmp], " STR(THREAD_EFLAGS) "(%[next])	\n"
	".word 0x70003ff4 | (__reg << 21)		\n"
	:
	: [next] "r" (next), [tmp] "r" (tmp)
	:
	);
}


static inline int thread_lbt_context_live(void)
{
	int ret = 0;

	if (__builtin_constant_p(loongson_cpu_has_lbt) && !loongson_cpu_has_lbt)
		goto out;

	ret = test_thread_flag(TIF_LBT_CTX_LIVE);
out:
	return ret;
}

static inline void enable_lbt(void)
{
	if (loongson_cpu_has_lbt)
		write_c0_config(read_c0_config() | (LS64_CONF_LBTEN));
}

static inline void disable_lbt(void)
{
	if (loongson_cpu_has_lbt)
		write_c0_config(read_c0_config() & (~(LS64_CONF_LBTEN)));
}

static inline int is_lbt_enabled(void)
{
	if (!loongson_cpu_has_lbt)
		return 0;

	return (read_c0_config() & (1<<20)) ?  1 : 0;
}

static inline int __is_lbt_owner(void)
{
	return test_thread_flag(TIF_USEDLBT);
}

static inline int is_lbt_owner(void)
{
	return loongson_cpu_has_lbt && __is_lbt_owner();
}

static inline void __own_lbt(void)
{
	enable_lbt();
	set_thread_flag(TIF_USEDLBT);
}

static inline void init_lbt(void)
{
	__own_lbt();
}

static inline void own_lbt_inatomic(int restore)
{
	if (loongson_cpu_has_lbt && !__is_lbt_owner()) {
		__own_lbt();
		if (restore)
			restore_lbt_registers(current);
	}
}

static inline void lose_lbt_inatomic(int save, struct task_struct *tsk)
{
	if (is_lbt_owner()) {
		if (save)
			save_lbt_registers(tsk);

		disable_lbt();
		clear_tsk_thread_flag(tsk, TIF_USEDLBT);
	}
}

static inline void lose_lbt(int save)
{
	preempt_disable();
	lose_lbt_inatomic(save, current);
	preempt_enable();
}

#else
static inline void own_lbt_inatomic(int restore)
{}
static inline void lose_lbt_inatomic(int save, struct task_struct *tsk)
{}
static inline void lose_lbt(int save)
{}
#endif

#endif
