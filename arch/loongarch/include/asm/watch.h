/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * Author: Chong Qiao <qiaochong@loongson.cn>
 */
#ifndef _ASM_WATCH_H
#define _ASM_WATCH_H

#include <linux/bitops.h>

#include <asm/loongarchregs.h>


void loongarch_install_watch_registers(struct task_struct *t);
void loongarch_update_watch_registers(struct task_struct *t);
void loongarch_clear_prev_watch_registers(struct task_struct *prev);
void loongarch_install_next_watch_registers(struct task_struct *next);
void loongarch_read_watch_registers(struct pt_regs *regs);
void loongarch_clear_watch_registers(void);
void loongarch_probe_watch_registers(struct cpuinfo_loongarch *c);
void watch_csrwr(unsigned long val, unsigned int reg);
unsigned long watch_csrrd(unsigned int reg);

#ifdef CONFIG_HARDWARE_WATCHPOINTS
#define __process_watch(prev, next) do {					\
	if (test_bit(TIF_LOAD_WATCH, &task_thread_info(prev)->flags) || \
	    test_bit(TIF_SINGLESTEP, &task_thread_info(prev)->flags)) \
		loongarch_clear_prev_watch_registers(prev); \
	if (test_bit(TIF_LOAD_WATCH, &task_thread_info(next)->flags) || \
	    test_bit(TIF_SINGLESTEP, &task_thread_info(next)->flags)) \
		loongarch_install_next_watch_registers(next); \
} while (0)

#else
#define __process_watch(prev, next) do {} while (0)
#endif

#endif /* _ASM_WATCH_H */
