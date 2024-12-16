/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 95, 96, 97, 98, 99, 2003, 06 by Ralf Baechle
 * Copyright (C) 1996 by Paul M. Antoine
 * Copyright (C) 1999 Silicon Graphics
 * Kevin D. Kissell, kevink@mips.org and Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 2000 MIPS Technologies, Inc.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_SWITCH_TO_H
#define _ASM_SWITCH_TO_H

#include <asm/cpu-features.h>
#include <asm/watch.h>
#include <asm/fpu.h>
#include <asm/lbt.h>
#include <asm/vdso.h>

struct task_struct;

/**
 * resume - resume execution of a task
 * @prev:	The task previously executed.
 * @next:	The task to begin executing.
 * @next_ti:	task_thread_info(next).
 * @sched_ra:	__schedule return address.
 * @sched_cfa:	__schedule call frame address.
 *
 * This function is used whilst scheduling to save the context of prev & load
 * the context of next. Returns prev.
 */
extern asmlinkage struct task_struct *resume(struct task_struct *prev,
		struct task_struct *next, struct thread_info *next_ti,
		void *sched_ra, void *sched_cfa);

/*
 * For newly created kernel threads switch_to() will return to
 * ret_from_kernel_thread, newly created user threads to ret_from_fork.
 * That is, everything following resume() will be skipped for new threads.
 * So everything that matters to new threads should be placed before resume().
 */
#define switch_to(prev, next, last)							\
do {											\
	lose_fpu_inatomic(1, prev);							\
	lose_lbt_inatomic(1, prev);							\
	vdso_per_cpu_switch_thread(prev, next);						\
	__process_watch(prev, next);							\
	(last) = resume(prev, next, task_thread_info(next),				\
			__builtin_return_address(0), __builtin_frame_address(0));	\
} while (0)

#endif /* _ASM_SWITCH_TO_H */
