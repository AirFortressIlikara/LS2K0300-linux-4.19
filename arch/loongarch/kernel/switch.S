/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 1995, 1996, 1998, 1999, 2002, 2003 Ralf Baechle
 * Copyright (C) 1996 David S. Miller (davem@davemloft.net)
 * Copyright (C) 1994, 1995, 1996, by Andreas Busse
 * Copyright (C) 1999 Silicon Graphics, Inc.
 * Copyright (C) 2000 MIPS Technologies, Inc.
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#include <asm/asm.h>
#include <asm/loongarchregs.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/thread_info.h>

#include <asm/asmmacro.h>

/*
 * task_struct *resume(task_struct *prev, task_struct *next,
 *		       struct thread_info *next_ti,
 *		       void *sched_ra, void *sched_cfa)
 */
	.align	5
SYM_FUNC_START(resume)
	cpu_save_nonscratch a0
	stptr.d	ra, a0, THREAD_REG01
	stptr.d a3, a0, THREAD_SCHED_RA
	stptr.d a4, a0, THREAD_SCHED_CFA

#if defined(CONFIG_STACKPROTECTOR) && !defined(CONFIG_SMP)
	PTR_LA	t8, __stack_chk_guard
	LONG_L	t9, a1, TASK_STACK_CANARY
	LONG_S	t9, t8, 0
#endif

	/*
	 * The order of restoring the registers takes care of the race
	 * updating $28, $29 and kernelsp without disabling ints.
	 */
	move	tp, a2
	cpu_restore_nonscratch a1

	li.w	t0, _THREAD_SIZE
	PTR_ADD	t0, t0, tp
	set_saved_sp	t0, t1, t2

	jr	ra
SYM_FUNC_END(resume)
