/*
 * LoongArch specific _mcount support
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive for
 * more details.
 *
 * Copyright (C) 2009 Lemote Inc. & DSLab, Lanzhou University, China
 * Copyright (C) 2010 DSLab, Lanzhou University, China
 * Author: Wu Zhangjin <wuzhangjin@gmail.com>
 */

#include <asm/export.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/ftrace.h>

	.text

#define MCOUNT_STACK_SIZE	(2 * SZREG)
#define MCOUNT_S0_OFFSET	(0)
#define MCOUNT_RA_OFFSET	(SZREG)

	.macro MCOUNT_SAVE_REGS
	PTR_ADDI sp, sp, -MCOUNT_STACK_SIZE
	PTR_S	s0, sp, MCOUNT_S0_OFFSET
	PTR_S	ra, sp, MCOUNT_RA_OFFSET
	move	s0, a0
	.endm

	.macro MCOUNT_RESTORE_REGS
	move	a0, s0
	PTR_L	ra, sp, MCOUNT_RA_OFFSET
	PTR_L	s0, sp, MCOUNT_S0_OFFSET
	PTR_ADDI sp, sp, MCOUNT_STACK_SIZE
	.endm


SYM_FUNC_START(_mcount)
	la	t1, ftrace_stub
	la	t2, ftrace_trace_function	/* Prepare t2 for (1) */
	PTR_L	t2, t2, 0
	beq	t1, t2, fgraph_trace

	MCOUNT_SAVE_REGS

	move	a0, ra				/* arg0: self return address */
	move	a1, s0				/* arg1: parent's return address */
	jirl	ra, t2, 0			/* (1) call *ftrace_trace_function */

	MCOUNT_RESTORE_REGS

fgraph_trace:
#ifdef	CONFIG_FUNCTION_GRAPH_TRACER
	la	t1, ftrace_stub
	la	t3, ftrace_graph_return
	PTR_L	t3, t3, 0
	bne	t1, t3, ftrace_graph_caller
	la	t1, ftrace_graph_entry_stub
	la	t3, ftrace_graph_entry
	PTR_L	t3, t3, 0
	bne	t1, t3, ftrace_graph_caller
#endif

	.globl ftrace_stub
ftrace_stub:
	jirl	zero, ra, 0
SYM_FUNC_END(_mcount)
EXPORT_SYMBOL(_mcount)

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
SYM_FUNC_START(ftrace_graph_caller)
	MCOUNT_SAVE_REGS

	PTR_ADDI	a0, ra, -4			/* arg0: Callsite self return addr */
	PTR_ADDI	a1, sp, MCOUNT_STACK_SIZE	/* arg1: Callsite sp */
	move	a2, s0					/* arg2: Callsite parent ra */
	bl	prepare_ftrace_return

	MCOUNT_RESTORE_REGS
	jirl	zero, ra, 0
SYM_FUNC_END(ftrace_graph_caller)

SYM_FUNC_START(return_to_handler)
	PTR_ADDI sp, sp, -2 * SZREG
	PTR_S	a0, sp, 0
	PTR_S	a1, sp, SZREG

	bl	ftrace_return_to_handler

	/* restore the real parent address: a0 -> ra */
	move	ra, a0

	PTR_L	a0, sp, 0
	PTR_L	a1, sp, SZREG
	PTR_ADDI	sp, sp, 2 * SZREG
	jirl	zero, ra, 0
SYM_FUNC_END(return_to_handler)
#endif /* CONFIG_FUNCTION_GRAPH_TRACER */
