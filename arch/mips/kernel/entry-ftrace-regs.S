/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * arch/mips/kernel/entry_ftrace.S
 *
 * Copyright (C) 2021 Loongson Corp
 * Author: Huang Pei <huangpei@loongson.cn>
 */

#include <asm/export.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>

/*
 * ftrace_regs_caller() is the function that replaces _mcount() when ftrace
 * is active.
 *
 * we arrive here after a function A calls function B, and B is what we
 * are tracing for. When we enter, sp points to A's stack frame, B has not
 * yet had a chance to allocate one yet. (This is different from -pg case
 * , in which the B's stack is allocated))

 * when ftrace initialized, it replace three nops from all function with
 * "lui + nop + move"
 * B:
 *	lui	at, %hi(ftrace_regs_caller)
 *	nop
 *	li	t0, 0
 * #	B's real start
 *
 * at B's entry, when tracing enabled, replace the 'nop' with 'jalr'
 *
 * #	B's entry, three nop for both in vmlinux and in kernel modules
 * B:
 *	lui	at, %hi(ftrace_regs_caller)
 *	jalr	at, at
 *	move	t0, zero
 * #	B's real start
 *
 * if set t0 to 1, then calling ftrace_regs_caller with partial regs saved
 *
 * B:
 *	lui	at, %hi(ftrace_regs_caller)
 *	jalr	at, at
 *	li	t0, 1
 * #	B's real start
 *
 * we make ftrace_regs_caller 64KB aligned, when entring ftrace_regs_caller
 * AT points to the return address to B, and ra points to return address
 * to A,
 *
 * if patched to new funcition, then clobbered the first real instruction
 *
 * B:
 *	lui	at, %hi(new_B)
 *	addiu	at, at, %lo(new_B)
 *	jr	at
 * #	B's real start, now clobbered with zero
 *	nop
 *
 */
	.text
	.set push
	.set noreorder
	.set noat
	.align 16
NESTED(ftrace_regs_caller, PT_SIZE, ra)
	PTR_ADDIU	sp, sp, -PT_SIZE
	.globl ftrace_caller
ftrace_caller:
#ifdef CONFIG_64BIT
	PTR_S	a4, PT_R8(sp)
	PTR_S	a5, PT_R9(sp)
	PTR_S	a6, PT_R10(sp)
	PTR_S	a7, PT_R11(sp)
#endif
	PTR_S	a0, PT_R4(sp)
	PTR_S	a1, PT_R5(sp)
	PTR_S	a2, PT_R6(sp)

	bnez	t0, 1f
	PTR_S	a3, PT_R7(sp)

	PTR_S	t0, PT_R12(sp)
	PTR_S	t1, PT_R13(sp)
	PTR_S	t2, PT_R14(sp)
	PTR_S	t3, PT_R15(sp)

	PTR_S	s0, PT_R16(sp)
	PTR_S	s1, PT_R17(sp)
	PTR_S	s2, PT_R18(sp)
	PTR_S	s3, PT_R19(sp)

	PTR_S	s4, PT_R20(sp)
	PTR_S	s5, PT_R21(sp)
	PTR_S	s6, PT_R22(sp)
	PTR_S	s7, PT_R23(sp)


	PTR_S	t8, PT_R24(sp)
	PTR_S	t9, PT_R25(sp)
	PTR_S	s8, PT_R30(sp)
	PTR_S	gp, PT_R28(sp)

	PTR_S	AT, PT_R1(sp)
1:
	PTR_LA	t0, PT_SIZE(sp)
	PTR_S	AT, PT_R0(sp)	//R0 for expected epc
	PTR_S	t0, PT_R29(sp)

	PTR_S	ra, PT_R31(sp)
	PTR_S	AT, PT_EPC(sp)		//PT_EPC maybe changed by kprobe handler

	END(ftrace_regs_caller)

ftrace_common:
	PTR_ADDIU	a0, AT, -12	//a0 points to B's entry address
	move	a1, ra			//a1 points to return address to A
	PTR_L	a2, function_trace_op	//a2 points to function_trace op

	.globl	ftrace_call
ftrace_call:
	jal	ftrace_stub
	move	a3, sp			//a3 point to pt_regs

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
	.globl ftrace_graph_call
ftrace_graph_call:
	nop
	nop
#endif

ftrace_common_return:
	PTR_L	AT, PT_R31(sp)
ftrace_graph_return:
	PTR_L	ra, PT_EPC(sp)
	PTR_L	a0, PT_R4(sp)
	PTR_L	a1, PT_R5(sp)
	PTR_L	a2, PT_R6(sp)
	PTR_L	a3, PT_R7(sp)
#ifdef CONFIG_64BIT
	PTR_L	a4, PT_R8(sp)
	PTR_L	a5, PT_R9(sp)
	PTR_L	a6, PT_R10(sp)
	PTR_L	a7, PT_R11(sp)
#endif
	PTR_ADDIU	sp, sp, PT_SIZE	//retore stack frame
	jr	ra
	move	ra, AT


	.globl ftrace_stub
ftrace_stub:
	jr	ra
	nop

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
	.globl	ftrace_graph_caller
ftrace_graph_caller:
	PTR_L	a0, PT_R31(sp)
	PTR_L	a1, PT_EPC(sp)
	jal	prepare_ftrace_return
	PTR_ADDIU	a2, sp, PT_SIZE

	b	ftrace_graph_return
	move	AT, v0


	.align	2
	.globl	return_to_handler
return_to_handler:
	PTR_SUBU	sp, PT_SIZE
	PTR_S	v0, PT_R2(sp)

	PTR_S	v1, PT_R3(sp)
	jal	ftrace_return_to_handler
	PTR_LA	a0, PT_SIZE(sp)

	/* restore the real parent address: v0 -> ra */
	move	ra, v0

	PTR_L	v0, PT_R2(sp)
	PTR_L	v1, PT_R3(sp)
	jr	ra
	 PTR_ADDIU	sp, PT_SIZE

	.set at
	.set reorder
#endif /* CONFIG_FUNCTION_GRAPH_TRACER */
