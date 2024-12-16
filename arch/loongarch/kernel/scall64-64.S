/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 96, 97, 98, 99, 2000, 01, 02 by Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2001 MIPS Technologies, Inc.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/errno.h>
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/irqflags.h>
#include <asm/loongarchregs.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/asm-offsets.h>
#include <asm/thread_info.h>
#include <asm/unistd.h>
#include <asm/unwind_hints.h>

	.text
	.cfi_sections	.debug_frame
	.align	5
SYM_FUNC_START(handle_sys)
	csrrd	t0, PERCPU_BASE_KS
	la.abs	t1, kernelsp
	LONG_ADD	t1, t1, t0
	or	t2, sp, zero
	LONG_L	sp, t1, 0

	PTR_ADDI sp, sp, -PT_SIZE
	cfi_st	t2, PT_R3, 1
	cfi_rel_offset  sp, PT_R3, 1
	cfi_st	$r4, PT_R4, 1
	cfi_st	$r5, PT_R5, 1
	csrrd	t2, LOONGARCH_CSR_PRMD
	LONG_S	t2, sp, PT_PRMD
	cfi_st	$r6, PT_R6, 1
	cfi_st	ra, PT_R1, 1
	cfi_st	$r7, PT_R7, 1
	cfi_st	$r8, PT_R8, 1
	cfi_st	$r9, PT_R9, 1
	cfi_st	$r10, PT_R10, 1
	cfi_st	$r11, PT_R11, 1
	csrrd	ra, LOONGARCH_CSR_ERA
	LONG_S	ra, sp, PT_ERA

	.cfi_rel_offset 1, PT_ERA

	cfi_st	tp, PT_R2, 1
	cfi_st	fp, PT_R22, 1
	cfi_st	$r21, PT_R21, 1
	or	$r21, t0, zero

	SAVE_STATIC
	UNWIND_HINT_REGS

	/* syscall coming from user mode */
	li.d	tp, ~_THREAD_MASK
	and	tp, tp, sp

	TRACE_IRQS_ON_RELOAD
	STI

	/* save the initial A0 value (needed in signal handlers) */
	st.d	a0, sp, PT_ORIG_A0
	ld.d	t1, sp, PT_ERA		# skip syscall on return
	addi.d	t1, t1, 4		# skip to next instruction
	st.d	t1, sp, PT_ERA

#ifdef CONFIG_CPU_HAS_LBT
	li.d	t1, TRANS_ARCH_MASK
	and	t1, t1, a7
	bnez	t1, lbt_scall
#endif

	li.d	t1, _TIF_WORK_SYSCALL_ENTRY
	LONG_L	t0, tp, TI_FLAGS	# syscall tracing enabled?
	and	t0, t1, t0
	bnez	t0, syscall_trace_entry

syscall_common:
	/* Check to make sure we don't jump to a bogus syscall number. */
	li.w t0, __NR_syscalls
	bgeu a7, t0, illegal_syscall

	/* Syscall number held in a7 */
	slli.d	t0, a7, 3		# offset into table
	la	t2, sys_call_table
	add.d	t0, t2, t0
	ld.d	t2, t0, 0		#syscall routine
	beqz    t2, illegal_syscall

	jalr	t2			# Do The Real Thing (TM)

	ld.d	t1, sp, PT_R11		# syscall number
	addi.d	t1, t1, 1		# +1 for handle_signal
	st.d	t1, sp, PT_R0		# save it for syscall restarting
	st.d	v0, sp, PT_R4		# result

n64_syscall_exit:
	NOT_SIBLING_CALL_HINT
	b	syscall_exit_partial

/* ------------------------------------------------------------------------ */

syscall_trace_entry:
	SAVE_STATIC
	move	a0, sp
	move	a1, a7
	bl	syscall_trace_enter

	blt	v0, zero, 1f			# seccomp failed? Skip syscall

	RESTORE_STATIC
	ld.d	a0, sp, PT_R4		# Restore argument registers
	ld.d	a1, sp, PT_R5
	ld.d	a2, sp, PT_R6
	ld.d	a3, sp, PT_R7
	ld.d	a4, sp, PT_R8
	ld.d	a5, sp, PT_R9
	ld.d	a6, sp, PT_R10
	ld.d	a7, sp, PT_R11		# Restore syscall (maybe modified)
	b	syscall_common

1:	b	syscall_exit

#ifdef CONFIG_CPU_HAS_LBT
lbt_scall:
	li.d	t0, TRANS_MIPS_N64
	and	t0, t0, t1
	bnez    t0, handle_sys_lat_mips64	# for mips binary translation.
	b	handle_sys_lat_i386		# for i386 binary translation.
#endif

	/*
	 * The system call does not exist in this kernel
	 */

illegal_syscall:
	/* This also isn't a valid syscall, throw an error.  */
	li.w	v0, -ENOSYS			# error
	st.d	v0, sp, PT_R4
	b	n64_syscall_exit
SYM_FUNC_END(handle_sys)
