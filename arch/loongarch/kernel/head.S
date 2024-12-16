/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 1995 Waldorf Electronics
 * Written by Ralf Baechle and Andreas Busse
 * Copyright (C) 1994 - 99, 2003, 06 Ralf Baechle
 * Copyright (C) 1996 Paul M. Antoine
 * Further modifications by David S. Miller and Harald Koerfgen
 * Copyright (C) 1999 Silicon Graphics, Inc.
 * Kevin Kissell, kevink@mips.com and Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 2000 MIPS Technologies, Inc.  All rights reserved.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/init.h>
#include <linux/threads.h>

#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/bug.h>
#include <asm/irqflags.h>
#include <asm/regdef.h>
#include <asm/loongarchregs.h>
#include <asm/stackframe.h>

	/*
	 * Reserved space for exception handlers.
	 * Necessary for machines which link their kernels at KPRANGEx.
	 */
	.fill	0x400

SYM_ENTRY(_stext, SYM_L_GLOBAL, SYM_A_NONE)

	__REF

.align	12
SYM_CODE_START(kernel_entry)			# kernel entry point
	# Config direct window and set PG
	# 0x8000 xxxx xxxx xxxx
	li.d	t0, CSR_DMW0_INIT	# UC, PLV0
	csrwr	t0, LOONGARCH_CSR_DMWIN0
	# 0x9000 xxxx xxxx xxxx
	li.d	t0, CSR_DMW1_INIT	# CA, PLV0
	csrwr	t0, LOONGARCH_CSR_DMWIN1
	# Enable PG
	li.w	t0, 0xb0	# PLV=0, IE=0, PG
	csrwr	t0, LOONGARCH_CSR_CRMD
	# Clear EUEN
	li.w	t0, 0xf
	csrxchg	zero, t0, LOONGARCH_CSR_EUEN

	/* We might not get launched at the address the kernel is linked to,
	   so we jump there.  */
	JUMP_CACHE_ADDR t0, t1

	la	t0, __bss_start		# clear .bss
	st.d	zero, t0, 0
	la	t1, __bss_stop - LONGSIZE
1:
	addi.d	t0, t0, LONGSIZE
	st.d	zero, t0, 0
	bne	t0, t1, 1b

	la	t0, fw_arg0
	st.d	a0, t0, 0		# firmware arguments
	la	t0, fw_arg1
	st.d	a1, t0, 0
	la	t0, fw_arg2
	st.d	a2, t0, 0

#ifdef CONFIG_PAGE_SIZE_4KB
	li.d	t0, 0
	li.d	t1, CSR_STFILL
	csrxchg	t0, t1, LOONGARCH_CSR_IMPCTL1
#endif

	csrwr	zero, PERCPU_BASE_KS
	/* gpr21 used for percpu base, it should be initialized as 0 */
	or	$r21, zero, zero

	la	tp, init_thread_union
	/* Set the SP after an empty pt_regs.  */
	li.d		sp, (_THREAD_SIZE - PT_SIZE)
	add.d		sp, sp, tp
	set_saved_sp	sp, t0, t1

#ifdef CONFIG_RELOCATABLE
#ifdef CONFIG_CRASH_DUMP
	/* Handle the a3 passed in by the firmware is not 0 */
	li.w		t1, 0x123
	bne		t1, a4, 1f

	beqz		a3, 1f
	move		a0, a3
	/* Apply the relocations for kdump */
	bl		relocate_kdump_kernel
	b		2f
#endif
1:
	/* Copy kernel and apply the relocations */
	bl		relocate_kernel
2:

	/* Repoint the sp into the new kernel image */
	PTR_LI		sp, _THREAD_SIZE - PT_SIZE
	PTR_ADD	sp, sp, $r2
	set_saved_sp	sp, t0, t1

	/*
	 * relocate_kernel returns the entry point either
	 * in the relocated kernel or the original if for
	 * some reason relocation failed - jump there now
	 * with instruction hazard barrier because of the
	 * newly sync'd icache.
	 */
	jirl		zero, v0, 0
#else
#ifdef CONFIG_KASAN
	bl		kasan_early_init
#endif
	bl		start_kernel
#endif
	ASM_BUG()

SYM_CODE_END(kernel_entry)

#ifdef CONFIG_SMP
/*
 * SMP slave cpus entry point. Board specific code for bootstrap calls this
 * function after setting up the stack and tp registers.
 */
	.macro	smp_slave_setup

#ifdef CONFIG_PAGE_SIZE_4KB
	li.d	t0, 0
	li.d	t1, CSR_STFILL
	csrxchg	t0, t1, LOONGARCH_CSR_IMPCTL1
#endif

	li.d	t0, CSR_DMW0_INIT	# UC, PLV0
	csrwr	t0, LOONGARCH_CSR_DMWIN0
	li.d	t0, CSR_DMW1_INIT	# CA, PLV0
	csrwr	t0, LOONGARCH_CSR_DMWIN1

	JUMP_CACHE_ADDR t0, t1

	li.w	t0, 0xb0	# PLV=0, IE=0, PG
	csrwr	t0, LOONGARCH_CSR_CRMD
	# Clear EUEN
	li.w	t0, 0xf
	csrxchg	zero, t0, LOONGARCH_CSR_EUEN

	la.abs  t0, cpuboot_data
	ld.d    sp, t0, CPU_BOOT_STACK
	ld.d    tp, t0, CPU_BOOT_TINFO
	.endm

SYM_CODE_START(smp_bootstrap)
	smp_slave_setup
	bl	start_secondary
	ASM_BUG()
SYM_CODE_END(smp_bootstrap)
SYM_ENTRY(kernel_entry_end, SYM_L_GLOBAL, SYM_A_NONE)
#endif /* CONFIG_SMP */
