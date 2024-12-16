/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 */
#include <linux/init.h>

#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/cacheops.h>
#include <asm/irqflags.h>
#include <asm/regdef.h>
#include <asm/fpregdef.h>
#include <asm/loongarchregs.h>
#include <asm/stackframe.h>
#include <asm/thread_info.h>
#include <asm/unwind_hints.h>

SYM_FUNC_START(except_vec_cex)
	b	cache_parity_error
	nop
SYM_FUNC_END(except_vec_cex)

	.align	5	/* 32 byte rollback region */
SYM_FUNC_START(__cpu_wait)
	/* start of rollback region */
	LONG_L	t0, tp, TI_FLAGS
	nop
	andi	t0, t0, _TIF_NEED_RESCHED
	bnez	t0, 1f
	nop
	nop
	nop
	idle	0
	/* end of rollback region (the region size must be power of two) */
1:
	jirl	zero, ra, 0
SYM_FUNC_END(__cpu_wait)

/*
 * Common Vectored Interrupt code
 * Complete the register saves and invoke the handler which is passed in $v0
 */
SYM_FUNC_START(except_vec_vi_handler)
	UNWIND_HINT_REGS
	la      t1, __cpu_wait
	ld.d    t0, sp, PT_ERA
	/* 32 byte rollback region */
	ori	t0, t0, 0x1f
	xori	t0, t0, 0x1f
	bne	t0, t1, 1f
	st.d    t0, sp, PT_ERA
1:
	LOAD_TEMP_FROM_KSCRATCH
	SAVE_TEMP
	SAVE_STATIC
	CLI
#ifdef CONFIG_TRACE_IRQFLAGS
	move	s0, v0
	TRACE_IRQS_OFF
	move	v0, s0
#endif

	LONG_L	s0, tp, TI_REGS
	LONG_S	sp, tp, TI_REGS

	/*
	 * SAVE_ALL ensures we are using a valid kernel stack for the thread.
	 * Check if we are already using the IRQ stack.
	 */
	move	s1, sp # Preserve the sp

	/* Get IRQ stack for this CPU */
	la		t1, irq_stack
	LONG_ADD	t1, t1, $r21
	LONG_L		t0, t1, 0

	# Check if already on IRQ stack
	PTR_LI		t1, ~(_THREAD_SIZE-1)
	and		t1, t1, sp
	beq		t0, t1, 2f

	/* Switch to IRQ stack */
	li.w		t1, _IRQ_STACK_START
	PTR_ADD	sp, t0, t1

	/* Save task's sp on IRQ stack so that unwinding can follow it */
	LONG_S		s1, sp, 0
2:
	/* v0 is equal to a0
	 * v0: irq number for do_vi
	 */
	la		t0, do_vi
	jirl		ra, t0, 0

	/* Restore sp */
	move	sp, s1

	la	t0, ret_from_irq
	jirl    zero, t0, 0
SYM_FUNC_END(except_vec_vi_handler)

	.macro	__build_clear_none
	.endm

	.macro	__build_clear_sti
	TRACE_IRQS_ON
	STI
	.endm

	.macro	__build_clear_cli
	CLI
	TRACE_IRQS_OFF
	.endm

	.macro	__build_clear_fpe
	movfcsr2gr	a1, fcsr0
	CLI
	TRACE_IRQS_OFF
	.endm

	.macro	__build_clear_csti
	csrrd	t0, LOONGARCH_CSR_CRMD
	PTR_S	t0, sp, PT_CRMD
	csrrd	t0, LOONGARCH_CSR_EUEN
	PTR_S	t0, sp, PT_EUEN
	csrrd	t0, LOONGARCH_CSR_ECFG
	PTR_S	t0, sp, PT_ECFG
	csrrd	t0, LOONGARCH_CSR_ESTAT
	PTR_S	t0, sp, PT_ESTAT
	csrrd	t0, LOONGARCH_CSR_BADV
	PTR_S	t0, sp, PT_BVADDR
	STI
	.endm

	.macro	__build_clear_kmode
	csrrd	t0, LOONGARCH_CSR_CRMD
	PTR_S	t0, sp, PT_CRMD
	csrrd	t0, LOONGARCH_CSR_EUEN
	PTR_S	t0, sp, PT_EUEN
	csrrd	t0, LOONGARCH_CSR_ECFG
	PTR_S	t0, sp, PT_ECFG
	csrrd	t0, LOONGARCH_CSR_ESTAT
	PTR_S	t0, sp, PT_ESTAT
	csrrd	t0, LOONGARCH_CSR_BADV
	PTR_S	t0, sp, PT_BVADDR
	KMODE
	.endm

	.macro	__BUILD_HANDLER exception handler clear ext
	.align	5
	SYM_FUNC_START(handle_\exception)
	csrwr	t0, LOONGARCH_CSR_KS0
	csrwr	t1, LOONGARCH_CSR_KS1
	SAVE_ALL
	SYM_INNER_LABEL(handle_\exception\ext, SYM_L_GLOBAL)
	__build_clear_\clear
	move	a0, sp
	la.abs	t0, do_\handler
	jirl    ra, t0, 0
	la.abs	t0, ret_from_exception
	jirl    zero, t0, 0
	SYM_FUNC_END(handle_\exception)
	.endm

	.macro	BUILD_HANDLER exception handler clear verbose
	__BUILD_HANDLER \exception \handler \clear \verbose _int
	.endm

	BUILD_HANDLER ade ade kmode
	BUILD_HANDLER bp bp kmode
	BUILD_HANDLER ri ri csti
	BUILD_HANDLER fpu fpu sti
	BUILD_HANDLER fpe fpe fpe
	BUILD_HANDLER lsx lsx sti
	BUILD_HANDLER lasx lasx sti
	BUILD_HANDLER lbt lbt kmode
	BUILD_HANDLER ale ale kmode
	BUILD_HANDLER bce bce kmode
	BUILD_HANDLER reserved reserved csti	/* others */
	BUILD_HANDLER watch watch none

SYM_FUNC_START(handle_sys_wrap)
	la.abs	t0, handle_sys
	jirl    zero, t0, 0
SYM_FUNC_END(handle_sys_wrap)


/*
 * Macro helper for vectored interrupt handler.
 */
	.macro	BUILD_VI_HANDLER num
	.align	5
SYM_FUNC_START(handle_vi_\num)
	csrwr	t0, LOONGARCH_CSR_KS0
	csrwr	t1, LOONGARCH_CSR_KS1
	SAVE_SOME #docfi=1
	addi.d	v0, zero, \num
	la.abs	v1, except_vec_vi_handler
	jirl	zero, v1, 0
SYM_FUNC_END(handle_vi_\num)
	.endm

	BUILD_VI_HANDLER 0
	BUILD_VI_HANDLER 1
	BUILD_VI_HANDLER 2
	BUILD_VI_HANDLER 3
	BUILD_VI_HANDLER 4
	BUILD_VI_HANDLER 5
	BUILD_VI_HANDLER 6
	BUILD_VI_HANDLER 7
	BUILD_VI_HANDLER 8
	BUILD_VI_HANDLER 9
	BUILD_VI_HANDLER 10
	BUILD_VI_HANDLER 11
	BUILD_VI_HANDLER 12
	BUILD_VI_HANDLER 13

	.section .data, "aw"
	.align	3
        .globl  vector_table
vector_table:
	PTR	handle_vi_0
	PTR	handle_vi_1
	PTR	handle_vi_2
	PTR	handle_vi_3
	PTR	handle_vi_4
	PTR	handle_vi_5
	PTR	handle_vi_6
	PTR	handle_vi_7
	PTR	handle_vi_8
	PTR	handle_vi_9
	PTR	handle_vi_10
	PTR	handle_vi_11
	PTR	handle_vi_12
	PTR	handle_vi_13
