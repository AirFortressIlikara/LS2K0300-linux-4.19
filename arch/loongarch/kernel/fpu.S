/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Pei Huang <huangpei@loongson.cn>
 * Copyright (C) 2019 Lu Zeng <zenglu@loongson.cn>
 * Copyright (C) 2020 Huacai Chen <chenhuacai@loongson.cn>
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/asm-extable.h>
#include <asm/errno.h>
#include <asm/export.h>
#include <asm/fpregdef.h>
#include <asm/loongarchregs.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>

#undef v0
#undef v1

#define FPU_SREG_WIDTH	32
#define SC_FPR0  0
#define SC_FPR1  (SC_FPR0  + FPU_SREG_WIDTH)
#define SC_FPR2  (SC_FPR1  + FPU_SREG_WIDTH)
#define SC_FPR3  (SC_FPR2  + FPU_SREG_WIDTH)
#define SC_FPR4  (SC_FPR3  + FPU_SREG_WIDTH)
#define SC_FPR5  (SC_FPR4  + FPU_SREG_WIDTH)
#define SC_FPR6  (SC_FPR5  + FPU_SREG_WIDTH)
#define SC_FPR7  (SC_FPR6  + FPU_SREG_WIDTH)
#define SC_FPR8  (SC_FPR7  + FPU_SREG_WIDTH)
#define SC_FPR9  (SC_FPR8  + FPU_SREG_WIDTH)
#define SC_FPR10 (SC_FPR9  + FPU_SREG_WIDTH)
#define SC_FPR11 (SC_FPR10 + FPU_SREG_WIDTH)
#define SC_FPR12 (SC_FPR11 + FPU_SREG_WIDTH)
#define SC_FPR13 (SC_FPR12 + FPU_SREG_WIDTH)
#define SC_FPR14 (SC_FPR13 + FPU_SREG_WIDTH)
#define SC_FPR15 (SC_FPR14 + FPU_SREG_WIDTH)
#define SC_FPR16 (SC_FPR15 + FPU_SREG_WIDTH)
#define SC_FPR17 (SC_FPR16 + FPU_SREG_WIDTH)
#define SC_FPR18 (SC_FPR17 + FPU_SREG_WIDTH)
#define SC_FPR19 (SC_FPR18 + FPU_SREG_WIDTH)
#define SC_FPR20 (SC_FPR19 + FPU_SREG_WIDTH)
#define SC_FPR21 (SC_FPR20 + FPU_SREG_WIDTH)
#define SC_FPR22 (SC_FPR21 + FPU_SREG_WIDTH)
#define SC_FPR23 (SC_FPR22 + FPU_SREG_WIDTH)
#define SC_FPR24 (SC_FPR23 + FPU_SREG_WIDTH)
#define SC_FPR25 (SC_FPR24 + FPU_SREG_WIDTH)
#define SC_FPR26 (SC_FPR25 + FPU_SREG_WIDTH)
#define SC_FPR27 (SC_FPR26 + FPU_SREG_WIDTH)
#define SC_FPR28 (SC_FPR27 + FPU_SREG_WIDTH)
#define SC_FPR29 (SC_FPR28 + FPU_SREG_WIDTH)
#define SC_FPR30 (SC_FPR29 + FPU_SREG_WIDTH)
#define SC_FPR31 (SC_FPR30 + FPU_SREG_WIDTH)

/* preprocessor replaces the fp in ".set fp=64" with $30 otherwise */
#undef fp

	.macro	EX insn, reg, src, offs
.ex\@:	\insn	\reg, \src, \offs
	_asm_extable .ex\@, fault
	.endm

	.macro	EX_V insn, reg, src, offs
	parse_v __insn, \insn
	parse_v __offs, \offs
	parse_r __src, \src
	parse_vr __reg, \reg

.ex\@:
	.word __insn << 22 | __offs << 10 | __src << 5 | __reg
	_asm_extable .ex\@, fault
	.endm

	.macro	EX_XV insn, reg, src, offs
	parse_v __insn, \insn
	parse_v __offs, \offs
	parse_r __src, \src
	parse_xr __reg, \reg

.ex\@:
	.word __insn << 22 | __offs << 10 | __src << 5 | __reg
	_asm_extable .ex\@, fault
	.endm

	.macro sc_save_fp base
	EX	fst.d $f0,  \base, SC_FPR0
	EX	fst.d $f1,  \base, SC_FPR1
	EX	fst.d $f2,  \base, SC_FPR2
	EX	fst.d $f3,  \base, SC_FPR3
	EX	fst.d $f4,  \base, SC_FPR4
	EX	fst.d $f5,  \base, SC_FPR5
	EX	fst.d $f6,  \base, SC_FPR6
	EX	fst.d $f7,  \base, SC_FPR7
	EX	fst.d $f8,  \base, SC_FPR8
	EX	fst.d $f9,  \base, SC_FPR9
	EX	fst.d $f10, \base, SC_FPR10
	EX	fst.d $f11, \base, SC_FPR11
	EX	fst.d $f12, \base, SC_FPR12
	EX	fst.d $f13, \base, SC_FPR13
	EX	fst.d $f14, \base, SC_FPR14
	EX	fst.d $f15, \base, SC_FPR15
	EX	fst.d $f16, \base, SC_FPR16
	EX	fst.d $f17, \base, SC_FPR17
	EX	fst.d $f18, \base, SC_FPR18
	EX	fst.d $f19, \base, SC_FPR19
	EX	fst.d $f20, \base, SC_FPR20
	EX	fst.d $f21, \base, SC_FPR21
	EX	fst.d $f22, \base, SC_FPR22
	EX	fst.d $f23, \base, SC_FPR23
	EX	fst.d $f24, \base, SC_FPR24
	EX	fst.d $f25, \base, SC_FPR25
	EX	fst.d $f26, \base, SC_FPR26
	EX	fst.d $f27, \base, SC_FPR27
	EX	fst.d $f28, \base, SC_FPR28
	EX	fst.d $f29, \base, SC_FPR29
	EX	fst.d $f30, \base, SC_FPR30
	EX	fst.d $f31, \base, SC_FPR31
	.endm

	.macro sc_restore_fp base
	EX	fld.d $f0,  \base, SC_FPR0
	EX	fld.d $f1,  \base, SC_FPR1
	EX	fld.d $f2,  \base, SC_FPR2
	EX	fld.d $f3,  \base, SC_FPR3
	EX	fld.d $f4,  \base, SC_FPR4
	EX	fld.d $f5,  \base, SC_FPR5
	EX	fld.d $f6,  \base, SC_FPR6
	EX	fld.d $f7,  \base, SC_FPR7
	EX	fld.d $f8,  \base, SC_FPR8
	EX	fld.d $f9,  \base, SC_FPR9
	EX	fld.d $f10, \base, SC_FPR10
	EX	fld.d $f11, \base, SC_FPR11
	EX	fld.d $f12, \base, SC_FPR12
	EX	fld.d $f13, \base, SC_FPR13
	EX	fld.d $f14, \base, SC_FPR14
	EX	fld.d $f15, \base, SC_FPR15
	EX	fld.d $f16, \base, SC_FPR16
	EX	fld.d $f17, \base, SC_FPR17
	EX	fld.d $f18, \base, SC_FPR18
	EX	fld.d $f19, \base, SC_FPR19
	EX	fld.d $f20, \base, SC_FPR20
	EX	fld.d $f21, \base, SC_FPR21
	EX	fld.d $f22, \base, SC_FPR22
	EX	fld.d $f23, \base, SC_FPR23
	EX	fld.d $f24, \base, SC_FPR24
	EX	fld.d $f25, \base, SC_FPR25
	EX	fld.d $f26, \base, SC_FPR26
	EX	fld.d $f27, \base, SC_FPR27
	EX	fld.d $f28, \base, SC_FPR28
	EX	fld.d $f29, \base, SC_FPR29
	EX	fld.d $f30, \base, SC_FPR30
	EX	fld.d $f31, \base, SC_FPR31
	.endm

	.macro sc_save_fcc base, tmp0, tmp1
	movcf2gr	\tmp0, $fcc0
	move	\tmp1, \tmp0
	movcf2gr	\tmp0, $fcc1
	bstrins.d	\tmp1, \tmp0, 15, 8
	movcf2gr	\tmp0, $fcc2
	bstrins.d	\tmp1, \tmp0, 23, 16
	movcf2gr	\tmp0, $fcc3
	bstrins.d	\tmp1, \tmp0, 31, 24
	movcf2gr	\tmp0, $fcc4
	bstrins.d	\tmp1, \tmp0, 39, 32
	movcf2gr	\tmp0, $fcc5
	bstrins.d	\tmp1, \tmp0, 47, 40
	movcf2gr	\tmp0, $fcc6
	bstrins.d	\tmp1, \tmp0, 55, 48
	movcf2gr	\tmp0, $fcc7
	bstrins.d	\tmp1, \tmp0, 63, 56
	EX	st.d \tmp1, \base, 0
	.endm

	.macro sc_restore_fcc base, tmp0, tmp1
	EX	ld.d \tmp0, \base, 0
	bstrpick.d	\tmp1, \tmp0, 7, 0
	movgr2cf	$fcc0, \tmp1
	bstrpick.d	\tmp1, \tmp0, 15, 8
	movgr2cf	$fcc1, \tmp1
	bstrpick.d	\tmp1, \tmp0, 23, 16
	movgr2cf	$fcc2, \tmp1
	bstrpick.d	\tmp1, \tmp0, 31, 24
	movgr2cf	$fcc3, \tmp1
	bstrpick.d	\tmp1, \tmp0, 39, 32
	movgr2cf	$fcc4, \tmp1
	bstrpick.d	\tmp1, \tmp0, 47, 40
	movgr2cf	$fcc5, \tmp1
	bstrpick.d	\tmp1, \tmp0, 55, 48
	movgr2cf	$fcc6, \tmp1
	bstrpick.d	\tmp1, \tmp0, 63, 56
	movgr2cf	$fcc7, \tmp1
	.endm

	.macro sc_save_fcsr base, tmp0
	movfcsr2gr	\tmp0, fcsr0
	EX	st.w \tmp0, \base, 0
#if defined(CONFIG_CPU_HAS_LBT)
	/* TM bit is always 0 if LBT not supported */
	andi	\tmp0, \tmp0, FPU_CSR_TM
	beqz	\tmp0, 1f
	bstrins.d \tmp0, $r0, FPU_CSR_TM_SHIFT, FPU_CSR_TM_SHIFT
	movgr2fcsr      fcsr0, \tmp0
	1:
#endif
	.endm

	.macro sc_restore_fcsr base, tmp0
	EX	ld.w \tmp0, \base, 0
	movgr2fcsr	fcsr0, \tmp0
	.endm

#if defined(CONFIG_CPU_HAS_LBT)
	.macro sc_save_scr base, tmp0
	parse_r	__reg, \tmp0
	.word 0x3 << 10 | 0x0 << 5 | __reg
	EX	st.d \tmp0, \base, 0
	.word 0x3 << 10 | 0x1 << 5 | __reg
	EX	st.d \tmp0, \base, 8
	.word 0x3 << 10 | 0x2 << 5 | __reg
	EX	st.d \tmp0, \base, 16
	.word 0x3 << 10 | 0x3 << 5 | __reg
	EX	st.d \tmp0, \base, 24
	.endm

	.macro sc_restore_scr base, tmp0
	parse_r	__reg, \tmp0
	EX	ld.d \tmp0, \base, 0
	.word 0x2 << 10 | __reg << 5 | 0x0
	EX	ld.d \tmp0, \base, 8
	.word 0x2 << 10 | __reg << 5 | 0x1
	EX	ld.d \tmp0, \base, 16
	.word 0x2 << 10 | __reg << 5 | 0x2
	EX	ld.d \tmp0, \base, 24
	.word 0x2 << 10 | __reg << 5 | 0x3
	.endm
#endif

	.macro sc_save_lsx base
	EX_V 0xb1 $vr0,  \base, SC_FPR0
	EX_V 0xb1 $vr1,  \base, SC_FPR1
	EX_V 0xb1 $vr2,  \base, SC_FPR2
	EX_V 0xb1 $vr3,  \base, SC_FPR3
	EX_V 0xb1 $vr4,  \base, SC_FPR4
	EX_V 0xb1 $vr5,  \base, SC_FPR5
	EX_V 0xb1 $vr6,  \base, SC_FPR6
	EX_V 0xb1 $vr7,  \base, SC_FPR7
	EX_V 0xb1 $vr8,  \base, SC_FPR8
	EX_V 0xb1 $vr9,  \base, SC_FPR9
	EX_V 0xb1 $vr10,  \base, SC_FPR10
	EX_V 0xb1 $vr11,  \base, SC_FPR11
	EX_V 0xb1 $vr12,  \base, SC_FPR12
	EX_V 0xb1 $vr13,  \base, SC_FPR13
	EX_V 0xb1 $vr14,  \base, SC_FPR14
	EX_V 0xb1 $vr15,  \base, SC_FPR15
	EX_V 0xb1 $vr16,  \base, SC_FPR16
	EX_V 0xb1 $vr17,  \base, SC_FPR17
	EX_V 0xb1 $vr18,  \base, SC_FPR18
	EX_V 0xb1 $vr19,  \base, SC_FPR19
	EX_V 0xb1 $vr20,  \base, SC_FPR20
	EX_V 0xb1 $vr21,  \base, SC_FPR21
	EX_V 0xb1 $vr22,  \base, SC_FPR22
	EX_V 0xb1 $vr23,  \base, SC_FPR23
	EX_V 0xb1 $vr24,  \base, SC_FPR24
	EX_V 0xb1 $vr25,  \base, SC_FPR25
	EX_V 0xb1 $vr26,  \base, SC_FPR26
	EX_V 0xb1 $vr27,  \base, SC_FPR27
	EX_V 0xb1 $vr28,  \base, SC_FPR28
	EX_V 0xb1 $vr29,  \base, SC_FPR29
	EX_V 0xb1 $vr30,  \base, SC_FPR30
	EX_V 0xb1 $vr31,  \base, SC_FPR31
	.endm

	.macro sc_restore_lsx base
	EX_V 0xb0 $vr0,  \base, SC_FPR0
	EX_V 0xb0 $vr1,  \base, SC_FPR1
	EX_V 0xb0 $vr2,  \base, SC_FPR2
	EX_V 0xb0 $vr3,  \base, SC_FPR3
	EX_V 0xb0 $vr4,  \base, SC_FPR4
	EX_V 0xb0 $vr5,  \base, SC_FPR5
	EX_V 0xb0 $vr6,  \base, SC_FPR6
	EX_V 0xb0 $vr7,  \base, SC_FPR7
	EX_V 0xb0 $vr8,  \base, SC_FPR8
	EX_V 0xb0 $vr9,  \base, SC_FPR9
	EX_V 0xb0 $vr10,  \base, SC_FPR10
	EX_V 0xb0 $vr11,  \base, SC_FPR11
	EX_V 0xb0 $vr12,  \base, SC_FPR12
	EX_V 0xb0 $vr13,  \base, SC_FPR13
	EX_V 0xb0 $vr14,  \base, SC_FPR14
	EX_V 0xb0 $vr15,  \base, SC_FPR15
	EX_V 0xb0 $vr16,  \base, SC_FPR16
	EX_V 0xb0 $vr17,  \base, SC_FPR17
	EX_V 0xb0 $vr18,  \base, SC_FPR18
	EX_V 0xb0 $vr19,  \base, SC_FPR19
	EX_V 0xb0 $vr20,  \base, SC_FPR20
	EX_V 0xb0 $vr21,  \base, SC_FPR21
	EX_V 0xb0 $vr22,  \base, SC_FPR22
	EX_V 0xb0 $vr23,  \base, SC_FPR23
	EX_V 0xb0 $vr24,  \base, SC_FPR24
	EX_V 0xb0 $vr25,  \base, SC_FPR25
	EX_V 0xb0 $vr26,  \base, SC_FPR26
	EX_V 0xb0 $vr27,  \base, SC_FPR27
	EX_V 0xb0 $vr28,  \base, SC_FPR28
	EX_V 0xb0 $vr29,  \base, SC_FPR29
	EX_V 0xb0 $vr30,  \base, SC_FPR30
	EX_V 0xb0 $vr31,  \base, SC_FPR31
	.endm

	.macro sc_save_lasx base
	EX_XV	0xb3 $xr0,  \base, SC_FPR0
	EX_XV	0xb3 $xr1,  \base, SC_FPR1
	EX_XV	0xb3 $xr2,  \base, SC_FPR2
	EX_XV	0xb3 $xr3,  \base, SC_FPR3
	EX_XV	0xb3 $xr4,  \base, SC_FPR4
	EX_XV	0xb3 $xr5,  \base, SC_FPR5
	EX_XV	0xb3 $xr6,  \base, SC_FPR6
	EX_XV	0xb3 $xr7,  \base, SC_FPR7
	EX_XV	0xb3 $xr8,  \base, SC_FPR8
	EX_XV	0xb3 $xr9,  \base, SC_FPR9
	EX_XV	0xb3 $xr10, \base, SC_FPR10
	EX_XV	0xb3 $xr11, \base, SC_FPR11
	EX_XV	0xb3 $xr12, \base, SC_FPR12
	EX_XV	0xb3 $xr13, \base, SC_FPR13
	EX_XV	0xb3 $xr14, \base, SC_FPR14
	EX_XV	0xb3 $xr15, \base, SC_FPR15
	EX_XV	0xb3 $xr16, \base, SC_FPR16
	EX_XV	0xb3 $xr17, \base, SC_FPR17
	EX_XV	0xb3 $xr18, \base, SC_FPR18
	EX_XV	0xb3 $xr19, \base, SC_FPR19
	EX_XV	0xb3 $xr20, \base, SC_FPR20
	EX_XV	0xb3 $xr21, \base, SC_FPR21
	EX_XV	0xb3 $xr22, \base, SC_FPR22
	EX_XV	0xb3 $xr23, \base, SC_FPR23
	EX_XV	0xb3 $xr24, \base, SC_FPR24
	EX_XV	0xb3 $xr25, \base, SC_FPR25
	EX_XV	0xb3 $xr26, \base, SC_FPR26
	EX_XV	0xb3 $xr27, \base, SC_FPR27
	EX_XV	0xb3 $xr28, \base, SC_FPR28
	EX_XV	0xb3 $xr29, \base, SC_FPR29
	EX_XV	0xb3 $xr30, \base, SC_FPR30
	EX_XV	0xb3 $xr31, \base, SC_FPR31
	.endm

	.macro sc_restore_lasx base
	EX_XV	0xb2 $xr0,  \base, SC_FPR0
	EX_XV	0xb2 $xr1,  \base, SC_FPR1
	EX_XV	0xb2 $xr2,  \base, SC_FPR2
	EX_XV	0xb2 $xr3,  \base, SC_FPR3
	EX_XV	0xb2 $xr4,  \base, SC_FPR4
	EX_XV	0xb2 $xr5,  \base, SC_FPR5
	EX_XV	0xb2 $xr6,  \base, SC_FPR6
	EX_XV	0xb2 $xr7,  \base, SC_FPR7
	EX_XV	0xb2 $xr8,  \base, SC_FPR8
	EX_XV	0xb2 $xr9,  \base, SC_FPR9
	EX_XV	0xb2 $xr10, \base, SC_FPR10
	EX_XV	0xb2 $xr11, \base, SC_FPR11
	EX_XV	0xb2 $xr12, \base, SC_FPR12
	EX_XV	0xb2 $xr13, \base, SC_FPR13
	EX_XV	0xb2 $xr14, \base, SC_FPR14
	EX_XV	0xb2 $xr15, \base, SC_FPR15
	EX_XV	0xb2 $xr16, \base, SC_FPR16
	EX_XV	0xb2 $xr17, \base, SC_FPR17
	EX_XV	0xb2 $xr18, \base, SC_FPR18
	EX_XV	0xb2 $xr19, \base, SC_FPR19
	EX_XV	0xb2 $xr20, \base, SC_FPR20
	EX_XV	0xb2 $xr21, \base, SC_FPR21
	EX_XV	0xb2 $xr22, \base, SC_FPR22
	EX_XV	0xb2 $xr23, \base, SC_FPR23
	EX_XV	0xb2 $xr24, \base, SC_FPR24
	EX_XV	0xb2 $xr25, \base, SC_FPR25
	EX_XV	0xb2 $xr26, \base, SC_FPR26
	EX_XV	0xb2 $xr27, \base, SC_FPR27
	EX_XV	0xb2 $xr28, \base, SC_FPR28
	EX_XV	0xb2 $xr29, \base, SC_FPR29
	EX_XV	0xb2 $xr30, \base, SC_FPR30
	EX_XV	0xb2 $xr31, \base, SC_FPR31
	.endm

/*
 * Save a thread's fp context.
 */
SYM_FUNC_START(_save_fp)
	/*
	 * since TM bit of FSCR may afftect fpr0-fp7
	 * fcsr save need before FPR save
	 */
	fpu_save_csr	a0 t1
	fpu_save_double a0 t1			# clobbers t1
	fpu_save_cc	a0 t1 t2		# clobbers t1, t2
	jirl zero, ra, 0
SYM_FUNC_END(_save_fp)
EXPORT_SYMBOL(_save_fp)

/*
 * Restore a thread's fp context.
 */
SYM_FUNC_START(_restore_fp)
	fpu_restore_double a0 t1		# clobbers t1
	/*
	 * since TM bit of FSCR may afftect fpr0-fp7
	 * fscr restore need be after FPR store
	 */
	fpu_restore_csr	a0 t1
	fpu_restore_cc	a0 t1 t2		# clobbers t1, t2
	jirl zero, ra, 0
SYM_FUNC_END(_restore_fp)

#ifdef CONFIG_CPU_HAS_LSX

/*
 * Save a thread's LSX vector context.
 */
SYM_FUNC_START(_save_lsx)
	lsx_save_all	a0 t1 t2
	jirl zero, ra, 0
SYM_FUNC_END(_save_lsx)
EXPORT_SYMBOL(_save_lsx)

/*
 * Restore a thread's LSX vector context.
 */
SYM_FUNC_START(_restore_lsx)
	lsx_restore_all	a0 t1 t2
	jirl zero, ra, 0
SYM_FUNC_END(_restore_lsx)

SYM_FUNC_START(_save_lsx_upper)
	lsx_save_all_upper a0 t0 t1
	jirl zero, ra, 0
SYM_FUNC_END(_save_lsx_upper)

SYM_FUNC_START(_restore_lsx_upper)
	lsx_restore_all_upper a0 t0 t1
	jirl zero, ra, 0
SYM_FUNC_END(_restore_lsx_upper)

SYM_FUNC_START(_init_lsx_upper)
	lsx_init_all_upper t1
	jirl zero, ra, 0
SYM_FUNC_END(_init_lsx_upper)
#endif

#ifdef CONFIG_CPU_HAS_LASX

/*
 * Save a thread's LASX vector context.
 */
SYM_FUNC_START(_save_lasx)
	lasx_save_all	a0 t1 t2
	jirl zero, ra, 0
SYM_FUNC_END(_save_lasx)
EXPORT_SYMBOL(_save_lasx)

/*
 * Restore a thread's LASX vector context.
 */
SYM_FUNC_START(_restore_lasx)
	lasx_restore_all a0 t1 t2
	jirl zero, ra, 0
SYM_FUNC_END(_restore_lasx)

SYM_FUNC_START(_save_lasx_upper)
	lasx_save_all_upper a0 t0 t1
	jirl zero, ra, 0
SYM_FUNC_END(_save_lasx_upper)

SYM_FUNC_START(_restore_lasx_upper)
	lasx_restore_all_upper a0 t0 t1
	jirl	zero, ra, 0
SYM_FUNC_END(_restore_lasx_upper)

SYM_FUNC_START(_init_lasx_upper)
	lasx_init_all_upper t1
	jirl	zero, ra, 0
SYM_FUNC_END(_init_lasx_upper)
#endif

/*
 * Load the FPU with signalling NANS.  This bit pattern we're using has
 * the property that no matter whether considered as single or as double
 * precision represents signaling NANS.
 *
 * The value to initialize fcsr0 to comes in $a0.
 */

SYM_FUNC_START(_init_fpu)
	li.w	t1, CSR_EUEN_FPEN
	csrxchg	t1, t1, LOONGARCH_CSR_EUEN

	movgr2fcsr	fcsr0, a0

	li.w	t1, -1				# SNaN

	movgr2fr.d	$f0, t1
	movgr2fr.d	$f1, t1
	movgr2fr.d	$f2, t1
	movgr2fr.d	$f3, t1
	movgr2fr.d	$f4, t1
	movgr2fr.d	$f5, t1
	movgr2fr.d	$f6, t1
	movgr2fr.d	$f7, t1
	movgr2fr.d	$f8, t1
	movgr2fr.d	$f9, t1
	movgr2fr.d	$f10, t1
	movgr2fr.d	$f11, t1
	movgr2fr.d	$f12, t1
	movgr2fr.d	$f13, t1
	movgr2fr.d	$f14, t1
	movgr2fr.d	$f15, t1
	movgr2fr.d	$f16, t1
	movgr2fr.d	$f17, t1
	movgr2fr.d	$f18, t1
	movgr2fr.d	$f19, t1
	movgr2fr.d	$f20, t1
	movgr2fr.d	$f21, t1
	movgr2fr.d	$f22, t1
	movgr2fr.d	$f23, t1
	movgr2fr.d	$f24, t1
	movgr2fr.d	$f25, t1
	movgr2fr.d	$f26, t1
	movgr2fr.d	$f27, t1
	movgr2fr.d	$f28, t1
	movgr2fr.d	$f29, t1
	movgr2fr.d	$f30, t1
	movgr2fr.d	$f31, t1

	jirl zero, ra, 0
SYM_FUNC_END(_init_fpu)

/*
 * a0: fpregs
 * a1: fcc
 * a2: fcsr
 */
SYM_FUNC_START(_save_fp_context)
	sc_save_fcc a1 t1 t2
	sc_save_fcsr a2 t1
	sc_save_fp a0
	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_save_fp_context)

/*
 * a0: fpregs
 * a1: fcc
 * a2: fcsr
 */
SYM_FUNC_START(_restore_fp_context)
	sc_restore_fp a0
	sc_restore_fcc a1 t1 t2
	sc_restore_fcsr a2 t1
	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_restore_fp_context)

/*
 * a0: fpregs
 * a1: fcc
 * a2: fcsr
 */
SYM_FUNC_START(_save_lsx_context)
	sc_save_fcc a1, t0, t1
	sc_save_fcsr a2, t0
	sc_save_lsx a0
	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_save_lsx_context)

/*
 * a0: fpregs
 * a1: fcc
 * a2: fcsr
 */
SYM_FUNC_START(_restore_lsx_context)
	sc_restore_lsx a0
	sc_restore_fcc a1, t1, t2
	sc_restore_fcsr a2, t1
	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_restore_lsx_context)

#if defined(CONFIG_CPU_HAS_LBT)
/*
 * a0: scr
 */
SYM_FUNC_START(_save_scr_context)
	parse_r	__reg, t0
	/* eflags */
	.word 0x17 << 18 | 0x3f << 10 | 0 << 5 | __reg
	EX st.w	t0, a1, 0

	sc_save_scr a0, t0

	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_save_scr_context)

/*
 * a0: scr
 */
SYM_FUNC_START(_restore_scr_context)
	parse_r	__reg, t0
	/* eflags */
	EX ld.w	t0, a1, 0
	.word 0x17 << 18 | 0x3f << 10 | 1 << 5 | __reg

	sc_restore_scr a0, t1

	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_restore_scr_context)
#endif

/*
 * a0: fpregs
 * a1: fcc
 * a2: fcsr
 */
SYM_FUNC_START(_save_lasx_context)
	sc_save_fcc a1, t0, t1
	sc_save_fcsr a2, t0
	sc_save_lasx a0
	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_save_lasx_context)

/*
 * a0: fpregs
 * a1: fcc
 * a2: fcsr
 */
SYM_FUNC_START(_restore_lasx_context)
	sc_restore_lasx a0
	sc_restore_fcc a1, t1, t2
	sc_restore_fcsr a2, t1
	li.w	a0, 0					# success
	jirl zero, ra, 0
SYM_FUNC_END(_restore_lasx_context)

	.type	fault, @function
fault:	li.w	a0, -EFAULT				# failure
	jirl zero, ra, 0
