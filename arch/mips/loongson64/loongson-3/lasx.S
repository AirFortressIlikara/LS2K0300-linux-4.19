/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Huang Pei <huangpei@loongson.cn>
 * Copyright (C) 2019 Zeng Lu <zenglu@loongson.cn>
 */
#include <asm/asmmacro.h>
#include <asm/errno.h>

/* preprocessor replaces the fp in ".set fp=64" with $30 otherwise */
#undef fp


#ifdef CONFIG_CPU_HAS_LASX
	.macro	lasx_init_upper	wd
	.set	push
	.set	noat
	xinsert_d \wd, 2
	xinsert_d \wd, 3
	.endm

	.macro	lasx_init_uppest	wd
	.set	push
	.set	noat
	insert_d \wd, 1
	xinsert_d \wd, 2
	xinsert_d \wd, 3
	.endm

/*
 * Save a thread's LASX vector context.
 */
LEAF(_save_lasx)
	.set	push
	.set	noat
	.set	noreorder
	SET_HARDFLOAT
	_cfcmsa	$1, MSA_CSR
	sw	$1, THREAD_MSA_CSR(a0)
	xvst_b	16, THREAD_FPR16, a0
	xvst_b_off	17, (THREAD_FPR1-THREAD_FPR0)
	xvst_b_off	18, (THREAD_FPR2-THREAD_FPR0)
	xvst_b_off	19, (THREAD_FPR3-THREAD_FPR0)
	xvst_b_off	20, (THREAD_FPR4-THREAD_FPR0)
	xvst_b_off	21, (THREAD_FPR5-THREAD_FPR0)
	xvst_b_off	22, (THREAD_FPR6-THREAD_FPR0)
	xvst_b_off	23, (THREAD_FPR7-THREAD_FPR0)
	xvst_b_off	24, (THREAD_FPR8-THREAD_FPR0)
	xvst_b_off	25, (THREAD_FPR9-THREAD_FPR0)
	xvst_b_off	26, (THREAD_FPR10-THREAD_FPR0)
	xvst_b_off	27, (THREAD_FPR11-THREAD_FPR0)
	xvst_b_off	28, (THREAD_FPR12-THREAD_FPR0)
	xvst_b_off	29, (THREAD_FPR13-THREAD_FPR0)
	xvst_b_off	30, (THREAD_FPR14-THREAD_FPR0)
	xvst_b_off	31, (THREAD_FPR15-THREAD_FPR0)
	xvst_b_off	0, (THREAD_FPR16-THREAD_FPR0)
	xvst_b_off	1, (THREAD_FPR17-THREAD_FPR0)
	xvst_b_off	2, (THREAD_FPR18-THREAD_FPR0)
	xvst_b_off	3, (THREAD_FPR19-THREAD_FPR0)
	xvst_b_off	4, (THREAD_FPR20-THREAD_FPR0)
	xvst_b_off	5, (THREAD_FPR21-THREAD_FPR0)
	xvst_b_off	6, (THREAD_FPR22-THREAD_FPR0)
	xvst_b_off	7, (THREAD_FPR23-THREAD_FPR0)
	xvst_b_off	8, (THREAD_FPR24-THREAD_FPR0)
	xvst_b_off	9, (THREAD_FPR25-THREAD_FPR0)
	xvst_b_off	10, (THREAD_FPR26-THREAD_FPR0)
	xvst_b_off	11, (THREAD_FPR27-THREAD_FPR0)
	xvst_b_off	12, (THREAD_FPR28-THREAD_FPR0)
	xvst_b_off	13, (THREAD_FPR29-THREAD_FPR0)
	xvst_b_off	14, (THREAD_FPR30-THREAD_FPR0)
	jr	ra
	xvst_b_off	15, (THREAD_FPR31-THREAD_FPR0)
END(_save_lasx)

/*
 * Restore a thread's LASX vector context.
 */
LEAF(_restore_lasx)
	.set	push
	.set	noat
	SET_HARDFLOAT
	lw	$1, THREAD_MSA_CSR(a0)
	_ctcmsa	MSA_CSR, $1
	.set	pop
	xvld_b	16, THREAD_FPR16, a0
	xvld_b_off	17, (THREAD_FPR1-THREAD_FPR0)
	xvld_b_off	18, (THREAD_FPR2-THREAD_FPR0)
	xvld_b_off	19, (THREAD_FPR3-THREAD_FPR0)
	xvld_b_off	20, (THREAD_FPR4-THREAD_FPR0)
	xvld_b_off	21, (THREAD_FPR5-THREAD_FPR0)
	xvld_b_off	22, (THREAD_FPR6-THREAD_FPR0)
	xvld_b_off	23, (THREAD_FPR7-THREAD_FPR0)
	xvld_b_off	24, (THREAD_FPR8-THREAD_FPR0)
	xvld_b_off	25, (THREAD_FPR9-THREAD_FPR0)
	xvld_b_off	26, (THREAD_FPR10-THREAD_FPR0)
	xvld_b_off	27, (THREAD_FPR11-THREAD_FPR0)
	xvld_b_off	28, (THREAD_FPR12-THREAD_FPR0)
	xvld_b_off	29, (THREAD_FPR13-THREAD_FPR0)
	xvld_b_off	30, (THREAD_FPR14-THREAD_FPR0)
	xvld_b_off	31, (THREAD_FPR15-THREAD_FPR0)
	xvld_b_off	0, (THREAD_FPR16-THREAD_FPR0)
	xvld_b_off	1, (THREAD_FPR17-THREAD_FPR0)
	xvld_b_off	2, (THREAD_FPR18-THREAD_FPR0)
	xvld_b_off	3, (THREAD_FPR19-THREAD_FPR0)
	xvld_b_off	4, (THREAD_FPR20-THREAD_FPR0)
	xvld_b_off	5, (THREAD_FPR21-THREAD_FPR0)
	xvld_b_off	6, (THREAD_FPR22-THREAD_FPR0)
	xvld_b_off	7, (THREAD_FPR23-THREAD_FPR0)
	xvld_b_off	8, (THREAD_FPR24-THREAD_FPR0)
	xvld_b_off	9, (THREAD_FPR25-THREAD_FPR0)
	xvld_b_off	10, (THREAD_FPR26-THREAD_FPR0)
	xvld_b_off	11, (THREAD_FPR27-THREAD_FPR0)
	xvld_b_off	12, (THREAD_FPR28-THREAD_FPR0)
	xvld_b_off	13, (THREAD_FPR29-THREAD_FPR0)
	xvld_b_off	14, (THREAD_FPR30-THREAD_FPR0)
	jr	ra
	xvld_b_off	15, (THREAD_FPR31-THREAD_FPR0)
END(_restore_lasx)

LEAF(_init_lasx_upper)
	.set	push
	.set	noat
	.set	noreorder
	SET_HARDFLOAT
	not	$1, zero
	lasx_init_upper	0
	xvseli_d 0xc, 0, 1
	xvseli_d 0xc, 0, 2
	xvseli_d 0xc, 0, 3
	xvseli_d 0xc, 0, 4
	xvseli_d 0xc, 0, 5
	xvseli_d 0xc, 0, 6
	xvseli_d 0xc, 0, 7
	xvseli_d 0xc, 0, 8
	xvseli_d 0xc, 0, 9
	xvseli_d 0xc, 0, 10
	xvseli_d 0xc, 0, 11
	xvseli_d 0xc, 0, 12
	xvseli_d 0xc, 0, 13
	xvseli_d 0xc, 0, 14
	xvseli_d 0xc, 0, 15
	xvseli_d 0xc, 0, 16
	xvseli_d 0xc, 0, 17
	xvseli_d 0xc, 0, 18
	xvseli_d 0xc, 0, 19
	xvseli_d 0xc, 0, 20
	xvseli_d 0xc, 0, 21
	xvseli_d 0xc, 0, 22
	xvseli_d 0xc, 0, 23
	xvseli_d 0xc, 0, 24
	xvseli_d 0xc, 0, 25
	xvseli_d 0xc, 0, 26
	xvseli_d 0xc, 0, 27
	xvseli_d 0xc, 0, 28
	xvseli_d 0xc, 0, 29
	xvseli_d 0xc, 0, 30
	jr	ra
	xvseli_d 0xc, 0, 31
	.set	pop
END(_init_lasx_upper)
#endif
