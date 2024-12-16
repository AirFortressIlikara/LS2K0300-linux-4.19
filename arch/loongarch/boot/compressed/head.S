/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 1995 Waldorf Electronics
 * Written by Ralf Baechle and Andreas Busse
 * Copyright (C) 1995 - 1999 Ralf Baechle
 * Copyright (C) 1996 Paul M. Antoine
 * Modified for DECStation and hence R3000 support by Paul M. Antoine
 * Further modifications by David S. Miller and Harald Koerfgen
 * Copyright (C) 1999 Silicon Graphics, Inc.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/addrspace.h>
#include <asm/loongarchregs.h>

SYM_CODE_START(start)
	/* Save boot rom start args */
	move	s0, a0
	move	s1, a1
	move	s2, a2
	move	s3, a3 /* for kdump */

	# 0x9000 xxxx xxxx xxxx
	li.d	t0, CSR_DMW1_INIT	# CA, PLV0
	csrwr	t0, LOONGARCH_CSR_DMWIN1

	/* Clear BSS */
	la	a0, _edata
	la	a2, _end
1:	st.d	zero, a0, 0
	addi.d	a0, a0, 8
	bne	a2, a0, 1b

	la	sp, .stack + 8192  /* stack address */

	la	a0, .heap	  /* heap address */
	move	a1, a3		  /* kdump relocate offset */
	li.w	t1, 0x123
	beq	t1, a4, 2f
	li.w	a1, 0
	li.w	s3, 0
2:
	la	ra, 3f
	la	t4, decompress_kernel
	jirl	zero, t4, 0
3:
	move	a0, s0
	move	a1, s1
	move	a2, s2
	move	a3, s3
	PTR_LI	t4, KERNEL_ENTRY
	add.d	t4, t4, a3
	jirl	zero, t4, 0
4:
	b	4b
SYM_CODE_END(start)

	.comm .heap,BOOT_HEAP_SIZE,4
	.comm .stack,4096*2,4
