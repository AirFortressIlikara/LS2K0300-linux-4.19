/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 */

#include <linux/linkage.h>

#include <asm/alternative-asm.h>
#include <asm/asmmacro.h>
#include <asm/cpu.h>
#include <asm/export.h>
#include <asm/regdef.h>
#include <asm/errno.h>
#include <asm/asm-extable.h>

.L_fixup_handle_unaligned:
	li.w	v0, -EFAULT
	jr	ra

/*
 * unsigned long probe_user_read(void *addr, void *value, unsigned long n, bool symbol)
 *
 * a0: addr
 * a1: value
 * a2: n
 * a3: symbol
 */
SYM_FUNC_START(unaligned_read)
	beqz	a2, 5f

	li.w	t1, 8
	li.w	t2, 0

	addi.d	t0, a2, -1
	mul.d	t1, t0, t1
	add.d 	a0, a0, t0

	beq	a3, zero, 2f
1:	ld.b	t3, a0, 0
	b	3f

2:	ld.bu	t3, a0, 0
3:	sll.d	t3, t3, t1
	or	t2, t2, t3
	addi.d	t1, t1, -8
	addi.d	a0, a0, -1
	addi.d	a2, a2, -1
	bgt	a2, zero, 2b
4:	st.d	t2, a1, 0

	move	v0, a2
	jr	ra

5:	li.w    v0, -EFAULT
	jr	ra

	_asm_extable 1b, .L_fixup_handle_unaligned
	_asm_extable 2b, .L_fixup_handle_unaligned
	_asm_extable 4b, .L_fixup_handle_unaligned
SYM_FUNC_END(unaligned_read)

/*
 * unsigned long unaligned_write(void *addr, unsigned long value, unsigned long n)
 *
 * a0: addr
 * a1: value
 * a2: n
 */
SYM_FUNC_START(unaligned_write)
	beqz	a2, 3f

	li.w	t0, 0
1:	srl.d	t1, a1, t0
2:	st.b	t1, a0, 0
	addi.d	t0, t0, 8
	addi.d	a2, a2, -1
	addi.d	a0, a0, 1
	bgt	a2, zero, 1b

	move	v0, a2
	jr	ra

3:	li.w    v0, -EFAULT
	jr	ra

	_asm_extable 2b, .L_fixup_handle_unaligned
SYM_FUNC_END(unaligned_write)
