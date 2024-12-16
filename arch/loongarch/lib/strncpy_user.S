/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 */

#include <linux/errno.h>
#include <linux/linkage.h>

#include <asm/asm.h>
#include <asm/export.h>
#include <asm/regdef.h>
#include <asm/asm-extable.h>

.L_fixup_handle_strncpy:
	/* return -EFAULT if exception before terminator */
	li.w	a0, -EFAULT
	jr	ra

/*
 * long __strncpy_from_user(char *to, const char *from, long len)
 *
 * a0: to
 * a1: from
 * a2: len
 */
SYM_FUNC_START(__strncpy_from_user)
	move	a3, zero

1:	ld.b	t0, a1, 0
	st.b	t0, a0, 0
	addi.d	a0, a0, 1
	addi.d	a1, a1, 1
	beqz	t0, 2f

	addi.d	a3, a3, 1
	blt	a3, a2, 1b

	/*
	 * return len if the entire buffer filled,
	 * return strlen else
	 */
2:	move	a0, a3
	jr	ra

	_asm_extable 1b, .L_fixup_handle_strncpy
SYM_FUNC_END(__strncpy_from_user)
EXPORT_SYMBOL(__strncpy_from_user)
