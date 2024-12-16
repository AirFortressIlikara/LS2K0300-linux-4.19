/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 */

#include <linux/linkage.h>

#include <asm/asm.h>
#include <asm/export.h>
#include <asm/regdef.h>
#include <asm/asm-extable.h>

.L_fixup_handle_strnlen:
	/* return 0 in case of error */
	move	a0, zero
	jr	ra

/*
 * long __strnlen_user(const char *s, long n)
 *
 * a0: s
 * a1: n
 */
SYM_FUNC_START(__strnlen_user)
	move	a2, zero

1:	ld.b	t0, a0, 0
	addi.d	a0, a0, 1
	addi.d	a2, a2, 1
	beqz	t0, 2f

	bge	a1, a2, 1b

	/*
	 * return the size of a string including the ending NUL character
	 * up to a maximum of n
	 */
2:	move	a0, a2
	jr	ra

	_asm_extable 1b, .L_fixup_handle_strnlen
SYM_FUNC_END(__strnlen_user)
