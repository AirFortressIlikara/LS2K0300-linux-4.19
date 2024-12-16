/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2006  Ralf Baechle (ralf@linux-mips.org)
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_FUTEX_H
#define _ASM_FUTEX_H

#ifdef __KERNEL__

#include <linux/futex.h>
#include <linux/uaccess.h>
#include <asm/asm-extable.h>
#include <asm/barrier.h>
#include <asm/errno.h>

#ifdef LAMO_INSTRUCTIONS
#define __futex_atomic_op(insn, ret, oldval, uaddr, oparg)	\
{								\
	__asm__ __volatile__(					\
	"1:	"	insn "			\n"		\
	"2:					\n"		\
	_ASM_EXTABLE_UACCESS_ERR(1b, 2b, %0)			\
	: "+r" (ret), "=&r" (oldval), "+ZB" (*uaddr)		\
	: "Jr" (oparg)						\
	: "memory");						\
}

#else

#define __futex_atomic_op(insn, ret, oldval, uaddr, oparg)	\
{								\
	__asm__ __volatile__(					\
	"1:	ll.w	%1, %4 # __futex_atomic_op\n"		\
	"	" insn	"				\n"	\
	"2:	sc.w	$t0, %2				\n"	\
	"	beq	$t0, $zero, 1b			\n"	\
	"3:						\n"	\
	_ASM_EXTABLE_UACCESS_ERR(1b, 3b, %0)			\
	_ASM_EXTABLE_UACCESS_ERR(2b, 3b, %0)			\
	: "=r" (ret), "=&r" (oldval),				\
	  "=ZC" (*uaddr)					\
	: "0" (0), "ZC" (*uaddr), "Jr" (oparg)			\
	: "memory", "t0");					\
}
#endif

static inline int
arch_futex_atomic_op_inuser(int op, int oparg, int *oval, u32 __user *uaddr)
{
	int oldval = 0, ret = 0;

	pagefault_disable();

	switch (op) {
#ifdef LAMO_INSTRUCTIONS
	case FUTEX_OP_SET:
		__futex_atomic_op("amswap_db.w %1, %z3, %2", ret, oldval, uaddr, oparg);
		break;
	case FUTEX_OP_ADD:
		__futex_atomic_op("amadd_db.w %1, %z3, %2", ret, oldval, uaddr, oparg);
		break;
	case FUTEX_OP_OR:
		__futex_atomic_op("amor_db.w  %1, %z3, %2", ret, oldval, uaddr, oparg);
		break;
	case FUTEX_OP_ANDN:
		__futex_atomic_op("amand_db.w %1, %z3, %2", ret, oldval, uaddr, ~oparg);
		break;
	case FUTEX_OP_XOR:
		__futex_atomic_op("amxor_db.w %1, %z3, %2", ret, oldval, uaddr, oparg);
		break;
#else
	case FUTEX_OP_SET:
		__futex_atomic_op("move $t0, %z5", ret, oldval, uaddr, oparg);
		break;
	case FUTEX_OP_ADD:
		__futex_atomic_op("add.w $t0, %1, %z5", ret, oldval, uaddr, oparg);
		break;
	case FUTEX_OP_OR:
		__futex_atomic_op("or	$t0, %1, %z5", ret, oldval, uaddr, oparg);
		break;
	case FUTEX_OP_ANDN:
		__futex_atomic_op("and	$t0, %1, %z5", ret, oldval, uaddr, ~oparg);
		break;
	case FUTEX_OP_XOR:
		__futex_atomic_op("xor	$t0, %1, %z5", ret, oldval, uaddr, oparg);
		break;
#endif
	default:
		ret = -ENOSYS;
	}

	pagefault_enable();

	if (!ret)
		*oval = oldval;

	return ret;
}

static inline int
futex_atomic_cmpxchg_inatomic(u32 *uval, u32 __user *uaddr,
			      u32 oldval, u32 newval)
{
	int ret = 0;
	u32 val = 0;

	if (!access_ok(VERIFY_WRITE, uaddr, sizeof(u32)))
		return -EFAULT;

	__asm__ __volatile__(
	"# futex_atomic_cmpxchg_inatomic			\n"
	"1:	ll.w	%1, %3					\n"
	"	bne	%1, %z4, 3f				\n"
	"	or	$t0, %z5, $zero				\n"
	"2:	sc.w	$t0, %2					\n"
	"	beq	$zero, $t0, 1b				\n"
	"3:							\n"
	__WEAK_LLSC_MB
	_ASM_EXTABLE_UACCESS_ERR(1b, 3b, %0)
	_ASM_EXTABLE_UACCESS_ERR(2b, 3b, %0)
	: "+r" (ret), "=&r" (val), "=ZC" (*uaddr)
	: "ZC" (*uaddr), "Jr" (oldval), "Jr" (newval)
	: "memory", "t0");

	*uval = val;
	return ret;
}

#endif
#endif /* _ASM_FUTEX_H */
