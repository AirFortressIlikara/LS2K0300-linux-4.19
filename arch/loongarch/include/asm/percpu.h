/*
 * Copyright (C) 2020 Loongson Technology Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_PERCPU_H
#define __ASM_PERCPU_H

#include <asm/cmpxchg.h>

#ifdef CONFIG_CPU_LOONGSON64
/* use r21 for fast access */
register unsigned long __my_cpu_offset __asm__("$r21");
static inline void set_my_cpu_offset(unsigned long off)
{
	__my_cpu_offset = off;
	csr_write64(off, PERCPU_BASE_KS);
}
#define __my_cpu_offset __my_cpu_offset
#else
#define set_my_cpu_offset(x) do {} while (0)
#endif

#define PERCPU_OP(op, asm_op, c_op)					\
static inline unsigned long __percpu_##op(void *ptr,			\
			unsigned long val, int size)			\
{									\
	unsigned long ret;						\
									\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__(					\
		"am"#asm_op".w"	" %[ret], %[val], %[ptr]	\n"		\
		: [ret] "=&r" (ret), [ptr] "+ZB"(*(u32 *)ptr)		\
		: [val] "r" (val));					\
		break;							\
	case 8:								\
		__asm__ __volatile__(					\
		"am"#asm_op".d" " %[ret], %[val], %[ptr]	\n"		\
		: [ret] "=&r" (ret), [ptr] "+ZB"(*(u64 *)ptr)		\
		: [val] "r" (val));					\
		break;							\
	default:							\
		ret = 0;						\
		BUILD_BUG();						\
	}								\
									\
	return ret c_op val;						\
}

PERCPU_OP(add, add, +)
PERCPU_OP(and, and, &)
PERCPU_OP(or, or, |)
#undef PERCPU_OP

static inline unsigned long __percpu_read(void *ptr, int size)
{
	unsigned long ret;

	switch (size) {
	case 1:
		__asm__ __volatile__ ("ldx.b %[ret], $r21, %[ptr]	\n"
		: [ret] "=&r"(ret)
		: [ptr] "r"(ptr)
		: "memory");
		break;
	case 2:
		__asm__ __volatile__ ("ldx.h %[ret], $r21, %[ptr]	\n"
		: [ret] "=&r"(ret)
		: [ptr] "r"(ptr)
		: "memory");
		break;
	case 4:
		__asm__ __volatile__ ("ldx.w %[ret], $r21, %[ptr]	\n"
		: [ret] "=&r"(ret)
		: [ptr] "r"(ptr)
		: "memory");
		break;
	case 8:
		__asm__ __volatile__ ("ldx.d %[ret], $r21, %[ptr]	\n"
		: [ret] "=&r"(ret)
		: [ptr] "r"(ptr)
		: "memory");
		break;
	default:
		ret = 0;
		BUILD_BUG();
	}

	return ret;
}

static inline void __percpu_write(void *ptr, unsigned long val, int size)
{
	switch (size) {
	case 1:
		__asm__ __volatile__("stx.b %[val], $r21, %[ptr]	\n"
		:
		: [val] "r" (val), [ptr] "r" (ptr)
		: "memory");
		break;
	case 2:
		__asm__ __volatile__("stx.h %[val], $r21, %[ptr]	\n"
		:
		: [val] "r" (val), [ptr] "r" (ptr)
		: "memory");
		break;
	case 4:
		__asm__ __volatile__("stx.w %[val], $r21, %[ptr]	\n"
		:
		: [val] "r" (val), [ptr] "r" (ptr)
		: "memory");
		break;
	case 8:
		__asm__ __volatile__("stx.d %[val], $r21, %[ptr]	\n"
		:
		: [val] "r" (val), [ptr] "r" (ptr)
		: "memory");
		break;
	default:
		BUILD_BUG();
	}
}

static inline unsigned long __percpu_xchg(void *ptr, unsigned long val,
						int size)
{
	switch (size) {
	case 1:
	case 2:
		return __xchg_small(ptr, val, size);

	case 4:
		return __xchg_asm("amswap.w", (volatile u32 *)ptr, (u32)val);

	case 8:
		if (!IS_ENABLED(CONFIG_64BIT))
			return __xchg_called_with_bad_pointer();

		return __xchg_asm("amswap.d", (volatile u64 *)ptr, val);

	default:
		return __xchg_called_with_bad_pointer();
	}
}

/* this_cpu_cmpxchg */
#define _protect_cmpxchg_local(pcp, o, n)			\
({								\
	typeof(*raw_cpu_ptr(&(pcp))) __ret;			\
	preempt_disable();					\
	__ret = cmpxchg_local(raw_cpu_ptr(&(pcp)), o, n);	\
	preempt_enable();					\
	__ret;							\
})

#define this_cpu_cmpxchg_1(ptr, o, n) _protect_cmpxchg_local(ptr, o, n)
#define this_cpu_cmpxchg_2(ptr, o, n) _protect_cmpxchg_local(ptr, o, n)
#define this_cpu_cmpxchg_4(ptr, o, n) _protect_cmpxchg_local(ptr, o, n)
#define this_cpu_cmpxchg_8(ptr, o, n) _protect_cmpxchg_local(ptr, o, n)
#define _percpu_read(pcp)						\
({									\
	typeof(pcp) __retval;						\
	__retval = (typeof(pcp))__percpu_read(&(pcp), sizeof(pcp));	\
	__retval;							\
})

#define _percpu_write(pcp, val)						\
do {									\
	__percpu_write(&(pcp), (unsigned long)(val), sizeof(pcp));	\
} while (0)								\

#define _pcp_protect(operation, pcp, val)			\
({								\
	typeof(pcp) __retval;					\
	preempt_disable();					\
	__retval = (typeof(pcp))operation(raw_cpu_ptr(&(pcp)),	\
					  (val), sizeof(pcp));	\
	preempt_enable();					\
	__retval;						\
})

#define _percpu_add(pcp, val) \
	_pcp_protect(__percpu_add, pcp, val)

#define _percpu_add_return(pcp, val) _percpu_add(pcp, val)

#define _percpu_and(pcp, val) \
	_pcp_protect(__percpu_and, pcp, val)

#define _percpu_or(pcp, val) \
	_pcp_protect(__percpu_or, pcp, val)

#define _percpu_xchg(pcp, val) ((typeof(pcp)) \
	_pcp_protect(__percpu_xchg, pcp, (unsigned long)(val)))

#define this_cpu_add_4(pcp, val) _percpu_add(pcp, val)
#define this_cpu_add_8(pcp, val) _percpu_add(pcp, val)

#define this_cpu_add_return_4(pcp, val) _percpu_add_return(pcp, val)
#define this_cpu_add_return_8(pcp, val) _percpu_add_return(pcp, val)

#define this_cpu_and_4(pcp, val) _percpu_and(pcp, val)
#define this_cpu_and_8(pcp, val) _percpu_and(pcp, val)

#define this_cpu_or_4(pcp, val) _percpu_or(pcp, val)
#define this_cpu_or_8(pcp, val) _percpu_or(pcp, val)

#define this_cpu_read_1(pcp) _percpu_read(pcp)
#define this_cpu_read_2(pcp) _percpu_read(pcp)
#define this_cpu_read_4(pcp) _percpu_read(pcp)
#define this_cpu_read_8(pcp) _percpu_read(pcp)

#define this_cpu_write_1(pcp, val) _percpu_write(pcp, val)
#define this_cpu_write_2(pcp, val) _percpu_write(pcp, val)
#define this_cpu_write_4(pcp, val) _percpu_write(pcp, val)
#define this_cpu_write_8(pcp, val) _percpu_write(pcp, val)

#define this_cpu_xchg_1(pcp, val) _percpu_xchg(pcp, val)
#define this_cpu_xchg_2(pcp, val) _percpu_xchg(pcp, val)
#define this_cpu_xchg_4(pcp, val) _percpu_xchg(pcp, val)
#define this_cpu_xchg_8(pcp, val) _percpu_xchg(pcp, val)

#include <asm-generic/percpu.h>

#endif /* __ASM_PERCPU_H */
