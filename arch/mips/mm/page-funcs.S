/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Micro-assembler generated clear_page/copy_page functions.
 *
 * Copyright (C) 2012  MIPS Technologies, Inc.
 * Copyright (C) 2012  Ralf Baechle <ralf@linux-mips.org>
 */
#include <asm/asm.h>
#include <asm/export.h>
#include <asm/regdef.h>

#ifdef CONFIG_SIBYTE_DMA_PAGEOPS
#define cpu_clear_page_function_name	clear_page_cpu
#define cpu_copy_page_function_name	copy_page_cpu
#else
#define cpu_clear_page_function_name	clear_page
#define cpu_copy_page_function_name	copy_page
#endif

#ifdef CONFIG_PAGE_SIZE_4KB
#define PAGE_SHIFT      12
#endif
#ifdef CONFIG_PAGE_SIZE_8KB
#define PAGE_SHIFT      13
#endif
#ifdef CONFIG_PAGE_SIZE_16KB
#define PAGE_SHIFT      14
#endif
#ifdef CONFIG_PAGE_SIZE_32KB
#define PAGE_SHIFT      15
#endif
#ifdef CONFIG_PAGE_SIZE_64KB
#define PAGE_SHIFT      16
#endif

#define PAGE_SIZE	(1 << PAGE_SHIFT)

#if !defined(CONFIG_CPU_LOONGSON3) && !defined(CONFIG_CPU_LOONGSON2K)
/*
 * Maximum sizes:
 *
 * R4000 128 bytes S-cache:		0x058 bytes
 * R4600 v1.7:				0x05c bytes
 * R4600 v2.0:				0x060 bytes
 * With prefetching, 16 word strides	0x120 bytes
 */
EXPORT(__clear_page_start)
LEAF(cpu_clear_page_function_name)
EXPORT_SYMBOL(cpu_clear_page_function_name)
1:	j	1b		/* Dummy, will be replaced. */
	.space 288
END(cpu_clear_page_function_name)
EXPORT(__clear_page_end)

/*
 * Maximum sizes:
 *
 * R4000 128 bytes S-cache:		0x11c bytes
 * R4600 v1.7:				0x080 bytes
 * R4600 v2.0:				0x07c bytes
 * With prefetching, 16 word strides	0x540 bytes
 */
EXPORT(__copy_page_start)
LEAF(cpu_copy_page_function_name)
EXPORT_SYMBOL(cpu_copy_page_function_name)
1:	j	1b		/* Dummy, will be replaced. */
	.space 1344
END(cpu_copy_page_function_name)
EXPORT(__copy_page_end)
#endif

#if defined(CONFIG_CPU_LOONGSON3) || defined(CONFIG_CPU_LOONGSON2K)
/*
 * As we are sharing code base with the mips32 tree (which use the o32 ABI
 * register definitions). We need to redefine the register definitions from
 * the n64 ABI register naming to the o32 ABI register naming.
 */
#undef t0
#undef t1
#undef t2
#undef t3
#define t0      $8
#define t1      $9
#define t2      $10
#define t3      $11
#define t4      $12
#define t5      $13
#define t6      $14
#define t7      $15


.set noreorder
.align 5
LEAF(clear_page)
EXPORT_SYMBOL(clear_page)
ori      a2, a0,  (PAGE_SIZE - 128)
	daddiu   a2, a2,  128
	1:
	gssq    zero, zero,   0(a0)
	gssq    zero, zero,  16(a0)
	gssq    zero, zero,  32(a0)
	gssq    zero, zero,  48(a0)
	daddiu  a0,  a0,  128
	gssq    zero, zero,  -64(a0)
	gssq    zero, zero,  -48(a0)
	gssq    zero, zero,  -32(a0)
	bne      a2, a0, 1b
	gssq    zero, zero,  -16(a0)

	jr      ra
	nop
END(clear_page)

.set noreorder
.align 5
LEAF(copy_page)
EXPORT_SYMBOL(copy_page)
ori      a2, a0,  (PAGE_SIZE - 128)
	daddiu   a2, a2,  128
	1:
	gslq     t1, t0,   0(a1)
	gslq     t3, t2,  16(a1)
	gslq     t5, t4,  32(a1)
	gslq     t7, t6,  48(a1)
	gssq     t1, t0,   0(a0)
	gslq     t1, t0,  64(a1)
	gssq     t3, t2,  16(a0)
	gslq     t3, t2,  80(a1)
	gssq     t5, t4,  32(a0)
	gslq     t5, t4,  96(a1)
	gssq     t7, t6,  48(a0)
	gslq     t7, t6,  112(a1)
	daddiu   a0, a0,  128
	daddiu   a1, a1,  128
	gssq     t1, t0,  -64(a0)
	gssq     t3, t2,  -48(a0)
	gssq     t5, t4,  -32(a0)
	bne      a2, a0, 1b
	gssq     t7, t6,  -16(a0)

	jr      ra
	nop
END(copy_page)
#endif
