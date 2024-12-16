/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LOONGARCH_SPARSEMEM_H
#define _LOONGARCH_SPARSEMEM_H
#ifdef CONFIG_SPARSEMEM

/*
 * SECTION_SIZE_BITS		2^N: how big each section will be
 * MAX_PHYSMEM_BITS		2^N: how much memory we can have in that space
 */
#if defined(CONFIG_LOONGARCH_HUGE_TLB_SUPPORT) && defined(CONFIG_PAGE_SIZE_64KB)
# define SECTION_SIZE_BITS	29 /* 2^29 = Largest Huge Page Size */
#else
# define SECTION_SIZE_BITS	28
#endif
#define MAX_PHYSMEM_BITS	48

#endif /* CONFIG_SPARSEMEM */
#ifdef CONFIG_SPARSEMEM_VMEMMAP
#include <linux/mm_types.h>
#define VMEMMAP_SIZE	(sizeof(struct page) * (1UL << (cpu_pabits + 1 - PAGE_SHIFT)))
#endif
#endif /* _LOONGARCH_SPARSEMEM_H */
