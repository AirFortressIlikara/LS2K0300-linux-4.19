// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#define pr_fmt(fmt) "kasan: " fmt
#include <linux/bootmem.h>
#include <linux/kasan.h>
#include <linux/sched/task.h>
#include <linux/memblock.h>

#include <asm/pgalloc.h>

#define __pgd_none(early, pgd) (early ? (pgd_val(pgd) == 0) : \
(__pa(pgd_val(pgd)) == (unsigned long)__pa(kasan_zero_pmd)))

#define __pmd_none(early, pmd) (early ? (pmd_val(pmd) == 0) : \
(__pa(pmd_val(pmd)) == (unsigned long)__pa(kasan_zero_pte)))

#define __pte_none(early, pte) (early ? pte_none(pte) : \
((pte_val(pte) & _PFN_MASK) == (unsigned long)__pa(kasan_zero_page)))

bool kasan_early_stage = true;

/*
 * Alloc memory for shadow memory page table.
 */
static phys_addr_t __init kasan_alloc_zeroed_page(int node)
{
	void *p = memblock_virt_alloc_try_nid(PAGE_SIZE, PAGE_SIZE,
					__pa(MAX_DMA_ADDRESS),
						MEMBLOCK_ALLOC_ACCESSIBLE,
						node);
	return __pa(p);
}

static pte_t *kasan_pte_offset(pmd_t *pmdp, unsigned long addr, int node,
				      bool early)
{
	if (__pmd_none(early, READ_ONCE(*pmdp))) {
		phys_addr_t pte_phys = early ?
				__pa_symbol(kasan_zero_pte)
					: kasan_alloc_zeroed_page(node);
		if (!early)
			memcpy(__va(pte_phys), kasan_zero_pte,
				sizeof(kasan_zero_pte));

		pmd_populate_kernel(NULL, pmdp, (pte_t *)__va(pte_phys));
	}

	return pte_offset_kernel(pmdp, addr);
}

static inline void kasan_set_pgd(pgd_t *pgdp, pgd_t pgdval)
{
	WRITE_ONCE(*pgdp, pgdval);
}

static pmd_t *kasan_pmd_offset(pgd_t *pgdp, unsigned long addr, int node,
				      bool early)
{
	if (__pgd_none(early, READ_ONCE(*pgdp))) {
		phys_addr_t pmd_phys = early ?
				__pa_symbol(kasan_zero_pmd)
					: kasan_alloc_zeroed_page(node);
		if (!early)
			memcpy(__va(pmd_phys), kasan_zero_pmd,
				sizeof(kasan_zero_pmd));
		kasan_set_pgd(pgdp, __pgd((unsigned long)__va(pmd_phys)));
	}

	return (pmd_t *)((pmd_t *)pgd_val(*pgdp) + pmd_index(addr));
}

static void  kasan_pte_populate(pmd_t *pmdp, unsigned long addr,
				      unsigned long end, int node, bool early)
{
	unsigned long next;
	pte_t *ptep = kasan_pte_offset(pmdp, addr, node, early);

	do {
		phys_addr_t page_phys = early ?
					__pa_symbol(kasan_zero_page)
					      : kasan_alloc_zeroed_page(node);
		next = addr + PAGE_SIZE;
		set_pte(ptep, pfn_pte(__phys_to_pfn(page_phys), PAGE_KERNEL));
	} while (ptep++, addr = next, addr != end && __pte_none(early, READ_ONCE(*ptep)));
}

static void kasan_pmd_populate(pgd_t *pgdp, unsigned long addr,
				      unsigned long end, int node, bool early)
{
	unsigned long next;
	pmd_t *pmdp = kasan_pmd_offset(pgdp, addr, node, early);

	do {
		next = pmd_addr_end(addr, end);
		kasan_pte_populate(pmdp, addr, next, node, early);
	} while (pmdp++, addr = next, addr != end && __pmd_none(early, READ_ONCE(*pmdp)));
}

static void __init kasan_pgd_populate(unsigned long addr, unsigned long end,
				      int node, bool early)
{
	unsigned long next;
	pgd_t *pgdp;

	pgdp = pgd_offset_k(addr);

	do {
		next = pgd_addr_end(addr, end);
		kasan_pmd_populate(pgdp, addr, next, node, early);
	} while (pgdp++, addr = next, addr != end);

}

asmlinkage void __init kasan_early_init(void)
{
	BUILD_BUG_ON(!IS_ALIGNED(KASAN_SHADOW_START, PGDIR_SIZE));
	BUILD_BUG_ON(!IS_ALIGNED(KASAN_SHADOW_END, PGDIR_SIZE));
}

/* Set up full kasan mappings, ensuring that the mapped pages are zeroed */
static void __init kasan_map_populate(unsigned long start, unsigned long end,
				      int node)
{
	kasan_pgd_populate(start & PAGE_MASK, PAGE_ALIGN(end), node, false);
}

static void __init clear_pgds(unsigned long start,
			unsigned long end)
{
	for (; start < end; start += PGDIR_SIZE)
		kasan_set_pgd((pgd_t *)pgd_offset_k(start), __pgd(0));
}

void __init kasan_init(void)
{
	struct memblock_region *reg;
	int i;

	/*
	 * Pgd was populated as invalid_pmd_table or invalid_pud_table
	 * in pagetable_init() which depends on how many levels of page
	 * table you are using, but we had to clean the gpd of kasan
	 * shadow memory, as the pgd value is none-zero.
	 * The assertion pgd_none is gong to be false and the formal populate
	 * afterwards is not going to create any new pgd at all.
	 */
	clear_pgds(KASAN_SHADOW_START, KASAN_SHADOW_END);

	/* Maps everything to a single page of zeroes */
	kasan_pgd_populate(KASAN_SHADOW_START, KASAN_SHADOW_END, NUMA_NO_NODE,
			true);

	kasan_populate_zero_shadow(kasan_mem_to_shadow((void *)MODULES_END),
		kasan_mem_to_shadow((void *)VMEMMAP_END));

	kasan_early_stage = false;

	/* Populate the linear mapping */
	for_each_memblock(memory, reg) {
		void *start = (void *)phys_to_virt(reg->base);
		void *end = (void *)phys_to_virt(reg->base + reg->size);

		if (start >= end)
			break;

		kasan_map_populate((unsigned long)kasan_mem_to_shadow(start),
				   (unsigned long)kasan_mem_to_shadow(end),
				   NUMA_NO_NODE);
	}

	/*
	 * KAsan may reuse the contents of kasan_zero_pte directly, so we
	 * should make sure that it maps the zero page read-only.
	 */
	for (i = 0; i < PTRS_PER_PTE; i++)
		set_pte(&kasan_zero_pte[i],
			pfn_pte(__phys_to_pfn(__pa_symbol(kasan_zero_page)),
				PAGE_KERNEL_RO));

	memset(kasan_zero_page, 0, PAGE_SIZE);

	/* At this point kasan is fully initialized. Enable error messages */
	init_task.kasan_depth = 0;
	pr_info("KernelAddressSanitizer initialized.\n");
}
