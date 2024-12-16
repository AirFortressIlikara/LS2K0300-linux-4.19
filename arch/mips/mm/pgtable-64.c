/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1999, 2000 by Silicon Graphics
 * Copyright (C) 2003 by Ralf Baechle
 */
#include <linux/export.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <asm/fixmap.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>

#ifdef CONFIG_QUICKLIST
DECLARE_PER_CPU(struct quicklist, quicklist)[CONFIG_NR_QUICK];
#endif

void pgd_init(void *page)
{
	unsigned long *p, *end;
	unsigned long entry;

#if !defined(__PAGETABLE_PUD_FOLDED)
	entry = (unsigned long)invalid_pud_table;
#elif !defined(__PAGETABLE_PMD_FOLDED)
	entry = (unsigned long)invalid_pmd_table;
#else
	entry = (unsigned long)invalid_pte_table;
#endif

	p = (unsigned long *) page;
	end = p + PTRS_PER_PGD;

	do {
		p[0] = entry;
		p[1] = entry;
		p[2] = entry;
		p[3] = entry;
		p[4] = entry;
		p += 8;
		p[-3] = entry;
		p[-2] = entry;
		p[-1] = entry;
	} while (p != end);
}

#ifndef __PAGETABLE_PMD_FOLDED
void pmd_init(void *addr)
{
	unsigned long *p, *end;
	unsigned long pagetable;

	pagetable = (unsigned long)invalid_pte_table;
	p = (unsigned long *) addr;
	end = p + PTRS_PER_PMD;

	do {
		p[0] = pagetable;
		p[1] = pagetable;
		p[2] = pagetable;
		p[3] = pagetable;
		p[4] = pagetable;
		p += 8;
		p[-3] = pagetable;
		p[-2] = pagetable;
		p[-1] = pagetable;
	} while (p != end);
}
EXPORT_SYMBOL_GPL(pmd_init);
#endif

#ifndef __PAGETABLE_PUD_FOLDED
void pud_init(unsigned long addr, unsigned long pagetable)
{
	unsigned long *p, *end;

	p = (unsigned long *)addr;
	end = p + PTRS_PER_PUD;

	do {
		p[0] = pagetable;
		p[1] = pagetable;
		p[2] = pagetable;
		p[3] = pagetable;
		p[4] = pagetable;
		p += 8;
		p[-3] = pagetable;
		p[-2] = pagetable;
		p[-1] = pagetable;
	} while (p != end);
}
#endif

pmd_t mk_pmd(struct page *page, pgprot_t prot)
{
	pmd_t pmd;

	pmd_val(pmd) = (page_to_pfn(page) << _PFN_SHIFT) | pgprot_val(prot);

	return pmd;
}

void set_pmd_at(struct mm_struct *mm, unsigned long addr,
		pmd_t *pmdp, pmd_t pmd)
{
	*pmdp = pmd;
	flush_tlb_all();
}

#ifdef CONFIG_QUICKLIST
static void quicklist_init(void)
{
	int cpu;
	struct quicklist *ql;
	unsigned long entry;

#if !defined(__PAGETABLE_PUD_FOLDED)
	entry = (unsigned long)invalid_pud_table;
#elif !defined(__PAGETABLE_PMD_FOLDED)
	entry = (unsigned long)invalid_pmd_table;
#else
	entry = (unsigned long)invalid_pte_table;
#endif

	for_each_online_cpu(cpu) {
		ql = per_cpu(quicklist, cpu);
		ql[QUICKLIST_PGD].order = PGD_ORDER;
		ql[QUICKLIST_PGD].val = entry;
		ql[QUICKLIST_PGD].hit = 0;
		ql[QUICKLIST_PGD].miss = 0;
		ql[QUICKLIST_PGD].threshold = 4096;

#ifndef __PAGETABLE_PMD_FOLDED
		ql[QUICKLIST_PMD].order = PMD_ORDER;
		ql[QUICKLIST_PMD].val = (unsigned long)invalid_pte_table;
		ql[QUICKLIST_PMD].hit = 0;
		ql[QUICKLIST_PMD].miss = 0;
		ql[QUICKLIST_PMD].threshold = 4096;
#endif

		ql[QUICKLIST_PTE].order = PTE_ORDER;
		ql[QUICKLIST_PTE].val = 0;
		ql[QUICKLIST_PTE].hit = 0;
		ql[QUICKLIST_PTE].miss = 0;
		ql[QUICKLIST_PTE].threshold = 4096;
	}
}
#endif

void __init pagetable_init(void)
{
	unsigned long vaddr;
	pgd_t *pgd_base;

	/* Initialize the entire pgd.  */
	pgd_init(swapper_pg_dir);
#ifndef __PAGETABLE_PUD_FOLDED
	pud_init((unsigned long)invalid_pud_table, (unsigned long)invalid_pmd_table);
#endif
#ifndef __PAGETABLE_PMD_FOLDED
	pmd_init(invalid_pmd_table);
#endif
	pgd_base = swapper_pg_dir;
	/*
	 * Fixed mappings:
	 */
	vaddr = __fix_to_virt(__end_of_fixed_addresses - 1) & PMD_MASK;
	fixrange_init(vaddr, vaddr + FIXADDR_SIZE, pgd_base);

#ifdef CONFIG_QUICKLIST
	quicklist_init();
#endif
}
