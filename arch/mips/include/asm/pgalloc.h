/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 2001, 2003 by Ralf Baechle
 * Copyright (C) 1999, 2000, 2001 Silicon Graphics, Inc.
 */
#ifndef _ASM_PGALLOC_H
#define _ASM_PGALLOC_H

#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/quicklist.h>

#define QUICKLIST_PGD	0
#define QUICKLIST_PMD	1
#define QUICKLIST_PTE	2

static inline void pmd_populate_kernel(struct mm_struct *mm, pmd_t *pmd,
	pte_t *pte)
{
	set_pmd(pmd, __pmd((unsigned long)pte));
}

static inline void pmd_populate(struct mm_struct *mm, pmd_t *pmd,
	pgtable_t pte)
{
	set_pmd(pmd, __pmd((unsigned long)page_address(pte)));
}
#define pmd_pgtable(pmd) pmd_page(pmd)

/*
 * Initialize a new pmd table with invalid pointers.
 */
#ifndef __PAGETABLE_PMD_FOLDED

static inline void pud_populate(struct mm_struct *mm, pud_t *pud, pmd_t *pmd)
{
	set_pud(pud, __pud((unsigned long)pmd));
}
#endif

/*
 * Initialize a new pgd / pmd table with invalid pointers.
 */
extern void pgd_init(void *page);
extern pgd_t *pgd_alloc(struct mm_struct *mm);

static inline void pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
#ifdef CONFIG_QUICKLIST
	quicklist_free(QUICKLIST_PGD, NULL, pgd);
#else
	free_pages((unsigned long)pgd, PGD_ORDER);
#endif
}

static inline pte_t *pte_alloc_one_kernel(struct mm_struct *mm,
	unsigned long address)
{
	void *pte;

#ifdef CONFIG_QUICKLIST
	pte = quicklist_alloc(QUICKLIST_PTE, GFP_KERNEL | __GFP_ZERO, NULL);
#else
	pte = (pte_t *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, PTE_ORDER);
#endif
	return (pte_t *)pte;
}

static inline struct page *pte_alloc_one(struct mm_struct *mm,
	unsigned long address)
{
	struct page *pte;
	void *addr;

#ifdef CONFIG_QUICKLIST
	addr = quicklist_alloc(QUICKLIST_PTE, GFP_KERNEL | __GFP_ZERO, NULL);
#else
	addr = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, PTE_ORDER);
#endif
	if (addr == NULL)
		return NULL;

	pte = virt_to_page((void *)addr);
#ifndef CONFIG_QUICKLIST
	clear_highpage(pte);
#endif
	if (!pgtable_page_ctor(pte)) {
#ifdef CONFIG_QUICKLIST
		quicklist_free(QUICKLIST_PTE, NULL, addr);
#else
		__free_page(pte);
#endif
		return NULL;
	}
	return pte;
}

static inline void pte_free_kernel(struct mm_struct *mm, pte_t *pte)
{
#ifdef CONFIG_QUICKLIST
	quicklist_free(QUICKLIST_PTE, NULL, pte);
#else
	free_pages((unsigned long)pte, PTE_ORDER);
#endif
}

static inline void pte_free(struct mm_struct *mm, pgtable_t pte)
{
	pgtable_page_dtor(pte);
#ifdef CONFIG_QUICKLIST
	quicklist_free_page(QUICKLIST_PTE, NULL, pte);
#else
	__free_pages(pte, PTE_ORDER);
#endif
}

#ifdef CONFIG_QUICKLIST
#define __pte_free_tlb(tlb, pte, address)		\
do {							\
	pgtable_page_dtor(pte);				\
	quicklist_free_page(QUICKLIST_PTE, NULL, pte);	\
} while (0)
#else
#define __pte_free_tlb(tlb, pte, address)		\
do {							\
	pgtable_page_dtor(pte);				\
	tlb_remove_page((tlb), pte);			\
} while (0)

#endif

#ifndef __PAGETABLE_PMD_FOLDED

static inline pmd_t *pmd_alloc_one(struct mm_struct *mm, unsigned long address)
{
	pmd_t *pmd;

#ifdef CONFIG_QUICKLIST
	pmd = (pmd_t *)quicklist_alloc(QUICKLIST_PMD, GFP_KERNEL, pmd_init);
#else
	pmd = (pmd_t *) __get_free_pages(GFP_KERNEL, PMD_ORDER);
	if (pmd)
		pmd_init((void *)pmd);
#endif

	return pmd;
}

static inline void pmd_free(struct mm_struct *mm, pmd_t *pmd)
{
#ifdef CONFIG_QUICKLIST
	quicklist_free(QUICKLIST_PMD, NULL, pmd);
#else
	free_pages((unsigned long)pmd, PMD_ORDER);
#endif
}

#define __pmd_free_tlb(tlb, x, addr)	pmd_free((tlb)->mm, x)

#endif

#ifndef __PAGETABLE_PUD_FOLDED

static inline pud_t *pud_alloc_one(struct mm_struct *mm, unsigned long address)
{
	pud_t *pud;

	pud = (pud_t *) __get_free_pages(GFP_KERNEL, PUD_ORDER);
	if (pud)
		pud_init((unsigned long)pud, (unsigned long)invalid_pmd_table);
	return pud;
}

static inline void pud_free(struct mm_struct *mm, pud_t *pud)
{
	free_pages((unsigned long)pud, PUD_ORDER);
}

static inline void pgd_populate(struct mm_struct *mm, pgd_t *pgd, pud_t *pud)
{
	set_pgd(pgd, __pgd((unsigned long)pud));
}

#define __pud_free_tlb(tlb, x, addr)	pud_free((tlb)->mm, x)

#endif /* __PAGETABLE_PUD_FOLDED */


extern void pagetable_init(void);
static inline void check_pgt_cache(void)
{
#ifdef CONFIG_QUICKLIST
	quicklist_trim(QUICKLIST_PGD, NULL, 128, 16);
	quicklist_trim(QUICKLIST_PMD, NULL, 128, 16);
	quicklist_trim(QUICKLIST_PTE, NULL, 128, 16);
#endif
}

#endif /* _ASM_PGALLOC_H */
