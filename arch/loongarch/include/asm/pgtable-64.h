/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_PGTABLE_64_H
#define _ASM_PGTABLE_64_H

#include <linux/kconfig.h>
#include <linux/compiler.h>
#include <linux/linkage.h>
#include <asm/addrspace.h>
#include <asm/page.h>

#define __ARCH_USE_5LEVEL_HACK
#if defined(CONFIG_PAGE_SIZE_64KB) && !defined(CONFIG_VA_BITS_48)
#include <asm-generic/pgtable-nopmd.h>
#elif !(defined(CONFIG_PAGE_SIZE_4KB) && defined(CONFIG_VA_BITS_48))
#include <asm-generic/pgtable-nopud.h>
#else
#include <asm-generic/5level-fixup.h>
#endif

#if CONFIG_PGTABLE_LEVELS == 2
#define PGDIR_SHIFT	(PAGE_SHIFT + PAGE_SHIFT + PTE_ORDER - 3)
#elif CONFIG_PGTABLE_LEVELS == 3
#define PMD_SHIFT	(PAGE_SHIFT + (PAGE_SHIFT + PTE_ORDER - 3))
#define PMD_SIZE	(1UL << PMD_SHIFT)
#define PMD_MASK	(~(PMD_SIZE-1))
#define PGDIR_SHIFT	(PMD_SHIFT + (PAGE_SHIFT + PMD_ORDER - 3))
#elif CONFIG_PGTABLE_LEVELS == 4
#define PMD_SHIFT	(PAGE_SHIFT + (PAGE_SHIFT + PTE_ORDER - 3))
#define PMD_SIZE	(1UL << PMD_SHIFT)
#define PMD_MASK	(~(PMD_SIZE-1))
#define PUD_SHIFT	(PMD_SHIFT + (PAGE_SHIFT + PMD_ORDER - 3))
#define PUD_SIZE	(1UL << PUD_SHIFT)
#define PUD_MASK	(~(PUD_SIZE-1))
#define PGDIR_SHIFT	(PUD_SHIFT + (PAGE_SHIFT + PUD_ORDER - 3))
#endif

#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

#ifdef CONFIG_VA_BITS_40
#ifdef CONFIG_PAGE_SIZE_4KB
#define PGD_ORDER		0
#define PUD_ORDER		aieeee_attempt_to_allocate_pud
#define PMD_ORDER		0
#define PTE_ORDER		0
#endif
#ifdef CONFIG_PAGE_SIZE_16KB
#define PGD_ORDER               0
#define PUD_ORDER		aieeee_attempt_to_allocate_pud
#define PMD_ORDER		0
#define PTE_ORDER		0
#endif
#ifdef CONFIG_PAGE_SIZE_64KB
#define PGD_ORDER		0
#define PUD_ORDER		aieeee_attempt_to_allocate_pud
#define PMD_ORDER		aieeee_attempt_to_allocate_pmd
#define PTE_ORDER		0
#endif
#endif

#ifdef CONFIG_VA_BITS_48
#ifdef CONFIG_PAGE_SIZE_4KB
#define PGD_ORDER		0
#define PUD_ORDER		0
#define PMD_ORDER		0
#define PTE_ORDER		0
#endif
#ifdef CONFIG_PAGE_SIZE_16KB
#define PGD_ORDER               0
#define PUD_ORDER		aieeee_attempt_to_allocate_pud
#define PMD_ORDER		0
#define PTE_ORDER		0
#endif
#ifdef CONFIG_PAGE_SIZE_64KB
#define PGD_ORDER		0
#define PUD_ORDER		aieeee_attempt_to_allocate_pud
#define PMD_ORDER		0
#define PTE_ORDER		0
#endif
#endif

#define PTRS_PER_PGD	((PAGE_SIZE << PGD_ORDER) >> 3)
#if CONFIG_PGTABLE_LEVELS > 3
#define PTRS_PER_PUD	((PAGE_SIZE << PUD_ORDER) >> 3)
#endif
#if CONFIG_PGTABLE_LEVELS > 2
#define PTRS_PER_PMD	((PAGE_SIZE << PMD_ORDER) >> 3)
#endif
#define PTRS_PER_PTE	((PAGE_SIZE << PTE_ORDER) >> 3)
#define USER_PTRS_PER_PGD       ((TASK_SIZE64 / PGDIR_SIZE)?(TASK_SIZE64 / PGDIR_SIZE):1)
#define FIRST_USER_ADDRESS	0UL

#ifndef __ASSEMBLY__

/*
 * TLB refill handlers also map the vmalloc area into xuseg.  Avoid
 * the first couple of pages so NULL pointer dereferences will still
 * reliably trap.
 */
#define MODULES_VADDR		(loongson_map_base + PCI_IOSIZE + (2 * PAGE_SIZE))
#define MODULES_END		(MODULES_VADDR + SZ_4G)

#define VMALLOC_START		MODULES_END
#ifdef CONFIG_KASAN
#ifndef CONFIG_SPARSEMEM_VMEMMAP
#define VMALLOC_END     \
	((loongson_map_base + \
	min(PTRS_PER_PGD * PTRS_PER_PUD * PTRS_PER_PMD * PTRS_PER_PTE * PAGE_SIZE, \
	    (1UL << cpu_vabits)) / 2) - 1)
#define VMEMMAP_END	VMALLOC_END
#else
#define VMALLOC_END     \
	((loongson_map_base + \
	min(PTRS_PER_PGD * PTRS_PER_PUD * PTRS_PER_PMD * PTRS_PER_PTE * PAGE_SIZE, \
	    (1UL << cpu_vabits)) / 2) - VMEMMAP_SIZE - PMD_SIZE - 1)
#define vmemmap ((struct page *)((VMALLOC_END + PMD_SIZE - 1) & PMD_MASK))
#define VMEMMAP_END	((unsigned long)vmemmap + VMEMMAP_SIZE)
#endif
#else
#ifndef CONFIG_SPARSEMEM_VMEMMAP
#define VMALLOC_END	\
	(loongson_map_base + \
	 min(PTRS_PER_PGD * PTRS_PER_PUD * PTRS_PER_PMD * PTRS_PER_PTE * PAGE_SIZE, \
	     (1UL << cpu_vabits)) - 1)
#define VMEMMAP_END	VMALLOC_END
#else
#define VMALLOC_END	\
	(loongson_map_base + \
	 min(PTRS_PER_PGD * PTRS_PER_PUD * PTRS_PER_PMD * PTRS_PER_PTE * PAGE_SIZE, \
	     (1UL << cpu_vabits)) - VMEMMAP_SIZE - PMD_SIZE - 1)
#define vmemmap ((struct page *)((VMALLOC_END + PMD_SIZE - 1) & PMD_MASK))
#define VMEMMAP_END	((unsigned long)vmemmap + VMEMMAP_SIZE)
#endif
#endif

#define pte_ERROR(e) \
	printk("%s:%d: bad pte %016lx.\n", __FILE__, __LINE__, pte_val(e))
#ifndef __PAGETABLE_PMD_FOLDED
#define pmd_ERROR(e) \
	printk("%s:%d: bad pmd %016lx.\n", __FILE__, __LINE__, pmd_val(e))
#endif
#ifndef __PAGETABLE_PUD_FOLDED
#define pud_ERROR(e) \
	printk("%s:%d: bad pud %016lx.\n", __FILE__, __LINE__, pud_val(e))
#endif
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd %016lx.\n", __FILE__, __LINE__, pgd_val(e))

extern pte_t invalid_pte_table[PTRS_PER_PTE];

#ifndef __PAGETABLE_PUD_FOLDED
/*
 * For 4-level pagetables we defines these ourselves, for 3-level the
 * definitions are below, for 2-level the
 * definitions are supplied by <asm-generic/pgtable-nopmd.h>.
 */
typedef struct { unsigned long pud; } pud_t;
#define pud_val(x)	((x).pud)
#define __pud(x)	((pud_t) { (x) })

extern pud_t invalid_pud_table[PTRS_PER_PUD];

/*
 * Empty pgd entries point to the invalid_pud_table.
 */
static inline int pgd_none(pgd_t pgd)
{
	return pgd_val(pgd) == (unsigned long)invalid_pud_table;
}

static inline int pgd_bad(pgd_t pgd)
{
	if (unlikely(pgd_val(pgd) & ~PAGE_MASK))
		return 1;

	return 0;
}

static inline int pgd_present(pgd_t pgd)
{
	return pgd_val(pgd) != (unsigned long)invalid_pud_table;
}

static inline void pgd_clear(pgd_t *pgdp)
{
	pgd_val(*pgdp) = (unsigned long)invalid_pud_table;
}

#define pud_index(address)	(((address) >> PUD_SHIFT) & (PTRS_PER_PUD - 1))

static inline unsigned long pgd_page_vaddr(pgd_t pgd)
{
	return pgd_val(pgd);
}

static inline pud_t *pud_offset(pgd_t *pgd, unsigned long address)
{
	return (pud_t *)pgd_page_vaddr(*pgd) + pud_index(address);
}

static inline void set_pgd(pgd_t *pgd, pgd_t pgdval)
{
	*pgd = pgdval;
}

#endif

#ifndef __PAGETABLE_PMD_FOLDED
/*
 * For 3-level pagetables we defines these ourselves, for 2-level the
 * definitions are supplied by <asm-generic/pgtable-nopmd.h>.
 */
typedef struct { unsigned long pmd; } pmd_t;
#define pmd_val(x)	((x).pmd)
#define __pmd(x)	((pmd_t) { (x) } )


extern pmd_t invalid_pmd_table[PTRS_PER_PMD];
#endif

/*
 * Empty pgd/pmd entries point to the invalid_pte_table.
 */
static inline int pmd_none(pmd_t pmd)
{
	return pmd_val(pmd) == (unsigned long) invalid_pte_table;
}

static inline int pmd_bad(pmd_t pmd)
{
	/* pmd_huge(pmd) but inline */
	if (unlikely(pmd_val(pmd) & _PAGE_HUGE))
		return 0;

	if (unlikely(pmd_val(pmd) & ~PAGE_MASK))
		return 1;

	return 0;
}

static inline int pmd_present(pmd_t pmd)
{
	if (unlikely(pmd_val(pmd) & _PAGE_HUGE))
		return !!(pmd_val(pmd) & (_PAGE_PRESENT | _PAGE_PROTNONE));

	return pmd_val(pmd) != (unsigned long) invalid_pte_table;
}

static inline void pmd_clear(pmd_t *pmdp)
{
	pmd_val(*pmdp) = ((unsigned long) invalid_pte_table);
}
#ifndef __PAGETABLE_PMD_FOLDED

/*
 * Empty pud entries point to the invalid_pmd_table.
 */
static inline int pud_none(pud_t pud)
{
	return pud_val(pud) == (unsigned long) invalid_pmd_table;
}

static inline int pud_bad(pud_t pud)
{
	return pud_val(pud) & ~PAGE_MASK;
}

static inline int pud_present(pud_t pud)
{
	return pud_val(pud) != (unsigned long) invalid_pmd_table;
}

static inline void pud_clear(pud_t *pudp)
{
	pud_val(*pudp) = ((unsigned long) invalid_pmd_table);
}
#endif

#define pte_page(x)		pfn_to_page(pte_pfn(x))

#define pte_pfn(x)		((unsigned long)(((x).pte & _PFN_MASK) >> _PFN_SHIFT))
#define pfn_pte(pfn, prot)	__pte(((pfn) << _PFN_SHIFT) | pgprot_val(prot))
#define pfn_pmd(pfn, prot)	__pmd(((pfn) << _PFN_SHIFT) | pgprot_val(prot))

#define __pgd_offset(address)	pgd_index(address)
#define __pud_offset(address)	(((address) >> PUD_SHIFT) & (PTRS_PER_PUD-1))
#define __pmd_offset(address)	pmd_index(address)

/* to find an entry in a kernel page-table-directory */
#define pgd_offset_k(address) pgd_offset(&init_mm, address)

#define pgd_index(address)	(((address) >> PGDIR_SHIFT) & (PTRS_PER_PGD-1))
#define pmd_index(address)	(((address) >> PMD_SHIFT) & (PTRS_PER_PMD-1))

/* to find an entry in a page-table-directory */
#define pgd_offset(mm, addr)	((mm)->pgd + pgd_index(addr))

#ifndef __PAGETABLE_PMD_FOLDED
static inline unsigned long pud_page_vaddr(pud_t pud)
{
	return pud_val(pud);
}
#define pud_phys(pud)		virt_to_phys((void *)pud_val(pud))
#define pud_page(pud)		(pfn_to_page(pud_phys(pud) >> PAGE_SHIFT))

/* Find an entry in the second-level page table.. */
static inline pmd_t *pmd_offset(pud_t * pud, unsigned long address)
{
	return (pmd_t *) pud_page_vaddr(*pud) + pmd_index(address);
}
#endif

/* Find an entry in the third-level page table.. */
#define __pte_offset(address)						\
	(((address) >> PAGE_SHIFT) & (PTRS_PER_PTE - 1))
#define pte_offset(dir, address)					\
	((pte_t *) pmd_page_vaddr(*(dir)) + __pte_offset(address))
#define pte_offset_kernel(dir, address)					\
	((pte_t *) pmd_page_vaddr(*(dir)) + __pte_offset(address))
#define pte_offset_map(dir, address)					\
	((pte_t *)page_address(pmd_page(*(dir))) + __pte_offset(address))
#define pte_unmap(pte) ((void)(pte))

/*
 * Initialize a new pgd / pmd table with invalid pointers.
 */
extern void pgd_init(void *page);
extern void pud_init(unsigned long page, unsigned long pagetable);
extern void pmd_init(void *page);

/*
 * Non-present pages:  high 40 bits are offset, next 8 bits type,
 * low 16 bits zero.
 */
static inline pte_t mk_swap_pte(unsigned long type, unsigned long offset)
{ pte_t pte; pte_val(pte) = (type << 16) | (offset << 24); return pte; }

#define __swp_type(x)		(((x).val >> 16) & 0xff)
#define __swp_offset(x)		((x).val >> 24)
#define __swp_entry(type, offset) ((swp_entry_t) { pte_val(mk_swap_pte((type), (offset))) })
#define __pte_to_swp_entry(pte) ((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(x)	((pte_t) { (x).val })
#ifdef CONFIG_ARCH_ENABLE_THP_MIGRATION
#define __pmd_to_swp_entry(pmd) ((swp_entry_t) { pmd_val(pmd) })
#define __swp_entry_to_pmd(x)	((pmd_t) { (x).val | _PAGE_HUGE })
#endif

#include <asm/fixmap.h>
#endif /* !__ASSEMBLY__ */
#endif /* _ASM_PGTABLE_64_H */
