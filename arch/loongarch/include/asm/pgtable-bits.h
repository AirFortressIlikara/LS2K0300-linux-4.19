/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_PGTABLE_BITS_H
#define _ASM_PGTABLE_BITS_H

#define	_PAGE_VALID_SHIFT	0
#define	_PAGE_DIRTY_SHIFT	1
#define	_PAGE_PLV_SHIFT		2  /* 2~3, two bits */
#define	_CACHE_SHIFT 		4  /* 4~5, two bits */
#define	_PAGE_GLOBAL_SHIFT 	6
#define	_PAGE_HUGE_SHIFT	6  /* HUGE is a PMD bit */
#define	_PAGE_PRESENT_SHIFT	7
#define	_PAGE_WRITE_SHIFT	8
#define	_PAGE_PROTNONE_SHIFT 	9
#define	_PAGE_SPECIAL_SHIFT 	10
#define _PAGE_MODIFIED_SHIFT  11
#define	_PAGE_HGLOBAL_SHIFT 	12 /* HGlobal is a PMD bit */
#define	_PAGE_PFN_SHIFT 	12
#define	_PAGE_PFN_END_SHIFT 	48
#define	_PAGE_NO_READ_SHIFT 	61
#define	_PAGE_NO_EXEC_SHIFT 	62
#define	_PAGE_RPLV_SHIFT 	63
/* Used only by software */
#define _PAGE_PRESENT		(_ULCAST_(1) << _PAGE_PRESENT_SHIFT)
#define _PAGE_WRITE		(_ULCAST_(1) << _PAGE_WRITE_SHIFT)
#define _PAGE_PROTNONE		(_ULCAST_(1) << _PAGE_PROTNONE_SHIFT)
#define _PAGE_SPECIAL		(_ULCAST_(1) << _PAGE_SPECIAL_SHIFT)
#define _PAGE_MODIFIED       (_ULCAST_(1) << _PAGE_MODIFIED_SHIFT)

/* Used by TLB hardware (placed in EntryLo*) */
#define _PAGE_VALID		(_ULCAST_(1) << _PAGE_VALID_SHIFT)
#define _PAGE_DIRTY		(_ULCAST_(1) << _PAGE_DIRTY_SHIFT)
#define _PAGE_PLV		(_ULCAST_(3) << _PAGE_PLV_SHIFT)
#define _PAGE_GLOBAL		(_ULCAST_(1) << _PAGE_GLOBAL_SHIFT)
#define _PAGE_HUGE		(_ULCAST_(1) << _PAGE_HUGE_SHIFT)
#define _PAGE_HGLOBAL		(_ULCAST_(1) << _PAGE_HGLOBAL_SHIFT)
#define _PAGE_NO_READ		(_ULCAST_(1) << _PAGE_NO_READ_SHIFT)
#define _PAGE_NO_EXEC		(_ULCAST_(1) << _PAGE_NO_EXEC_SHIFT)
#define _PAGE_RPLV		(_ULCAST_(1) << _PAGE_RPLV_SHIFT)
#define _CACHE_MASK		(_ULCAST_(3) << _CACHE_SHIFT)
#define _PFN_SHIFT		(PAGE_SHIFT - 12 + _PAGE_PFN_SHIFT)

#define _PAGE_USER	(PLV_USER << _PAGE_PLV_SHIFT)
#define _PAGE_KERN	(PLV_KERN << _PAGE_PLV_SHIFT)

#define _PAGE_ACCESSED _PAGE_VALID

#define _PFN_MASK (~((_ULCAST_(1) << (_PFN_SHIFT)) - 1) & \
		  ((_ULCAST_(1) << (_PAGE_PFN_END_SHIFT)) - 1))

#ifndef __ASSEMBLY__
/*
 * pte_to_entrylo converts a page table entry (PTE) into a
 * entrylo0/1 value.
 */
static inline uint64_t pte_to_entrylo(unsigned long pte_val)
{
	return pte_val;
}

/*
 * Cache attributes
 */

#ifndef _CACHE_SUC
#define _CACHE_SUC			(0<<_CACHE_SHIFT) /* Strong-ordered UnCached */
#endif
#ifndef _CACHE_CC
#define _CACHE_CC			(1<<_CACHE_SHIFT) /* Coherent Cached */
#endif
#ifndef _CACHE_WUC
#define _CACHE_WUC			(2<<_CACHE_SHIFT) /* Weak-ordered UnCached */
#endif

#define __READABLE	(_PAGE_VALID)
#define __WRITEABLE	(_PAGE_DIRTY | _PAGE_WRITE | _PAGE_MODIFIED)

#define _PAGE_CHG_MASK	(_PAGE_MODIFIED | _PAGE_SPECIAL | _PFN_MASK | _CACHE_MASK | _PAGE_PLV)
#define _HPAGE_CHG_MASK	(_PAGE_CHG_MASK | _PAGE_HUGE)
#endif
#endif /* _ASM_PGTABLE_BITS_H */
