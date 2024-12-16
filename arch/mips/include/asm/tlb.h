/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_TLB_H
#define __ASM_TLB_H

#include <asm/cpu-features.h>
#include <asm/mipsregs.h>
#include <linux/mm_types.h>

/*
 * MIPS doesn't need any special per-pte or per-vma handling, except
 * we need to flush cache for area to be unmapped.
 */
#define tlb_start_vma(tlb, vma)					\
	do {							\
		if (!tlb->fullmm)				\
			flush_cache_range(vma, vma->vm_start, vma->vm_end); \
	}  while (0)
#define tlb_end_vma(tlb, vma) do { } while (0)
#define __tlb_remove_tlb_entry(tlb, ptep, address) do { } while (0)

/*
 * .. because we flush the whole mm when it fills up.
 */
#define _UNIQUE_ENTRYHI(base, idx, asid)				\
		(((base) + ((idx) << (PAGE_SHIFT + 1))) |		\
		 (cpu_has_tlbinv ? MIPS_ENTRYHI_EHINV : 0) | asid)
#define UNIQUE_ENTRYHI(idx)		_UNIQUE_ENTRYHI(CKSEG0, idx, 0)
#define UNIQUE_GUEST_ENTRYHI(idx)	_UNIQUE_ENTRYHI(CKSEG1, idx, 0)
#define UNIQUE_ENTRYHI_ASID(idx, asid)	_UNIQUE_ENTRYHI(CKSEG0, idx, asid)

static inline unsigned int num_wired_entries(void)
{
	unsigned int wired = read_c0_wired();

	if (cpu_has_mips_r6)
		wired &= MIPSR6_WIRED_WIRED;

	return wired;
}

static void tlb_flush(struct mmu_gather *tlb);

#include <asm-generic/tlb.h>

static inline void tlb_flush(struct mmu_gather *tlb)
{
	struct vm_area_struct vma;

	vma.vm_mm = tlb->mm;
	vma.vm_flags = 0;
	if (tlb->fullmm) {
		flush_tlb_mm(tlb->mm);
		return;
	}

	flush_tlb_range(&vma, tlb->start, tlb->end);
}

#endif /* __ASM_TLB_H */
