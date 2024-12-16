/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_KASAN_H
#define __ASM_KASAN_H

#ifndef __ASSEMBLY__

#include <linux/linkage.h>
#include <asm/addrspace.h>
#include <asm/pgtable-64.h>

#define KASAN_SHADOW_SCALE_SHIFT 3
#define KASAN_SHADOW_OFFSET 0

#define XRANGE_SHIFT (48)

/* Valid address length. */
#define XXXRANGE_SHADOW_SHIFT (PGDIR_SHIFT + PGD_ORDER + PAGE_SHIFT - 3)
/* Used for taking out the valid address. */
#define XXXRANGE_SHADOW_MASK GENMASK_ULL(XXXRANGE_SHADOW_SHIFT - 1, 0)
/* One segment whole address space size. */
#define	XXXRANGE_SIZE (XXXRANGE_SHADOW_MASK + 1)

/* 64-bit segment value. */
#define XKPRANGE_CC_SEG		(0x9000)
#define XKPRANGE_UC_SEG	(0x8000)
#define XKVRANGE_SEG		(0xffff)

/* COH_SHAREABLE */
#define XKPRANGE_CC_START	(0x9000000000000000)
#define XKPRANGE_CC_SIZE	XXXRANGE_SIZE
#define XKPRANGE_CC_KASAN_OFFSET (0)
#define XKPRANGE_CC_SHADOW_SIZE (XKPRANGE_CC_SIZE >> KASAN_SHADOW_SCALE_SHIFT)
#define XKPRANGE_CC_SHADOW_END	(XKPRANGE_CC_KASAN_OFFSET + XKPRANGE_CC_SHADOW_SIZE)
/* IO/UNCACHED */
#define XKPRANGE_UC_START	(0x8000000000000000)
#define XKPRANGE_UC_SIZE	XXXRANGE_SIZE
#define XKPRANGE_UC_KASAN_OFFSET XKPRANGE_CC_SHADOW_END
#define XKPRANGE_UC_SHADOW_SIZE (XKPRANGE_UC_SIZE >> KASAN_SHADOW_SCALE_SHIFT)
#define XKPRANGE_UC_SHADOW_END	(XKPRANGE_UC_KASAN_OFFSET + XKPRANGE_UC_SHADOW_SIZE)

/* VMALLOC  */
#define XKVRANGE_VMALLOC_START	VMALLOC_START
#define XKVRANGE_VMALLOC_SIZE	round_up(VMEMMAP_END - MODULES_VADDR + 1, PGDIR_SIZE)
#define XKVRANGE_VMALLOC_KASAN_OFFSET	XKPRANGE_UC_SHADOW_END
#define XKVRANGE_VMALLOC_SHADOW_SIZE	(XKVRANGE_VMALLOC_SIZE >> KASAN_SHADOW_SCALE_SHIFT)
#define XKVRANGE_VMALLOC_SHADOW_END	(XKVRANGE_VMALLOC_KASAN_OFFSET + XKVRANGE_VMALLOC_SHADOW_SIZE)

/* Kasan shadow memory start right after vmalloc. */
#define KASAN_SHADOW_START	round_up(VMEMMAP_END, PGDIR_SIZE)
#define KASAN_SHADOW_SIZE	(XKVRANGE_VMALLOC_SHADOW_END - XKPRANGE_CC_KASAN_OFFSET)
#define KASAN_SHADOW_END	round_up(KASAN_SHADOW_START + KASAN_SHADOW_SIZE, PGDIR_SIZE)

#define XKPRANGE_CC_SHADOW_OFFSET	(KASAN_SHADOW_START + XKPRANGE_CC_KASAN_OFFSET)
#define XKPRANGE_UC_SHADOW_OFFSET	(KASAN_SHADOW_START + XKPRANGE_UC_KASAN_OFFSET)
#define XKVRANGE_SHADOW_OFFSET		(KASAN_SHADOW_START + XKVRANGE_VMALLOC_KASAN_OFFSET)

extern bool kasan_early_stage;
extern unsigned char kasan_zero_page[PAGE_SIZE];
static inline void *kasan_mem_to_shadow(const void *addr)
{
	if (kasan_early_stage) {
		return (void *)(kasan_zero_page);
	} else {
		unsigned long maddr = (unsigned long)addr;
		unsigned long xrange = (maddr >> XRANGE_SHIFT) & 0xffff;
		unsigned long offset = 0;

		maddr &= XXXRANGE_SHADOW_MASK;
		switch (xrange) {
		case XKPRANGE_CC_SEG:
			offset = XKPRANGE_CC_SHADOW_OFFSET;
			break;
		case XKPRANGE_UC_SEG:
			offset = XKPRANGE_UC_SHADOW_OFFSET;
			break;
		case XKVRANGE_SEG:
			offset = XKVRANGE_SHADOW_OFFSET;
			break;
		default:
			WARN_ON(1);
			return NULL;
		}

		return (void *)((maddr >> KASAN_SHADOW_SCALE_SHIFT) + offset);
	}
}

static inline const void *kasan_shadow_to_mem(const void *shadow_addr)
{
	unsigned long addr = (unsigned long)shadow_addr;

	if (unlikely(addr > KASAN_SHADOW_END) ||
		unlikely(addr < KASAN_SHADOW_START)) {
		WARN_ON(1);
		return NULL;
	}

	if (addr >= XKVRANGE_SHADOW_OFFSET)
		return (void *)(((addr - XKVRANGE_SHADOW_OFFSET) << KASAN_SHADOW_SCALE_SHIFT) + XKVRANGE_VMALLOC_START);
	else if (addr >= XKPRANGE_UC_SHADOW_OFFSET)
		return (void *)(((addr - XKPRANGE_UC_SHADOW_OFFSET) << KASAN_SHADOW_SCALE_SHIFT) + XKPRANGE_UC_START);
	else if (addr >= XKPRANGE_CC_SHADOW_OFFSET)
		return (void *)(((addr - XKPRANGE_CC_SHADOW_OFFSET) << KASAN_SHADOW_SCALE_SHIFT) + XKPRANGE_CC_START);
	else
		WARN_ON(1);
		return NULL;
}

#define __HAVE_ARCH_SHADOW_MAP

void kasan_init(void);
asmlinkage void kasan_early_init(void);

#endif
#endif
