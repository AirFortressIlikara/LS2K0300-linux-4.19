// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#include <linux/bug.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/pagemap.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/highmem.h>
#include <linux/hugetlb.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/pfn.h>
#include <linux/hardirq.h>
#include <linux/gfp.h>
#include <linux/initrd.h>
#include <linux/mmzone.h>
#include <linux/memblock.h>
#include <asm/asm-offsets.h>
#include <asm/bootinfo.h>
#include <asm/cpu.h>
#include <asm/dma.h>
#include <asm/kmap_types.h>
#include <asm/mmu_context.h>
#include <asm/sections.h>
#include <asm/pgtable.h>
#include <asm/pgtable-64.h>
#include <asm/pgalloc.h>
#include <asm/tlb.h>

/*
 * We have up to 8 empty zeroed pages so we can map one of the right colour
 * when needed.	 Since page is never written to after the initialization we
 * don't have to care about aliases on other CPUs.
 */
unsigned long empty_zero_page, zero_page_mask;
EXPORT_SYMBOL(empty_zero_page);
EXPORT_SYMBOL(zero_page_mask);
extern unsigned long max_mapnr;

void setup_zero_pages(void)
{
	unsigned int order, i;
	struct page *page;

	order = 0;

	empty_zero_page = __get_free_pages(GFP_KERNEL | __GFP_ZERO, order);
	if (!empty_zero_page)
		panic("Oh boy, that early out of memory?");

	page = virt_to_page((void *)empty_zero_page);
	split_page(page, order);
	for (i = 0; i < (1 << order); i++, page++)
		mark_page_reserved(page);

	zero_page_mask = ((PAGE_SIZE << order) - 1) & PAGE_MASK;
}

void copy_user_highpage(struct page *to, struct page *from,
	unsigned long vaddr, struct vm_area_struct *vma)
{
	void *vfrom, *vto;

	vto = kmap_atomic(to);
	vfrom = kmap_atomic(from);
	copy_page(vto, vfrom);
	kunmap_atomic(vfrom);
	kunmap_atomic(vto);
	/* Make sure this page is cleared on other CPU's too before using it */
	smp_wmb();
}

void copy_to_user_page(struct vm_area_struct *vma,
	struct page *page, unsigned long vaddr, void *dst, const void *src,
	unsigned long len)
{
	memcpy(dst, src, len);
}

void copy_from_user_page(struct vm_area_struct *vma,
	struct page *page, unsigned long vaddr, void *dst, const void *src,
	unsigned long len)
{
	memcpy(dst, src, len);
}
EXPORT_SYMBOL_GPL(copy_from_user_page);

#ifndef CONFIG_NEED_MULTIPLE_NODES
int page_is_ram(unsigned long pagenr)
{
	return memblock_is_memory(PFN_PHYS(pagenr)) &&
		!memblock_is_reserved(PFN_PHYS(pagenr));
}

void __init paging_init(void)
{
	unsigned long max_zone_pfns[MAX_NR_ZONES];

#ifdef CONFIG_ZONE_DMA
	max_zone_pfns[ZONE_DMA] = MAX_DMA_PFN;
#endif
#ifdef CONFIG_ZONE_DMA32
	max_zone_pfns[ZONE_DMA32] = MAX_DMA32_PFN;
#endif
	max_zone_pfns[ZONE_NORMAL] = max_low_pfn;

	free_area_init_nodes(max_zone_pfns);
}

void __init mem_init(void)
{
	max_mapnr = max_low_pfn;
	high_memory = (void *) __va(max_low_pfn << PAGE_SHIFT);

	free_all_bootmem();
	setup_zero_pages();	/* Setup zeroed pages.  */
	mem_init_print_info(NULL);
}
#endif /* !CONFIG_NEED_MULTIPLE_NODES */

void free_init_pages(const char *what, unsigned long begin, unsigned long end)
{
	unsigned long pfn;

	for (pfn = PFN_UP(begin); pfn < PFN_DOWN(end); pfn++) {
		struct page *page = pfn_to_page(pfn);
		void *addr = phys_to_virt(PFN_PHYS(pfn));

		memset(addr, POISON_FREE_INITMEM, PAGE_SIZE);
		free_reserved_page(page);
	}
	printk(KERN_INFO "Freeing %s: %ldk freed\n", what, (end - begin) >> 10);
}

#ifdef CONFIG_BLK_DEV_INITRD
void free_initrd_mem(unsigned long start, unsigned long end)
{
	free_reserved_area((void *)start, (void *)end, POISON_FREE_INITMEM,
			   "initrd");
}
#endif

void (*free_init_pages_eva)(void *begin, void *end) = NULL;

void __ref free_initmem(void)
{
	prom_free_prom_memory();
	/*
	 * Let the platform define a specific function to free the
	 * init section since EVA may have used any possible mapping
	 * between virtual and physical addresses.
	 */
	if (free_init_pages_eva)
		free_init_pages_eva((void *)&__init_begin, (void *)&__init_end);
	else
		free_initmem_default(POISON_FREE_INITMEM);
}

#ifdef CONFIG_MEMORY_HOTPLUG
int arch_add_memory(int nid, u64 start, u64 size, struct vmem_altmap *altmap, bool want_memblock)
{
	unsigned long start_pfn = start >> PAGE_SHIFT;
	unsigned long nr_pages = size >> PAGE_SHIFT;
	int ret;

	ret = __add_pages(nid, start_pfn, nr_pages, altmap, want_memblock);

	if (ret)
		printk("%s: Problem encountered in __add_pages() as ret=%d\n",
				__func__,  ret);

	return ret;
}

#ifdef CONFIG_HAVE_ARCH_NODEDATA_EXTENSION
pg_data_t *arch_alloc_nodedata(int nid)
{
	return kzalloc(sizeof(pg_data_t), GFP_KERNEL);
}

void arch_free_nodedata(pg_data_t *pgdat)
{
	kfree(pgdat);
}

void arch_refresh_nodedata(int nid, pg_data_t *pgdat)
{
        BUG();
}
#endif

#ifdef CONFIG_NUMA
int memory_add_physaddr_to_nid(u64 start)
{
	int nid;

	nid = pa_to_nid(start);
	return nid;
}
EXPORT_SYMBOL_GPL(memory_add_physaddr_to_nid);
#endif

#ifdef CONFIG_MEMORY_HOTREMOVE
void arch_remove_memory(int nid, u64 start,
		u64 size, struct vmem_altmap *altmap)
{
	unsigned long start_pfn = start >> PAGE_SHIFT;
	unsigned long nr_pages = size >> PAGE_SHIFT;
	struct page *page = pfn_to_page(start_pfn);

	/* With altmap the first mapped page is offset from @start */
	if (altmap)
		page += vmem_altmap_offset(altmap);
	__remove_pages(start_pfn, nr_pages, altmap);
}
#endif
#endif

#ifdef CONFIG_SPARSEMEM_VMEMMAP
void __meminit arch_vmemmap_verify(pte_t *pte, int node,
				unsigned long start, unsigned long end)
{
	unsigned long pfn = pte_pfn(*pte);
	int actual_node = early_pfn_to_nid(pfn);

	if (node_distance(actual_node, node) > LOCAL_DISTANCE)
		pr_warn("[%lx-%lx] potential offnode page_structs\n",
			start, end - 1);
}

void * __meminit arch_vmemmap_alloc_block_zero(unsigned long size, int node)
{
	void *p = vmemmap_alloc_block(size, node);

	if (!p)
		return NULL;
	memset(p, 0, size);

	return p;
}

pte_t * __meminit arch_vmemmap_pte_populate(pmd_t *pmd, unsigned long addr, int node)
{
	pte_t *pte = pte_offset_kernel(pmd, addr);
	if (pte_none(*pte)) {
		pte_t entry;
		void *p = arch_vmemmap_alloc_block_zero(PAGE_SIZE, node);
		if (!p)
			return NULL;
		entry = pfn_pte(__pa(p) >> PAGE_SHIFT, PAGE_KERNEL);
		set_pte_at(&init_mm, addr, pte, entry);
	}
	return pte;
}

pmd_t * __meminit arch_vmemmap_pmd_populate(pud_t *pud, unsigned long addr, int node)
{
	pmd_t *pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd)) {
		void *p = arch_vmemmap_alloc_block_zero(PAGE_SIZE, node);
		if (!p)
			return NULL;
		pmd_populate_kernel(&init_mm, pmd, p);
	}
	return pmd;
}

pud_t * __meminit arch_vmemmap_pud_populate(p4d_t *p4d, unsigned long addr, int node)
{
	pud_t *pud = pud_offset(p4d, addr);
	if (pud_none(*pud)) {
		void *p = arch_vmemmap_alloc_block_zero(PAGE_SIZE, node);
		if (!p)
			return NULL;
#ifndef __PAGETABLE_PMD_FOLDED
		pmd_init(p);
#endif
		pud_populate(&init_mm, pud, p);
	}
	return pud;
}

p4d_t * __meminit arch_vmemmap_p4d_populate(pgd_t *pgd, unsigned long addr, int node)
{
	p4d_t *p4d = p4d_offset(pgd, addr);
	if (p4d_none(*p4d)) {
		void *p = arch_vmemmap_alloc_block_zero(PAGE_SIZE, node);
		if (!p)
			return NULL;
#ifndef __PAGETABLE_PUD_FOLDED
		pud_init((unsigned long)p, (unsigned long)invalid_pmd_table);
#endif
		p4d_populate(&init_mm, p4d, p);
	}
	return p4d;
}

pgd_t * __meminit arch_vmemmap_pgd_populate(unsigned long addr, int node)
{
	pgd_t *pgd = pgd_offset_k(addr);
	if (pgd_none(*pgd)) {
		void *p = arch_vmemmap_alloc_block_zero(PAGE_SIZE, node);
		if (!p)
			return NULL;
		pgd_populate(&init_mm, pgd, p);
	}
	return pgd;
}

int __meminit arch_vmemmap_populate_basepages(unsigned long start,
					 unsigned long end, int node)
{
	unsigned long addr = start;
	pgd_t *pgd;
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	for (; addr < end; addr += PAGE_SIZE) {
		pgd = arch_vmemmap_pgd_populate(addr, node);
		if (!pgd)
			return -ENOMEM;
		p4d = arch_vmemmap_p4d_populate(pgd, addr, node);
		if (!p4d)
			return -ENOMEM;
		pud = arch_vmemmap_pud_populate(p4d, addr, node);
		if (!pud)
			return -ENOMEM;
		pmd = arch_vmemmap_pmd_populate(pud, addr, node);
		if (!pmd)
			return -ENOMEM;
		pte = arch_vmemmap_pte_populate(pmd, addr, node);
		if (!pte)
			return -ENOMEM;
		arch_vmemmap_verify(pte, node, addr, addr + PAGE_SIZE);
	}

	return 0;
}

int __meminit arch_vmemmap_populate_hugepages(unsigned long start,
					 unsigned long end, int node)
{
	unsigned long addr = start;
	unsigned long next;
	pgd_t *pgd;
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;

	for (addr = start; addr < end; addr = next) {
		next = pmd_addr_end(addr, end);

		pgd = arch_vmemmap_pgd_populate(addr, node);
		if (!pgd)
			return -ENOMEM;
		p4d = arch_vmemmap_p4d_populate(pgd, addr, node);
		if (!p4d)
			return -ENOMEM;
		pud = arch_vmemmap_pud_populate(p4d, addr, node);
		if (!pud)
			return -ENOMEM;

		pmd = pmd_offset(pud, addr);
		if (pmd_none(*pmd)) {
			void *p = NULL;

			p = arch_vmemmap_alloc_block_zero(PMD_SIZE, node);
			if (p) {
				pmd_t entry;

				entry = pfn_pmd(virt_to_pfn(p), PAGE_KERNEL);
				entry = pmd_mkhuge(entry);
				set_pmd_at(&init_mm, addr, pmd, entry);

				continue;
			}
		} else if (pmd_huge(*pmd)) {
			arch_vmemmap_verify((pte_t *)pmd, node, addr, next);
			continue;
		}
		if (arch_vmemmap_populate_basepages(addr, next, node))
			return -ENOMEM;
	}

	return 0;
}

int __meminit vmemmap_populate(unsigned long start, unsigned long end, int node,
		struct vmem_altmap *altmap)
{
	return arch_vmemmap_populate_hugepages(start, end, node);
}
void vmemmap_free(unsigned long start, unsigned long end,
		struct vmem_altmap *altmap)
{
}
#endif

static pte_t *__get_pte_phys(unsigned long addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;

	pgd = pgd_offset_k(addr);
	if (pgd_none(*pgd)) {
		pud_t *new;

		new = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
		pgd_populate(&init_mm, pgd, new);
#ifndef __PAGETABLE_PUD_FOLDED
		pud_init((unsigned long)new, (unsigned long)invalid_pmd_table);
#endif
	}

	pud = pud_offset(pgd, addr);
	if (pud_none(*pud)) {
		pmd_t *new;

		new = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
		pud_populate(&init_mm, pud, new);
#ifndef __PAGETABLE_PMD_FOLDED
		pmd_init((void *)new);
#endif
	}

	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd)) {
		pte_t *new;

		new = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
		pmd_populate_kernel(&init_mm, pmd, new);
	}

	return pte_offset_kernel(pmd, addr);
}

void __set_fixmap(enum fixed_addresses idx,
			       phys_addr_t phys, pgprot_t flags)
{
	unsigned long addr = __fix_to_virt(idx);
	pte_t *ptep;

	BUG_ON(idx <= FIX_HOLE || idx >= __end_of_fixed_addresses);

	ptep = __get_pte_phys(addr);
	if (!pte_none(*ptep)) {
		pte_ERROR(*ptep);
		return;
	}

	if (pgprot_val(flags)) {
		set_pte(ptep, pfn_pte(phys >> PAGE_SHIFT, flags));
	} else {
		pte_clear(&init_mm, addr, ptep);
		flush_tlb_kernel_range(addr, addr+PAGE_SIZE);
	}
}


/*
 * Align swapper_pg_dir in to 64K, allows its address to be loaded
 * with a single LUI instruction in the TLB handlers.  If we used
 * __aligned(64K), its size would get rounded up to the alignment
 * size, and waste space.  So we place it in its own section and align
 * it in the linker script.
 */
pgd_t swapper_pg_dir[_PTRS_PER_PGD] __section(.bss..swapper_pg_dir);
pgd_t invalid_pgd[_PTRS_PER_PGD] __page_aligned_bss;
#ifndef __PAGETABLE_PUD_FOLDED
pud_t invalid_pud_table[PTRS_PER_PUD] __page_aligned_bss;
EXPORT_SYMBOL_GPL(invalid_pud_table);
#endif
#ifndef __PAGETABLE_PMD_FOLDED
pmd_t invalid_pmd_table[PTRS_PER_PMD] __page_aligned_bss;
EXPORT_SYMBOL(invalid_pmd_table);
#endif
pte_t invalid_pte_table[PTRS_PER_PTE] __page_aligned_bss;
EXPORT_SYMBOL(invalid_pte_table);
