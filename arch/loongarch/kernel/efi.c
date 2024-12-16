// SPDX-License-Identifier: GPL-2.0
/*
 * EFI partition
 *
 * Just for ACPI here, complete it when implementing EFI runtime.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * lvjianmin: <lvjianmin@loongson.cn>
 * Huacai Chen: <chenhuacai@loongson.cn>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/efi.h>
#include <linux/acpi.h>
#include <linux/efi-bgrt.h>
#include <linux/export.h>
#include <linux/bootmem.h>
#include <linux/slab.h>
#include <linux/memblock.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/reboot.h>
#include <linux/bcd.h>
#include <linux/initrd.h>
#include <asm/tlb.h>
#include <asm/bootinfo.h>
#include <asm/efi.h>
#include <boot_param.h>
#include <loongson.h>

static __initdata unsigned long screen_info_table = EFI_INVALID_TABLE_ADDR;
static __initdata unsigned long new_memmap = EFI_INVALID_TABLE_ADDR;
static __initdata unsigned long initrd = EFI_INVALID_TABLE_ADDR;
static __initdata pgd_t *pgd_efi;

efi_config_table_type_t arch_tables[] __initdata = {
	{LINUX_EFI_LARCH_SCREEN_INFO_TABLE_GUID, "LA SCREENINFO", &screen_info_table},
	{LINUX_EFI_ARM_SCREEN_INFO_TABLE_GUID, "SCREENINFO", &screen_info_table},
	{LINUX_EFI_NEW_MEMMAP_GUID,   "NEWMEM",  &new_memmap},
	{LINUX_EFI_INITRD_MEDIA_GUID,	"INITRD", &initrd},
	{NULL_GUID, NULL, NULL},
};

static void __init init_screen_info(void)
{
	struct screen_info *si;

	if (screen_info_table != EFI_INVALID_TABLE_ADDR) {
		si = early_memremap_ro(screen_info_table, sizeof(*si));
		if (!si) {
			pr_err("Could not map screen_info config table\n");
			return;
		}
		screen_info = *si;
		memset(si, 0, sizeof(*si));
		early_memunmap(si, sizeof(*si));
	}

	if (screen_info.orig_video_isVGA == VIDEO_TYPE_EFI)
		memblock_reserve(screen_info.lfb_base, screen_info.lfb_size);
}

static void __init init_new_memmap(void)
{
	struct efi_new_memmap *tbl;

	if (new_memmap == EFI_INVALID_TABLE_ADDR)
		return;

	tbl = early_memremap_ro(new_memmap, sizeof(*tbl));
	if (tbl) {
		struct efi_memory_map_data data;

		data.phys_map           = new_memmap + sizeof(*tbl);
		data.size               = tbl->map_size;
		data.desc_size          = tbl->desc_size;
		data.desc_version       = tbl->desc_ver;

		if (efi_memmap_init_early(&data) < 0)
			panic("Unable to map EFI memory map.\n");

		early_memunmap(tbl, sizeof(*tbl));
	}
}

static int __init efimap_populate_hugepages(
		unsigned long start, unsigned long end,
		pgprot_t prot)
{
	unsigned long addr;
	unsigned long next;
	pmd_t entry;
	pud_t *pud;
	pmd_t *pmd;

	for (addr = start; addr < end; addr = next) {
		next = pmd_addr_end(addr, end);
		pud = pud_offset(pgd_efi + pgd_index(addr), addr);
		if (pud_none(*pud)) {
			void *p = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
			if (!p)
				return -1;
			pmd_init(p);
			pud_populate(&init_mm, pud, p);
		}
		pmd = pmd_offset(pud, addr);
		if (pmd_none(*pmd)) {
			entry = pfn_pmd((addr >> PAGE_SHIFT), prot);
			entry = pmd_mkhuge(entry);
			set_pmd_at(&init_mm, addr, pmd, entry);
		}
	}
	return 0;
}

static void __init efi_map_pgt(void)
{
	unsigned long node;
	unsigned long start, end;
	unsigned long start_pfn, end_pfn;

	pgd_efi = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
	if (!pgd_efi) {
		pr_err("alloc efi pgd failed!\n");
		return;
	}
	pgd_init(pgd_efi);
	csr_write64((long)pgd_efi, LOONGARCH_CSR_PGDL);

	/* Low Memory, Cached */
	efimap_populate_hugepages(0, SZ_256M, PAGE_KERNEL);

	for_each_node_mask(node, node_possible_map) {
		/* MMIO Registers, Uncached */
		efimap_populate_hugepages(SZ_256M | (node << 44),
				SZ_512M | (node << 44), PAGE_KERNEL_SUC);

		get_pfn_range_for_nid(node, &start_pfn, &end_pfn);
		start = ALIGN_DOWN(start_pfn << PAGE_SHIFT, PMD_SIZE);
		end = ALIGN(end_pfn << PAGE_SHIFT, PMD_SIZE);

		/* System memory, Cached */
		efimap_populate_hugepages(node ? start : SZ_512M, end, PAGE_KERNEL);
	}
}

static int __init efimap_free_pgt(unsigned long start, unsigned long end)
{
	unsigned long addr;
	unsigned long next;
	pud_t *pud;
	pmd_t *pmd;

	for (addr = start; addr < end; addr = next) {
		next = pmd_addr_end(addr, end);

		pud = pud_offset(pgd_efi + pgd_index(addr), addr);
		if (!pud_present(*pud))
			continue;
		pmd = pmd_offset(pud, addr);
		free_bootmem(virt_to_phys((void *)pmd), PAGE_SIZE);
		pud_clear(pud);
	}
	return 0;
}

static void __init efi_unmap_pgt(void)
{
	unsigned long node;
	unsigned long start, end;
	unsigned long start_pfn, end_pfn;

	for_each_node_mask(node, node_possible_map) {
		get_pfn_range_for_nid(node, &start_pfn, &end_pfn);
		start = ALIGN_DOWN(start_pfn << PAGE_SHIFT, PMD_SIZE);
		end = ALIGN(end_pfn << PAGE_SHIFT, PMD_SIZE);

		/* Free pagetable memory */
		efimap_free_pgt(start, end);
	}

	free_bootmem(virt_to_phys((void *)pgd_efi), PAGE_SIZE);
	csr_write64((long)invalid_pgd, LOONGARCH_CSR_PGDL);
	local_flush_tlb_all();

	return;
}
/*
 * enter_virt_mode() - create a virtual mapping for the EFI memory map and call
 * efi_set_virtual_address_map enter virtual for runtime service
 *
 * This function populates the virt_addr fields of all memory region descriptors
 * in @memory_map whose EFI_MEMORY_RUNTIME attribute is set. Those descriptors
 * are also copied to @runtime_map, and their total count is returned in @count.
 */
unsigned int __init enter_virt_mode(void)
{
	int count, entry;
	unsigned int size;
	unsigned long attr;
	unsigned long virt_base;
	efi_status_t status;
	efi_runtime_services_t *rt;
	efi_set_virtual_address_map_t *svam;
	efi_memory_desc_t *in, *runtime_map;
	efi_memory_desc_t *runtime_phy;

	if (efi_bp)
		return EFI_SUCCESS;

	entry = count = 0;
	size = sizeof(efi_memory_desc_t);
	for_each_efi_memory_desc(in) {
		attr = in->attribute;
		if (!(attr & EFI_MEMORY_RUNTIME))
			continue;
		entry++;
	}
	runtime_phy = (efi_memory_desc_t *)memblock_alloc(size * entry, PAGE_SIZE);
	if (!runtime_phy) {
		pr_err("Alloc runtime map failed.\n");
		return -1;
	}
	runtime_map = early_memremap_ro((unsigned long)runtime_phy, size * entry);

	for_each_efi_memory_desc(in) {
		attr = in->attribute;
		if (!(attr & EFI_MEMORY_RUNTIME))
			continue;

		if (attr & (EFI_MEMORY_WB | EFI_MEMORY_WT))
			virt_base = CAC_BASE;
		else
			virt_base = UNCAC_BASE;

		in->virt_addr = in->phys_addr + virt_base;
		memcpy(&runtime_map[count++], in, size);
	}

	rt = early_memremap_ro((unsigned long)efi.systab->runtime, sizeof(*rt));

	/* Install the new virtual address map */
	svam = rt->set_virtual_address_map;

	efi_map_pgt();
	local_flush_tlb_all();

	status = svam(size * count, size, efi.memmap.desc_version, runtime_phy);

	efi_unmap_pgt();
	local_flush_tlb_all();

	if (status != EFI_SUCCESS)
		return -1;

	return 0;
}

void __init efi_runtime_init(void)
{
	efi_status_t status;

	if (!efi_enabled(EFI_BOOT) || !efi.systab->runtime)
		goto skip;

	status = enter_virt_mode();

	if (!efi_runtime_disabled() && !status) {
		efi.runtime	= (unsigned long)efi.systab->runtime;
		efi.runtime_version = efi.systab->hdr.revision;

		efi_native_runtime_setup();
		set_bit(EFI_RUNTIME_SERVICES, &efi.flags);
		return;
	}
skip:
	pr_warning("UEFI runtime services will not be available!\n");
}

static void __init get_initrd(void)
{
	if (IS_ENABLED(CONFIG_BLK_DEV_INITRD) && initrd != EFI_INVALID_TABLE_ADDR) {
		struct linux_efi_initrd *tbl;

		tbl = early_memremap(initrd, sizeof(*tbl));
		if (tbl) {
			initrd_start = (unsigned long)__va(tbl->base);
			initrd_end = initrd_start + tbl->size;
			early_memunmap(tbl, sizeof(*tbl));
		}
	}
}

void __init efi_init(void)
{
	if (efi_bp) {
		efi.systab = (efi_system_table_t *)efi_bp->systemtable;
	} else {
		if (!efi_system_table)
			return;

		efi.systab = (efi_system_table_t *)early_memremap_ro(efi_system_table,
				sizeof(efi.systab));
	}

	if (!efi.systab) {
		pr_err("Can't find EFI system table.\n");
		return;
	}
	set_bit(EFI_64BIT, &efi.flags);
	efi.config_table = (unsigned long)efi.systab->tables;

	efi_config_init(arch_tables);

	get_initrd();

	init_screen_info();

	init_new_memmap();
}
