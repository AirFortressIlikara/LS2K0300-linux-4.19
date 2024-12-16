// SPDX-License-Identifier: GPL-2.0
/*
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/acpi.h>

#include <asm/bootinfo.h>
#include <asm/numa.h>

#include <loongson.h>
#include <boot_param.h>
#include <mem.h>
#include <asm/acpi.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/efi.h>

void __init memblock_and_maxpfn_init(void)
{
	int i;
	u32 mem_type;
	u64 mem_start, mem_end, mem_size;
	efi_memory_desc_t *md;

	if (!g_mmap) {
		/* Parse memory information */
		for_each_efi_memory_desc(md) {
			mem_type = md->type;
			mem_start = md->phys_addr;
			mem_size = md->num_pages << EFI_PAGE_SHIFT;
			mem_end = mem_start + mem_size;

			switch (mem_type) {
			case EFI_LOADER_CODE:
			case EFI_LOADER_DATA:
			case EFI_BOOT_SERVICES_CODE:
			case EFI_BOOT_SERVICES_DATA:
			case EFI_PERSISTENT_MEMORY:
			case EFI_CONVENTIONAL_MEMORY:
				memblock_add(mem_start, mem_size);
				if (max_low_pfn < (mem_end >> PAGE_SHIFT))
					max_low_pfn = mem_end >> PAGE_SHIFT;
				break;
			case EFI_PAL_CODE:
			case EFI_UNUSABLE_MEMORY:
			case EFI_ACPI_RECLAIM_MEMORY:
			case EFI_RUNTIME_SERVICES_CODE:
			case EFI_RUNTIME_SERVICES_DATA:
			case EFI_RESERVED_TYPE:
			case EFI_MEMORY_MAPPED_IO:
			case EFI_MEMORY_MAPPED_IO_PORT_SPACE:
				memblock_reserve(mem_start, mem_size);
				break;
			}
		}
		memblock_set_current_limit(PFN_PHYS(max_low_pfn));
		memblock_set_node(0, PHYS_ADDR_MAX, &memblock.memory, 0);

		/* Reserve the first 2MB for kexec */
		memblock_reserve(PHYS_OFFSET, 0x200000);

		/* Reserve the kernel text/data/bss */
		memblock_reserve(__pa_symbol(&_text),
				__pa_symbol(&_end) - __pa_symbol(&_text));
		return;
	}

	/* parse memory information */
	for (i = 0; i < g_mmap->map_count; i++) {
		mem_type = g_mmap->map[i].mem_type;
		mem_start = g_mmap->map[i].mem_start;
		mem_size = g_mmap->map[i].mem_size;
		mem_end = mem_start + mem_size;

		switch (mem_type) {
		case ADDRESS_TYPE_SYSRAM:
			memblock_add(mem_start, mem_size);
			if (max_low_pfn < (mem_end >> PAGE_SHIFT))
				max_low_pfn = mem_end >> PAGE_SHIFT;
			break;
		}
	}
	memblock_set_current_limit(PFN_PHYS(max_low_pfn));
}

void __init memblock_remove_mem(void)
{
	int i;
	u64 mem_start, mem_size;

	/* Parse memory information and activate */
	for (i = 0; i < g_mmap->map_count; i++) {
		mem_start = g_mmap->map[i].mem_start;
		mem_size = g_mmap->map[i].mem_size;

		memblock_remove(mem_start, mem_size);
	}
}

void __init fw_init_memory(void)
{
	int i;
	u32 mem_type;
	u64 mem_start, mem_end, mem_size;
	static unsigned long num_physpages;
	unsigned long start_pfn, end_pfn;
	unsigned long kernel_end_pfn;

	if (!g_mmap)
		return;
	/* parse memory information */
	for (i = 0; i < g_mmap->map_count; i++) {
		mem_type = g_mmap->map[i].mem_type;
		mem_start = g_mmap->map[i].mem_start;
		mem_size = g_mmap->map[i].mem_size;
		mem_end = mem_start + mem_size;

		switch (mem_type) {
		case ADDRESS_TYPE_SYSRAM:
			mem_start = PFN_ALIGN(mem_start);
			mem_end = PFN_ALIGN(mem_end - PAGE_SIZE + 1);
			if (mem_start >= mem_end)
				break;
			num_physpages += (mem_size >> PAGE_SHIFT);
			pr_info("mem_start:0x%llx, mem_size:0x%llx Bytes\n",
				mem_start, mem_size);
			pr_info("start_pfn:0x%llx, end_pfn:0x%llx\n",
			mem_start >> PAGE_SHIFT, (mem_start + mem_size) >>
			PAGE_SHIFT);

			memblock_set_node(mem_start, mem_size,
					&memblock.memory, 0);
			break;
		case ADDRESS_TYPE_ACPI:
			pr_info("mem_type:%d ", mem_type);
			pr_info("mem_start:0x%llx, mem_size:0x%llx Bytes\n",
				mem_start, mem_size);
			mem_start = PFN_ALIGN(mem_start - PAGE_SIZE + 1);
			mem_end = PFN_ALIGN(mem_end);
			mem_size = mem_end - mem_start;
			memblock_add(mem_start, mem_size);
			memblock_mark_nomap(mem_start, mem_size);
			memblock_set_node(mem_start, mem_size,
					&memblock.memory, 0);
			memblock_reserve(mem_start, mem_size);
			break;
		case ADDRESS_TYPE_RESERVED:
			pr_info("mem_type:%d ", mem_type);
			pr_info("mem_start:0x%llx, mem_size:0x%llx Bytes\n",
				mem_start, mem_size);
			memblock_reserve(mem_start, mem_size);
			break;
		}
	}

	get_pfn_range_for_nid(0, &start_pfn, &end_pfn);
	pr_info("start_pfn=0x%lx, end_pfn=0x%lx, num_physpages:0x%lx\n",
				start_pfn, end_pfn, num_physpages);

	NODE_DATA(0)->node_start_pfn = start_pfn;
	NODE_DATA(0)->node_spanned_pages = end_pfn - start_pfn;

	/* used by finalize_initrd() */
	max_low_pfn = end_pfn;

	/* kernel end address */
	kernel_end_pfn = PFN_UP(__pa_symbol(&_end));

	/* Reserve the kernel text/data/bss */
	memblock_reserve(start_pfn << PAGE_SHIFT,
		((kernel_end_pfn - start_pfn) << PAGE_SHIFT));

	if (node_end_pfn(0) >= (0xffffffff >> PAGE_SHIFT))
		memblock_reserve(0xfe000000, 32 << 20);
}
