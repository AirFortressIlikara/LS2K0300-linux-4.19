// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 * Author: Huacai Chen <chenhuacai@loongson.cn>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/export.h>
#include <linux/acpi.h>
#include <linux/efi.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>
#include <asm/fw.h>
#include <asm/time.h>
#include <asm/bootinfo.h>
#include <asm/dma.h>
#include <loongson.h>
#include <loongson-pch.h>

struct loongsonlist_mem_map *g_mmap;
u64 efi_system_table;
struct boot_params *efi_bp;
struct loongsonlist_vbios *pvbios;
struct loongson_system_configuration loongson_sysconf;

void *loongson_fdt_blob;
EXPORT_SYMBOL(loongson_sysconf);

static u8 ext_listhdr_checksum(u8 *buffer, u32 length)
{
	u8 sum = 0;
	u8 *end = buffer + length;

	while (buffer < end) {
		sum = (u8)(sum + *(buffer++));
	}

	return (sum);
}

static int parse_mem(struct _extention_list_hdr *head)
{
	g_mmap = (struct loongsonlist_mem_map *)head;
	if (ext_listhdr_checksum((u8 *)g_mmap, head->length)) {
		printk("mem checksum error\n");
		return -EPERM;
	}
	return 0;
}

static int parse_vbios(struct _extention_list_hdr *head)
{
	pvbios = (struct loongsonlist_vbios *)head;

	if (ext_listhdr_checksum((u8 *)pvbios, head->length)) {
		printk("vbios_addr checksum error\n");
		return -EPERM;
	}
	loongson_sysconf.vgabios_addr =
		(unsigned long)early_memremap_ro(pvbios->vbios_addr,
				sizeof(unsigned long));

	return 0;
}

static int parse_screeninfo(struct _extention_list_hdr *head)
{
	struct loongsonlist_screeninfo *pscreeninfo;

	pscreeninfo = (struct loongsonlist_screeninfo *)head;
	if (ext_listhdr_checksum((u8 *)pscreeninfo, head->length)) {
		printk("screeninfo_addr checksum error\n");
		return -EPERM;
	}

	memcpy(&screen_info, &pscreeninfo->si, sizeof(screen_info));
	return 0;
}

static int list_find(struct boot_params *bp)
{
	struct _extention_list_hdr *fhead = NULL;
	unsigned long index;

	fhead = bp->extlist;
	if (!fhead) {
		printk("the bp ext struct empty!\n");
		return -1;
	}
	do {
		if (memcmp(&(fhead->signature), LOONGSON_MEM_SIGNATURE, 3) == 0) {
			if (parse_mem(fhead) !=0) {
				printk("parse mem failed\n");
				return -EPERM;
			}
		} else if (memcmp(&(fhead->signature), LOONGSON_VBIOS_SIGNATURE, 5) == 0) {
			if (parse_vbios(fhead) != 0) {
				printk("parse vbios failed\n");
				return -EPERM;
			}
		} else if (memcmp(&(fhead->signature), LOONGSON_SCREENINFO_SIGNATURE, 5) == 0) {
			if (parse_screeninfo(fhead) != 0) {
				printk("parse screeninfo failed\n");
				return -EPERM;
			}
		}
		fhead = (struct _extention_list_hdr *)fhead->next;
		index = (unsigned long)fhead;
	} while (index);
	return 0;
}

static int get_bpi_version(u64 *signature)
{
	u8 data[9];
	int version = BPI_VERSION_NONE;

	data[8] = 0;
	memcpy(data, signature, sizeof(*signature));
	if (kstrtoint(&data[3], 10, &version))
		return BPI_VERSION_NONE;
	return version;
}

static void __init parse_bpi_flags(void)
{
	if (efi_bp->flags & BPI_FLAGS_UEFI_SUPPORTED) {
		set_bit(EFI_BOOT, &efi.flags);
	} else {
		clear_bit(EFI_BOOT, &efi.flags);
	}
}

static void *remap_fdt(phys_addr_t dt_phys)
{
	int fdt_size;
	void *fdt_ptr;

	fdt_ptr = early_memremap_ro(dt_phys, SZ_16K);
	fdt_size = fdt_totalsize(fdt_ptr);

	if (fdt_size > SZ_16K)
		return early_memremap_ro((unsigned long)fdt_ptr, fdt_size);

	return fdt_ptr;
}

void __init fw_init_env(void)
{
	int efi_boot;
	char *cmdline;
	void *fdt_ptr;
	struct efi_memory_map_data data;
	struct efi_fdt_params params;

	if (fw_arg0 > 1)
		goto parse_bpi;

	efi_boot = fw_arg0;
	if (efi_boot)
		set_bit(EFI_BOOT, &efi.flags);
	else
		clear_bit(EFI_BOOT, &efi.flags);

	if (fw_arg2 == 0)
		goto parse_fdt;

	cmdline = early_memremap_ro(fw_arg1, COMMAND_LINE_SIZE);
	strscpy(boot_command_line, cmdline, COMMAND_LINE_SIZE);
	early_memunmap(cmdline, COMMAND_LINE_SIZE);

	if (IS_ENABLED(CONFIG_BUILTIN_DTB))
		memcpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);

	efi_system_table = fw_arg2;

	return;

parse_bpi:
	efi_bp = (struct boot_params *)fw_arg2;
	loongson_sysconf.bpi_version = get_bpi_version(&efi_bp->signature);
	pr_info("BPI%d with boot flags %llx.\n", loongson_sysconf.bpi_version, efi_bp->flags);
	if (loongson_sysconf.bpi_version == BPI_VERSION_NONE)
		pr_info("Fatal error, incorrect BPI version: %d\n",
				loongson_sysconf.bpi_version);
	else if (loongson_sysconf.bpi_version == BPI_VERSION_V2)
		parse_bpi_flags();

	if (list_find(efi_bp))
		printk("Scan bootparm failed\n");

	return;

parse_fdt:
	fdt_ptr = remap_fdt(fw_arg1);

	early_init_dt_scan(fdt_ptr);
	early_init_fdt_reserve_self();
	if (IS_ENABLED(CONFIG_BUILTIN_DTB))
		memcpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);

	/* Grab UEFI information placed in FDT by stub */
	if (!efi_get_fdt_params(&params))
		return;

	efi_system_table = params.system_table;

	data.desc_version = params.desc_ver;
	data.desc_size = params.desc_size;
	data.size = params.mmap_size;
	data.phys_map = params.mmap;

	efi_memmap_init_early(&data);
	memblock_reserve(data.phys_map & PAGE_MASK,
			 PAGE_ALIGN(data.size + (data.phys_map & ~PAGE_MASK)));
}

static int __init init_cpu_fullname(void)
{
 	int cpu;

	if (loongson_sysconf.cpuname && !strncmp(loongson_sysconf.cpuname, "Loongson", 8)) {
		for (cpu = 0; cpu < NR_CPUS; cpu++) {
			__cpu_full_name[cpu] = loongson_sysconf.cpuname;
		}
	}
 	return 0;
}
arch_initcall(init_cpu_fullname);
