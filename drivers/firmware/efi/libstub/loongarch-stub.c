// SPDX-License-Identifier: GPL-2.0
/*
 * Author: Yun Liu <liuyun@loongson.cn>
 *         Huacai Chen <chenhuacai@loongson.cn>
 * Copyright (C) 2020-2022 Loongson Technology Corporation Limited
 */

#include <linux/efi.h>
#include <asm/efi.h>
#include <asm/addrspace.h>
#include "efistub.h"

#define BOOT_HEAP_SIZE 0x400000
#define EFI_MMAP_NR_SLACK_SLOTS	8

typedef void __noreturn (*kernel_entry_t)(bool efi, unsigned long cmdline,
					unsigned long systab);

extern long kernel_entaddr;
extern void decompress_kernel(unsigned long boot_heap_start, long kdump_reloc_offset);

static unsigned char efi_heap[BOOT_HEAP_SIZE];
static efi_guid_t screen_info_guid = LINUX_EFI_LARCH_SCREEN_INFO_TABLE_GUID;
static kernel_entry_t kernel_entry;
static unsigned long efi_sys_table;
static unsigned long efi_cmdline_ptr;

/**
 * efi_get_new_memory_map() - get new memory map
 * @map:		pointer to memory map pointer to which to assign the
 *			newly allocated memory map
 * @install_cfg_tbl:	whether or not to install the boot memory map as a
 *			configuration table
 *
 * Retrieve the UEFI memory map. The allocated memory leaves room for
 * up to EFI_MMAP_NR_SLACK_SLOTS additional memory map entries.
 *
 * Return:	status code
 */
static efi_status_t efi_get_new_memory_map(efi_system_table_t *sys_table_arg,
				struct efi_new_memmap **map,
				bool install_cfg_tbl)
{
	int memtype = install_cfg_tbl ? EFI_ACPI_RECLAIM_MEMORY
				      : EFI_LOADER_DATA;
	efi_guid_t tbl_guid = LINUX_EFI_NEW_MEMMAP_GUID;
	struct efi_new_memmap *m, tmp;
	efi_status_t status;
	unsigned long size;

	tmp.map_size = 0;
	status = efi_call_early(get_memory_map, &tmp.map_size, NULL,
			&tmp.map_key, &tmp.desc_size, &tmp.desc_ver);
	if (status != EFI_BUFFER_TOO_SMALL)
		return EFI_LOAD_ERROR;

	size = tmp.map_size + tmp.desc_size * EFI_MMAP_NR_SLACK_SLOTS;
	status = efi_call_early(allocate_pool, memtype, sizeof(*m) + size,
			     (void **)&m);
	if (status != EFI_SUCCESS)
		return status;

	if (install_cfg_tbl) {
		/*
		 * Installing a configuration table might allocate memory, and
		 * this may modify the memory map. This means we should install
		 * the configuration table first, and re-install or delete it
		 * as needed.
		 */
		status = efi_call_early(install_configuration_table,
					&tbl_guid,
					m);
		if (status != EFI_SUCCESS)
			goto free_map;
	}

	m->buff_size = m->map_size = size;
	status = efi_call_early(get_memory_map, &m->map_size, m->map,
			&m->map_key, &m->desc_size, &m->desc_ver);
	if (status != EFI_SUCCESS)
		goto uninstall_table;

	*map = m;
	return EFI_SUCCESS;

uninstall_table:
	if (install_cfg_tbl)
		efi_call_early(install_configuration_table, &tbl_guid, NULL);
free_map:
	efi_call_early(free_pool, m);
	return status;
}

static efi_status_t install_boot_memmap(efi_system_table_t *sys_table)
{
	efi_status_t status;
	struct efi_new_memmap *new_map;

	pr_efi(sys_table, "Installing boot memory map\n");
	status = efi_get_new_memory_map(sys_table, &new_map, true);

	return status;
}

struct screen_info *alloc_screen_info(efi_system_table_t *sys_table_arg)
{
	efi_status_t status;
	struct screen_info *si;

	status = efi_call_early(allocate_pool,
			EFI_RUNTIME_SERVICES_DATA, sizeof(*si), (void **)&si);
	if (status != EFI_SUCCESS)
		return NULL;

	memset(si, 0, sizeof(*si));

	status = efi_call_early(install_configuration_table, &screen_info_guid, si);
	if (status == EFI_SUCCESS)
		return si;

	efi_call_early(free_pool, si);

	return NULL;
}

void free_screen_info(efi_system_table_t *sys_table_arg, struct screen_info *si)
{
	if (!si)
		return;

	efi_call_early(install_configuration_table, &screen_info_guid, NULL);
	efi_call_early(free_pool, si);
}

efi_status_t check_platform_features(efi_system_table_t *sys_table_arg)
{
	/* Config Direct Mapping */
	csr_write64(CSR_DMW0_INIT, LOONGARCH_CSR_DMWIN0);
	csr_write64(CSR_DMW1_INIT, LOONGARCH_CSR_DMWIN1);

	return EFI_SUCCESS;
}

static efi_status_t efi_load_initrd(efi_system_table_t *sys_table_arg,
				unsigned long *initrd_addr,
				unsigned long *initrd_size,
				unsigned long *image_addr,
				efi_loaded_image_t *image,
				char *cmdline_ptr,
				unsigned long dram_base)
{
	efi_status_t status;
	efi_guid_t tbl_guid = LINUX_EFI_INITRD_MEDIA_GUID;
	struct linux_efi_initrd *tbl;

	status = handle_cmdline_files(sys_table_arg, image, cmdline_ptr,
				"initrd=",
				efi_get_max_initrd_addr(dram_base,
							*image_addr),
				initrd_addr,
				initrd_size);
	if (status != EFI_SUCCESS)
		goto failed;

	if ((*initrd_addr == 0) || (*initrd_size == 0)) {
		pr_efi(sys_table_arg, "Initrd info is invalid\n");
		goto failed;
	}

	status = efi_call_early(allocate_pool, EFI_LOADER_DATA,
				sizeof(struct linux_efi_initrd),
				(void **)&tbl);
	if (status != EFI_SUCCESS)
		goto free_initrd;

	tbl->base = *initrd_addr;
	tbl->size = *initrd_size;
	status = efi_call_early(install_configuration_table, &tbl_guid, tbl);
	if (status != EFI_SUCCESS)
		goto free_tbl;

	return EFI_SUCCESS;

free_tbl:
	efi_call_early(free_pool, tbl);
free_initrd:
	efi_free(sys_table_arg, *initrd_size, *initrd_addr);
failed:
	pr_efi(sys_table_arg, "Failed to load initrd\n");
	return status;
}

efi_status_t handle_kernel_image(efi_system_table_t *sys_table_arg,
				 unsigned long *image_addr,
				 unsigned long *image_size,
				 unsigned long *reserve_addr,
				 unsigned long *reserve_size,
				 unsigned long dram_base,
				 efi_loaded_image_t *image)
{
	efi_status_t status;
	char *cmdline_ptr = NULL;
	int cmdline_size = 0;
	unsigned long initrd_addr = 0;
	unsigned long initrd_size = 0;

	decompress_kernel((unsigned long)efi_heap, 0);
	kernel_entry = (kernel_entry_t)kernel_entaddr;

	efi_sys_table = (unsigned long)sys_table_arg;
	if (!efi_sys_table) {
		status = EFI_UNSUPPORTED;
		goto fail;
	}

	cmdline_ptr = efi_convert_cmdline(sys_table_arg, image, &cmdline_size);
	if (!cmdline_ptr) {
		pr_efi_err(sys_table_arg,
			"getting command line via LOADED_IMAGE_PROTOCOL\n");
		status = EFI_OUT_OF_RESOURCES;
		goto fail;
	}
	efi_cmdline_ptr = (unsigned long)cmdline_ptr;

	efi_load_initrd(sys_table_arg, &initrd_addr, &initrd_size,
			image_addr, image, cmdline_ptr, dram_base);

	status = install_boot_memmap(sys_table_arg);
	if (status != EFI_SUCCESS) {
		pr_efi_err(sys_table_arg, "Failed install boot memmap!\n");
		goto fail_free_mem;
	}

	return status;

fail_free_mem:
	efi_free(sys_table_arg, initrd_size, initrd_addr);
	efi_free(sys_table_arg, cmdline_size, (unsigned long)cmdline_ptr);
fail:
	pr_efi_err(sys_table_arg, "Failed to boot kernel\n");
	return status;
}

void __noreturn efi_enter_kernel(unsigned long entrypoint,
				unsigned long fdt,
				unsigned long fdt_size)
{
	kernel_entry(true, efi_cmdline_ptr, efi_sys_table);
}
