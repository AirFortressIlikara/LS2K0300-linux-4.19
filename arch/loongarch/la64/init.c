// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/bootmem.h>
#include <linux/dmi.h>
#include <asm/bootinfo.h>
#include <asm/cacheflush.h>
#include <loongson.h>
#include <loongson-pch.h>
#include <asm/time.h>
#include <linux/memblock.h>
#include <asm/efi.h>
#include <asm/fw.h>
#include <asm/smp.h>
#include <boot_param.h>
#include <linux/efi.h>
#include <linux/acpi.h>
#include <asm/acpi.h>
#include <asm/topology.h>
#include <linux/libfdt.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/logic_pio.h>
#define SMBIOS_BIOSSIZE_OFFSET		0x9
#define SMBIOS_BIOSEXTERN_OFFSET	0x13
#define SMBIOS_FREQLOW_OFFSET		0x16
#define SMBIOS_FREQHIGH_OFFSET		0x17
#define SMBIOS_FREQLOW_MASK		0xFF
#define SMBIOS_CORE_PACKAGE_OFFSET	0x23
#define LOONGSON_EFI_ENABLE     	(1 << 3)

struct loongson_board_info b_info;
extern void __init memblock_and_maxpfn_init(void);
extern void __init memblock_remove_mem(void);
extern char cpu_full_name[64];
static bool delayed_panic;

static const char dmi_empty_string[] = "        ";

extern void *loongson_fdt_blob;

struct loongsonlist_mem_map global_mem_map;

static int __init parse_cluster(struct device_node *cluster, int depth)
{
	char name[10];
	bool leaf = true;
	bool has_cores = false;
	struct device_node *c;
	int core_id = 0;
	int i, ret;

	i = 0;
	do {
		snprintf(name, sizeof(name), "cluster%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			leaf = false;
			ret = parse_cluster(c, depth + 1);
			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	i = 0;
	do {
		snprintf(name, sizeof(name), "core%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			has_cores = true;

			if (depth == 0) {
				pr_err("%pOF: cpu-map children should be clusters\n",
				       c);
				of_node_put(c);
				return -EINVAL;
			}

			if (leaf) {
				core_id++;
			} else {
				pr_err("%pOF: Non-leaf cluster with core %s\n",
				       cluster, name);
				ret = -EINVAL;
			}
			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	if (leaf && !has_cores)
		pr_warn("%pOF: empty cluster\n", cluster);

	if (loongson_sysconf.cores_per_package == 0)
		loongson_sysconf.cores_per_package = core_id;

	return 0;
}
static int __init parse_dt_topology(void)
{
	struct device_node *cn, *map;
	int ret = 0;

	cn = of_find_node_by_path("/cpus");
	if (!cn) {
		return 0;
	}

	map = of_get_child_by_name(cn, "cpu-map");
	if (!map)
		goto out;

	ret = parse_cluster(map, 0);
	if (ret != 0)
		goto out_map;

out_map:
	of_node_put(map);
out:
	of_node_put(cn);
	return ret;
}

#define INVALID_HWID	0xFFFF
static u64 __init of_get_hwid(struct device_node *dn)
{
	const __be32 *cell = of_get_property(dn, "reg", NULL);

	if (!cell) {
		pr_err("%pOF: missing reg property\n", dn);
		return INVALID_HWID;
	}

	return of_read_number(cell, of_n_addr_cells(dn));
}

static void __init parse_dt_cpus(void)
{
	struct device_node *dn;
	int i;
	int nid = 0;
	int hwids[NR_CPUS];
	nodemask_t nodes_mask;
	int nr_nodes;

	loongson_sysconf.reserved_cpus_mask = -1;
	loongson_sysconf.boot_cpu_id = read_csr_cpuid();

	nodes_clear(nodes_mask);
	for_each_node_by_type(dn, "cpu") {
		u64 hwid = of_get_hwid(dn);

		if (hwid >= INVALID_HWID)
			continue;

		for (i = 0; i < loongson_sysconf.nr_cpus; i++) {
			if (hwids[i] == hwid) {
				pr_err("%pOF: duplicate cpu reg properties in the DT\n", dn);
				continue;
			}
		}

		nid = of_node_to_nid(dn);
		if (nid != NUMA_NO_NODE)
			node_set(nid, nodes_mask);

		if (of_node_to_nid(dn) == 0)
			loongson_sysconf.cores_per_node++;

		if (loongson_sysconf.nr_cpus >= NR_CPUS)
			break;

		hwids[loongson_sysconf.nr_cpus] = hwid;
		loongson_sysconf.reserved_cpus_mask &= (~(1 << hwid));
		loongson_sysconf.nr_cpus++;
	}
	nr_nodes = nodes_weight(nodes_mask);
	if (nr_nodes)
		loongson_sysconf.nr_nodes = nodes_weight(nodes_mask);
}

static void *get_fdt(efi_system_table_t *sys_table)
{
	int i;
	void *fdt = 0;
	efi_config_table_t *tables;

	if (fw_arg2 == 0) {
		fdt = (void *)TO_CAC((u64)fw_arg1);
	} else {
		tables = (efi_config_table_t *) sys_table->tables;

		for (i = 0; i < sys_table->nr_tables; i++)
			if (efi_guidcmp(tables[i].guid, DEVICE_TREE_GUID) == 0) {
				fdt = (void *) tables[i].table;
				fdt = (void *)TO_CAC((u64)fdt);
				break;
			}
	}

	if (IS_ENABLED(CONFIG_BUILTIN_DTB) && (&__dtb_start != &__dtb_end))
		fdt = &__dtb_start;

	if (!fdt || fdt_check_header(fdt) != 0)
		return NULL;

	return fdt;
}

static bool __init acpi_tables_present(void)
{
	return efi.acpi20 != EFI_INVALID_TABLE_ADDR ||
			efi.acpi != EFI_INVALID_TABLE_ADDR;
}

static void __init fdt_setup(void)
{
	unsigned long fdt_addr;

	if (efi.systab) {
		/* Look for a device tree configuration table entry. */
		fdt_addr = (uintptr_t)get_fdt(efi.systab);
		if (fdt_addr)
			loongson_fdt_blob = (void *)fdt_addr;
	}

	if (!loongson_fdt_blob) {
		/*
		 * Since both acpi tables and dtb are not found, we have to
		 * panic. Considering earlycon with parameters may exist in
		 * cmdline, delay the panic to let the uart show the details.
		 */
		delayed_panic = true;
		return;
	}

	if (g_mmap != NULL) {
		memblock_remove_mem();
		pr_warn("Ext list mem info is ignored!\n");
	}
	if (efi_bp || IS_ENABLED(CONFIG_BUILTIN_DTB)) {
		__dt_setup_arch(loongson_fdt_blob);
		early_init_fdt_reserve_self();
	}
	early_init_fdt_scan_reserved_mem();

	device_tree_init();
	parse_dt_cpus();
	parse_dt_topology();

	disable_acpi();
}

static int __init add_legacy_isa_io(struct fwnode_handle *fwnode, unsigned long isa_base)
{
	int ret = 0;
	unsigned long vaddr;
	struct logic_pio_hwaddr *range;

	range = kzalloc(sizeof(*range), GFP_ATOMIC);
	if (!range)
		return -ENOMEM;

	range->fwnode = fwnode;
	range->size = ISA_IOSIZE;
	range->hw_start = isa_base;
	range->flags = LOGIC_PIO_CPU_MMIO;

	ret = logic_pio_register_range(range);
	if (ret) {
		kfree(range);
		return ret;
	}

	if (range->io_start != 0) {
		logic_pio_unregister_range(range);
		kfree(range);
		return -EINVAL;
	}

	vaddr = (unsigned long)(PCI_IOBASE + range->io_start);
	ret = ioremap_page_range(vaddr, vaddr + range->size, range->hw_start, pgprot_device(PAGE_KERNEL));
	return ret;
}

static struct fwnode_handle * __init parse_isa_base(u64 *cpu_addr)
{
	struct device_node *np;
	const __be32 *ranges = NULL;
	int len;
	struct device_node *node;

	for_each_node_by_name(np, "isa") {
		node = of_node_get(np);

		if (!node)
			break;

		ranges = of_get_property(node, "ranges", &len);

		if (!ranges || (ranges && len > 0))
			break;
	}
	if (ranges) {
		ranges += 2;
		*cpu_addr = of_translate_address(np, ranges);
		return &np->fwnode;
	}

	return NULL;
}

static int __init register_legacy_isa_io(void)
{
	struct fwnode_handle *fwnode;
	u64 cpu_addr;

	if (!acpi_disabled) {
		cpu_addr = ISA_PHY_IOBASE;
		fwnode = kzalloc(sizeof(*fwnode), GFP_ATOMIC);
	} else {
		fwnode = parse_isa_base(&cpu_addr);
	}

	if (fwnode)
		add_legacy_isa_io(fwnode, cpu_addr);

	return 0;
}
arch_initcall(register_legacy_isa_io);

static const char *dmi_string_parse(const struct dmi_header *dm, u8 s)
{
	const u8 *bp = ((u8 *) dm) + dm->length;

	if (s) {
		s--;
		while (s > 0 && *bp) {
			bp += strlen(bp) + 1;
			s--;
		}

		if (*bp != 0) {
			size_t len = strlen(bp)+1;
			size_t cmp_len = len > 8 ? 8 : len;

			if (!memcmp(bp, dmi_empty_string, cmp_len))
				return dmi_empty_string;

			return bp;
		}
	}

	return "";

}

static void __init parse_cpu_table(const struct dmi_header *dm)
{
	u64 freq_temp = 0;
	char *dmi_data = (char *)dm;

	freq_temp = ((*(dmi_data + SMBIOS_FREQHIGH_OFFSET) << 8) + \
			((*(dmi_data + SMBIOS_FREQLOW_OFFSET)) & SMBIOS_FREQLOW_MASK));
	cpu_clock_freq = freq_temp * 1000000;

	loongson_sysconf.cpuname = (void *)dmi_string_parse(dm, dmi_data[16]);
	loongson_sysconf.cores_per_package = *(dmi_data + SMBIOS_CORE_PACKAGE_OFFSET);

	pr_info("CpuClock = %llu\n", cpu_clock_freq);

}

static void __init parse_bios_table(const struct dmi_header *dm)
{
	int bios_extern;

	char *dmi_data = (char *)dm;

	bios_extern = *(dmi_data + SMBIOS_BIOSEXTERN_OFFSET);
	b_info.bios_size = (*(dmi_data + SMBIOS_BIOSSIZE_OFFSET) + 1) << 6;

	if (loongson_sysconf.bpi_version == BPI_VERSION_V2) {
		if ((!!(efi_bp->flags & BPI_FLAGS_UEFI_SUPPORTED)) != (!!(bios_extern & LOONGSON_EFI_ENABLE)))
			pr_err("There is a conflict of definitions between efi_bp->flags and smbios\n");
		return ;
	}

	if (bios_extern & LOONGSON_EFI_ENABLE)
		set_bit(EFI_BOOT, &efi.flags);
	else
		clear_bit(EFI_BOOT, &efi.flags);
}

static void __init find_tokens(const struct dmi_header *dm, void *dummy)
{
	switch (dm->type) {
	case 0x0: /* Extern BIOS */
		parse_bios_table(dm);
		break;
	case 0x4: /* Calling interface */
		parse_cpu_table(dm);
		break;
	}
}

static void __init set_pcie_wakeup(void)
{
	acpi_status status;
	u32 value;

	if (loongson_sysconf.is_soc_cpu || acpi_gbl_reduced_hardware)
		return;

	status = acpi_read_bit_register(ACPI_BITREG_PCIEXP_WAKE_DISABLE, &value);
	if (ACPI_FAILURE(status)) {
		return;
	}
	loongson_sysconf.pcie_wake_enabled = !value;
}

static void __init smbios_parse(void)
{
	b_info.bios_vendor = (void *)dmi_get_system_info(DMI_BIOS_VENDOR);
	b_info.bios_version = (void *)dmi_get_system_info(DMI_BIOS_VERSION);
	b_info.bios_release_date = (void *)dmi_get_system_info(DMI_BIOS_DATE);
	b_info.board_vendor = (void *)dmi_get_system_info(DMI_BOARD_VENDOR);
	b_info.board_name = (void *)dmi_get_system_info(DMI_BOARD_NAME);
	dmi_walk(find_tokens, NULL);
}

void __init early_init(void)
{
	fw_init_cmdline();
	fw_init_env();
	efi_init();
	memblock_and_maxpfn_init();
	if (!acpi_tables_present()) {
		fdt_setup();
		if (!of_machine_is_compatible("loongson,ls2k2000") &&
		    !of_machine_is_compatible("loongson,ls2k1500"))
			loongson_sysconf.is_soc_cpu = 1;
	}
}
void __init platform_init(void)
{
	if (delayed_panic)
		panic("Both acpi tables and dtb are not found.");

#if defined(CONFIG_ACPI) && defined(CONFIG_BLK_DEV_INITRD)
	acpi_table_upgrade();
#endif
#ifdef CONFIG_ACPI
	acpi_gbl_use_default_register_widths = false;
	acpi_boot_table_init();
#endif

	set_pcie_wakeup();

#ifdef CONFIG_NUMA
	fw_init_numa_memory();
#else
	fw_init_memory();
#endif
	dmi_scan_machine();
	if (dmi_available) {
		dmi_set_dump_stack_arch_desc();
		smbios_parse();
	}
	pr_info("The BIOS Version: %s\n", b_info.bios_version);

	efi_runtime_init();

	register_smp_ops(&loongson3_smp_ops);
}
void __init prom_free_prom_memory(void)
{
}
