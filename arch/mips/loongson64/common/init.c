/*
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/bootmem.h>
#include <linux/dmi.h>
#include <asm/bootinfo.h>
#include <asm/traps.h>
#include <asm/smp-ops.h>
#include <asm/cacheflush.h>

#include <loongson.h>
#include <loongson-pch.h>
#include <asm/time.h>
#include <linux/memblock.h>
#include <linux/dmi.h>
#include <asm/uasm.h>
#include <asm/efi.h>
#include <workarounds.h>
#include <boot_param.h>
#include <linux/acpi.h>
#include <linux/efi.h>

#define SMBIOS_FREQHIGH_OFFSET		19
#define SMBIOS_FREQLOW_OFFSET		18
#define SMBIOS_FREQLOW_MASK		0xFF
#define SMBIOS_CORE_PACGE_OFFSET	31
#define SMBIOS_BOISSIZE_OFFSET		5
#define SMBIOS_BOISEXTERN_OFFSET 	15
#define LOONGSON_EFI_ENABLE     	(1 << 3)

/* Loongson CPU address windows config space base address */
unsigned long __maybe_unused _loongson_addrwincfg_base;

struct interface_info einter_smbios;
struct board_devices eboard_smbios;

const char *loongson_cpuname;
static const char dmi_empty_string[] = "        ";

extern u32 cpu_clock_freq;
extern void *loongson_fdt_blob;
extern bool loongson_acpiboot_flag;
extern char *bios_vendor;
extern char *bios_release_date;
extern char *board_manufacturer;
extern struct board_devices *eboard;
extern struct interface_info *einter;
extern struct platform_controller_hub ls7a_pch;
extern struct platform_controller_hub rs780_pch;

static void __init mips_nmi_setup(void)
{
	void *base;
	extern char except_vec_nmi;

	base = (void *)(CAC_BASE + 0x380);
	memcpy(base, &except_vec_nmi, 0x80);
	flush_icache_range((unsigned long)base, (unsigned long)base + 0x80);
}

const char *dmi_string_parse(const struct dmi_header *dm, u8 s)
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
	char *dmi_data = (char *)(dm + 1);
	int freq_temp = 0;

	freq_temp = ((*(dmi_data + SMBIOS_FREQHIGH_OFFSET) << 8) + \
			((*(dmi_data + SMBIOS_FREQLOW_OFFSET)) & SMBIOS_FREQLOW_MASK));
	cpu_clock_freq = freq_temp * 1000000;
	loongson_sysconf.cores_per_package = *(dmi_data + SMBIOS_CORE_PACGE_OFFSET);
	loongson_cpuname =  dmi_string_parse(dm, dmi_data[12]);

	mips_cpu_frequency = cpu_clock_freq;
	pr_info("CpuClock = %u\n", cpu_clock_freq);

}

static void __init parse_bios_table(const struct dmi_header *dm)
{
	char *dmi_data = (char *)(dm + 1);
	int bios_extern;

	einter = &einter_smbios;
	einter_smbios.size = *(dmi_data + SMBIOS_BOISSIZE_OFFSET);

	bios_extern = *(dmi_data + SMBIOS_BOISEXTERN_OFFSET);

	if (bios_extern & LOONGSON_EFI_ENABLE)
		set_bit(EFI_BOOT, &efi.flags);
	else
		clear_bit(EFI_BOOT, &efi.flags);
}

#ifdef CONFIG_EFI_PARTITION
static void __init parse_bios_extern(const struct dmi_header *dm)
{
	char *dmi_data = (char *)(dm + 1);
	int bios_extern;

	bios_extern = *(dmi_data + SMBIOS_BOISEXTERN_OFFSET);

	if (bios_extern & LOONGSON_EFI_ENABLE)
		set_bit(EFI_BOOT, &efi.flags);
	else
		clear_bit(EFI_BOOT, &efi.flags);

}

static void __init find_token_pmon(const struct dmi_header *dm, void *dummy)
{
	if (dm->type == 0)
		parse_bios_extern(dm);
}
#endif

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


static void __init smbios_parse(void)
{

	eboard = &eboard_smbios;
	bios_vendor = (void *)dmi_get_system_info(DMI_BIOS_VENDOR);
	strcpy(einter_smbios.description,dmi_get_system_info(DMI_BIOS_VERSION));
	bios_release_date = (void *)dmi_get_system_info(DMI_BIOS_DATE);
	board_manufacturer = (void *)dmi_get_system_info(DMI_BOARD_VENDOR);
	strcpy(eboard_smbios.name, dmi_get_system_info(DMI_BOARD_NAME));
	dmi_walk(find_tokens, NULL);
}

static void loongson_pch_init(void)
{
	switch (acpi_irq_model) {
	case ACPI_IRQ_MODEL_IOAPIC:
		loongson_pch = &ls7a_pch;
		loongson_sysconf.msi_address_lo = 0x2FF00000;
		loongson_sysconf.msi_address_hi = 0;
		loongson_sysconf.msi_base_irq = LOONGSON_PCI_MSI_IRQ_BASE;
		loongson_sysconf.msi_last_irq = LOONGSON_PCI_MSI_IRQ_BASE + (loongson_cpu_has_msi256 ? 192 : 64);
		loongson_sysconf.io_base_irq = LOONGSON_PCH_IRQ_BASE;
		loongson_sysconf.io_last_irq = LOONGSON_PCH_IRQ_BASE + (loongson_cpu_has_msi256 ? 256 : 128);
		loongson_sysconf.ec_sci_irq = 0x07;
		break;
	case ACPI_IRQ_MODEL_PIC:

		loongson_pch = &rs780_pch;

		loongson_sysconf.msi_address_lo = 0xFEE00000;
		loongson_sysconf.msi_address_hi = 0;
		loongson_sysconf.msi_base_irq = 16;
		loongson_sysconf.msi_last_irq = 128;
		loongson_sysconf.io_base_irq = 0;
		loongson_sysconf.io_last_irq = 128;
		loongson_sysconf.ec_sci_irq = 0x07;
		break;
	default:
		BUG_ON(true);
	}
}

unsigned long max_low_pfn_mapped;
extern unsigned long initrd_start, initrd_end;
extern void __init memblock_and_maxpfn_init(void);
void plat_early_init(void)
{
	prom_init_cmdline();
	prom_init_env();
	if (loongson_acpiboot_flag)
		memblock_and_maxpfn_init();
}
void __init prom_init(void)
{
#ifdef CONFIG_CPU_SUPPORTS_ADDRWINCFG
	_loongson_addrwincfg_base = (unsigned long)
		ioremap(LOONGSON_ADDRWINCFG_BASE, LOONGSON_ADDRWINCFG_SIZE);
#endif

	/* init base address of io space */
	set_io_port_base((unsigned long)
		ioremap(LOONGSON_PCIIO_BASE, LOONGSON_PCIIO_SIZE));


	if (loongson_acpiboot_flag) {
#ifdef CONFIG_EFI
		efi_init();
#endif

#if defined(CONFIG_ACPI) && defined(CONFIG_BLK_DEV_INITRD)
		acpi_table_upgrade();
#endif
#ifdef CONFIG_ACPI
		acpi_gbl_use_default_register_widths = false;
#endif
		acpi_boot_table_init();
		acpi_boot_init();
 	} else {
		register_pch_pic(0, LS7A_PCH_CFG_BASE, LOONGSON_PCH_IRQ_BASE);
 	}

#ifdef CONFIG_NUMA
	prom_init_numa_memory();
#else
	if (loongson_acpiboot_flag) {
		prom_init_memory_new();
	} else {
		prom_init_memory();
	}
#endif

	dmi_scan_machine();
	dmi_set_dump_stack_arch_desc();

	if (loongson_acpiboot_flag) {
		smbios_parse();
		loongson_pch_init();
	}

#ifdef CONFIG_EFI_PARTITION
	if (strstr(einter->description,"uefi") || strstr(einter->description,"UEFI"))
		set_bit(EFI_BOOT, &efi.flags);
	else if (dmi_get_system_info(DMI_SYS_VENDOR) != NULL)
		dmi_walk(find_token_pmon, NULL);
#endif
	pr_info("The BIOS Version: %s\n",einter->description);

	if ((loongson_pch != NULL) && (loongson_pch->early_config))
		loongson_pch->early_config();
	else
		prom_printf("Can't get Bridge information!\n");


	/*init the uart base address */
	prom_init_uart_base();
	register_smp_ops(&loongson3_smp_ops);
	board_nmi_handler_setup = mips_nmi_setup;

	init_hypervisor_platform();
}

void __init prom_free_prom_memory(void)
{
}
