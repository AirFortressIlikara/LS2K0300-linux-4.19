/*
 * Based on Ocelot Linux port, which is
 * Copyright 2001 MontaVista Software Inc.
 * Author: jsun@mvista.com or jsun@junsun.net
 *
 * Copyright 2003 ICT CAS
 * Author: Michael Guo <guoyi@ict.ac.cn>
 *
 * Copyright (C) 2007 Lemote Inc. & Institute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 * Copyright (C) 2009 Lemote Inc.
 * Author: Wu Zhangjin, wuzhangjin@gmail.com
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/export.h>
#include <asm/time.h>
#include <asm/bootinfo.h>
#include <asm/dma-coherence.h>
#include <loongson.h>
#include <loongson-pch.h>
#include <boot_param.h>
#include <workarounds.h>
#include <linux/efi.h>
#include <linux/acpi.h>

struct boot_params *boot_p;
struct board_devices *eboard;
struct interface_info *einter;
struct system_loongson *esys;
struct loongson_params *loongson_p;
struct bootparamsinterface *efi_bp;
struct efi_cpuinfo_loongson *ecpu;
struct efi_memory_map_loongson *loongson_memmap;
struct irq_source_routing_table *eirq_source;
struct loongson_special_attribute *especial;
struct loongson_system_configuration loongson_sysconf;

struct loongsonlist_vbios *pvbios;
struct loongsonlist_mem_map *loongson_mem_map;

u64 loongson_chipcfg[MAX_PACKAGES] = {0xffffffffbfc00180};
u64 loongson_chiptemp[MAX_PACKAGES];
u64 loongson_freqctrl[MAX_PACKAGES];
unsigned long long smp_group[4];

struct platform_controller_hub dummy_pch;
struct platform_controller_hub *loongson_pch;

bool loongson_acpiboot_flag;

u32 cpu_guestmode;
u32 cpu_clock_freq;
char cpu_full_name[64];
void *loongson_fdt_blob;
unsigned int has_systab = 0;
unsigned long systab_addr;

extern char *bios_vendor;
extern char *loongson_cpuname;
extern char *bios_release_date;
extern char *board_manufacturer;
extern char _bios_info[];
extern char _board_info[];

EXPORT_SYMBOL(eboard);
EXPORT_SYMBOL(cpu_clock_freq);
EXPORT_SYMBOL(loongson_sysconf);
EXPORT_SYMBOL(loongson_pch);

#define parse_even_earlier(res, option, p)				\
do {									\
	unsigned int tmp __maybe_unused;				\
									\
	if (strncmp(option, (char *)p, strlen(option)) == 0)		\
		tmp = kstrtou32((char *)p + strlen(option"="), 10, &res); \
} while (0)

void __init no_efiboot_env(void)
{
	/* pmon passes arguments in 32bit pointers */
	char freq[12];
	unsigned int processor_id;
	char *bios_info __maybe_unused;
	char *board_info __maybe_unused;

#ifndef CONFIG_LEFI_FIRMWARE_INTERFACE
	int *_prom_envp;
	long l;

	/* firmware arguments are initialized in head.S */
	_prom_envp = (int *)fw_arg2;

	l = (long)*_prom_envp;
	while (l != 0) {
		parse_even_earlier(cpu_clock_freq, "cpuclock", l);
		parse_even_earlier(memsize, "memsize", l);
		parse_even_earlier(highmemsize, "highmemsize", l);
		_prom_envp++;
		l = (long)*_prom_envp;
	}
	if (memsize == 0)
		memsize = 256;

	loongson_pch = &dummy_pch;
	loongson_sysconf.nr_uarts = 1;

	pr_info("memsize=%u, highmemsize=%u\n", memsize, highmemsize);
#else
	int i;

	/* firmware arguments are initialized in head.S */
	boot_p = (struct boot_params *)fw_arg2;
	loongson_p = &(boot_p->efi.sysinfo.lp);

	esys = (struct system_loongson *)
		((u64)loongson_p + loongson_p->system_offset);
	ecpu = (struct efi_cpuinfo_loongson *)
		((u64)loongson_p + loongson_p->cpu_offset);
	eboard	= (struct board_devices *)
		((u64)loongson_p + loongson_p->boarddev_table_offset);
	einter  = (struct interface_info *)((u64)loongson_p + loongson_p->interface_offset);
	especial = (struct loongson_special_attribute *)((u64)loongson_p + loongson_p->special_offset);
	eirq_source = (struct irq_source_routing_table *)
		((u64)loongson_p + loongson_p->irq_offset);
	loongson_memmap = (struct efi_memory_map_loongson *)
		((u64)loongson_p + loongson_p->memory_offset);

	cpu_clock_freq = ecpu->cpu_clock_freq;
	loongson_sysconf.cputype = ecpu->cputype;
	switch (ecpu->cputype) {
	case Legacy_3A:
	case Loongson_3A:
		loongson_sysconf.cores_per_node = 4;
		loongson_sysconf.cores_per_package = 4;
		smp_group[0] = 0x900000003ff01000;
		smp_group[1] = 0x900010003ff01000;
		smp_group[2] = 0x900020003ff01000;
		smp_group[3] = 0x900030003ff01000;
		loongson_chipcfg[0] = 0x900000001fe00180;
		loongson_chipcfg[1] = 0x900010001fe00180;
		loongson_chipcfg[2] = 0x900020001fe00180;
		loongson_chipcfg[3] = 0x900030001fe00180;
		loongson_chiptemp[0] = 0x900000001fe0019c;
		loongson_chiptemp[1] = 0x900010001fe0019c;
		loongson_chiptemp[2] = 0x900020001fe0019c;
		loongson_chiptemp[3] = 0x900030001fe0019c;
		loongson_freqctrl[0] = 0x900000001fe001d0;
		loongson_freqctrl[1] = 0x900010001fe001d0;
		loongson_freqctrl[2] = 0x900020001fe001d0;
		loongson_freqctrl[3] = 0x900030001fe001d0;
		loongson_sysconf.ht_control_base = 0x90000EFDFB000000;
		loongson_sysconf.workarounds = WORKAROUND_CPUFREQ;
		break;
	case Legacy_3B:
	case Loongson_3B:
		loongson_sysconf.cores_per_node = 4; /* One chip has 2 nodes */
		loongson_sysconf.cores_per_package = 8;
		smp_group[0] = 0x900000003ff01000;
		smp_group[1] = 0x900010003ff05000;
		smp_group[2] = 0x900020003ff09000;
		smp_group[3] = 0x900030003ff0d000;
		loongson_chipcfg[0] = 0x900000001fe00180;
		loongson_chipcfg[1] = 0x900020001fe00180;
		loongson_chipcfg[2] = 0x900040001fe00180;
		loongson_chipcfg[3] = 0x900060001fe00180;
		loongson_chiptemp[0] = 0x900000001fe0019c;
		loongson_chiptemp[1] = 0x900020001fe0019c;
		loongson_chiptemp[2] = 0x900040001fe0019c;
		loongson_chiptemp[3] = 0x900060001fe0019c;
		loongson_freqctrl[0] = 0x900000001fe001d0;
		loongson_freqctrl[1] = 0x900020001fe001d0;
		loongson_freqctrl[2] = 0x900040001fe001d0;
		loongson_freqctrl[3] = 0x900060001fe001d0;
		loongson_sysconf.ht_control_base = 0x90001EFDFB000000;
		loongson_sysconf.workarounds = WORKAROUND_CPUHOTPLUG;
		break;
	default:
		loongson_sysconf.cores_per_node = 1;
		loongson_sysconf.cores_per_package = 1;
		loongson_chipcfg[0] = 0x900000001fe00180;
	}

	/* parse bios info */
	strcpy(_bios_info, einter->description);
	bios_info = _bios_info;
	bios_vendor = strsep(&bios_info, "-");
	strsep(&bios_info, "-");
	strsep(&bios_info, "-");
	bios_release_date = strsep(&bios_info, "-");
	if (!bios_release_date)
		bios_release_date = especial->special_name;

	/* parse board info */
	strcpy(_board_info, eboard->name);
	board_info = _board_info;
	board_manufacturer = strsep(&board_info, "-");

	loongson_sysconf.nr_cpus = ecpu->nr_cpus;
	loongson_sysconf.nr_nodes = ecpu->total_node;
	loongson_sysconf.boot_cpu_id = ecpu->cpu_startup_core_id;
	loongson_sysconf.reserved_cpus_mask = ecpu->reserved_cores_mask;
#ifdef CONFIG_KEXEC
	loongson_sysconf.boot_cpu_id = get_ebase_cpunum();
	for (i = 0; i < loongson_sysconf.boot_cpu_id; i++)
		loongson_sysconf.reserved_cpus_mask |= (1<<i);
	pr_info("Boot CPU ID is being fixed from %d to %d\n",
		ecpu->cpu_startup_core_id, loongson_sysconf.boot_cpu_id);
#endif
	if (ecpu->nr_cpus > NR_CPUS || ecpu->nr_cpus == 0)
		loongson_sysconf.nr_cpus = NR_CPUS;

	if (loongson_sysconf.nr_nodes * loongson_sysconf.cores_per_node < loongson_sysconf.nr_cpus)
		loongson_sysconf.nr_nodes = (loongson_sysconf.nr_cpus +
			loongson_sysconf.cores_per_node - 1) /
			loongson_sysconf.cores_per_node;

	if (!strncmp(ecpu->cpuname, "Loongson", 8))
		strncpy(cpu_full_name, ecpu->cpuname, sizeof(cpu_full_name));
	if (cpu_full_name[0] == 0)
		strncpy(cpu_full_name, __cpu_full_name[0], sizeof(cpu_full_name));

	loongson_sysconf.pci_mem_start_addr = eirq_source->pci_mem_start_addr;
	loongson_sysconf.pci_mem_end_addr = eirq_source->pci_mem_end_addr;
	loongson_sysconf.pci_io_base = eirq_source->pci_io_start_addr;
	loongson_sysconf.dma_mask_bits = eirq_source->dma_mask_bits;
	if (loongson_sysconf.dma_mask_bits < 32 ||
		loongson_sysconf.dma_mask_bits > 64)
		loongson_sysconf.dma_mask_bits = 32;

	hw_coherentio = !eirq_source->dma_noncoherent;
	pr_info("BIOS configured I/O coherency: %s\n", hw_coherentio?"ON":"OFF");

	cpu_guestmode = !cpu_has_vz;

	if (strstr(eboard->name,"2H")) {
		loongson_pch = &ls2h_pch;
		loongson_sysconf.ec_sci_irq = 0x80;
		loongson_sysconf.msi_address_lo = 0;
		loongson_sysconf.msi_address_hi = 0;
		loongson_sysconf.msi_base_irq = 160;
		loongson_sysconf.msi_last_irq = 192;
		loongson_sysconf.io_base_irq = 64; /* ? Fix me*/
		loongson_sysconf.io_last_irq = 192;
		loongson_fdt_blob = __dtb_loongson3_ls2h_begin;
	} else if (strstr(eboard->name,"7A")) {
		loongson_pch = &ls7a_pch;
		loongson_sysconf.ec_sci_irq = 0x7B;
		loongson_sysconf.msi_address_lo = 0x2FF00000;
		loongson_sysconf.msi_address_hi = 0;
		loongson_sysconf.msi_base_irq = LOONGSON_PCI_MSI_IRQ_BASE;
		loongson_sysconf.msi_last_irq = LOONGSON_PCI_MSI_IRQ_BASE + (loongson_cpu_has_msi256 ? 192 : 64);
		loongson_sysconf.io_base_irq = LOONGSON_PCH_IRQ_BASE;
		loongson_sysconf.io_last_irq = LOONGSON_PCH_IRQ_BASE + (loongson_cpu_has_msi256 ? 256 : 128);
		loongson_fdt_blob = __dtb_loongson3_ls7a_begin;
	} else {
		loongson_pch = &rs780_pch;
		loongson_sysconf.ec_sci_irq = 0x07;
		loongson_sysconf.msi_address_lo = 0xFEE00000;
		loongson_sysconf.msi_address_hi = 0;
		loongson_sysconf.msi_base_irq = 16;
		loongson_sysconf.msi_last_irq = 128;
		loongson_sysconf.io_base_irq = 0;
		loongson_sysconf.io_last_irq = 128;
		loongson_fdt_blob = __dtb_loongson3_rs780_begin;
	}
	if (esys->vers >= 2 && esys->of_dtb_addr)
		loongson_fdt_blob = (void *)(esys->of_dtb_addr);

	loongson_sysconf.restart_addr = boot_p->reset_system.ResetWarm;
	loongson_sysconf.poweroff_addr = boot_p->reset_system.Shutdown;
	loongson_sysconf.suspend_addr = boot_p->reset_system.DoSuspend;

	loongson_sysconf.vgabios_addr = boot_p->efi.sysinfo.vga_bios;
	pr_debug("Shutdown Addr: %llx, Restart Addr: %llx, VBIOS Addr: %llx\n",
		loongson_sysconf.poweroff_addr, loongson_sysconf.restart_addr,
		loongson_sysconf.vgabios_addr);

	memset(loongson_sysconf.ecname, 0, 32);
	if (esys->has_ec)
		memcpy(loongson_sysconf.ecname, esys->ec_name, 32);
	loongson_sysconf.workarounds |= esys->workarounds;

	loongson_sysconf.nr_uarts = esys->nr_uarts;
	if (esys->nr_uarts < 1 || esys->nr_uarts > MAX_UARTS)
		loongson_sysconf.nr_uarts = 1;
	memcpy(loongson_sysconf.uarts, esys->uarts,
		sizeof(struct uart_device) * loongson_sysconf.nr_uarts);

	loongson_sysconf.nr_sensors = esys->nr_sensors;
	if (loongson_sysconf.nr_sensors > MAX_SENSORS)
		loongson_sysconf.nr_sensors = 0;
	if (loongson_sysconf.nr_sensors)
		memcpy(loongson_sysconf.sensors, esys->sensors,
			sizeof(struct sensor_device) * loongson_sysconf.nr_sensors);
#endif
	if (cpu_clock_freq == 0) {
		processor_id = (&current_cpu_data)->processor_id;
		switch (processor_id & PRID_REV_MASK) {
		case PRID_REV_LOONGSON2E:
			cpu_clock_freq = 533080000;
			break;
		case PRID_REV_LOONGSON2F:
			cpu_clock_freq = 797000000;
			break;
		case PRID_REV_LOONGSON3A_R1:
		case PRID_REV_LOONGSON3A_R2_0:
		case PRID_REV_LOONGSON3A_R2_1:
		case PRID_REV_LOONGSON3A_R3_0:
		case PRID_REV_LOONGSON3A_R3_1:
			cpu_clock_freq = 900000000;
			break;
		case PRID_REV_LOONGSON3B_R1:
		case PRID_REV_LOONGSON3B_R2:
			cpu_clock_freq = 1000000000;
			break;
		default:
			cpu_clock_freq = 100000000;
			break;
		}
	}
	mips_cpu_frequency = cpu_clock_freq;
	pr_info("CpuClock = %u\n", cpu_clock_freq);

	/* Append default cpu frequency with round-off */
	sprintf(freq, " @ %uMHz", (cpu_clock_freq + 500000) / 1000000);
	strncat(cpu_full_name, freq, sizeof(cpu_full_name));
	__cpu_full_name[0] = cpu_full_name;
}


u8 ext_listhdr_checksum(u8 *buffer, u32 length)
{
	u8 sum = 0;
	u8 *end = buffer + length;

	while (buffer < end) {
		sum = (u8)(sum + *(buffer++));
	}

	return (sum);
}
int parse_mem(struct _extention_list_hdr *head)
{
	loongson_mem_map = (struct loongsonlist_mem_map *)head;
	if (ext_listhdr_checksum((u8 *)loongson_mem_map, head->length)) {
		prom_printf("mem checksum error\n");
		return -EPERM;
	}
	return 0;
}


int parse_vbios(struct _extention_list_hdr *head)
{
	pvbios = (struct loongsonlist_vbios *)head;

	if (ext_listhdr_checksum((u8 *)pvbios, head->length)) {
		prom_printf("vbios_addr checksum error\n");
		return -EPERM;
	} else {
		loongson_sysconf.vgabios_addr = pvbios->vbios_addr;
	}
	return 0;
}

static int parse_screeninfo(struct _extention_list_hdr *head)
{
	struct loongsonlist_screeninfo *pscreeninfo;

	pscreeninfo = (struct loongsonlist_screeninfo *)head;
	if (ext_listhdr_checksum((u8 *)pscreeninfo, head->length)) {
		prom_printf("screeninfo_addr checksum error\n");
		return -EPERM;
	}

	memcpy(&screen_info, &pscreeninfo->si, sizeof(screen_info));
	return 0;
}

static int list_find(struct _extention_list_hdr *head)
{
	struct _extention_list_hdr *fhead = head;

	if (fhead == NULL) {
		prom_printf("the link is empty!\n");
		return -1;
	}

	while(fhead != NULL) {
		if (memcmp(&(fhead->signature), LOONGSON_MEM_LINKLIST, 3) == 0) {
			if (parse_mem(fhead) !=0) {
				prom_printf("parse mem failed\n");
				return -EPERM;
			}
		} else if (memcmp(&(fhead->signature), LOONGSON_VBIOS_LINKLIST, 5) == 0) {
			if (parse_vbios(fhead) != 0) {
				prom_printf("parse vbios failed\n");
				return -EPERM;
			}
		} else if (memcmp(&(fhead->signature), LOONGSON_SCREENINFO_LINKLIST, 5) == 0) {
			if (parse_screeninfo(fhead) != 0) {
				prom_printf("parse screeninfo failed\n");
				return -EPERM;
			}
		}
		fhead = fhead->next;
	}
	return 0;

}
#ifdef CONFIG_SUSPEND
extern void init_suspend_addr(void);
#endif
void __init prom_init_env(void)
{
	efi_bp = (struct bootparamsinterface *)fw_arg2;

	if (memcmp(&(efi_bp->signature), LOONGSON_EFIBOOT_SIGNATURE, 3) != 0) {
		acpi_disabled = 1;
		no_efiboot_env();
	}else {
		cpu_guestmode = !cpu_has_vz;
		smp_group[0] = 0x900000003ff01000;
		smp_group[1] = 0x900010003ff01000;
		smp_group[2] = 0x900020003ff01000;
		smp_group[3] = 0x900030003ff01000;

		loongson_sysconf.ht_control_base = 0x90000EFDFB000000;
		loongson_chipcfg[0] = 0x900000001fe00180;
		loongson_chipcfg[1] = 0x900010001fe00180;
		loongson_chipcfg[2] = 0x900020001fe00180;
		loongson_chipcfg[3] = 0x900030001fe00180;

		loongson_chiptemp[0] = 0x900000001fe0019c;
		loongson_chiptemp[1] = 0x900010001fe0019c;
		loongson_chiptemp[2] = 0x900020001fe0019c;
		loongson_chiptemp[3] = 0x900030001fe0019c;
		loongson_freqctrl[0] = 0x900000001fe001d0;
		loongson_freqctrl[1] = 0x900010001fe001d0;
		loongson_freqctrl[2] = 0x900020001fe001d0;
		loongson_freqctrl[3] = 0x900030001fe001d0;

		loongson_sysconf.workarounds = WORKAROUND_CPUFREQ;
		hw_coherentio = 1;
		loongson_sysconf.nr_uarts = 1;
		loongson_acpiboot_flag = 1;
		acpi_disabled = 0;
		loongson_sysconf.pci_mem_start_addr = PCI_MEM_START_ADDR;
		loongson_sysconf.pci_mem_end_addr = PCI_MEM_END_ADDR;
		loongson_sysconf.pci_io_base = LOONGSON_PCI_IOBASE;
		loongson_sysconf.dma_mask_bits = LOONGSON_DMA_MASK_BIT;

		if (list_find(efi_bp->extlist))
			prom_printf("Scan bootparm failed\n");

	}


}


u64 ls_get_vbios_addr(void)
{
       return loongson_sysconf.vgabios_addr;
}
EXPORT_SYMBOL(ls_get_vbios_addr);


static int __init init_cpu_fullname(void)
{
 	int cpu;

	/* get the __cpu_full_name from bios */
	if (ecpu) {
		if((ecpu->vers > 1) && (ecpu->cpuname[0] != 0))
			for(cpu = 0; cpu < NR_CPUS; cpu++)
				__cpu_full_name[cpu] = ecpu->cpuname;
	} else {
		if (loongson_acpiboot_flag == 1) {
			for(cpu = 0; cpu < NR_CPUS; cpu++) {
				__cpu_full_name[cpu] = loongson_cpuname;
			}
		}
	}
 	return 0;
}
arch_initcall(init_cpu_fullname);
