/*
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Copyright (C) 2007 Lemote, Inc. & Institute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 * Copyright (C) 2009 Lemote, Inc.
 * Author: Zhangjin Wu, wuzhangjin@gmail.com
 */
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <acpi/reboot.h>
#include <linux/acpi.h>
#include <asm/bootinfo.h>
#include <asm/idle.h>
#include <asm/page.h>
#include <asm/reboot.h>

#include <linux/efi.h>
#include <loongson.h>
#include <boot_param.h>

extern bool loongson_acpiboot_flag;

static inline void loongson_reboot(void)
{
#ifndef CONFIG_CPU_JUMP_WORKAROUNDS
	((void (*)(void))ioremap_nocache(LOONGSON_BOOT_BASE, 4)) ();
#else
	void (*func)(void);

	func = (void *)ioremap_nocache(LOONGSON_BOOT_BASE, 4);

	__asm__ __volatile__(
	"	.set	noat						\n"
	"	jr	%[func]						\n"
	"	.set	at						\n"
	: /* No outputs */
	: [func] "r" (func));
#endif
}

static void loongson_restart(char *command)
{
#ifndef CONFIG_LEFI_FIRMWARE_INTERFACE
	/* do preparation for reboot */
	mach_prepare_reboot();

	/* reboot via jumping to boot base address */
	loongson_reboot();
#else
#ifdef CONFIG_EFI
	if (efi_capsule_pending(NULL)) {
		pr_info("EFI capsule is pending, forcing EFI reboot.\n");
		efi_reboot(REBOOT_WARM, NULL);
	} else if (!acpi_disabled) {
#else
	if (!acpi_disabled) {
#endif
		acpi_reboot();
	} else {
		void (*fw_restart)(void) = (void *)loongson_sysconf.restart_addr;
		fw_restart();
	}
	while (1) {
		if (cpu_wait)
			cpu_wait();
	}
#endif
}

#include <asm/delay.h>
#include <linux/sched/debug.h>
static void loongson_poweroff(void)
{
#ifndef CONFIG_LEFI_FIRMWARE_INTERFACE
	mach_prepare_shutdown();

	/*
	 * It needs a wait loop here, but mips/kernel/reset.c already calls
	 * a generic delay loop, machine_hang(), so simply return.
	 */
	return;
#else
	void (*fw_poweroff)(void) = (void *)loongson_sysconf.poweroff_addr;
	fw_poweroff();
	while (1) {
		if (cpu_wait)
			cpu_wait();
	}
#endif
}

static void loongson_halt(void)
{
	pr_notice("\n\n** You can safely turn off the power now **\n\n");
	while (1) {
		if (cpu_wait)
			cpu_wait();
	}
}

#ifdef CONFIG_KEXEC

#define MAX_ARGS	64

static int kexec_argc;
static int kdump_argc;
static void *kexec_argv;
static void *kdump_argv;

static struct page *loongson_crash_control_pages(struct kimage *image)
{
	int i;
	unsigned long hole_start, size;
	struct kexec_segment kexec_segment;

	size = 1 << PAGE_SHIFT;
	for (i = image->nr_segments - 1; i > 0; i--) {
		kexec_segment = image->segment[i - 1];
		hole_start = image->segment[i].mem - size;
		if (hole_start >= kexec_segment.mem + kexec_segment.memsz)
			return pfn_to_page(hole_start >> PAGE_SHIFT);
	}

	return NULL;
}

static int loongson_kexec_prepare(struct kimage *image)
{
	int i, offt, argc = 0;
	int *argv;
	char *str, *ptr, *bootloader = "kexec";

	argv = kmalloc(COMMAND_LINE_SIZE, GFP_KERNEL);
	if (!argv)
		return -ENOMEM;

	for (i = 0; i < image->nr_segments; i++) {
		if (!strncmp(bootloader, (char *)image->segment[i].buf,
				strlen(bootloader))) {
			argv[argc++] = fw_arg1 + COMMAND_LINE_SIZE/2;
			str = (char *)argv + COMMAND_LINE_SIZE/2;
			memcpy(str, image->segment[i].buf, COMMAND_LINE_SIZE/2);
			ptr = strchr(str, ' ');
			while (ptr) {
				*ptr = '\0';
				if (ptr[1] != ' ') {
					offt = (int)(ptr - str + 1);
					argv[argc++] = fw_arg1 + COMMAND_LINE_SIZE/2 + offt;
				}
				ptr = strchr(ptr + 1, ' ');
			}
			break;
		}
	}

	if (image->type == KEXEC_TYPE_CRASH) {
		/* A specific way to find a hole in crashkernel. */
		image->control_code_page = loongson_crash_control_pages(image);
		if (!image->control_code_page) {
			pr_err("Could not allocate control_code_buffer\n");
			kfree(kdump_argv);
			kfree(argv);
			kfree(image);
			return -ENOMEM;
		}
		kfree(kdump_argv);
		kdump_argc = argc;
		kdump_argv = argv;
	} else {
		kfree(kexec_argv);
		kexec_argc = argc;
		kexec_argv = argv;
	}

	return 0;
}

#ifdef CONFIG_SMP
static void kexec_smp_down(void *ignored)
{
	int cpu = smp_processor_id();

	if (!cpu_online(cpu))
		return;

	set_cpu_online(cpu, false);
	local_irq_disable();
	while (!atomic_read(&kexec_ready_to_reboot))
		cpu_relax();

	kexec_reboot();
}
#endif

static void loongson_kexec_shutdown(void)
{
	int cpu;

	fw_arg0 = kexec_argc;
	memcpy((void *)fw_arg1, kexec_argv, COMMAND_LINE_SIZE);

	kexec_args[0] = fw_arg0;
	kexec_args[1] = fw_arg1;
	kexec_args[2] = fw_arg2;
#ifdef CONFIG_SMP
	secondary_kexec_args[0] = TO_UNCAC(0x3ff01000);
	for_each_possible_cpu(cpu)
		if (!cpu_online(cpu))
			cpu_up(cpu); /* Everyone should go to reboot_code_buffer */

	smp_call_function(kexec_smp_down, NULL, 0);
	smp_wmb();
	while (num_online_cpus() > 1) {
		mdelay(1);
		cpu_relax();
	}
#endif
}

static void loongson_crash_shutdown(struct pt_regs *regs)
{
	fw_arg0 = kdump_argc;
	memcpy((void *)fw_arg1, kdump_argv, COMMAND_LINE_SIZE);

	kexec_args[0] = fw_arg0;
	kexec_args[1] = fw_arg1;
	kexec_args[2] = fw_arg2;
#ifdef CONFIG_SMP
	secondary_kexec_args[0] = TO_UNCAC(0x3ff01000);
#endif
	default_machine_crash_shutdown(regs);
}

#endif

static int __init mips_reboot_setup(void)
{
	_machine_restart = loongson_restart;
	_machine_halt = loongson_halt;
	pm_power_off = loongson_poweroff;

#ifdef CONFIG_KEXEC
	_machine_kexec_prepare = loongson_kexec_prepare;
	_machine_kexec_shutdown = loongson_kexec_shutdown;
	_machine_crash_shutdown = loongson_crash_shutdown;
#endif

	return 0;
}

arch_initcall(mips_reboot_setup);
