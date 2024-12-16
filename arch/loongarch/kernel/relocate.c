// SPDX-License-Identifier: GPL-2.0
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Support for Kernel relocation at boot time
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 * Authors: Huacai Chen (chenhuacai@loongson.cn)
 */
#include <asm/bootinfo.h>
#include <asm/fw.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <linux/elf.h>
#include <linux/kernel.h>
#include <linux/start_kernel.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/mm.h>
#include <linux/of_fdt.h>

#define RELOCATED(x) ((void *)((long)x + offset))

extern u32 _relocation_start[];	/* End kernel image / start relocation table */
extern u32 _relocation_end[];	/* End relocation table */

static void __init apply_r_loongarch_64_rel(u32 *loc_new, long offset)
{
	*(u64 *)loc_new += offset;
}

static void __init apply_r_loongarch_32_rel(u32 *loc_new, long offset)
{
	*loc_new += offset;
}

/*
 * The details about la.abs r1, x on LoongArch
 *
 * lu12i.w $r1,0
 * ori     $r1,$r1,0x0
 * lu32i.d $r1,0
 * lu52i.d $r1,$r1,0
 *
 * LoongArch use lu12i.w, ori, lu32i.d, lu52i.d to load a 64bit imm.
 * lu12i.w load bit31~bit12, ori load bit11~bit0,
 * lu32i.d load bit51~bit32, lu52i.d load bit63~bit52
 */

static void __init apply_r_loongarch_la_rel(u32 *loc_new, long offset)
{
	unsigned long long dest;
	union loongarch_instruction *ori, *lu12iw, *lu32id, *lu52id;

	ori = (union loongarch_instruction *)&loc_new[1];
	lu12iw = (union loongarch_instruction *)&loc_new[0];
	lu32id = (union loongarch_instruction *)&loc_new[2];
	lu52id = (union loongarch_instruction *)&loc_new[3];

	dest = ori->reg2ui12_format.simmediate & 0xfff;
	dest |= (lu12iw->reg1i20_format.simmediate & 0xfffff) << 12;
	dest |= ((u64)lu32id->reg1i20_format.simmediate & 0xfffff) << 32;
	dest |= ((u64)lu52id->reg2i12_format.simmediate & 0xfff) << 52;
	dest += offset;

	ori->reg2ui12_format.simmediate = dest & 0xfff;
	lu12iw->reg1i20_format.simmediate = (dest >> 12) & 0xfffff;
	lu32id->reg1i20_format.simmediate = (dest >> 32) & 0xfffff;
	lu52id->reg2i12_format.simmediate = (dest >> 52) & 0xfff;
}

static int __init do_relocations(void *kbase_new, long offset, u32 *rstart, u32 *rend)
{
	u32 *r;
	u32 *loc_new;
	int type;

	for (r = rstart; r < rend; r++) {
		/* Sentinel for last relocation */
		if (*r == 0)
			break;

		type = (*r >> 28) & 0xf;
		loc_new = (void *)(kbase_new + ((*r & 0x0fffffff) << 2));

		switch (type) {
		case 1:
			apply_r_loongarch_32_rel(loc_new, offset);
			break;
		case 2:
			apply_r_loongarch_64_rel(loc_new, offset);
			break;
		case 3:
			apply_r_loongarch_la_rel(loc_new, offset);
			break;
		default:
			pr_err("Unhandled relocation type %d\n", type);
			return -ENOEXEC;
		}
	}

	return 0;
}

#ifdef CONFIG_RANDOMIZE_BASE

static inline __init unsigned long rotate_xor(unsigned long hash,
					      const void *area, size_t size)
{
	size_t i;
	unsigned long *ptr = (unsigned long *)area;

	for (i = 0; i < size / sizeof(hash); i++) {
		/* Rotate by odd number of bits and XOR. */
		hash = (hash << ((sizeof(hash) * 8) - 7)) | (hash >> 7);
		hash ^= ptr[i];
	}

	return hash;
}

static inline __init unsigned long get_random_boot(void)
{
	unsigned long entropy = random_get_entropy();
	unsigned long hash = 0;

	/* Attempt to create a simple but unpredictable starting entropy. */
	hash = rotate_xor(hash, linux_banner, strlen(linux_banner));

	/* Add in any runtime entropy we can get */
	hash = rotate_xor(hash, &entropy, sizeof(entropy));

	return hash;
}

static inline __init bool kaslr_disabled(void)
{
	char *str;

#if defined(CONFIG_CMDLINE_BOOL)
	const char *builtin_cmdline = CONFIG_CMDLINE;

	str = strstr(builtin_cmdline, "nokaslr");
	if (str == builtin_cmdline ||
	    (str > builtin_cmdline && *(str - 1) == ' '))
		return true;
#endif
	str = strstr(arcs_cmdline, "nokaslr");
	if (str == arcs_cmdline || (str > arcs_cmdline && *(str - 1) == ' '))
		return true;

	return false;
}

static inline void __init *determine_relocation_address(void)
{
	/* Choose a new address for the kernel */
	unsigned long kernel_length;
	void *dest = &_text;
	unsigned long offset;

	if (kaslr_disabled())
		return dest;

	kernel_length = (long)_end - (long)(&_text);

	offset = get_random_boot() << 16;
	offset &= (CONFIG_RANDOMIZE_BASE_MAX_OFFSET - 1);
	if (offset < kernel_length)
		offset += ALIGN(kernel_length, 0xffff);

	return RELOCATED(dest);
}

#else

static inline void __init *determine_relocation_address(void)
{
	/*
	 * Choose a new address for the kernel
	 * For now we'll hard code the destination
	 */
	return (void *)(CAC_BASE + 0x03000000);
}

#endif

static inline int __init relocation_addr_valid(void *loc_new)
{
	if ((unsigned long)loc_new & 0x0000ffff) {
		/* Inappropriately aligned new location */
		return 0;
	}
	if ((unsigned long)loc_new < (unsigned long)&_end) {
		/* New location overlaps original kernel */
		return 0;
	}
	return 1;
}

void *__init relocate_kernel(void)
{
	void *loc_new;
	unsigned long kernel_length;
	unsigned long bss_length;
	long offset = 0;
	int res = 1;
	/* Default to original kernel entry point */
	void *kernel_entry = start_kernel;
	char *cmdline;

	/* Get the command line */
	if (!fw_arg2) {
		/* a0 = efi flag, a1 = small fdt */
		early_init_dt_scan(early_ioremap(fw_arg1, SZ_64K));
		strscpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);
	} else if (fw_arg0 == 1 || fw_arg0 == 0) {
		/* a0 = efi flag, a1 = cmdline, a2 = systemtab */
		cmdline = early_ioremap(fw_arg1, COMMAND_LINE_SIZE);
		strscpy(arcs_cmdline, cmdline, COMMAND_LINE_SIZE);
		early_iounmap(cmdline, COMMAND_LINE_SIZE);
	} else {
		/* a0 = argc, a1 = argv, a3 = envp */
		fw_init_cmdline();
	}

	kernel_length = (long)(&_relocation_start) - (long)(&_text);
	bss_length = (long)&__bss_stop - (long)&__bss_start;

	loc_new = determine_relocation_address();

	/* Sanity check relocation address */
	if (relocation_addr_valid(loc_new))
		offset = (unsigned long)loc_new - (unsigned long)(&_text);

	/* Reset the command line now so we don't end up with a duplicate */
	arcs_cmdline[0] = '\0';

	if (offset) {
		/* Copy the kernel to it's new location */
		memcpy(loc_new, &_text, kernel_length);

		/* Perform relocations on the new kernel */
		res = do_relocations(loc_new, offset, _relocation_start,
			_relocation_end);
		if (res < 0)
			goto out;

		/* Sync the caches ready for execution of new kernel */
		asm volatile (
		"	ibar 0					\n"
		"	dbar 0					\n");

		/*
		 * The original .bss has already been cleared, and
		 * some variables such as command line parameters
		 * stored to it so make a copy in the new location.
		 */
		memcpy(RELOCATED(&__bss_start), &__bss_start, bss_length);

		/* The current thread is now within the relocated image */
		__current_thread_info = RELOCATED(__current_thread_info);

		/* Return the new kernel's entry point */
		kernel_entry = RELOCATED(start_kernel);
	}
out:
	return kernel_entry;
}

void *__init relocate_kdump_kernel(long offset)
{
#ifdef CONFIG_CRASH_DUMP
	void *loc_new;
	u32 *rstart, *rend;
	int res = 1;
	/* Default to original kernel entry point */
	void *kernel_entry = start_kernel;

	rstart = RELOCATED((unsigned long)_relocation_start - (unsigned long)&_text
			+ VMLINUX_LOAD_ADDRESS);
	rend = RELOCATED((unsigned long)_relocation_end - (unsigned long)&_text
			+ VMLINUX_LOAD_ADDRESS);

	loc_new = (void *)(offset + VMLINUX_LOAD_ADDRESS);

	/* Perform relocations on the new kernel */
	res = do_relocations(loc_new, offset, rstart, rend);
	if (res < 0)
		return NULL;

	/* Return the new kernel's entry point */
	kernel_entry = RELOCATED((unsigned long)start_kernel - (unsigned long)&_text
		+ VMLINUX_LOAD_ADDRESS);

	return kernel_entry;
#endif
	return NULL;
}

/*
 * Show relocation information on panic.
 */
void show_kernel_relocation(const char *level)
{
	unsigned long offset;

	offset = __pa_symbol(_text) - __pa_symbol(VMLINUX_LOAD_ADDRESS);

	if (IS_ENABLED(CONFIG_RELOCATABLE) && offset > 0) {
		printk(level);
		pr_cont("Kernel relocated by 0x%pK\n", (void *)offset);
		pr_cont(" .text @ 0x%pK\n", _text);
		pr_cont(" .data @ 0x%pK\n", _sdata);
		pr_cont(" .bss  @ 0x%pK\n", __bss_start);
	}
}

static int kernel_location_notifier_fn(struct notifier_block *self,
				       unsigned long v, void *p)
{
	show_kernel_relocation(KERN_EMERG);
	return NOTIFY_DONE;
}

static struct notifier_block kernel_location_notifier = {
	.notifier_call = kernel_location_notifier_fn
};

static int __init register_kernel_offset_dumper(void)
{
	atomic_notifier_chain_register(&panic_notifier_list,
				       &kernel_location_notifier);
	return 0;
}
__initcall(register_kernel_offset_dumper);
