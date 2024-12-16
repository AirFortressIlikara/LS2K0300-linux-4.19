// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/binfmts.h>
#include <linux/cred.h>
#include <linux/elf.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timekeeper_internal.h>

#include <asm/page.h>
#include <asm/vdso.h>

extern struct user_namespace init_user_ns;
extern struct pid_namespace init_pid_ns;

/* Kernel-provided data used by the VDSO. */
static union {
	struct loongarch_vdso_data data;
	u8 pages[PAGE_ALIGN(sizeof(struct loongarch_vdso_data))];
} vdso __page_aligned_data;

/*
 * Mapping for the VDSO data pages. The real pages are mapped manually, as
 * what we map and where within the area they are mapped is determined at
 * runtime.
 */
static struct page *no_pages[] = { NULL };
static struct vm_special_mapping vdso_vvar_mapping = {
	.name = "[vvar]",
	.pages = no_pages,
};

static void __init init_vdso_image(struct loongarch_vdso_image *image)
{
	unsigned long num_pages, i;
	unsigned long data_pfn;

	BUG_ON(!PAGE_ALIGNED(image->data));
	BUG_ON(!PAGE_ALIGNED(image->size));

	num_pages = image->size / PAGE_SIZE;

	data_pfn = __phys_to_pfn(__pa_symbol(image->data));
	for (i = 0; i < num_pages; i++)
		image->mapping.pages[i] = pfn_to_page(data_pfn + i);
}

static void __init init_vdso_data(void)
{
	unsigned cpu;

	for_each_possible_cpu(cpu)
		vdso.data.pcpu_data[cpu].node = cpu_to_node(cpu);
}

static int __init init_vdso(void)
{
	init_vdso_data();
	init_vdso_image(&vdso_image);
	return 0;
}
subsys_initcall(init_vdso);

void update_vsyscall(struct timekeeper *tk)
{
	vdso_data_write_begin(&vdso.data);

	vdso.data.xtime_sec = tk->xtime_sec;
	vdso.data.xtime_nsec = tk->tkr_mono.xtime_nsec;
	vdso.data.wall_to_mono_sec = tk->wall_to_monotonic.tv_sec;
	vdso.data.wall_to_mono_nsec = tk->wall_to_monotonic.tv_nsec;
	vdso.data.cs_shift = tk->tkr_mono.shift;

	vdso.data.clock_mode = tk->tkr_mono.clock->archdata.vdso_clock_mode;
	if (vdso.data.clock_mode != VDSO_CLOCK_NONE) {
		vdso.data.cs_mult = tk->tkr_mono.mult;
		vdso.data.cs_cycle_last = tk->tkr_mono.cycle_last;
		vdso.data.cs_mask = tk->tkr_mono.mask;
	}

	vdso_data_write_end(&vdso.data);
}

void update_vsyscall_tz(void)
{
	if (vdso.data.clock_mode != VDSO_CLOCK_NONE) {
		vdso.data.tz_minuteswest = sys_tz.tz_minuteswest;
		vdso.data.tz_dsttime = sys_tz.tz_dsttime;
	}
}

void update_vsyscall_cred(struct cred *cred)
{
	unsigned long flags;
	unsigned int cpu_id;

	local_irq_save(flags);
	cpu_id = (current_thread_info())->cpu;

	if (cred->user_ns != &init_user_ns) {
		vdso.data.pcpu_data[cpu_id].uid = VDSO_INVALID_UID;
	} else {
		vdso.data.pcpu_data[cpu_id].uid = cred->uid.val;
	}

	vdso_pcpu_data_update_seq(&vdso.data.pcpu_data[cpu_id]);
	local_irq_restore(flags);
}

void vdso_per_cpu_switch_thread(struct task_struct *prev,
	struct task_struct *next)
{
	unsigned int cpu_id;

	cpu_id = (current_thread_info())->cpu;

	if (next->cred->user_ns != &init_user_ns) {
		vdso.data.pcpu_data[cpu_id].uid = VDSO_INVALID_UID;
	} else {
		vdso.data.pcpu_data[cpu_id].uid = task_uid(next).val;
	}

	if (!pid_alive(next) ||
		next->thread_pid->numbers[next->thread_pid->level].ns !=
			&init_pid_ns) {
		vdso.data.pcpu_data[cpu_id].pid = VDSO_INVALID_PID;
	} else {
		vdso.data.pcpu_data[cpu_id].pid = next->tgid;
	}

	if (test_tsk_thread_flag(next, TIF_SINGLESTEP))
		vdso.data.pcpu_data[cpu_id].singlestep_mask = 1;
	else
		vdso.data.pcpu_data[cpu_id].singlestep_mask = 0;

	vdso_pcpu_data_update_seq(&vdso.data.pcpu_data[cpu_id]);
}

static unsigned long vdso_base(void)
{
	unsigned long base;

	base = STACK_TOP;

	if (current->flags & PF_RANDOMIZE) {
		base += get_random_int() & (VDSO_RANDOMIZE_SIZE - 1);
		base = PAGE_ALIGN(base);
	}

	return base;
}

int arch_setup_additional_pages(struct linux_binprm *bprm, int uses_interp)
{
	struct loongarch_vdso_image *image = current->thread.vdso;
	struct mm_struct *mm = current->mm;
	unsigned long vvar_size, size, base, data_addr, vdso_addr;
	struct vm_area_struct *vma;
	int ret;

	if (down_write_killable(&mm->mmap_sem))
		return -EINTR;

	/*
	 * Determine total area size. This includes the VDSO data itself, the
	 * data page, and the GIC user page if present. Always create a mapping
	 * for the GIC user area if the GIC is present regardless of whether it
	 * is the current clocksource, in case it comes into use later on. We
	 * only map a page even though the total area is 64K, as we only need
	 * the counter registers at the start.
	 */
	vvar_size = sizeof(vdso);
	size = vvar_size + image->size;

	base = get_unmapped_area(NULL, vdso_base(), size, 0, 0);
	if (IS_ERR_VALUE(base)) {
		ret = base;
		goto out;
	}

	data_addr = base;
	vdso_addr = data_addr + vvar_size;

	vma = _install_special_mapping(mm, base, vvar_size,
				       VM_READ | VM_MAYREAD,
				       &vdso_vvar_mapping);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto out;
	}

	/* Map data page. */
	ret = remap_pfn_range(vma, data_addr,
			      virt_to_phys(&vdso) >> PAGE_SHIFT,
			      vvar_size, PAGE_READONLY);
	if (ret)
		goto out;

	/* Map VDSO image. */
	vma = _install_special_mapping(mm, vdso_addr, image->size,
				       VM_READ | VM_EXEC |
				       VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC,
				       &image->mapping);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto out;
	}

	mm->context.vdso = (void *)vdso_addr;
	ret = 0;

out:
	up_write(&mm->mmap_sem);
	return ret;
}
