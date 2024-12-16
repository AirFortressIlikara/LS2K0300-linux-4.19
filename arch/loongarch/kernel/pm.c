// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014 Imagination Technologies Ltd.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * CPU PM notifiers for saving/restoring general CPU state.
 */

#include <linux/cpu_pm.h>
#include <linux/init.h>

#include <asm/fpu.h>
#include <asm/mmu_context.h>
#include <asm/pm.h>

/* Used by PM helper macros in asm/pm.h */
struct loongarch_static_suspend_state loongarch_static_suspend_state;

/**
 * loongarch_cpu_save() - Save general CPU state.
 * Ensures that general CPU context is saved, notably FPU and DSP.
 */
static int loongarch_cpu_save(void)
{
	/* Save FPU state */
	lose_fpu(1);

	return 0;
}

/**
 * loongarch_cpu_restore() - Restore general CPU state.
 * Restores important CPU context.
 */
static void loongarch_cpu_restore(void)
{
	unsigned int cpu = smp_processor_id();

	/* Restore ASID */
	if (current->mm)
		write_csr_asid(cpu_asid(cpu, current->mm));
}

/**
 * loongarch_pm_notifier() - Notifier for preserving general CPU context.
 * @self:	Notifier block.
 * @cmd:	CPU PM event.
 * @v:		Private data (unused).
 *
 * This is called when a CPU power management event occurs, and is used to
 * ensure that important CPU context is preserved across a CPU power down.
 */
static int loongarch_pm_notifier(struct notifier_block *self, unsigned long cmd,
			    void *v)
{
	int ret;

	switch (cmd) {
	case CPU_PM_ENTER:
		ret = loongarch_cpu_save();
		if (ret)
			return NOTIFY_STOP;
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		loongarch_cpu_restore();
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block loongarch_pm_notifier_block = {
	.notifier_call = loongarch_pm_notifier,
};

static int __init loongarch_pm_init(void)
{
	return cpu_pm_register_notifier(&loongarch_pm_notifier_block);
}
arch_initcall(loongarch_pm_init);
