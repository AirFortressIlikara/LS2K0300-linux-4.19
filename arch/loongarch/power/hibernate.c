// SPDX-License-Identifier: GPL-2.0
#include <asm/tlbflush.h>

extern int restore_image(void);
extern void enable_pcie_wakeup(void);
extern int swsusp_arch_save(void);

int swsusp_arch_suspend(void)
{
	enable_pcie_wakeup();
	return swsusp_arch_save();
}

int swsusp_arch_resume(void)
{
	/* Avoid TLB mismatch during and after kernel resume */
	local_flush_tlb_all();
	return restore_image();
}
