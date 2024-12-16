/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_IDLE_H
#define __ASM_IDLE_H

#include <linux/cpuidle.h>
#include <linux/linkage.h>

extern void cpu_wait(void);
extern asmlinkage void __cpu_wait(void);

extern int cpuidle_wait_enter(struct cpuidle_device *dev,
				   struct cpuidle_driver *drv, int index);

#define CPUIDLE_WAIT_STATE {\
	.enter			= cpuidle_wait_enter,\
	.exit_latency		= 1,\
	.target_residency	= 1,\
	.power_usage		= UINT_MAX,\
	.name			= "wait",\
	.desc			= "LoongArch wait",\
}

#endif /* __ASM_IDLE_H  */
