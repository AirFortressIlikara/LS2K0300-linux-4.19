/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */

#ifndef _LOONGARCH_SETUP_H
#define _LOONGARCH_SETUP_H

#include <linux/types.h>
#include <uapi/asm/setup.h>

#define VECSIZE 0x200

extern void set_handler(unsigned long offset, void *addr, unsigned long len);
extern void set_merr_handler(unsigned long offset, void *addr, unsigned long len);

typedef void (*vi_handler_t)(int irq);
extern void set_vi_handler(int n, vi_handler_t addr);

extern unsigned long eentry;
extern unsigned long tlbrentry;
extern void cpu_cache_init(void);
extern void per_cpu_trap_init(int cpu);

#ifdef CONFIG_BLK_DEV_INITRD
#define __early_init_dt_declare_initrd(__start, __end)			\
	do {								\
		initrd_start = (unsigned long)__va(__start);				\
		initrd_end = (unsigned long)__va(__end);					\
		memblock_reserve(__pa(initrd_start), initrd_end - initrd_start);	\
		initrd_below_start_ok = 1;		\
	} while (0)
#endif

#ifdef CONFIG_USE_OF

struct boot_param_header;

extern void __dt_setup_arch(void *bph);
extern void device_tree_init(void);

#else /* CONFIG_OF */
static inline void device_tree_init(void) { }
#endif /* CONFIG_OF */
extern void set_tlb_handler(void);

#endif /* __SETUP_H */
