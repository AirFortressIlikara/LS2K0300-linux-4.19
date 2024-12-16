/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 * Authors: Huacai Chen (chenhuacai@loongson.cn)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <asm/abidefs.h>

#if _LOONGARCH_SIM != _LOONGARCH_SIM_ABILP64 && defined(CONFIG_64BIT)

/* Building 32-bit VDSO for the 64-bit kernel. Fake a 32-bit Kconfig. */
#undef CONFIG_64BIT
#define CONFIG_32BIT 1
#ifndef __ASSEMBLY__
#include <asm-generic/atomic64.h>
#endif
#endif

#ifndef __ASSEMBLY__

#include <asm/asm.h>
#include <asm/page.h>
#include <asm/vdso.h>

static inline unsigned long get_vdso_base(void)
{
	unsigned long addr;

	__asm__("la.pcrel	%0, _start	\n" : "=r" (addr));

	return addr;
}

static inline const struct loongarch_vdso_data *get_vdso_data(void)
{
	unsigned long vvar_size;

	vvar_size = ALIGN(sizeof(struct loongarch_vdso_data), PAGE_SIZE);
	return (const struct loongarch_vdso_data *)(get_vdso_base() - vvar_size);
}

#endif /* __ASSEMBLY__ */
