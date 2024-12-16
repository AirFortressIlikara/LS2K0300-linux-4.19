/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_CLOCKSOURCE_H
#define __ASM_CLOCKSOURCE_H

#include <linux/types.h>

/* VDSO clocksources. */
#define VDSO_CLOCK_NONE		0	/* No suitable clocksource. */
#define VDSO_CLOCK_STABLE      1       /* Use the stable counter. */

/**
 * struct arch_clocksource_data - Architecture-specific clocksource information.
 * @vdso_clock_mode: Method the VDSO should use to access the clocksource.
 */
struct arch_clocksource_data {
	u8 vdso_clock_mode;
};

#endif /* __ASM_CLOCKSOURCE_H */
