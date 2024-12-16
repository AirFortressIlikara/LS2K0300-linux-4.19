/* SPDX-License-Identifier: GPL-2.0 */
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

#include "vdso.h"

#include <linux/linkage.h>
#include <uapi/asm/unistd.h>

#include <asm/regdef.h>
#include <asm/asm.h>

	.section	.text
	.cfi_sections	.debug_frame

SYM_FUNC_START(__vdso_rt_sigreturn)

	li.w	a7, __NR_rt_sigreturn
	syscall	0

SYM_FUNC_END(__vdso_rt_sigreturn)
