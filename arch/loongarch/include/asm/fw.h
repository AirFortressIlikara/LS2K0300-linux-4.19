/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef __ASM_FW_H_
#define __ASM_FW_H_

#include <asm/bootinfo.h>

extern long *_fw_argv;

#define fw_argv(index)		((char *)TO_CAC((long)_fw_argv[(index)]))

extern void fw_init_cmdline(void);

#endif /* __ASM_FW_H_ */
