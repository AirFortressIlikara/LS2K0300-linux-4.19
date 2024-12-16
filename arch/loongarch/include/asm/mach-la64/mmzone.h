/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010 Loongson Inc. & Lemote Inc. &
 *                    Institute of Computing Technology
 * Author:  Xiang Gao, gaoxiang@ict.ac.cn
 *          Huacai Chen, chenhc@lemote.com
 *          Xiaofu Meng, Shuangshuang Zhang
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef _ASM_MACH_MMZONE_H
#define _ASM_MACH_MMZONE_H

#ifdef CONFIG_NEED_MULTIPLE_NODES
extern struct pglist_data *node_data[];
#define NODE_DATA(nid)         (node_data[(nid)])

extern void setup_zero_pages(void);
#endif
#endif /* _ASM_MACH_MMZONE_H */
