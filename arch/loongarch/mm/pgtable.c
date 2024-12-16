// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/quicklist.h>
#include <linux/string.h>
#include <asm/pgalloc.h>

pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *ret;

	ret = pgd_alloc_one(mm);
	return ret;
}
EXPORT_SYMBOL_GPL(pgd_alloc);
