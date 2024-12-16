/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/quicklist.h>
#include <asm/pgalloc.h>

pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *ret, *init;

#ifdef CONFIG_QUICKLIST
	ret = quicklist_alloc(QUICKLIST_PGD, GFP_KERNEL, pgd_init);
#else
	ret = (pgd_t *) __get_free_pages(GFP_KERNEL, PGD_ORDER);
#endif
	if (ret) {
#ifndef CONFIG_QUICKLIST
		pgd_init((void *)ret);
#endif
		init = pgd_offset(&init_mm, 0UL);
		memcpy(ret + USER_PTRS_PER_PGD, init + USER_PTRS_PER_PGD,
		       (PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));
	}

	return ret;
}
EXPORT_SYMBOL_GPL(pgd_alloc);
