// SPDX-License-Identifier: GPL-3.0
/*
 * Quicklist support.
 *
 * Quicklists are light weight lists of pages that have a defined state
 * on alloc and free. Pages must be in the quicklist specific defined state
 * (zero by default) when the page is freed. It seems that the initial idea
 * for such lists first came from Dave Miller and then various other people
 * improved on it.
 *
 * Copyright (C) 2007 SGI,
 * 	Christoph Lameter <cl@linux.com>
 * 		Generalized, added support for multiple lists and
 * 		constructors / destructors.
 */
#include <linux/kernel.h>

#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/quicklist.h>

DEFINE_PER_CPU(struct quicklist [CONFIG_NR_QUICK], quicklist);

#define FRACTION_OF_NODE_MEM_SHIFT	4

#if !defined(CONFIG_CPU_LOONGSON3) && !defined(CONFIG_CPU_LOONGSON2K) && \
	!defined(CONFIG_LOONGARCH)
static unsigned long max_pages(unsigned long min_pages)
{
	unsigned long node_free_pages, max;
	int node = numa_node_id();
	struct zone *zones = NODE_DATA(node)->node_zones;

	node_free_pages =
#ifdef CONFIG_ZONE_DMA
		zone_page_state(&zones[ZONE_DMA], NR_FREE_PAGES) +
#endif
#ifdef CONFIG_ZONE_DMA32
		zone_page_state(&zones[ZONE_DMA32], NR_FREE_PAGES) +
#endif
		zone_page_state(&zones[ZONE_NORMAL], NR_FREE_PAGES);

	max = node_free_pages >> FRACTION_OF_NODE_MEM_SHIFT;
	max = max >> 2;

	return max(max, min_pages);
}
#endif

static long min_pages_to_free(struct quicklist *q,
	unsigned long min_pages, long max_free)
{
	long pages_to_free;

#if defined(CONFIG_CPU_LOONGSON3) || defined(CONFIG_CPU_LOONGSON2K) || \
	defined(CONFIG_LOONGARCH)
	pages_to_free = q->nr_pages - q->threshold;
#else
	pages_to_free = q->nr_pages - max_pages(min_pages);
#endif

	return min(pages_to_free, max_free);
}

/*
 * Trim down the number of pages in the quicklist
 */
void quicklist_trim(int nr, void (*dtor)(void *),
	unsigned long min_pages, unsigned long max_free)
{
	long pages_to_free;
	struct quicklist *q;

	q = &get_cpu_var(quicklist)[nr];
#if defined(CONFIG_CPU_LOONGSON3) || defined(CONFIG_CPU_LOONGSON2K) || \
	defined(CONFIG_LOONGARCH)
	if (q->nr_pages < q->threshold) {
		put_cpu_var(quicklist);
		return;
	}

	if ((q->hit + q->miss) > 512) {
		if (q->hit > (q->miss << 3)) {
			/* hit rate larger than 88% */
			if (q->threshold >= (min_pages * 2))
				q->threshold -= min_pages;
		} else if (q->hit < (q->miss << 2)) {
			/* hit rate less than 80% */
			q->threshold = 4096;
		}
		q->hit = 0;
		q->miss = 0;
	}
#endif
	if (q->nr_pages > min_pages) {
		pages_to_free = min_pages_to_free(q, min_pages, max_free);

		while (pages_to_free > 0) {
			/*
			 * We pass a gfp_t of 0 to quicklist_alloc here
			 * because we will never call into the page allocator.
			 */
			void *p = quicklist_alloc(nr, 0, NULL);

			if (dtor)
				dtor(p);
#if !defined(CONFIG_CPU_LOONGSON3) && !defined(CONFIG_CPU_LOONGSON2K) && \
	!defined(CONFIG_LOONGARCH)
			free_page((unsigned long)p);
#else
			free_pages((unsigned long)p, q->order);
#endif
			pages_to_free--;
		}
	}
	put_cpu_var(quicklist);
}

unsigned long quicklist_total_size(void)
{
	unsigned long count = 0;
	int cpu;
	struct quicklist *ql, *q;

	for_each_online_cpu(cpu) {
		ql = per_cpu(quicklist, cpu);
		for (q = ql; q < ql + CONFIG_NR_QUICK; q++)
			count += q->nr_pages;
	}
	return count;
}

#if defined(CONFIG_CPU_LOONGSON3) || defined(CONFIG_CPU_LOONGSON2K) || \
	defined(CONFIG_LOONGARCH)
/*
 * The two key functions quicklist_alloc and quicklist_free are inline so
 * that they may be custom compiled for the platform.
 * Specifying a NULL ctor can remove constructor support. Specifying
 * a constant quicklist allows the determination of the exact address
 * in the per cpu area.
 *
 * The fast patch in quicklist_alloc touched only a per cpu cacheline and
 * the first cacheline of the page itself. There is minmal overhead involved.
 */
void *quicklist_alloc(int nr, gfp_t flags, void (*ctor)(void *))
{
	struct quicklist *q;
	void **p = NULL;
	unsigned int order;
	struct page *page;

	q = &get_cpu_var(quicklist)[nr];
	p = q->page;
	if (likely(p)) {
		q->page = p[0];
		p[0] = (void *)q->val;
		q->nr_pages--;
		q->hit++;
	}
	if (likely(p)) {
		put_cpu_var(quicklist);
		return p;
	}

	q->miss++;
	order = q->order;
	put_cpu_var(quicklist);

	page = alloc_pages(flags, order);
	if (!page)
		return NULL;
	p = (void *)page_address(page);
	if (ctor && p)
		ctor(p);
	return p;
}

void __quicklist_free(int nr, void (*dtor)(void *), void *p,
		struct page *page)
{
	struct quicklist *q;

	q = &get_cpu_var(quicklist)[nr];
	*(void **)p = q->page;
	q->page = p;
	q->nr_pages++;
	put_cpu_var(quicklist);
}
EXPORT_SYMBOL_GPL(__quicklist_free);

#endif
