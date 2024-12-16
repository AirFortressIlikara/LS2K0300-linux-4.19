/*
 * MIPS cacheinfo support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/cacheinfo.h>
#include <linux/of.h>

/* Populates leaf and increments to next leaf */
#define populate_cache(cache, leaf, c_level, c_type)		\
do {								\
	leaf->type = c_type;					\
	leaf->level = c_level;					\
	leaf->coherency_line_size = c->cache.linesz;		\
	leaf->number_of_sets = c->cache.sets;			\
	leaf->ways_of_associativity = c->cache.ways;		\
	leaf->size = c->cache.linesz * c->cache.sets *		\
		c->cache.ways;					\
	leaf++;							\
} while (0)

static int __init_cache_level(unsigned int cpu)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	struct cpu_cacheinfo *this_cpu_ci = get_cpu_cacheinfo(cpu);
	int levels = 0, leaves = 0;

	/*
	 * If Dcache is not set, we assume the cache structures
	 * are not properly initialized.
	 */
	if (c->dcache.waysize)
		levels += 1;
	else
		return -ENOENT;


	leaves += (c->icache.waysize) ? 2 : 1;

#if defined(CONFIG_CPU_LOONGSON3)
	if (c->vcache.waysize) {
		levels++;
		leaves++;
	}
#endif

	if (c->scache.waysize) {
		levels++;
		leaves++;
	}

	if (c->tcache.waysize) {
		levels++;
		leaves++;
	}

	this_cpu_ci->num_levels = levels;
	this_cpu_ci->num_leaves = leaves;
	return 0;
}

static inline bool cache_leaves_are_shared(struct cacheinfo *this_leaf,
					   struct cacheinfo *sib_leaf)
{
#if defined(CONFIG_CPU_LOONGSON3)
	return !((this_leaf->level == 1) || (this_leaf->level == 2));
#endif
	return !(this_leaf->level == 1);
}

#ifdef CONFIG_CPU_LOONGSON3
static void __cache_cpumap_setup(unsigned int cpu)
{
	struct cpu_cacheinfo *this_cpu_ci = get_cpu_cacheinfo(cpu);
	struct cacheinfo *this_leaf, *sib_leaf;
	unsigned int index;

	for (index = 0; index < this_cpu_ci->num_leaves; index++) {
		unsigned int i;

		this_leaf = this_cpu_ci->info_list + index;
		/* skip if shared_cpu_map is already populated */
		if (!cpumask_empty(&this_leaf->shared_cpu_map))
			continue;

		cpumask_set_cpu(cpu, &this_leaf->shared_cpu_map);
		for_each_online_cpu(i) {
			struct cpu_cacheinfo *sib_cpu_ci = get_cpu_cacheinfo(i);

			if (i == cpu || !sib_cpu_ci->info_list)
				continue;/* skip if itself or no cacheinfo */
			sib_leaf = sib_cpu_ci->info_list + index;
			if (cache_leaves_are_shared(this_leaf, sib_leaf)) {
				cpumask_set_cpu(cpu, &sib_leaf->shared_cpu_map);
				cpumask_set_cpu(i, &this_leaf->shared_cpu_map);
			}
		}
	}
}
#endif

static void fill_cpumask_siblings(int cpu, cpumask_t *cpu_map)
{
       int cpu1;

       for_each_possible_cpu(cpu1)
               if (cpus_are_siblings(cpu, cpu1))
                       cpumask_set_cpu(cpu1, cpu_map);
}

static void fill_cpumask_cluster(int cpu, cpumask_t *cpu_map)
{
       int cpu1;
       int cluster = cpu_cluster(&cpu_data[cpu]);

       for_each_possible_cpu(cpu1)
               if (cpu_cluster(&cpu_data[cpu1]) == cluster)
                       cpumask_set_cpu(cpu1, cpu_map);
}

static int __populate_cache_leaves(unsigned int cpu)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	struct cpu_cacheinfo *this_cpu_ci = get_cpu_cacheinfo(cpu);
	struct cacheinfo *this_leaf = this_cpu_ci->info_list;
	int level=1;

	if (c->icache.waysize) {
		/* L1 caches are per core */
		fill_cpumask_siblings(cpu, &this_leaf->shared_cpu_map);
		populate_cache(dcache, this_leaf, level, CACHE_TYPE_DATA);
		fill_cpumask_siblings(cpu, &this_leaf->shared_cpu_map);
		populate_cache(icache, this_leaf, level++, CACHE_TYPE_INST);
	} else {
		populate_cache(dcache, this_leaf, level++, CACHE_TYPE_UNIFIED);
	}

#if defined(CONFIG_CPU_LOONGSON3)
	if (c->vcache.waysize)
		populate_cache(vcache, this_leaf, level++, CACHE_TYPE_UNIFIED);
#endif
	if (c->scache.waysize) {
		/* L2 cache is per cluster */
		fill_cpumask_cluster(cpu, &this_leaf->shared_cpu_map);
		populate_cache(scache, this_leaf, level++, CACHE_TYPE_UNIFIED);
	}

	if (c->tcache.waysize)
		populate_cache(tcache, this_leaf, level++, CACHE_TYPE_UNIFIED);

#if defined(CONFIG_CPU_LOONGSON3)
	if (!of_have_populated_dt()){
		__cache_cpumap_setup(cpu);
		this_cpu_ci->cpu_map_populated = true;
	}
#else
	this_cpu_ci->cpu_map_populated = true;
#endif

	return 0;
}

DEFINE_SMP_CALL_CACHE_FUNCTION(init_cache_level)
DEFINE_SMP_CALL_CACHE_FUNCTION(populate_cache_leaves)
