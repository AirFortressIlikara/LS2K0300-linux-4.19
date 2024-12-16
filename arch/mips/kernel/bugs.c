#include <asm/alternative.h>
#include <asm/bugs.h>

void __init check_bugs(void)
{
	unsigned int cpu = smp_processor_id();

	cpu_data[cpu].udelay_val = loops_per_jiffy;
	check_bugs32();
#ifdef CONFIG_64BIT
	check_bugs64();
#endif

#ifdef CONFIG_CPU_LOONGSON3
	alternative_instructions();
#endif
}
