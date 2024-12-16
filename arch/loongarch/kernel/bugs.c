#include <asm/alternative.h>
#include <asm/bugs.h>

void __init check_bugs(void)
{
	unsigned int cpu = smp_processor_id();

	cpu_data[cpu].udelay_val = loops_per_jiffy;

	alternative_instructions();
}
