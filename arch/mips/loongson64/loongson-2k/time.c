/* SPDX-License-Identifier: GPL-2.0 */
#include <asm/time.h>
#include <asm/hpet.h>
#include <loongson.h>
#include <loongson-2k.h>

u32 cpu_clock_freq;
u32 node_pll_l2_out;
EXPORT_SYMBOL_GPL(node_pll_l2_out);

unsigned long get_cpu_clock(void)
{
	unsigned long node_pll_hi;
	unsigned long node_pll_lo;
	unsigned long ret;
	int div_loopc;
	int div_ref;
	int div_out;
	int l2_out;

	node_pll_lo =  readq((void *)CKSEG1ADDR(CONF_BASE + PLL_SYS0_OFF));
	node_pll_hi =  readq((void *)CKSEG1ADDR(CONF_BASE + PLL_SYS1_OFF));

	div_ref = (node_pll_lo >> 26) & 0x3f;
	div_loopc = (node_pll_lo >> 32) & 0x3ff;
	div_out = (node_pll_lo >> 42) & 0x3f;
	l2_out = node_pll_hi & 0x3f;
	node_pll_l2_out = l2_out;

	ret = 100000000UL * div_loopc;
	l2_out = l2_out * div_out * div_ref;

	do_div(ret, l2_out);

	return ret;
}

void __init plat_time_init(void)
{
	cpu_clock_freq = get_cpu_clock();

	/* setup mips r4k timer */
	mips_hpt_frequency = cpu_clock_freq / 2;

#ifdef CONFIG_LOONGSON_HPET
	setup_hpet_timer();
#endif
}

void read_persistent_clock64(struct timespec64 *ts)
{
	unsigned int year, mon, day, hour, min, sec;
	unsigned int value;

	value = readl((void *)TO_UNCAC(LS2K_TOY_READ0_REG));
	sec = (value >> 4) & 0x3f;
	min = (value >> 10) & 0x3f;
	hour = (value >> 16) & 0x1f;
	day = (value >> 21) & 0x1f;
	mon = (value >> 26) & 0x3f;
	year = readl((void *)TO_UNCAC(LS2K_TOY_READ1_REG));
	year += 1900;
	ts->tv_sec = mktime(year, mon, day, hour, min, sec);
	ts->tv_nsec = (value&0xf) * NSEC_PER_SEC / 10 ;
}
