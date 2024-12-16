#include <linux/init.h>
#include <linux/clockchips.h>
#include <linux/sched_clock.h>
#include <asm/time.h>
#include <loongson.h>
#include <asm/mipsregs.h>
#include <asm/cevt-r4k.h>
#include <loongson.h>

#define stabel_irq    7

unsigned int ls_stable_freq;
int stable_timer_irq_installed;
int stable_timer_enabled = 0;

static DEFINE_SPINLOCK(stable_lock);
DEFINE_PER_CPU(struct clock_event_device, stable_clockevent_device);

irqreturn_t stable_irq_handler(int irq, void *data)
{
	const int r2 = cpu_has_mips_r2;
	struct clock_event_device *cd;
	int cpu = smp_processor_id();

	if ((cp0_perfcount_irq < 0) && perf_irq() == IRQ_HANDLED && !r2)
		goto out;

	/*
	 * The same applies to performance counter interrupts.	But with the
	 * above we now know that the reason we got here must be a timer
	 * interrupt.  Being the paranoiacs we are we check anyway.
	 */
	if (!r2 || (read_c0_cause() & (1 << 30))) {
		/* Clear Count/Compare Interrupt */
		write_c0_compare(read_c0_compare());
		cd = &per_cpu(stable_clockevent_device, cpu);
		cd->event_handler(cd);
	}

out:
	return IRQ_HANDLED;
}

static struct irqaction stable_irq_irqaction = {
	.handler = stable_irq_handler,
	.flags = IRQF_PERCPU | IRQF_SHARED | IRQF_TIMER,
	.name = "stable timer",
};

void stable_event_handler(struct clock_event_device *dev)
{
}

static int stable_set_state_periodic(struct clock_event_device *evt)
{
	unsigned long  cfg;
	unsigned int period_init;
	unsigned int raw_cpuid;
	unsigned long addr;

	spin_lock(&stable_lock);

	raw_cpuid = cpu_logical_map(smp_processor_id());
	addr = CPU_TO_CONF(raw_cpuid) + LOONGSON_STABLE_TIMER_CFG;

	if (loongson_cpu_has_csr)
		cfg = csr_readq(LOONGSON_STABLE_TIMER_CFG);
	else
		cfg = ls64_conf_read64((void *)addr);

	cfg |= (LOONGSON_STABLE_TIMER_CFG_PERIODIC | LOONGSON_STABLE_TIMER_CFG_EN);
	cfg &= LOONGSON_STABLE_TIMER_INITVAL_RST;

	if (loongson_cpu_has_cpucfg)
		period_init = calc_const_freq();
	else
		period_init = cpu_clock_freq;

	period_init = period_init / HZ;

	if (loongson_cpu_has_csr)
		csr_writeq(cfg | period_init, LOONGSON_STABLE_TIMER_CFG);
	else
		ls64_conf_write64(cfg | period_init, (void *)addr);

	spin_unlock(&stable_lock);

	return 0;
}

static int stable_set_state_oneshot(struct clock_event_device *evt)
{
	unsigned long  cfg;
	unsigned int raw_cpuid;
	unsigned long addr;

	spin_lock(&stable_lock);

	raw_cpuid = cpu_logical_map(smp_processor_id());
	addr = CPU_TO_CONF(raw_cpuid) + LOONGSON_STABLE_TIMER_CFG;

	if (loongson_cpu_has_csr)
		cfg = csr_readq(LOONGSON_STABLE_TIMER_CFG);
	else
		cfg = ls64_conf_read64((void *)addr);

	/* set timer0 type
	 * 1 : periodic interrupt
	 * 0 : non-periodic(oneshot) interrupt
	 */

	cfg &= ~LOONGSON_STABLE_TIMER_CFG_PERIODIC;
	cfg |= LOONGSON_STABLE_TIMER_CFG_EN;

	if (loongson_cpu_has_csr)
		csr_writeq(cfg, LOONGSON_STABLE_TIMER_CFG);
	else
		ls64_conf_write64(cfg, (void *)addr);

	spin_unlock(&stable_lock);

	return 0;
}

static int stable_set_state_oneshot_stopped(struct clock_event_device *evt)
{
	return 0;
}

static int stable_set_state_shutdown(struct clock_event_device *evt)
{
	return 0;
}

static int loongson3_next_event(unsigned long delta,
				struct clock_event_device *evt)
{
	unsigned long addr;
	unsigned long cnt;
	unsigned int raw_cpuid;

	raw_cpuid = cpu_logical_map(smp_processor_id());
	addr = CPU_TO_CONF(raw_cpuid) + LOONGSON_STABLE_TIMER_CFG;
	cnt = (delta | LOONGSON_STABLE_TIMER_CFG_RESERVED | LOONGSON_STABLE_TIMER_CFG_EN);
	ls64_conf_write64(cnt, (void *)addr);
	return 0;
}

static int sbl_cnt_next_event(unsigned long delta,
                struct clock_event_device *evt)
{
	csr_writeq(delta | LOONGSON_STABLE_TIMER_CFG_RESERVED | LOONGSON_STABLE_TIMER_CFG_EN, LOONGSON_STABLE_TIMER_CFG);
	return 0;
}

static void stable_event_enable(void)
{
	unsigned long long GSconfigFlag = 0;

	GSconfigFlag = read_c0_config6();
	GSconfigFlag &= ~MIPS_CONF6_INNTIMER;
	GSconfigFlag |= MIPS_CONF6_EXTIMER;
	write_c0_config6(GSconfigFlag);
}

int stable_clockevent_init(void)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *cd;
	unsigned int irq;
	unsigned long min_delta = 0x600;
	unsigned long max_delta = (1UL << 48) - 1;
	unsigned int const_freq;

	stable_event_enable();

	if (loongson_cpu_has_cpucfg)
		const_freq = calc_const_freq();
	else
		const_freq = cpu_clock_freq;

	irq = MIPS_CPU_IRQ_BASE + stabel_irq;

	cd = &per_cpu(stable_clockevent_device, cpu);

	cd->name = "stable timer";

	if(cpu_guestmode)
		cd->features = CLOCK_EVT_FEAT_ONESHOT;
	else
		cd->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;

	cd->rating = 380;
	cd->irq = irq;
	cd->cpumask = cpumask_of(cpu);
	cd->set_state_periodic = stable_set_state_periodic;
	cd->set_state_oneshot = stable_set_state_oneshot;
	cd->set_state_oneshot_stopped = stable_set_state_oneshot_stopped;
	cd->set_state_shutdown = stable_set_state_shutdown;
	if (loongson_cpu_has_csr)
		cd->set_next_event = sbl_cnt_next_event;
	else
		cd->set_next_event = loongson3_next_event;

	cd->event_handler   = stable_event_handler;

	clockevents_config_and_register(cd, const_freq, min_delta, max_delta);

	if (stable_timer_irq_installed)
		return 0;

	stable_timer_irq_installed = 1;
	setup_irq(irq, &stable_irq_irqaction);
	stable_timer_enabled = 1;

	pr_info("stable clock event device register\n");

	return 0;
}

static u64 read_const_cntr64(struct clocksource *clk)
{
	u64 count;

	__asm__ __volatile__(
		" .set push\n"
		" .set mips32r2\n"
		" rdhwr   %0, $30\n"
		" .set pop\n"
		: "=r" (count));

	return count;
}

static struct clocksource csrc_stable_counter = {
	.name = "stable counter",
	/* mips clocksource rating is less than 300, so stable_counter is better. */
	.rating = 380,
	.read = read_const_cntr64,
	.mask = CLOCKSOURCE_MASK(64),
	/* oneshot mode work normal with this flag */
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.mult = 0,
	.shift = 10,
	.archdata	= { .vdso_clock_mode = VDSO_CLOCK_STABLE },
};

extern void update_clocksource_for_loongson(struct clocksource *cs);
extern unsigned long loops_per_jiffy;

/*used for adjust clocksource and clockevent for guest
 *when migrate to a diffrent cpu freq
 */
void loongson_stablecounter_adjust(void)
{
	unsigned int cpu;
	struct clock_event_device *cd;
	struct clocksource *cs = &csrc_stable_counter;
	u32 new_stable_freq;

	ls_stable_freq = calc_const_freq();
	new_stable_freq = LOONGSON_FREQCTRL(0);

	printk("src_freq 0x%x,new_freq 0x%x\n", ls_stable_freq, new_stable_freq);

	for_each_online_cpu(cpu){
		cd = &per_cpu(stable_clockevent_device, cpu);
		clockevents_update_freq(cd, new_stable_freq);
		cpu_data[cpu].udelay_val = cpufreq_scale(loops_per_jiffy,
				 ls_stable_freq / 1000, new_stable_freq / 1000);
	}

	loops_per_jiffy = cpu_data[0].udelay_val;
	ls_stable_freq = new_stable_freq;
	mips_hpt_frequency = new_stable_freq / 2;
	__clocksource_update_freq_scale(cs, 1, new_stable_freq);
	update_clocksource_for_loongson(cs);
}

u64 notrace native_sched_clock(void)
{
	/* 64-bit arithmatic can overflow, so use 128-bit. */
	return read_const_cntr64(NULL);
}

int __init init_stable_clocksource(void)
{
	int res;
	unsigned long freq;

	if (loongson_cpu_has_cpucfg) {
		freq = calc_const_freq();
	} else
		freq = cpu_clock_freq;

	csrc_stable_counter.mult =
		clocksource_hz2mult(freq, csrc_stable_counter.shift);

	res = clocksource_register_hz(&csrc_stable_counter, freq);

	sched_clock_register(native_sched_clock, 64, freq);

	pr_info("stable counter clock source device register\n");

	return res;
}
