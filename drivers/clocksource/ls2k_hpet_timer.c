// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/percpu.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <asm/time.h>
#include <loongson-2k.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

void __iomem *hpet_mmio_base;
unsigned int hpet_freq;
unsigned int hpet_t0_irq;
unsigned int hpet_irq_flags;
unsigned int hpet_t0_cfg;

static DEFINE_SPINLOCK(hpet_lock);
DEFINE_PER_CPU(struct clock_event_device, hpet_clockevent_device);

#define HPET_MIN_CYCLES		16
#define HPET_MIN_PROG_DELTA	(HPET_MIN_CYCLES * 12)
#define HPET_COMPARE_VAL	((hpet_freq + HZ / 2) / HZ)

bool loongson_hpet_present(void)
{
	return IS_ENABLED(CONFIG_LS2K_HPET_TIMER) && hpet_mmio_base;
}

static int hpet_read(int offset)
{
	return *(volatile unsigned int *)(hpet_mmio_base + offset);
}

static void hpet_write(int offset, int data)
{
	*(volatile unsigned int *)(hpet_mmio_base + offset) = data;
}

static void hpet_start_counter(void)
{
	unsigned int cfg = hpet_read(HPET_CFG);

	cfg |= HPET_CFG_ENABLE;
	hpet_write(HPET_CFG, cfg);
}

static void hpet_stop_counter(void)
{
	unsigned int cfg = hpet_read(HPET_CFG);

	cfg &= ~HPET_CFG_ENABLE;
	hpet_write(HPET_CFG, cfg);
}

static void hpet_reset_counter(void)
{
	hpet_write(HPET_COUNTER, 0);
	hpet_write(HPET_COUNTER + 4, 0);
}

static void hpet_restart_counter(void)
{
	hpet_stop_counter();
	hpet_reset_counter();
	hpet_start_counter();
}

static void hpet_enable_legacy_int(void)
{
	/* Do nothing on Loongson-2k */
}

static int hpet_set_state_periodic(struct clock_event_device *evt)
{
	int cfg;

	spin_lock(&hpet_lock);

	pr_info("set clock event to periodic mode!\n");
	/* stop counter */
	hpet_stop_counter();
	hpet_reset_counter();
	hpet_write(HPET_T0_CMP, 0);

	/* enables the timer0 to generate a periodic interrupt */
	cfg = hpet_read(HPET_T0_CFG);
	cfg &= ~HPET_TN_LEVEL;
	cfg |= HPET_TN_ENABLE | HPET_TN_PERIODIC | HPET_TN_SETVAL |
		HPET_TN_32BIT | hpet_irq_flags;
	hpet_write(HPET_T0_CFG, cfg);

	/* set the comparator */
	hpet_write(HPET_T0_CMP, HPET_COMPARE_VAL);
	udelay(1);
	hpet_write(HPET_T0_CMP, HPET_COMPARE_VAL);

	/* start counter */
	hpet_start_counter();

	spin_unlock(&hpet_lock);
	return 0;
}

static int hpet_set_state_shutdown(struct clock_event_device *evt)
{
	int cfg;

	spin_lock(&hpet_lock);

	cfg = hpet_read(HPET_T0_CFG);
	cfg &= ~HPET_TN_ENABLE;
	hpet_write(HPET_T0_CFG, cfg);

	spin_unlock(&hpet_lock);
	return 0;
}

static int hpet_set_state_oneshot(struct clock_event_device *evt)
{
	int cfg;

	spin_lock(&hpet_lock);

	pr_info("set clock event to one shot mode!\n");
	cfg = hpet_read(HPET_T0_CFG);
	/*
	 * set timer0 type
	 * 1 : periodic interrupt
	 * 0 : non-periodic(oneshot) interrupt
	 */
	cfg &= ~HPET_TN_PERIODIC;
	cfg |= HPET_TN_ENABLE | HPET_TN_32BIT |
		hpet_irq_flags;
	hpet_write(HPET_T0_CFG, cfg);

	/* start counter */
	hpet_start_counter();

	spin_unlock(&hpet_lock);
	return 0;
}

static int hpet_tick_resume(struct clock_event_device *evt)
{
	spin_lock(&hpet_lock);
	hpet_enable_legacy_int();
	spin_unlock(&hpet_lock);

	return 0;
}

static int hpet_next_event(unsigned long delta,
		struct clock_event_device *evt)
{
	u32 cnt;
	s32 res;

	cnt = hpet_read(HPET_COUNTER);
	cnt += (u32) delta;
	hpet_write(HPET_T0_CMP, cnt);

	res = (s32)(cnt - hpet_read(HPET_COUNTER));

	return res < HPET_MIN_CYCLES ? -ETIME : 0;
}

static irqreturn_t hpet_irq_handler(int irq, void *data)
{
	int is_irq;
	struct clock_event_device *cd;
	unsigned int cpu = smp_processor_id();

	is_irq = hpet_read(HPET_STATUS);
	if (is_irq & HPET_T0_IRS) {
		/* clear the TIMER0 irq status register */
		hpet_write(HPET_STATUS, HPET_T0_IRS);
		cd = &per_cpu(hpet_clockevent_device, cpu);
		cd->event_handler(cd);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static struct irqaction hpet_irq_str = {
	.handler = hpet_irq_handler,
	.flags = IRQD_NO_BALANCING | IRQF_TIMER,
	.name = "hpet",
};

/*
 * HPET address assignation and irq setting should be done in bios.
 * But, sometimes bios don't do this, we just setup here directly.
 */
static void hpet_setup(void)
{
	hpet_enable_legacy_int();
}

static int hpet_setup_irq(struct clock_event_device *cd)
{
	setup_irq(cd->irq, &hpet_irq_str);

	disable_irq(cd->irq);
	irq_set_affinity(cd->irq, cd->cpumask);
	enable_irq(cd->irq);

	return 0;
}

static int __init ls2k_hpet_clockevent_init(void)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *cd;

	hpet_setup();

	cd = &per_cpu(hpet_clockevent_device, cpu);
	cd->name = "hpet";
	cd->rating = 300;
	cd->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	cd->set_state_shutdown = hpet_set_state_shutdown;
	cd->set_state_periodic = hpet_set_state_periodic;
	cd->set_state_oneshot = hpet_set_state_oneshot;
	cd->tick_resume = hpet_tick_resume;
	cd->set_next_event = hpet_next_event;
	cd->irq = hpet_t0_irq;
	cd->cpumask = cpumask_of(cpu);
	clockevent_set_clock(cd, hpet_freq);
	cd->max_delta_ns = clockevent_delta2ns(0x7fffffff, cd);
	cd->max_delta_ticks = 0x7fffffff;
	cd->min_delta_ns = clockevent_delta2ns(HPET_MIN_PROG_DELTA, cd);
	cd->min_delta_ticks = HPET_MIN_PROG_DELTA;

	clockevents_register_device(cd);
	hpet_setup_irq(cd);

	pr_info("hpet clock event device register\n");

	return 0;
}

static u64 hpet_read_counter(struct clocksource *cs)
{
	return (u64)hpet_read(HPET_COUNTER);
}

static void hpet_suspend(struct clocksource *cs)
{
	hpet_t0_cfg = hpet_read(HPET_T0_CFG);
}

static void hpet_resume(struct clocksource *cs)
{
	hpet_write(HPET_T0_CFG, hpet_t0_cfg);
	hpet_setup();
	hpet_restart_counter();
}

struct clocksource csrc_hpet = {
	.name = "hpet",
	.rating = 300,
	.read = hpet_read_counter,
	.mask = CLOCKSOURCE_MASK(32),
	/* oneshot mode work normal with this flag */
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = hpet_suspend,
	.resume = hpet_resume,
	.mult = 0,
	.shift = 10,
};

static int __init ls2k_hpet_clocksource_init(void)
{
	csrc_hpet.mult = clocksource_hz2mult(hpet_freq, csrc_hpet.shift);

	/* start counter */
	hpet_start_counter();

	return clocksource_register_hz(&csrc_hpet, hpet_freq);
}

static int __init ls2k_hpet_timer_init(struct device_node *np)
{
	int retval;
	u32 clk;

	hpet_mmio_base = of_iomap(np, 0);
	if (!hpet_mmio_base) {
		pr_err("hpet: unable to map ls2k hpet timer registers\n");
		return -ENODEV;
	}

	hpet_t0_irq = irq_of_parse_and_map(np, 0);
	if (hpet_t0_irq < 0) {
		retval = hpet_t0_irq;
		return -1;
	}

	if (!of_property_read_u32(np, "clock-frequency", &clk))
		hpet_freq = clk;
	else
		return -1;

	hpet_irq_flags = HPET_TN_LEVEL;

	ls2k_hpet_clocksource_init();

	ls2k_hpet_clockevent_init();

	return 0;
}

TIMER_OF_DECLARE(ls2k_hpet_timer, "loongson,loongson2-hpet", ls2k_hpet_timer_init);
