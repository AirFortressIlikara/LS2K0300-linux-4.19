// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * Common time service routines for LoongArch machines.
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/clockchips.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched_clock.h>
#include <linux/spinlock.h>

#include <asm/cpu-features.h>
#include <asm/loongarchregs.h>
#include <asm/time.h>
#include <asm/paravirt.h>

DEFINE_SPINLOCK(rtc_lock);
EXPORT_SYMBOL(rtc_lock);

u64 cpu_clock_freq;
EXPORT_SYMBOL(cpu_clock_freq);
u64 const_clock_freq;
EXPORT_SYMBOL(const_clock_freq);

static DEFINE_SPINLOCK(state_lock);
static DEFINE_PER_CPU(struct clock_event_device, constant_clockevent_device);

static void constant_event_handler(struct clock_event_device *dev)
{
}

irqreturn_t constant_timer_interrupt(int irq, void *data)
{
	int cpu = smp_processor_id();
	struct clock_event_device *cd;

	/* Clear Timer Interrupt */
	write_csr_tintclear(CSR_TINTCLR_TI);
	cd = &per_cpu(constant_clockevent_device, cpu);
	cd->event_handler(cd);

	return IRQ_HANDLED;
}

static int constant_set_state_oneshot(struct clock_event_device *evt)
{
	unsigned long timer_config;

	spin_lock(&state_lock);

	timer_config = csr_read64(LOONGARCH_CSR_TCFG);
	timer_config |=  CSR_TCFG_EN;
	timer_config &= ~CSR_TCFG_PERIOD;
	csr_write64(timer_config, LOONGARCH_CSR_TCFG);

	spin_unlock(&state_lock);

	return 0;
}

static int constant_set_state_oneshot_stopped(struct clock_event_device *evt)
{
	unsigned long timer_config;

	spin_lock(&state_lock);

	timer_config = csr_read64(LOONGARCH_CSR_TCFG);
	timer_config &=  ~CSR_TCFG_EN;
	csr_write64(timer_config, LOONGARCH_CSR_TCFG);

	spin_unlock(&state_lock);

	return 0;
}

static int constant_set_state_periodic(struct clock_event_device *evt)
{
	unsigned long period;
	unsigned long timer_config;

	spin_lock(&state_lock);

	period = const_clock_freq / HZ;
	timer_config = period & CSR_TCFG_VAL;
	timer_config |= (CSR_TCFG_PERIOD | CSR_TCFG_EN);
	csr_write64(timer_config, LOONGARCH_CSR_TCFG);

	spin_unlock(&state_lock);

	return 0;
}

static int constant_set_state_shutdown(struct clock_event_device *evt)
{
	return 0;
}

static int constant_timer_next_event(unsigned long delta, struct clock_event_device *evt)
{
	unsigned long timer_config;

	delta &= CSR_TCFG_VAL;
	timer_config = delta | CSR_TCFG_EN;
	csr_write64(timer_config, LOONGARCH_CSR_TCFG);

	return 0;
}

static unsigned long __init get_loops_per_jiffy(void)
{
	unsigned long lpj = (unsigned long)const_clock_freq;

	do_div(lpj, HZ);
	return lpj;
}

/* This shouldn't resume during hibernate */
static long init_offset __nosavedata;

void save_counter(void)
{
	init_offset = drdtime();
}

void sync_counter(void)
{
	/* Ensure counter begin at 0 */
	csr_write64(init_offset, LOONGARCH_CSR_CNTC);
}

int constant_clockevent_init(void)
{
	unsigned int irq;
	unsigned int cpu = smp_processor_id();
	unsigned long min_delta = 0x600;
	unsigned long max_delta = (1UL << 48) - 1;
	struct clock_event_device *cd;
	static int timer_irq_installed = 0;

	irq = LOONGSON_TIMER_IRQ;

	cd = &per_cpu(constant_clockevent_device, cpu);

	cd->name = "Constant";
	cd->features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_PERCPU;

	cd->irq = irq;
	cd->rating = 320;
	cd->cpumask = cpumask_of(cpu);
	cd->set_state_oneshot = constant_set_state_oneshot;
	cd->set_state_oneshot_stopped = constant_set_state_oneshot_stopped;
	cd->set_state_periodic = constant_set_state_periodic;
	cd->set_state_shutdown = constant_set_state_shutdown;
	cd->set_next_event = constant_timer_next_event;
	cd->event_handler = constant_event_handler;

	clockevents_config_and_register(cd, const_clock_freq, min_delta, max_delta);

	if (timer_irq_installed)
		return 0;

	timer_irq_installed = 1;

	sync_counter();

	if (request_irq(irq, constant_timer_interrupt, IRQF_PERCPU | IRQF_TIMER, "timer", NULL))
		pr_err("Failed to request irq %d (timer)\n", irq);

	lpj_fine = get_loops_per_jiffy();
	pr_info("Constant clock event device register\n");

	return 0;
}

static u64 read_const_counter(struct clocksource *clk)
{
	return drdtime();
}

u64 native_sched_clock(void)
{
	return read_const_counter(NULL);
}

static struct clocksource clocksource_const = {
	.name = "Constant",
	.rating = 400,
	.read = read_const_counter,
	.mask = CLOCKSOURCE_MASK(64),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.mult = 0,
	.shift = 10,
	.archdata	= { .vdso_clock_mode = VDSO_CLOCK_STABLE },
};

int __init constant_clocksource_init(void)
{
	int res;
	unsigned long freq;

	freq = const_clock_freq;

	clocksource_const.mult =
		clocksource_hz2mult(freq, clocksource_const.shift);

	res = clocksource_register_hz(&clocksource_const, freq);

	sched_clock_register(native_sched_clock, 64, freq);

	pr_info("Constant clock source device register\n");

	return res;
}

void __init time_init(void)
{
#ifdef CONFIG_TIMER_PROBE
	timer_probe();
#endif
	if (!cpu_has_cpucfg)
		const_clock_freq = cpu_clock_freq;
	else
		const_clock_freq = calc_const_freq();

	init_offset = -(drdtime() - csr_read64(LOONGARCH_CSR_CNTC));

	constant_clockevent_init();
	constant_clocksource_init();

	pv_time_init();
}

#ifdef CONFIG_SMP
/*
 * If we have a stable timer are using it for the delay loop, we can
 * skip clock calibration if another cpu in the same socket has already
 * been calibrated. This assumes that stable timer applies to all
 * cpus in the socket - this should be a safe assumption.
 */
unsigned long calibrate_delay_is_known(void)
{
	int next, cpu = smp_processor_id();
	const struct cpumask *mask = topology_core_cpumask(cpu);

	if (!mask)
		return 0;

	next = cpumask_any_but(mask, cpu);
	if (next < nr_cpu_ids)
		return cpu_data[next].udelay_val;
	return 0;
}
#endif
