/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * KVM/MIPS: Instruction/Exception emulation
 *
 * Copyright (C) 2019  Loongson Inc.  All rights reserved.
 * Author: Xing Li, lixing@loongson.cn
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/ktime.h>
#include <linux/kvm_host.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/bootmem.h>
#include <linux/random.h>
#include <asm/page.h>
#include <asm/cacheflush.h>
#include <asm/cacheops.h>
#include <asm/cpu-info.h>
#include <asm/mmu_context.h>
#include <asm/tlbflush.h>
#include <asm/inst.h>
#include <loongson.h>

#undef CONFIG_MIPS_MT
#include <asm/r4kcache.h>
#define CONFIG_MIPS_MT

#include "interrupt.h"
#include "commpage.h"

#include "trace.h"
#include "ls7a_irq.h"
#include "ls3a_ipi.h"

/**
 * kvm_loongson_stable_timer_disabled() - Find whether the stable timer is disabled.
 * @vcpu:	Virtual CPU.
 *
 * Returns:	1 if the CP0_Count timer is disabled by either the guest
 *		CP0_Cause.DC bit or the stable_timer_ctl.DC bit.
 *		0 otherwise (in which case stable timer is running).
 */
int kvm_loongson_stable_timer_disabled(struct kvm_vcpu *vcpu)
{
	return	(vcpu->arch.stable_timer_ctl & KVM_REG_MIPS_COUNT_CTL_DC);
}

/**
 * kvm_mips_ktime_to_stable_timer() - Scale ktime_t to a 64-bit stable timer.
 *
 * Caches the dynamic nanosecond bias in vcpu->arch.stable_timer_dyn_bias.
 *
 * Assumes !kvm_loongson_stable_timer_disabled(@vcpu) (guest stable timer is running).
 */
static u64 kvm_mips_ktime_to_stable_timer(struct kvm_vcpu *vcpu, ktime_t now)
{
	s64 now_ns, periods;
	u64 delta;

	now_ns = ktime_to_ns(now);
	delta = now_ns + vcpu->arch.stable_timer_dyn_bias;

	if (delta >= vcpu->arch.stable_timer_period) {
		/* If delta is out of safe range the bias needs adjusting */
		periods = div64_s64(now_ns, vcpu->arch.stable_timer_period);
		vcpu->arch.stable_timer_dyn_bias = -periods * vcpu->arch.stable_timer_period;
		/* Recalculate delta with new bias */
		delta = now_ns + vcpu->arch.stable_timer_dyn_bias;
	}

	/*
	 * We've ensured that:
	 *   delta < stable_timer_period
	 *
	 * Therefore the intermediate delta*stable_timer_hz will never overflow since
	 * at the boundary condition:
	 *   delta = stable_timer_period
	 *   delta = NSEC_PER_SEC * 2^32 / stable_timer_hz
	 *   delta * stable_timer_hz = NSEC_PER_SEC * 2^32
	 */
	return div_u64(delta * vcpu->arch.stable_timer_hz, NSEC_PER_SEC);
}

/**
 * kvm_loongson_stable_timer_time() - Get effective current time.
 * @vcpu:	Virtual CPU.
 *
 * Get effective monotonic ktime. This is usually a straightforward ktime_get(),
 * except when the master disable bit is set in stable_timer_ctl, in which case it is
 * stable_timer_resume, i.e. the time that the count was disabled.
 *
 * Returns:	Effective monotonic ktime for stable timer.
 */
static inline ktime_t kvm_loongson_stable_timer_time(struct kvm_vcpu *vcpu)
{
	if (unlikely(vcpu->arch.stable_timer_ctl & KVM_REG_MIPS_COUNT_CTL_DC))
		return vcpu->arch.stable_timer_resume;

	return ktime_get();
}

/**
 * kvm_loongson_read_stable_running() - Read the current count value as if running.
 * @vcpu:	Virtual CPU.
 * @now:	Kernel time to read stable timer at.
 *
 * Returns the current guest stable timer at time @now and handles if the
 * timer interrupt is pending and hasn't been handled yet.
 *
 * Returns:	The current value of the guest stable timer.
 */
static u64 kvm_loongson_read_stable_running(struct kvm_vcpu *vcpu, ktime_t now)
{
	ktime_t expires, threshold;
	u64 stable_timer;
	int running;

	/* Calculate the biased and scaled guest stable timer */
	stable_timer = vcpu->arch.stable_timer_bias + kvm_mips_ktime_to_stable_timer(vcpu, now);
	stable_timer &= LOONGSON_STABLE_TIMER_MASK;

	/*
	 * Find whether stable timer has reached the closest timer interrupt. If
	 * not, we shouldn't inject it.
	 */
	if (stable_timer < ((u64)(1ULL << 48) - 1))
		return stable_timer;

	/*
	 * The stable timer we're going to return has already reached the closest
	 * timer interrupt. Quickly check if it really is a new interrupt by
	 * looking at whether the interval until the hrtimer expiry time is
	 * less than 1/4 of the timer period.
	 */
	expires = hrtimer_get_expires(&vcpu->arch.comparecount_timer);
	threshold = ktime_add_ns(now, vcpu->arch.stable_timer_period / 4);
	if (ktime_before(expires, threshold)) {
		/*
		 * Cancel it while we handle it so there's no chance of
		 * interference with the timeout handler.
		 */
		running = hrtimer_cancel(&vcpu->arch.comparecount_timer);

		/* Nothing should be waiting on the timeout */
		kvm_mips_callbacks->queue_timer_int(vcpu);

		/*
		 * Restart the timer if it was running based on the expiry time
		 * we read, so that we don't push it back 2 periods.
		 */
		if (running) {
			expires = ktime_add_ns(expires,
					       vcpu->arch.stable_timer_period);
			hrtimer_start(&vcpu->arch.comparecount_timer, expires,
				      HRTIMER_MODE_ABS);
		}
	}

	return stable_timer;
}

/**
 * kvm_mips_loongson_read_stable_timer() - Read the current stable timer value.
 * @vcpu:	Virtual CPU.
 *
 * Read the current guest stable timer value, taking into account whether the timer
 * is stopped.
 *
 * Returns:	The current guest stable timer value.
 */
u32 kvm_loongson_read_stable_timer(struct kvm_vcpu *vcpu)
{
	/* If count disabled just read static copy of count */
	if (kvm_loongson_stable_timer_disabled(vcpu))
		return vcpu->arch.stable_timer_tick;

	return kvm_loongson_read_stable_running(vcpu, ktime_get());
}

/**
 * kvm_loongson_freeze_hrtimer() - Safely stop the hrtimer.
 * @vcpu:	Virtual CPU.
 * @count:	Output pointer for stable timer value at point of freeze.
 *
 * Freeze the hrtimer safely and return both the ktime and the stable timer value
 * at the point it was frozen. It is guaranteed that any pending interrupts at
 * the point it was frozen are handled, and none after that point.
 *
 * This is useful where the time is needed in the calculation of the
 * new parameters.
 *
 * Assumes !kvm_loongson_stable_timer_disabled(@vcpu) (guest stable timer is running).
 *
 * Returns:	The ktime at the point of freeze.
 */
ktime_t kvm_loongson_freeze_hrtimer(struct kvm_vcpu *vcpu, u64 *stable_timer)
{
	ktime_t now;

	/* stop hrtimer before finding time */
	hrtimer_cancel(&vcpu->arch.comparecount_timer);
	now = ktime_get();

	/* find count at this point and handle pending hrtimer */
	*stable_timer = kvm_loongson_read_stable_running(vcpu, now);

	return now;
}

/**
 * kvm_loongson_resume_hrtimer() - Resume hrtimer, updating expiry.
 * @vcpu:	Virtual CPU.
 * @now:	ktime at point of resume.
 * @stable_timer:	stable timer at point of resume.
 *
 * Resumes the timer and updates the timer expiry based on @now and @count.
 * This can be used in conjunction with kvm_mips_freeze_timer() when timer
 * parameters need to be changed.
 *
 * It is guaranteed that a timer interrupt immediately after resume will be
 * handled. That case is already handled by kvm_mips_freeze_timer().
 *
 * Assumes !kvm_loongson_stable_timer_disabled(@vcpu) (guest stable timer is running).
 */
static void kvm_loongson_resume_hrtimer(struct kvm_vcpu *vcpu,
				    ktime_t now, u64 stable_timer)
{
	u64 delta;
	ktime_t expire;

	/* Stable timer decreased to zero or
	 * initialize to zero, set 4 second timer
	*/
	delta = div_u64(stable_timer * NSEC_PER_SEC, vcpu->arch.stable_timer_hz);
	expire = ktime_add_ns(now, delta);

	/* Update hrtimer to use new timeout */
	hrtimer_cancel(&vcpu->arch.comparecount_timer);
	hrtimer_start(&vcpu->arch.comparecount_timer, expire, HRTIMER_MODE_ABS);
}

/**
 * kvm_mips_restore_hrtimer() - Restore hrtimer after a gap, updating expiry.
 * @vcpu:	Virtual CPU.
 * @before:	Time before Count was saved, lower bound of drift calculation.
 * @stable_timer:	stable timer at point of restore.
 * @min_drift:	Minimum amount of drift permitted before correction.
 *		Must be <= 0.
 *
 * Restores the timer from a particular @count, accounting for drift. This can
 * be used in conjunction with kvm_mips_freeze_timer() when a hardware timer is
 * to be used for a period of time, but the exact ktime corresponding to the
 * final Count that must be restored is not known.
 *
 * It is gauranteed that a timer interrupt immediately after restore will be
 * handled. That case should already be handled when the hardware timer state is saved.
 *
 * Assumes !kvm_loongson_stable_timer_disabled(@vcpu) (guest stable timer is not
 * stopped).
 *
 * Returns:	Amount of correction to stable_timer_bias due to drift.
 */
int kvm_loongson_restore_hrtimer(struct kvm_vcpu *vcpu, ktime_t before,
			     u64 stable_timer, int min_drift)
{
	int ret = 0;

	/* Resume using the calculated ktime */
	kvm_loongson_resume_hrtimer(vcpu, before, stable_timer);
	return ret;
}

/**
 * kvm_loongson_write_stable_timer() - Modify the count and update timer.
 * @vcpu:	Virtual CPU.
 * @stable_timer:	Guest stable timer value to set.
 *
 * Sets the stable timer value and updates the timer accordingly.
 */
void kvm_loongson_write_stable_timer(struct kvm_vcpu *vcpu, u64 stable_timer)
{
	ktime_t now;

	/* Calculate bias */
	now = kvm_loongson_stable_timer_time(vcpu);
	vcpu->arch.stable_timer_bias = stable_timer - kvm_mips_ktime_to_stable_timer(vcpu, now);
	vcpu->arch.stable_timer_bias &= LOONGSON_STABLE_TIMER_MASK;

	if (kvm_loongson_stable_timer_disabled(vcpu)) {
		/* The timer's disabled, adjust the static stable timer */
		vcpu->arch.stable_timer_tick = stable_timer;
	} else {
		/* Update timeout */
		kvm_loongson_resume_hrtimer(vcpu, now, stable_timer);
	}
}

/**
 * kvm_loongson_init_stable_timer() - Initialise stable timer.
 * @vcpu:	Virtual CPU.
 * @stable_timer_hz:	Frequency of timer.
 *
 * Initialise the timer to the specified frequency, zero it, and set it going if
 * it's enabled.
 */
void kvm_loongson_init_stable_timer(struct kvm_vcpu *vcpu, unsigned long stable_timer_hz)
{
	vcpu->arch.stable_timer_hz = stable_timer_hz;
	vcpu->arch.stable_timer_period = div_u64((u64)NSEC_PER_SEC << 32, stable_timer_hz);
	vcpu->arch.stable_timer_dyn_bias = 0;

	/* Starting at 0 */
	kvm_timer_callbacks->write_stable_timer(vcpu, 0);
}

/**
 * kvm_loongson_set_stable_timer_hz() - Update the frequency of the timer.
 * @vcpu:	Virtual CPU.
 * @stable_timer_hz:	Frequency of stable timer in Hz.
 *
 * Change the frequency of the stable timer. This is done atomically so that
 * stable timer is continuous and no timer interrupt is lost.
 *
 * Returns:	-EINVAL if @stable_timer_hz is out of range.
 *		0 on success.
 */
int kvm_loongson_set_stable_timer_hz(struct kvm_vcpu *vcpu, s64 stable_timer_hz)
{
	int dc;
	ktime_t now;
	u64 stable_timer;

	/* ensure the frequency is in a sensible range... */
	if (stable_timer_hz <= 0 || stable_timer_hz > NSEC_PER_SEC)
		return -EINVAL;
	/* ... and has actually changed */
	if (vcpu->arch.stable_timer_hz == stable_timer_hz)
		return 0;

	/* Safely freeze timer so we can keep it continuous */
	dc = kvm_loongson_stable_timer_disabled(vcpu);
	if (dc) {
		now = kvm_loongson_stable_timer_time(vcpu);
		stable_timer = vcpu->arch.stable_timer_tick;
	} else {
		now = kvm_loongson_freeze_hrtimer(vcpu, &stable_timer);
	}

	/* Update the frequency */
	vcpu->arch.stable_timer_hz = stable_timer_hz;
	vcpu->arch.stable_timer_period = div_u64((u64)NSEC_PER_SEC << 32, stable_timer_hz);
	vcpu->arch.stable_timer_dyn_bias = 0;

	/* Calculate adjusted bias so dynamic count is unchanged */
	vcpu->arch.stable_timer_bias = stable_timer - kvm_mips_ktime_to_stable_timer(vcpu, now);
	vcpu->arch.stable_timer_bias &= LOONGSON_STABLE_TIMER_MASK;

	/* Update and resume hrtimer */
	if (!dc)
		kvm_loongson_resume_hrtimer(vcpu, now, stable_timer);
	return 0;
}

/**
 * kvm_loongson_stable_timer_disable() - Disable stable timer.
 * @vcpu:	Virtual CPU.
 *
 * Disable the stable timer. A timer interrupt on or before the final stop
 * time will be handled but not after.
 *
 * Assumes stable timer was previously enabled but now Guest.CP0_Cause.DC or
 * stable_timer_ctl.DC has been set (count disabled).
 *
 * Returns:	The time that the timer was stopped.
 */
static ktime_t kvm_loongson_stable_timer_disable(struct kvm_vcpu *vcpu)
{
	u64 stable_timer;
	ktime_t now;

	/* Stop hrtimer */
	hrtimer_cancel(&vcpu->arch.comparecount_timer);

	/* Set the static count from the dynamic count, handling pending TI */
	now = ktime_get();
	stable_timer = kvm_loongson_read_stable_running(vcpu, now);

	vcpu->arch.stable_timer_tick = stable_timer;

	return now;
}

/**
 * kvm_loongson_set_stable_timer_ctl() - Update the count control KVM register.
 * @vcpu:	Virtual CPU.
 * @stable_timer_ctl:	Count control register new value.
 *
 * Set the count control KVM register. The timer is updated accordingly.
 *
 * Returns:	-EINVAL if reserved bits are set.
 *		0 on success.
 */
int kvm_loongson_set_stable_timer_ctl(struct kvm_vcpu *vcpu, s64 stable_timer_ctl)
{
	struct mips_coproc *cop0 = vcpu->arch.cop0;
	s64 changed = stable_timer_ctl ^ vcpu->arch.stable_timer_ctl;
	s64 delta;
	ktime_t expire, now;
	u64 stable_timer;

	/* Only allow defined bits to be changed */
	if (changed & ~(s64)(KVM_REG_MIPS_COUNT_CTL_DC))
		return -EINVAL;

	/* Apply new value */
	vcpu->arch.stable_timer_ctl = stable_timer_ctl;

	/* Master CP0_Count disable */
	if (changed & KVM_REG_MIPS_COUNT_CTL_DC) {
		/* Is CP0_Cause.DC already disabling CP0_Count? */
		if (kvm_read_c0_guest_cause(cop0) & CAUSEF_DC) {
			if (stable_timer_ctl & KVM_REG_MIPS_COUNT_CTL_DC)
				/* Just record the current time */
				vcpu->arch.stable_timer_resume = ktime_get();
		} else if (stable_timer_ctl & KVM_REG_MIPS_COUNT_CTL_DC) {
			/* disable timer and record current time */
			vcpu->arch.stable_timer_resume = kvm_loongson_stable_timer_disable(vcpu);
		} else {
			/*
			 * Calculate timeout relative to static count at resume
			 * time (wrap 0 to 2^32).
			 */
			stable_timer = vcpu->arch.stable_timer_tick;
			delta = div_u64(stable_timer * NSEC_PER_SEC,
					vcpu->arch.stable_timer_hz);
			expire = ktime_add_ns(vcpu->arch.stable_timer_resume, delta);

			/* Handle pending interrupt */
			now = ktime_get();
			if (ktime_compare(now, expire) >= 0)
				/* Nothing should be waiting on the timeout */
				kvm_mips_callbacks->queue_timer_int(vcpu);

			/* Resume hrtimer without changing bias */
			stable_timer = kvm_loongson_read_stable_running(vcpu, now);
			kvm_loongson_resume_hrtimer(vcpu, now, stable_timer);
		}
	}

	return 0;
}

/**
 * kvm_loongson_set_stable_timer_resume() - Update the count resume KVM register.
 * @vcpu:		Virtual CPU.
 * @stable_timer_resume:	Count resume register new value.
 *
 * Set the count resume KVM register.
 *
 * Returns:	-EINVAL if out of valid range (0..now).
 *		0 on success.
 */
int kvm_loongson_set_stable_timer_resume(struct kvm_vcpu *vcpu, s64 stable_timer_resume)
{
	/*
	 * It doesn't make sense for the resume time to be in the future, as it
	 * would be possible for the next interrupt to be more than a full
	 * period in the future.
	 */
	if (stable_timer_resume < 0 || stable_timer_resume > ktime_to_ns(ktime_get()))
		return -EINVAL;

	vcpu->arch.stable_timer_resume = ns_to_ktime(stable_timer_resume);
	return 0;
}

/**
 * kvm_loongson_count_timeout() - Push timer forward on timeout.
 * @vcpu:	Virtual CPU.
 *
 * Handle an hrtimer event by push the hrtimer forward a period.
 *
 * Returns:	The hrtimer_restart value to return to the hrtimer subsystem.
 */
enum hrtimer_restart kvm_loongson_count_timeout(struct kvm_vcpu *vcpu)
{
	/* Add the Count period to the current expiry time */
	hrtimer_add_expires_ns(&vcpu->arch.comparecount_timer,
			       vcpu->arch.stable_timer_period);
	return HRTIMER_RESTART;
}
