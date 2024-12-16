// SPDX-License-Identifier: GPL-2.0
/*
 * Split spinlock implementation out into its own file, so it can be
 * compiled in a FTRACE-compatible way.
 */
#include <linux/spinlock.h>
#include <linux/export.h>

#include <asm/paravirt.h>

__visible void paravirt_wait_nop(u8 *ptr, u8 val)
{
}

__visible void paravirt_kick_nop(int cpu)
{
}

__visible void __native_queued_spin_unlock(struct qspinlock *lock)
{
	native_queued_spin_unlock(lock);
}

bool pv_is_native_spin_unlock(void)
{
	return pv_lock_ops.queued_spin_unlock ==  __native_queued_spin_unlock;
}

__visible bool __native_vcpu_is_preempted(long cpu)
{
	return false;
}

bool pv_is_native_vcpu_is_preempted(void)
{
	return pv_lock_ops.vcpu_is_preempted == __native_vcpu_is_preempted;
}

struct pv_lock_ops pv_lock_ops = {
#ifdef CONFIG_SMP
	.queued_spin_lock_slowpath = native_queued_spin_lock_slowpath,
	.queued_spin_unlock = __native_queued_spin_unlock,
	.wait = paravirt_wait_nop,
	.kick = paravirt_kick_nop,
	.vcpu_is_preempted = __native_vcpu_is_preempted,
#endif /* SMP */
};
EXPORT_SYMBOL(pv_lock_ops);
