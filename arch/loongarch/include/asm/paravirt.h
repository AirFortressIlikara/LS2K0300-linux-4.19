/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_LOONGARCH_PARAVIRT_H
#define _ASM_LOONGARCH_PARAVIRT_H
#include <asm/kvm_para.h>

#ifdef CONFIG_PARAVIRT
static inline bool kvm_para_available(void)
{
	return true;
}
struct static_key;
extern struct static_key paravirt_steal_enabled;
extern struct static_key paravirt_steal_rq_enabled;

struct pv_time_ops {
	unsigned long long (*steal_clock)(int cpu);
};
struct kvm_steal_time {
	__u64 steal;
	__u32 version;
	__u32 flags;
	__u8  preempted;
	__u8  pad[47];
};
extern struct pv_time_ops pv_time_ops;

static inline u64 paravirt_steal_clock(int cpu)
{
	return pv_time_ops.steal_clock(cpu);
}

static inline bool pv_feature_support(int feature)
{
	return kvm_hypercall1(KVM_HC_FUNC_FEATURE, feature) == KVM_RET_SUC;
}
static inline void pv_notify_host(int feature, unsigned long data)
{
	kvm_hypercall2(KVM_HC_FUNC_NOTIFY, feature, data);
}

#if defined(CONFIG_SMP) && defined(CONFIG_PARAVIRT_SPINLOCKS)
struct qspinlock;

struct pv_lock_ops {
	void (*queued_spin_lock_slowpath)(struct qspinlock *lock, u32 val);
	void (*queued_spin_unlock)(struct qspinlock *lock);
	void (*wait)(u8 *ptr, u8 val);
	void (*kick)(int cpu);
	bool (*vcpu_is_preempted)(long cpu);
};

extern struct pv_lock_ops pv_lock_ops;

void __init kvm_spinlock_init(void);

static __always_inline void pv_queued_spin_lock_slowpath(struct qspinlock *lock,
		u32 val)
{
	pv_lock_ops.queued_spin_lock_slowpath(lock, val);
}

static __always_inline void pv_queued_spin_unlock(struct qspinlock *lock)
{
	pv_lock_ops.queued_spin_unlock(lock);
}

static __always_inline void pv_wait(u8 *ptr, u8 val)
{
	pv_lock_ops.wait(ptr, val);
}

static __always_inline void pv_kick(int cpu)
{
	pv_lock_ops.kick(cpu);
}

static __always_inline bool pv_vcpu_is_preempted(long cpu)
{
	return pv_lock_ops.vcpu_is_preempted(cpu);
}

#endif /* SMP && PARAVIRT_SPINLOCKS */

int __init pv_time_init(void);
int __init pv_ipi_init(void);
#else
static inline bool kvm_para_available(void)
{
	return false;
}

#define pv_time_init() do {} while (0)
#define pv_ipi_init() do {} while (0)
#endif
#endif /* _ASM_LOONGARCH_PARAVIRT_H */
