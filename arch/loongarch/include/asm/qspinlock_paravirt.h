/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_QSPINLOCK_PARAVIRT_H
#define __ASM_QSPINLOCK_PARAVIRT_H

#ifdef CONFIG_64BIT

#define __pv_queued_spin_unlock	__pv_queued_spin_unlock
void __pv_queued_spin_unlock(struct qspinlock *lock)
{
	u8 lockval = cmpxchg(&lock->locked, _Q_LOCKED_VAL, 0);

	if (likely(lockval == _Q_LOCKED_VAL))
		return;
	__pv_queued_spin_unlock_slowpath(lock, lockval);
}

#endif /* CONFIG_64BIT */
#endif
