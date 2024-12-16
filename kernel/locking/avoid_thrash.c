#include <linux/types.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/topology.h>
#include <linux/preempt.h>

#define FREQ_OFFSET 0
#define FREQ_SIZE 20

#define PREV_JIFF_OFFSET 20
#define PREV_JIFF_SIZE 32

#define PREV_PID_OFFSET 52
#define PREV_PID_SIZE 8

#define NID_OFFSET 60
#define NID_SIZE 4

static inline u64 value_normalize(u64 value, int cnt)
{
	return ~0UL >> (64 - cnt) & value;
}

static inline u64 get_field(u64 val, u32 offset, u32 cnt)
{
	return (u64)(val >> offset) & (~0UL >> (64 - cnt));
}

static inline u64 set_field(u64 old, u32 offset, u32 cnt, u64 field_val)
{
	u64 mask = 0;

	if (offset + cnt >= 63)
		mask = ~0UL >> (64 - offset);
	if (64 - offset >= 63)
		mask = ~0UL << (offset + cnt);
	if (mask == 0)
		mask = ~0UL >> (64 - offset) | ~0UL << (offset + cnt);

	return (old & mask) | (field_val << offset);
}

static inline u64 get_freq(u64 val)
{
	return get_field(val, FREQ_OFFSET, FREQ_SIZE);
}

static inline u64 set_freq(u64 val, u64 field_val)
{
	return set_field(val, FREQ_OFFSET, FREQ_SIZE, field_val);
}

static inline u64 get_prev_jiff(u64 val)
{
	return get_field(val, PREV_JIFF_OFFSET, PREV_JIFF_SIZE);
}

static inline u64 set_prev_jiff(u64 val, u64 field_val)
{
	return set_field(val, PREV_JIFF_OFFSET, PREV_JIFF_SIZE, field_val);
}

static inline u64 get_prev_pid(u64 val)
{
	return get_field(val, PREV_PID_OFFSET, PREV_PID_SIZE);
}

static inline u64 set_prev_pid(u64 val, u64 field_val)
{
	return set_field(val, PREV_PID_OFFSET, PREV_PID_SIZE, field_val);
}

static inline u64 get_nid(u64 val)
{
	return get_field(val, NID_OFFSET, NID_SIZE);
}

static inline u64 set_nid(u64 val, u64 field_val)
{
	return set_field(val, NID_OFFSET, NID_SIZE, field_val);
}

#ifndef CONFIG_NODE_CACHE_THRASH_OPTIMIZATION
static inline void do_fixed_set(u64 nid)
{
	return;
}

static inline int make_decision_fixed(u64 *ptr)
{
	return -1;
}

void update_access_stat(u64 *ptr, int kind)
{
	return;
}
#else
static inline void do_fixed_set(u64 nid)
{
	if (atomic_cmpxchg(&current->in_progress, 0, 1) != 0)
		return;

	if (in_task()) {
		sched_setaffinity(current->pid, cpumask_of_node(nid));
#ifdef CONFIG_DEBUG_AVOID_NODE_THRASH
		printk("pinned, jiffies: %lu pid: %d comm: %s nid: %llu\n",
			   jiffies, current->pid, current->comm, nid);
#endif
		current->pinned = nid;
		smp_mb();
	}
	atomic_set(&current->in_progress, 0);

	return;
}

static inline int make_decision_fixed(u64 *ptr)
{
	u64 jiff_normal = value_normalize(jiffies, PREV_JIFF_SIZE);
	u64 jiff_diff;
	int ret = -1;

	if (unlikely(current->init_pin == -1)) {
		if (current->nr_cpus_allowed >= nr_cpu_ids)
			current->init_pin = 0;
		else
			current->init_pin = 1;
	}

	jiff_diff = jiff_normal - get_prev_jiff(*ptr);
	if (jiff_diff > 2 && jiff_diff * 256 < get_freq(*ptr)) {
		if (current->pinned == -1) {
			if (!current->init_pin) {
				ret = get_nid(*ptr);
#ifdef CONFIG_DEBUG_AVOID_NODE_THRASH
				printk("decision1, jiffies: %lu pid: %d comm: %s freq: %llu nid: %llu diff: %llu\n",
					   jiffies, current->pid, current->comm, get_freq(*ptr), get_nid(*ptr), jiff_diff);
#endif
			}
		} else {
			if (get_nid(*ptr) < current->pinned) {
				ret = get_nid(*ptr);
#ifdef CONFIG_DEBUG_AVOID_NODE_THRASH
				printk("decision2, jiffies: %lu pid: %d comm: %s freq: %llu nid: %llu diff: %llu\n",
					   jiffies, current->pid, current->comm, get_freq(*ptr), get_nid(*ptr), jiff_diff);
#endif
			}
		}
		current->fixed_stamp = jiffies;
	}

	return ret;
}

/*
  Call this func where the lock is requested
 */
void update_access_stat(u64 *ptr, int kind)
{
	u64 jiff_normal = value_normalize(jiffies, PREV_JIFF_SIZE);
	u64 jiff_diff;
	u64 pid_normal = value_normalize(current->pid, PREV_PID_SIZE);
	int nid;

	if (unlikely(*ptr == 0)) {
		*ptr = set_prev_jiff(*ptr, jiff_normal);
		*ptr = set_nid(*ptr, numa_node_id());
	}

	if (pid_normal != get_prev_pid(*ptr)) {
		*ptr = set_freq(*ptr, get_freq(*ptr) + 1);
		*ptr = set_prev_pid(*ptr, pid_normal);
	}

	jiff_diff = jiff_normal - get_prev_jiff(*ptr);
	if (jiff_diff > 4) {
		*ptr = set_freq(*ptr, 0);
		*ptr = set_prev_jiff(*ptr, jiff_normal);
	}
	current->stat = *ptr;

	nid = make_decision_fixed(&current->stat);
	if (nid >= 0)
		do_fixed_set(nid);

	return;
}
#endif
EXPORT_SYMBOL(update_access_stat);
EXPORT_SYMBOL(make_decision_fixed);
EXPORT_SYMBOL(do_fixed_set);
