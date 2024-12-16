// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include "vdso.h"

#include <linux/compiler.h>
#include <linux/getcpu.h>
#include <linux/time.h>

#include <asm/clocksource.h>
#include <asm/io.h>
#include <asm/unistd.h>
#include <asm/vdso.h>

static __always_inline long gettimeofday_fallback(struct timeval *_tv,
					  struct timezone *_tz)
{
	register struct timezone *tz asm("a1") = _tz;
	register struct timeval *tv asm("a0") = _tv;
	register long ret asm("v0");
	register long nr asm("a7") = __NR_gettimeofday;

	asm volatile(
	"       syscall 0\n"
	: "=r" (ret)
	: "r" (nr), "r" (tv), "r" (tz)
	: "$t0", "$t1", "$t2", "$t3", "$t4", "$t5", "$t6", "$t7",
	  "$t8", "memory");

	return ret;
}

static __always_inline long clock_gettime_fallback(clockid_t _clkid,
					   struct timespec *_ts)
{
	register struct timespec *ts asm("a1") = _ts;
	register clockid_t clkid asm("a0") = _clkid;
	register long ret asm("v0");
	register long nr asm("a7") = __NR_clock_gettime;

	asm volatile(
	"       syscall 0\n"
	: "=r" (ret)
	: "r" (nr), "r" (clkid), "r" (ts)
	: "$t0", "$t1", "$t2", "$t3", "$t4", "$t5", "$t6", "$t7",
	  "$t8", "memory");

	return ret;
}

static __always_inline long getpid_fallback(void)
{
	register long ret asm("v0");
	register long nr asm("a7") = __NR_getpid;
	register long error asm("a3");

	asm volatile(
	"       syscall 0\n"
	: "=r" (ret), "=r" (error)
	: "r" (nr)
	: "$t0", "$t1", "$t2", "$t3", "$t4", "$t5", "$t6", "$t7",
	  "$t8", "memory");

	return ret;
}

static __always_inline long getuid_fallback(void)
{
	register long ret asm("v0");
	register long nr asm("a7") = __NR_getuid;
	register long error asm("a3");

	asm volatile(
	"       syscall 0\n"
	: "=r" (ret), "=r" (error)
	: "r" (nr)
	: "$t0", "$t1", "$t2", "$t3", "$t4", "$t5", "$t6", "$t7",
	  "$t8", "memory");

	return ret;
}

static __always_inline long getcpu_fallback(unsigned *_cpu, unsigned *_node)
{
	register unsigned *node asm("a1") = _node;
	register unsigned *cpu asm("a0") = _cpu;
	register long ret asm("v0");
	register long nr asm("a7") = __NR_getcpu;

	asm volatile(
	"       syscall 0\n"
	: "=r" (ret)
	: "r" (nr), "r"(cpu), "r"(node)
	: "$t0", "$t1", "$t2", "$t3", "$t4", "$t5", "$t6", "$t7",
	  "$t8", "memory");

	return ret;
}

static __always_inline int do_realtime_coarse(struct timespec *ts,
					      const struct loongarch_vdso_data *data)
{
	u32 start_seq;

	do {
		start_seq = vdso_data_read_begin(data);

		ts->tv_sec = data->xtime_sec;
		ts->tv_nsec = data->xtime_nsec >> data->cs_shift;
	} while (vdso_data_read_retry(data, start_seq));

	return 0;
}

static __always_inline int do_monotonic_coarse(struct timespec *ts,
					       const struct loongarch_vdso_data *data)
{
	u32 start_seq;
	u64 to_mono_sec;
	u64 to_mono_nsec;

	do {
		start_seq = vdso_data_read_begin(data);

		ts->tv_sec = data->xtime_sec;
		ts->tv_nsec = data->xtime_nsec >> data->cs_shift;

		to_mono_sec = data->wall_to_mono_sec;
		to_mono_nsec = data->wall_to_mono_nsec;
	} while (vdso_data_read_retry(data, start_seq));

	ts->tv_sec += to_mono_sec;
	timespec_add_ns(ts, to_mono_nsec);

	return 0;
}

static __always_inline u32 read_cpu_id(void)
{
	unsigned long cpu_id;

	__asm__ __volatile__(
	"	rdtime.d $zero, %0\n"
	: "=r" (cpu_id)
	:
	: "memory");

	return cpu_id;
}

static __always_inline u64 read_stable_count(void)
{
	unsigned long count;

	__asm__ __volatile__(
	"	rdtime.d %0, $zero\n"
	: "=r" (count));

	return count;
}

static __always_inline u64 get_ns(const struct loongarch_vdso_data *data)
{
	u64 cycle_now, delta, nsec;

	switch (data->clock_mode) {
	case VDSO_CLOCK_STABLE:
		cycle_now = read_stable_count();
		break;
	default:
		return 0;
	}

	delta = (cycle_now - data->cs_cycle_last) & data->cs_mask;

	nsec = (delta * data->cs_mult) + data->xtime_nsec;
	nsec >>= data->cs_shift;

	return nsec;
}

static __always_inline int do_realtime(struct timespec *ts,
				       const struct loongarch_vdso_data *data)
{
	u32 start_seq;
	u64 ns;

	do {
		start_seq = vdso_data_read_begin(data);

		if (data->clock_mode == VDSO_CLOCK_NONE)
			return -ENOSYS;

		ts->tv_sec = data->xtime_sec;
		ns = get_ns(data);
	} while (vdso_data_read_retry(data, start_seq));

	ts->tv_nsec = 0;
	timespec_add_ns(ts, ns);

	return 0;
}

static __always_inline int do_monotonic(struct timespec *ts,
					const struct loongarch_vdso_data *data)
{
	u32 start_seq;
	u64 ns;
	u64 to_mono_sec;
	u64 to_mono_nsec;

	do {
		start_seq = vdso_data_read_begin(data);

		if (data->clock_mode == VDSO_CLOCK_NONE)
			return -ENOSYS;

		ts->tv_sec = data->xtime_sec;
		ns = get_ns(data);

		to_mono_sec = data->wall_to_mono_sec;
		to_mono_nsec = data->wall_to_mono_nsec;
	} while (vdso_data_read_retry(data, start_seq));

	ts->tv_sec += to_mono_sec;
	ts->tv_nsec = 0;
	timespec_add_ns(ts, ns + to_mono_nsec);

	return 0;
}

/*
 * This is behind the ifdef so that we don't provide the symbol when there's no
 * possibility of there being a usable clocksource, because there's nothing we
 * can do without it. When libc fails the symbol lookup it should fall back on
 * the standard syscall path.
 */
int __vdso_gettimeofday(struct timeval *tv, struct timezone *tz)
{
	const struct loongarch_vdso_data *data = get_vdso_data();
	struct timespec ts;
	int ret;

	ret = do_realtime(&ts, data);
	if (ret)
		return gettimeofday_fallback(tv, tz);

	if (tv) {
		tv->tv_sec = ts.tv_sec;
		tv->tv_usec = ts.tv_nsec / 1000;
	}

	if (tz) {
		tz->tz_minuteswest = data->tz_minuteswest;
		tz->tz_dsttime = data->tz_dsttime;
	}

	return 0;
}

int __vdso_clock_gettime(clockid_t clkid, struct timespec *ts)
{
	const struct loongarch_vdso_data *data = get_vdso_data();
	int ret = -1;

	switch (clkid) {
	case CLOCK_REALTIME_COARSE:
		ret = do_realtime_coarse(ts, data);
		break;
	case CLOCK_MONOTONIC_COARSE:
		ret = do_monotonic_coarse(ts, data);
		break;
	case CLOCK_REALTIME:
		ret = do_realtime(ts, data);
		break;
	case CLOCK_MONOTONIC:
		ret = do_monotonic(ts, data);
		break;
	default:
		break;
	}

	if (ret)
		ret = clock_gettime_fallback(clkid, ts);

	return ret;
}

static __always_inline pid_t __do_getpid(const struct loongarch_vdso_data *data)
{
	u32 start_seq;
	u32 cpu_id;
	pid_t pid;

	do {
		cpu_id = read_cpu_id();
		start_seq = vdso_pcpu_data_read_begin(&data->pcpu_data[cpu_id]);
		pid = data->pcpu_data[cpu_id].pid;

	} while (cpu_id != read_cpu_id() ||
		vdso_pcpu_data_read_retry(&data->pcpu_data[cpu_id], start_seq));

    return pid;
}

pid_t __vdso_getpid(void)
{
	const struct loongarch_vdso_data *data;
	pid_t pid;

	data = get_vdso_data();
	if (!data) {
		return getpid_fallback();
	}

	pid = __do_getpid(data);
	if (pid == VDSO_INVALID_PID) {
		return getpid_fallback();
	}

	return pid;
}

static __always_inline uid_t __do_getuid(const struct loongarch_vdso_data *data)
{
	u32 start_seq;
	u32 cpu_id;
	uid_t uid;

	do {
		cpu_id = read_cpu_id();
		start_seq = vdso_pcpu_data_read_begin(&data->pcpu_data[cpu_id]);
		uid = data->pcpu_data[cpu_id].uid;

	} while (cpu_id != read_cpu_id() ||
		vdso_pcpu_data_read_retry(&data->pcpu_data[cpu_id], start_seq));

	return uid;
}

uid_t __vdso_getuid(void)
{
	const struct loongarch_vdso_data *data;
	uid_t uid;

	data = get_vdso_data();
	if (!data) {
		return getuid_fallback();
	}

	uid = __do_getuid(data);
	if (uid == VDSO_INVALID_UID) {
		return getuid_fallback();
	}

	return uid;
}

long __vdso_getcpu(unsigned *cpu, unsigned *node, struct getcpu_cache *unused)
{
	u32 cpu_id;

	cpu_id = read_cpu_id();

	if (cpu)
		*cpu = cpu_id;

	if (node) {
		const struct loongarch_vdso_data *data;

		data = get_vdso_data();
		if (!data)
			return getcpu_fallback(cpu, node);

		*node = data->pcpu_data[cpu_id].node;
	}

	return 0;
}
