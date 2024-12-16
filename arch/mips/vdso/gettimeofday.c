/*
 * Copyright (C) 2015 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include "vdso.h"

#include <linux/compiler.h>
#include <linux/time.h>

#include <asm/clocksource.h>
#include <asm/io.h>
#include <asm/unistd.h>
#include <asm/vdso.h>

static __always_inline long getpid_fallback(void)
{
	register long ret asm("v0");
	register long nr asm("v0") = __NR_getpid;
	register long error asm("a3");

	asm volatile(
	"       syscall\n"
	: "=r" (ret), "=r" (error)
	: "r" (nr)
	: "$1", "$3", "$8", "$9", "$10", "$11", "$12", "$13",
	  "$14", "$15", "$24", "$25", "hi", "lo", "memory");

	return ret;
}

static __always_inline long getuid_fallback(void)
{
	register long ret asm("v0");
	register long nr asm("v0") = __NR_getuid;
	register long error asm("a3");

	asm volatile(
	"       syscall\n"
	: "=r" (ret), "=r" (error)
	: "r" (nr)
	: "$1", "$3", "$8", "$9", "$10", "$11", "$12", "$13",
	  "$14", "$15", "$24", "$25", "hi", "lo", "memory");

	return ret;
}

#if MIPS_ISA_REV < 6
#define VDSO_SYSCALL_CLOBBERS "hi", "lo",
#else
#define VDSO_SYSCALL_CLOBBERS
#endif

#ifdef CONFIG_MIPS_CLOCK_VSYSCALL

static __always_inline long gettimeofday_fallback(struct timeval *_tv,
					  struct timezone *_tz)
{
	register struct timezone *tz asm("a1") = _tz;
	register struct timeval *tv asm("a0") = _tv;
	register long ret asm("v0");
	register long nr asm("v0") = __NR_gettimeofday;
	register long error asm("a3");

	asm volatile(
	"       syscall\n"
	: "=r" (ret), "=r" (error)
	: "r" (tv), "r" (tz), "r" (nr)
	: "$1", "$3", "$8", "$9", "$10", "$11", "$12", "$13",
	  "$14", "$15", "$24", "$25",
	  VDSO_SYSCALL_CLOBBERS
	  "memory");

	return error ? -ret : ret;
}

#endif

static __always_inline long clock_gettime_fallback(clockid_t _clkid,
					   struct timespec *_ts)
{
	register struct timespec *ts asm("a1") = _ts;
	register clockid_t clkid asm("a0") = _clkid;
	register long ret asm("v0");
	register long nr asm("v0") = __NR_clock_gettime;
	register long error asm("a3");

	asm volatile(
	"       syscall\n"
	: "=r" (ret), "=r" (error)
	: "r" (clkid), "r" (ts), "r" (nr)
	: "$1", "$3", "$8", "$9", "$10", "$11", "$12", "$13",
	  "$14", "$15", "$24", "$25",
	  VDSO_SYSCALL_CLOBBERS
	  "memory");

	return error ? -ret : ret;
}

static __always_inline int do_realtime_coarse(struct timespec *ts,
					      const union mips_vdso_data *data)
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
					       const union mips_vdso_data *data)
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

#ifdef CONFIG_CSRC_R4K

static __always_inline u64 read_r4k_count(void)
{
	unsigned int count;

	__asm__ __volatile__(
	"	.set push\n"
	"	.set mips32r2\n"
	"	rdhwr	%0, $2\n"
	"	.set pop\n"
	: "=r" (count));

	return count;
}

#endif

static __always_inline u32 read_cpu_id(void)
{
	unsigned long cpu_id;

	__asm__ __volatile__(
	"	.set push\n"
	"	.set mips64r2\n"
	"	rdhwr	%0, $0\n"
	"	.set pop\n"
	: "=r" (cpu_id)
	:
	: "memory");

	return cpu_id;
}

static __always_inline u64 read_stable_count(void)
{
	unsigned long count;

	__asm__ __volatile__(
	"	.set push\n"
	"	.set mips64r2\n"
	"	rdhwr	%0, $30\n"
	"	.set pop\n"
	: "=r" (count));

	return count;
}

#ifdef CONFIG_LOONGSON_HPET

#ifdef CONFIG_CPU_LOONGSON2K
#define HPET_MAIN_COUNTER_OFFSET	0xf0
#else	/* LS7A1000 */
#define HPET_MAIN_COUNTER_OFFSET	0x10f0
#endif

static __always_inline u64 read_hpet_count(const union mips_vdso_data *data)
{
	void __iomem *hpet = get_extimer(data);

	return ____raw_readq(hpet + HPET_MAIN_COUNTER_OFFSET);
}

#endif

#ifdef CONFIG_CLKSRC_MIPS_GIC

static __always_inline u64 read_gic_count(const union mips_vdso_data *data)
{
	void __iomem *gic = get_extimer(data);
	u32 hi, hi2, lo;

	do {
		hi = __raw_readl(gic + sizeof(lo));
		lo = __raw_readl(gic);
		hi2 = __raw_readl(gic + sizeof(lo));
	} while (hi2 != hi);

	return (((u64)hi) << 32) + lo;
}

#endif

static __always_inline u64 get_ns(const union mips_vdso_data *data)
{
	u64 cycle_now, delta, nsec;

	switch (data->clock_mode) {
#ifdef CONFIG_CSRC_R4K
	case VDSO_CLOCK_R4K:
		cycle_now = read_r4k_count();
		break;
#endif
#ifdef CONFIG_CLKSRC_MIPS_GIC
	case VDSO_CLOCK_GIC:
		cycle_now = read_gic_count(data);
		break;
#endif
	case VDSO_CLOCK_STABLE:
		cycle_now = read_stable_count();
		break;
#ifdef CONFIG_LOONGSON_HPET
	case VDSO_CLOCK_HPET:
		cycle_now = read_hpet_count(data);
		break;
#endif
	default:
		return 0;
	}

	delta = (cycle_now - data->cs_cycle_last) & data->cs_mask;

	nsec = (delta * data->cs_mult) + data->xtime_nsec;
	nsec >>= data->cs_shift;

	return nsec;
}

static __always_inline int do_realtime(struct timespec *ts,
				       const union mips_vdso_data *data)
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
					const union mips_vdso_data *data)
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

#ifdef CONFIG_MIPS_CLOCK_VSYSCALL

/*
 * This is behind the ifdef so that we don't provide the symbol when there's no
 * possibility of there being a usable clocksource, because there's nothing we
 * can do without it. When libc fails the symbol lookup it should fall back on
 * the standard syscall path.
 */
int __vdso_gettimeofday(struct timeval *tv, struct timezone *tz)
{
#if _MIPS_SIM == _MIPS_SIM_ABI32
	return gettimeofday_fallback(tv, tz);
#else
	const union mips_vdso_data *data = get_vdso_data();
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
#endif
}

#endif /* CONFIG_MIPS_CLOCK_VSYSCALL */

int __vdso_clock_gettime(clockid_t clkid, struct timespec *ts)
{
#if _MIPS_SIM == _MIPS_SIM_ABI32
	return clock_gettime_fallback(clkid, ts);
#else
	const union mips_vdso_data *data = get_vdso_data();
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
#endif
}

static __always_inline pid_t __do_getpid(const union mips_vdso_data *data)
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
#if _MIPS_SIM == _MIPS_SIM_ABI32
	return getpid_fallback();
#else
	const union mips_vdso_data *data;
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
#endif
}

static __always_inline uid_t __do_getuid(const union mips_vdso_data *data)
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
#if _MIPS_SIM == _MIPS_SIM_ABI32
	return getuid_fallback();
#else
	const union mips_vdso_data *data;
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
#endif
}
