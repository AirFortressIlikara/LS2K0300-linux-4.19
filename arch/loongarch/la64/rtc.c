// SPDX-License-Identifier: GPL-2.0
/*
 *  Lemote Fuloong platform support
 *
 *  Copyright(c) 2010 Arnaud Patard <apatard@mandriva.com>
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <loongson-pch.h>

#ifndef loongson_decode_year
#define loongson_decode_year(year) ((year) + 1900)
#endif

#define RTC_TOYREAD0    0x2C
#define RTC_YEAR        0x30

unsigned long loongson_get_rtc_time(void)
{
	unsigned int year, mon, day, hour, min, sec;
	unsigned int value;

	value = ls7a_readl(LS7A_RTC_REG_BASE + RTC_TOYREAD0);
	sec = (value >> 4) & 0x3f;
	min = (value >> 10) & 0x3f;
	hour = (value >> 16) & 0x1f;
	day = (value >> 21) & 0x1f;
	mon = (value >> 26) & 0x3f;
	year = ls7a_readl(LS7A_RTC_REG_BASE + RTC_YEAR);

	year = loongson_decode_year(year);

	return mktime(year, mon, day, hour, min, sec);
}

void read_persistent_clock64(struct timespec64 *ts)
{
	if (loongson_sysconf.is_soc_cpu)
		ts->tv_sec = mktime(2020, 1, 1, 0, 0, 0);
	else
		ts->tv_sec = loongson_get_rtc_time();
	ts->tv_nsec = 0;
}
