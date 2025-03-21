/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/

#ifndef __HALRF_PSD_H__
#define __HALRF_PSD_H__

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)

struct _halrf_psd_data {
	u32		point;
	u32		start_point;
	u32		stop_point;
	u32		average;
	u32		buf_size;
	u32		psd_data[256];
	u32		psd_progress;
};



enum rt_status halrf_psd_init(
	struct	dm_struct	*dm
);



enum rt_status halrf_psd_query(
	struct	dm_struct	*dm,
	u32		*outbuf,
	u32		buf_size
);

enum rt_status halrf_psd_init_query(
	struct	dm_struct	*dm,
	u32		*outbuf,
	u32		point,
	u32		start_point,
	u32		stop_point,
	u32		average,
	u32		buf_size
);

#endif	/*#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)*/
#endif	/*#ifndef __HALRF_PSD_H__*/

