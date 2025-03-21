/******************************************************************************
 *
 * Copyright(c) 2007 - 2017  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/


#ifndef	_HALRF_H__
#define _HALRF_H__

/*============================================================*/
/*include files*/
/*============================================================*/
#include "halrf/halrf_psd.h"


/*============================================================*/
/*Definition */
/*============================================================*/
/*IQK version*/
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN))
#define IQK_VERSION_8188E	"0x14"
#define IQK_VERSION_8192E	"0x01"
#define IQK_VERSION_8723B	"0x1e"
#define IQK_VERSION_8812A	"0x01"
#define IQK_VERSION_8821A	"0x01"
#elif (DM_ODM_SUPPORT_TYPE & (ODM_CE))
#define IQK_VERSION_8188E	"0x01"
#define IQK_VERSION_8192E	"0x01"
#define IQK_VERSION_8723B	"0x1e"
#define IQK_VERSION_8812A	"0x01"
#define IQK_VERSION_8821A	"0x01"
#elif (DM_ODM_SUPPORT_TYPE & (ODM_AP))
#define IQK_VERSION_8188E	"0x01"
#define IQK_VERSION_8192E	"0x01"
#define IQK_VERSION_8723B	"0x1e"
#define IQK_VERSION_8812A	"0x01"
#define IQK_VERSION_8821A	"0x01"
#endif
#define IQK_VERSION_8814A	"0x0f"
#define IQK_VERSION_8188F	"0x01"
#define IQK_VERSION_8197F	"0x01"
#define IQK_VERSION_8703B	"0x05"
#define IQK_VERSION_8710B	"0x01"
#define IQK_VERSION_8723D	"0x02"
#define IQK_VERSION_8822B	"0x2f"
#define IQK_VERSION_8821C	"0x23"

/*LCK version*/
#define LCK_VERSION_8188E	"0x01"
#define LCK_VERSION_8192E	"0x01"
#define LCK_VERSION_8723B	"0x01"
#define LCK_VERSION_8812A	"0x01"
#define LCK_VERSION_8821A	"0x01"
#define LCK_VERSION_8814A	"0x01"
#define LCK_VERSION_8188F	"0x01"
#define LCK_VERSION_8197F	"0x01"
#define LCK_VERSION_8703B	"0x01"
#define LCK_VERSION_8710B	"0x01"
#define LCK_VERSION_8723D	"0x01"
#define LCK_VERSION_8822B	"0x01"
#define LCK_VERSION_8821C	"0x01"

/*power tracking version*/
#define POWERTRACKING_VERSION_8188E	"0x01"
#define POWERTRACKING_VERSION_8192E	"0x01"
#define POWERTRACKING_VERSION_8723B	"0x01"
#define POWERTRACKING_VERSION_8812A	"0x01"
#define POWERTRACKING_VERSION_8821A	"0x01"
#define POWERTRACKING_VERSION_8814A	"0x01"
#define POWERTRACKING_VERSION_8188F	"0x01"
#define POWERTRACKING_VERSION_8197F	"0x01"
#define POWERTRACKING_VERSION_8703B	"0x01"
#define POWERTRACKING_VERSION_8710B	"0x01"
#define POWERTRACKING_VERSION_8723D	"0x01"
#define POWERTRACKING_VERSION_8822B	"0x01"
#define POWERTRACKING_VERSION_8821C	"0x01"

/*DPK tracking version*/
#define DPK_VERSION_8188E	"NONE"
#define DPK_VERSION_8192E	"NONE"
#define DPK_VERSION_8723B	"NONE"
#define DPK_VERSION_8812A	"NONE"
#define DPK_VERSION_8821A	"NONE"
#define DPK_VERSION_8814A	"NONE"
#define DPK_VERSION_8188F	"NONE"
#define DPK_VERSION_8197F	"NONE"
#define DPK_VERSION_8703B	"NONE"
#define DPK_VERSION_8710B	"NONE"
#define DPK_VERSION_8723D	"NONE"
#define DPK_VERSION_8822B	"NONE"
#define DPK_VERSION_8821C	"NONE"

/*Kfree tracking version*/
#define KFREE_VERSION_8188E	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8192E	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8723B	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8812A	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8821A	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8814A	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8188F	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8197F	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8703B	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8710B	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8723D	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8822B	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"
#define KFREE_VERSION_8821C	(dm->power_trim_data.flag & KFREE_FLAG_ON)? "0x01" : "NONE"

/*PA Bias Calibration version*/
#define PABIASK_VERSION_8188E	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8192E	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8723B	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8812A	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8821A	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8814A	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8188F	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8197F	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8703B	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8710B	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8723D	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8822B	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"
#define PABIASK_VERSION_8821C	(dm->power_trim_data.pa_bias_flag & PA_BIAS_FLAG_ON)? "0x01" : "NONE"



#define HALRF_IQK_VER	(dm->support_ic_type == ODM_RTL8188E)? IQK_VERSION_8188E :\
						(dm->support_ic_type == ODM_RTL8192E)? IQK_VERSION_8192E :\
						(dm->support_ic_type == ODM_RTL8723B)? IQK_VERSION_8723B :\
						(dm->support_ic_type == ODM_RTL8812)? IQK_VERSION_8812A :\
						(dm->support_ic_type == ODM_RTL8821)? IQK_VERSION_8821A :\
						(dm->support_ic_type == ODM_RTL8814A)? IQK_VERSION_8814A :\
						(dm->support_ic_type == ODM_RTL8188F)? IQK_VERSION_8188F :\
						(dm->support_ic_type == ODM_RTL8197F)? IQK_VERSION_8197F :\
						(dm->support_ic_type == ODM_RTL8703B)? IQK_VERSION_8703B :\
						(dm->support_ic_type == ODM_RTL8710B)? IQK_VERSION_8710B :\
						(dm->support_ic_type == ODM_RTL8723D)? IQK_VERSION_8723D :\
						(dm->support_ic_type == ODM_RTL8822B)? IQK_VERSION_8822B :\
						(dm->support_ic_type == ODM_RTL8821C)? IQK_VERSION_8821C :"unknown"


#define HALRF_LCK_VER	(dm->support_ic_type == ODM_RTL8188E)? LCK_VERSION_8188E :\
						(dm->support_ic_type == ODM_RTL8192E)? LCK_VERSION_8192E :\
						(dm->support_ic_type == ODM_RTL8723B)? LCK_VERSION_8723B :\
						(dm->support_ic_type == ODM_RTL8812)? LCK_VERSION_8812A :\
						(dm->support_ic_type == ODM_RTL8821)? LCK_VERSION_8821A :\
						(dm->support_ic_type == ODM_RTL8814A)? LCK_VERSION_8814A :\
						(dm->support_ic_type == ODM_RTL8188F)? LCK_VERSION_8188F :\
						(dm->support_ic_type == ODM_RTL8197F)? LCK_VERSION_8197F :\
						(dm->support_ic_type == ODM_RTL8703B)? LCK_VERSION_8703B :\
						(dm->support_ic_type == ODM_RTL8710B)? LCK_VERSION_8710B :\
						(dm->support_ic_type == ODM_RTL8723D)? LCK_VERSION_8723D :\
						(dm->support_ic_type == ODM_RTL8822B)? LCK_VERSION_8822B :\
						(dm->support_ic_type == ODM_RTL8821C)? LCK_VERSION_8821C :"unknown"


#define HALRF_POWRTRACKING_VER	(dm->support_ic_type == ODM_RTL8188E)? POWERTRACKING_VERSION_8188E :\
								(dm->support_ic_type == ODM_RTL8192E)? POWERTRACKING_VERSION_8192E :\
								(dm->support_ic_type == ODM_RTL8723B)? POWERTRACKING_VERSION_8723B :\
								(dm->support_ic_type == ODM_RTL8812)? POWERTRACKING_VERSION_8812A :\
								(dm->support_ic_type == ODM_RTL8821)? POWERTRACKING_VERSION_8821A :\
								(dm->support_ic_type == ODM_RTL8814A)? POWERTRACKING_VERSION_8814A :\
								(dm->support_ic_type == ODM_RTL8188F)? POWERTRACKING_VERSION_8188F :\
								(dm->support_ic_type == ODM_RTL8197F)? POWERTRACKING_VERSION_8197F :\
								(dm->support_ic_type == ODM_RTL8703B)? POWERTRACKING_VERSION_8703B :\
								(dm->support_ic_type == ODM_RTL8710B)? POWERTRACKING_VERSION_8710B :\
								(dm->support_ic_type == ODM_RTL8723D)? POWERTRACKING_VERSION_8723D :\
								(dm->support_ic_type == ODM_RTL8822B)? POWERTRACKING_VERSION_8822B :\
								(dm->support_ic_type == ODM_RTL8821C)? POWERTRACKING_VERSION_8821C :"unknown"

#define HALRF_DPK_VER	(dm->support_ic_type == ODM_RTL8188E)? DPK_VERSION_8188E :\
						(dm->support_ic_type == ODM_RTL8192E)? DPK_VERSION_8192E :\
						(dm->support_ic_type == ODM_RTL8723B)? DPK_VERSION_8723B :\
						(dm->support_ic_type == ODM_RTL8812)? DPK_VERSION_8812A :\
						(dm->support_ic_type == ODM_RTL8821)? DPK_VERSION_8821A :\
						(dm->support_ic_type == ODM_RTL8814A)? DPK_VERSION_8814A :\
						(dm->support_ic_type == ODM_RTL8188F)? DPK_VERSION_8188F :\
						(dm->support_ic_type == ODM_RTL8197F)? DPK_VERSION_8197F :\
						(dm->support_ic_type == ODM_RTL8703B)? DPK_VERSION_8703B :\
						(dm->support_ic_type == ODM_RTL8710B)? DPK_VERSION_8710B :\
						(dm->support_ic_type == ODM_RTL8723D)? DPK_VERSION_8723D :\
						(dm->support_ic_type == ODM_RTL8822B)? DPK_VERSION_8822B :\
						(dm->support_ic_type == ODM_RTL8821C)? DPK_VERSION_8821C :"unknown"

#define HALRF_KFREE_VER (dm->support_ic_type == ODM_RTL8188E)? KFREE_VERSION_8188E :\
						(dm->support_ic_type == ODM_RTL8192E)? KFREE_VERSION_8192E :\
						(dm->support_ic_type == ODM_RTL8723B)? KFREE_VERSION_8723B :\
						(dm->support_ic_type == ODM_RTL8812)? KFREE_VERSION_8812A :\
						(dm->support_ic_type == ODM_RTL8821)? KFREE_VERSION_8821A :\
						(dm->support_ic_type == ODM_RTL8814A)? KFREE_VERSION_8814A :\
						(dm->support_ic_type == ODM_RTL8188F)? KFREE_VERSION_8188F :\
						(dm->support_ic_type == ODM_RTL8197F)? KFREE_VERSION_8197F :\
						(dm->support_ic_type == ODM_RTL8703B)? KFREE_VERSION_8703B :\
						(dm->support_ic_type == ODM_RTL8710B)? KFREE_VERSION_8710B :\
						(dm->support_ic_type == ODM_RTL8723D)? KFREE_VERSION_8723D :\
						(dm->support_ic_type == ODM_RTL8822B)? KFREE_VERSION_8822B :\
						(dm->support_ic_type == ODM_RTL8821C)? KFREE_VERSION_8821C :"unknown"

#define HALRF_PABIASK_VER	(dm->support_ic_type == ODM_RTL8188E)? PABIASK_VERSION_8188E :\
								(dm->support_ic_type == ODM_RTL8192E)? PABIASK_VERSION_8192E :\
								(dm->support_ic_type == ODM_RTL8723B)? PABIASK_VERSION_8723B :\
								(dm->support_ic_type == ODM_RTL8812)? PABIASK_VERSION_8812A :\
								(dm->support_ic_type == ODM_RTL8821)? PABIASK_VERSION_8821A :\
								(dm->support_ic_type == ODM_RTL8814A)? PABIASK_VERSION_8814A :\
								(dm->support_ic_type == ODM_RTL8188F)? PABIASK_VERSION_8188F :\
								(dm->support_ic_type == ODM_RTL8197F)? PABIASK_VERSION_8197F :\
								(dm->support_ic_type == ODM_RTL8703B)? PABIASK_VERSION_8703B :\
								(dm->support_ic_type == ODM_RTL8710B)? PABIASK_VERSION_8710B :\
								(dm->support_ic_type == ODM_RTL8723D)? PABIASK_VERSION_8723D :\
								(dm->support_ic_type == ODM_RTL8822B)? PABIASK_VERSION_8822B :\
								(dm->support_ic_type == ODM_RTL8821C)? PABIASK_VERSION_8821C :"unknown"



#define IQK_THRESHOLD			8
#define DPK_THRESHOLD			4

/*===========================================================*/
/*AGC RX High Power mode*/
/*===========================================================*/
#define	lna_low_gain_1		0x64
#define	lna_low_gain_2		0x5A
#define	lna_low_gain_3		0x58

/*============================================================*/
/* enumeration */
/*============================================================*/
enum halrf_ability {
	HAL_RF_TX_PWR_TRACK	= BIT(0),
	HAL_RF_IQK				= BIT(1),
	HAL_RF_LCK				= BIT(2),
	HAL_RF_DPK				= BIT(3),
	HAL_RF_TXGAPK			= BIT(4)
};

enum halrf_cmninfo_init {
	HALRF_CMNINFO_ABILITY = 0,
	HALRF_CMNINFO_DPK_EN = 1,
	HALRF_CMNINFO_EEPROM_THERMAL_VALUE,
	HALRF_CMNINFO_FW_VER,
	HALRF_CMNINFO_RFK_FORBIDDEN,
	HALRF_CMNINFO_IQK_SEGMENT,
	HALRF_CMNINFO_RATE_INDEX,
	HALRF_CMNINFO_MP_PSD_POINT,
	HALRF_CMNINFO_MP_PSD_START_POINT,
	HALRF_CMNINFO_MP_PSD_STOP_POINT,
	HALRF_CMNINFO_MP_PSD_AVERAGE
};

enum halrf_cmninfo_hook {
	HALRF_CMNINFO_CON_TX,
	HALRF_CMNINFO_SINGLE_TONE,
	HALRF_CMNINFO_CARRIER_SUPPRESSION,
	HALRF_CMNINFO_MP_RATE_INDEX
};

enum phydm_lna_set {
	phydm_lna_disable		= 0,
	phydm_lna_enable		= 1,
};


/*============================================================*/
/* structure */
/*============================================================*/

struct _hal_rf_ {
	/*hook*/
	u8		*test1;

	/*update*/
	u32		rf_supportability;

	u8		eeprom_thermal;
	u8		dpk_en;			/*Enable Function DPK OFF/ON = 0/1*/
	boolean	dpk_done;
	u32		fw_ver;

	boolean	*is_con_tx;
	boolean	*is_single_tone;
	boolean	*is_carrier_suppresion;

	u8		*mp_rate_index;
	u32		p_rate_index;
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
	struct	_halrf_psd_data	halrf_psd_data;
#endif
};

/*============================================================*/
/* function prototype */
/*============================================================*/

void halrf_basic_profile(
	void			*dm_void,
	u32			*_used,
	char			*output,
	u32			*_out_len
);
#if (RTL8822B_SUPPORT == 1 || RTL8821C_SUPPORT == 1)
void halrf_iqk_info_dump(
	void *dm_void,
	u32 *_used,
	char *output,
	u32 *_out_len
);

void halrf_iqk_hwtx_check(
	void *dm_void,
	boolean		is_check
);
#endif

u8 halrf_match_iqk_version(
	void	*dm_void
);

void halrf_support_ability_debug(
	void		*dm_void,
	char		input[][16],
	u32		*_used,
	char		*output,
	u32		*_out_len
);

void halrf_cmn_info_init(
	void		*dm_void,
	enum halrf_cmninfo_init	cmn_info,
	u32		value
);

void halrf_cmn_info_hook(
	void		*dm_void,
	u32		cmn_info,
	void		*value
);

void halrf_cmn_info_set(
	void		*dm_void,
	u32			cmn_info,
	u64			value
);

u64 halrf_cmn_info_get(
	void		*dm_void,
	u32			cmn_info
);

void halrf_watchdog(
	void			*dm_void
);

void halrf_supportability_init(
	void		*dm_void
);

void halrf_init(
	void			*dm_void
);

void halrf_iqk_trigger(
	void			*dm_void,
	boolean		is_recovery
);

void halrf_segment_iqk_trigger(
	void			*dm_void,
	boolean		clear,
	boolean		segment_iqk
);

void halrf_lck_trigger(
	void			*dm_void
);

void halrf_iqk_debug(
	void		*dm_void,
	u32		*const dm_value,
	u32		*_used,
	char		*output,
	u32		*_out_len
);

void phydm_get_iqk_cfir(
	void		*dm_void,
	u8 idx,
	u8 path,
	boolean debug
);

void halrf_iqk_xym_read(
	void *dm_void,
	u8 path,
	u8 xym_type
);

void halrf_rf_lna_setting(
	void	*dm_void,
	enum phydm_lna_set type
);


void halrf_do_imr_test(
	void	*dm_void,
	u8 data
);

u32 halrf_psd_log2base(
	u32 val
);


#if (RTL8822B_SUPPORT == 1 || RTL8821C_SUPPORT == 1)
void halrf_iqk_dbg(void	*dm_void);
#endif
#endif


