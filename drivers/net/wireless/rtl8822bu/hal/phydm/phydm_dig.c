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

/* ************************************************************
 * include files
 * ************************************************************ */
#include "mp_precomp.h"
#include "phydm_precomp.h"


boolean phydm_dig_go_up_check(
	void		*dm_void
)
{
	struct dm_struct		*dm = (struct dm_struct *)dm_void;
	struct ccx_info			*ccx_info = &dm->dm_ccx_info;
	struct phydm_dig_struct		*dig_t = &dm->dm_dig_table;
	u8		cur_ig_value = dig_t->cur_ig_value;
	u8		max_cover_bond;
	u8		rx_gain_range_max = dig_t->rx_gain_range_max;
	u8		i = 0, j = 0;
	u8		total_nhm_cnt = ccx_info->nhm_rpt_sum;
	u32		dig_cover_cnt = 0;
	u32		over_dig_cover_cnt = 0;
	boolean		ret = true;

	if (*dm->bb_op_mode == PHYDM_PERFORMANCE_MODE)
		return ret;

	max_cover_bond = DIG_MAX_BALANCE_MODE - dig_t->dig_upcheck_initial_value;

	if (cur_ig_value < max_cover_bond - 6)
		dig_t->dig_go_up_check_level = DIG_GOUPCHECK_LEVEL_0;
	else if (cur_ig_value <= DIG_MAX_BALANCE_MODE)
		dig_t->dig_go_up_check_level = DIG_GOUPCHECK_LEVEL_1;
	else	/* cur_ig_value > DM_DIG_MAX_AP, foolproof */
		dig_t->dig_go_up_check_level = DIG_GOUPCHECK_LEVEL_2;


	PHYDM_DBG(dm, DBG_DIG, "check_lv = %d, max_cover_bond = 0x%x\n",
		  dig_t->dig_go_up_check_level,
		  max_cover_bond);

	if (total_nhm_cnt == 0)
		return true;

	if (dig_t->dig_go_up_check_level == DIG_GOUPCHECK_LEVEL_0) {
		for (i = 3; i <= 11; i++)
			dig_cover_cnt += ccx_info->nhm_result[i];
		ret = ((dig_t->dig_level0_ratio_reciprocal * dig_cover_cnt) >=
		       total_nhm_cnt) ? true : false;
	} else if (dig_t->dig_go_up_check_level == DIG_GOUPCHECK_LEVEL_1) {

		/* search index */
		for (i = 0; i <= 10; i++) {
			if ((max_cover_bond * 2) == ccx_info->nhm_th[i]) {
				for (j = (i + 1); j <= 11; j++)
					over_dig_cover_cnt += ccx_info->nhm_result[j];
				break;
			}
		}
		ret = (dig_t->dig_level1_ratio_reciprocal * over_dig_cover_cnt <
		       total_nhm_cnt) ? true : false;

		if (!ret) {
			/* update dig_t->rx_gain_range_max */
			dig_t->rx_gain_range_max = (rx_gain_range_max >= max_cover_bond - 6) ?
						   (max_cover_bond - 6) : rx_gain_range_max;

			PHYDM_DBG(dm, DBG_DIG,
				  "Noise pwr over DIG can filter, lock rx_gain_range_max to 0x%x\n",
				  dig_t->rx_gain_range_max);
		}
	} else if (dig_t->dig_go_up_check_level == DIG_GOUPCHECK_LEVEL_2) {
		/* cur_ig_value > DM_DIG_MAX_AP, foolproof */
		ret = true;
	}

	return ret;
}

void odm_fa_threshold_check(
	void			*dm_void,
	boolean			is_dfs_band,
	boolean			is_performance
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;

	if (dig_t->is_dbg_fa_th) {

		PHYDM_DBG(dm, DBG_DIG, "Manual Fix FA_th\n");

	} else if (dm->is_linked && (is_performance || is_dfs_band)) {
		if (dm->rssi_min < 20) {	/*[PHYDM-252]*/
			dig_t->fa_th[0] = 500;
			dig_t->fa_th[1] = 750;
			dig_t->fa_th[2] = 1000;
		} else if (((dm->rx_tp >> 2) > dm->tx_tp) && /*Test RX TP*/
			   (dm->rx_tp < 10) && (dm->rx_tp > 1)) {	/*RXTP = 1 ~ 10Mbps*/
			dig_t->fa_th[0] = 125;
			dig_t->fa_th[1] = 250;
			dig_t->fa_th[2] = 500;
		} else {
			dig_t->fa_th[0] = 250;
			dig_t->fa_th[1] = 500;
			dig_t->fa_th[2] = 750;
		}
	} else {
		if (is_dfs_band) {	/* For DFS band and no link */

			dig_t->fa_th[0] = 250;
			dig_t->fa_th[1] = 1000;
			dig_t->fa_th[2] = 2000;
		} else {
			dig_t->fa_th[0] = 2000;
			dig_t->fa_th[1] = 4000;
			dig_t->fa_th[2] = 5000;
		}
	}

	PHYDM_DBG(dm, DBG_DIG, "FA_th={%d,%d,%d}\n",
		  dig_t->fa_th[0], dig_t->fa_th[1], dig_t->fa_th[2]);

}

void phydm_set_big_jump_step(
	void			*dm_void,
	u8			current_igi
)
{
#if (RTL8822B_SUPPORT == 1 || RTL8197F_SUPPORT == 1)
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	u8		step1[8] = {24, 30, 40, 50, 60, 70, 80, 90};
	u8		i;

	if (dig_t->enable_adjust_big_jump == 0)
		return;

	for (i = 0; i <= dig_t->big_jump_step1; i++) {
		if ((current_igi + step1[i]) > dig_t->big_jump_lmt[dig_t->agc_table_idx]) {
			if (i != 0)
				i = i - 1;
			break;
		} else if (i == dig_t->big_jump_step1)
			break;
	}
	if (dm->support_ic_type & ODM_RTL8822B)
		odm_set_bb_reg(dm, 0x8c8, 0xe, i);
	else if (dm->support_ic_type & ODM_RTL8197F)
		odm_set_bb_reg(dm, ODM_REG_BB_AGC_SET_2_11N, 0xe, i);

	PHYDM_DBG(dm, DBG_DIG,
		  "phydm_set_big_jump_step(): bigjump = %d (ori = 0x%x), LMT=0x%x\n",
		  i, dig_t->big_jump_step1, dig_t->big_jump_lmt[dig_t->agc_table_idx]);
#endif
}

void odm_write_dig(
	void			*dm_void,
	u8			current_igi
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	struct phydm_adaptivity_struct	*adaptivity = (struct
			phydm_adaptivity_struct *)phydm_get_structure(dm, PHYDM_ADAPTIVITY);

	PHYDM_DBG(dm, DBG_DIG, "odm_write_dig===>\n");

	/* 1 Check IGI by upper bound */
	if (adaptivity->igi_lmt_en &&
	    (current_igi > adaptivity->adapt_igi_up) && dm->is_linked) {

		current_igi = adaptivity->adapt_igi_up;

		PHYDM_DBG(dm, DBG_DIG,
			  "Force to Adaptivity Upper bound=((0x%x))\n", current_igi);
	}

	if (dig_t->cur_ig_value != current_igi) {
#if (RTL8822B_SUPPORT == 1 || RTL8197F_SUPPORT == 1)
		/* Modify big jump step for 8822B and 8197F */
		if (dm->support_ic_type & (ODM_RTL8822B | ODM_RTL8197F))
			phydm_set_big_jump_step(dm, current_igi);
#endif

#if (ODM_PHY_STATUS_NEW_TYPE_SUPPORT == 1)
		/* Set IGI value of CCK for new CCK AGC */
		if (dm->cck_new_agc && (dm->support_ic_type & PHYSTS_2ND_TYPE_IC))
			odm_set_bb_reg(dm, 0xa0c, 0x3f00, (current_igi >> 1));
#endif

		/*Add by YuChen for USB IO too slow issue*/
		if (dm->support_ic_type &
		    (ODM_IC_11AC_GAIN_IDX_EDCCA | ODM_IC_11N_GAIN_IDX_EDCCA)) {
			if ((dm->support_ability & ODM_BB_ADAPTIVITY) &&
			    (current_igi < dig_t->cur_ig_value)) {
				dig_t->cur_ig_value = current_igi;
				phydm_adaptivity(dm);
			}
		} else {
			if ((dm->support_ability & ODM_BB_ADAPTIVITY) &&
			    (current_igi > dig_t->cur_ig_value)) {
				dig_t->cur_ig_value = current_igi;
				phydm_adaptivity(dm);
			}
		}

		/* Set IGI value */
		odm_set_bb_reg(dm, ODM_REG(IGI_A, dm), ODM_BIT(IGI, dm), current_igi);

#if (defined(PHYDM_COMPILE_ABOVE_2SS))
		if (dm->support_ic_type & PHYDM_IC_ABOVE_2SS)
			odm_set_bb_reg(dm, ODM_REG(IGI_B, dm), ODM_BIT(IGI, dm), current_igi);
#endif

#if (defined(PHYDM_COMPILE_ABOVE_4SS))
		if (dm->support_ic_type & PHYDM_IC_ABOVE_4SS) {
			odm_set_bb_reg(dm, ODM_REG(IGI_C, dm), ODM_BIT(IGI, dm), current_igi);
			odm_set_bb_reg(dm, ODM_REG(IGI_D, dm), ODM_BIT(IGI, dm), current_igi);
		}
#endif

		dig_t->cur_ig_value = current_igi;
	}

	PHYDM_DBG(dm, DBG_DIG, "New_igi=((0x%x))\n\n", current_igi);
}

void phydm_set_dig_val(
	void			*dm_void,
	u32			*val_buf,
	u8			val_len
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;

	if (val_len != 1) {
		PHYDM_DBG(dm, ODM_COMP_API, "[Error][DIG]Need val_len=1\n");
		return;
	}

	odm_write_dig(dm, (u8)(*val_buf));
}

void odm_pause_dig(
	void					*dm_void,
	enum phydm_pause_type		type,
	enum phydm_pause_level		lv,
	u8				igi_input
)
{
	struct	dm_struct	*dm = (struct dm_struct *)dm_void;
	u8	rpt = false;
	u32	igi = (u32)igi_input;

	PHYDM_DBG(dm, DBG_DIG, "[%s]type = %s, LV = %d, igi = 0x%x\n",
		  __func__,
		  ((type == PHYDM_PAUSE) ? "Pause" : ((type == PHYDM_RESUME) ? "Resume" :
				  "PauseNoSet")),
		  lv, igi);

	switch (type) {

	case PHYDM_PAUSE:
	case PHYDM_PAUSE_NO_SET: {
		rpt = phydm_pause_func(dm, F00_DIG, PHYDM_PAUSE, lv, 1, &igi);
		break;
	}

	case PHYDM_RESUME: {
		rpt = phydm_pause_func(dm, F00_DIG, PHYDM_RESUME, lv, 1, &igi);
		break;
	}
	default:
		PHYDM_DBG(dm, DBG_DIG, "Wrong type\n");
		break;
	}

	PHYDM_DBG(dm, DBG_DIG, "pause_result=%d\n", rpt);
}


boolean odm_dig_abort(
	void			*dm_void
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
	void		*adapter	= dm->adapter;
#endif

	/* support_ability */
	if ((!(dm->support_ability & ODM_BB_FA_CNT)) ||
	    (!(dm->support_ability & ODM_BB_DIG)) ||
	    *dm->is_scan_in_process) {
		PHYDM_DBG(dm, DBG_DIG, "Not Support\n");
		return true;
	}

	if (dm->pause_ability & ODM_BB_DIG) {

		PHYDM_DBG(dm, DBG_DIG, "Return: Pause DIG in LV=%d\n",
			  dm->pause_lv_table.lv_dig);
		return true;
	}

	if (dig_t->is_ignore_dig) {
		dig_t->is_ignore_dig = false;
		PHYDM_DBG(dm, DBG_DIG, "Return: Ignore DIG\n");
		return true;
	}

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
#if OS_WIN_FROM_WIN7(OS_VERSION)
	if (IsAPModeExist(adapter) && ((PADAPTER)(adapter))->bInHctTest) {
		PHYDM_DBG(dm, DBG_DIG, " Return: Is AP mode or In HCT Test\n");
		return true;
	}
#endif
#endif

	return false;
}

void phydm_dig_init(
	void		*dm_void
)
{
	struct dm_struct		*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct		*dig_t = &dm->dm_dig_table;
#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
	struct phydm_fa_struct	*false_alm_cnt = (struct phydm_fa_struct *)
			phydm_get_structure(dm, PHYDM_FALSEALMCNT);
#endif
	u32			ret_value = 0;
	u8			i;

	dig_t->dm_dig_max = DIG_MAX_BALANCE_MODE;
	dig_t->dm_dig_min = DIG_MIN_PERFORMANCE;
	dig_t->dig_max_of_min = DIG_MAX_OF_MIN_BALANCE_MODE;

	dig_t->is_ignore_dig = false;
	dig_t->cur_ig_value = (u8) odm_get_bb_reg(dm, ODM_REG(IGI_A, dm),
			      ODM_BIT(IGI, dm));
	dig_t->is_media_connect = false;

	dig_t->fa_th[0] = 250;
	dig_t->fa_th[1] = 500;
	dig_t->fa_th[2] = 750;
	dig_t->is_dbg_fa_th = false;
#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
	/* For RTL8881A */
	false_alm_cnt->cnt_ofdm_fail_pre = 0;
#endif

	odm_memory_set(dm, dig_t->pause_dig_value, 0, PHYDM_PAUSE_MAX_NUM);
	dig_t->pause_lv_bitmap = 0;

	dig_t->rx_gain_range_max = DIG_MAX_BALANCE_MODE;
	dig_t->rx_gain_range_min = dig_t->cur_ig_value;

#if (RTL8822B_SUPPORT == 1 || RTL8197F_SUPPORT == 1)
	dig_t->enable_adjust_big_jump = 1;
	if (dm->support_ic_type & ODM_RTL8822B)
		ret_value = odm_get_bb_reg(dm, 0x8c8, MASKLWORD);
	else if (dm->support_ic_type & ODM_RTL8197F)
		ret_value = odm_get_bb_reg(dm, 0xc74, MASKLWORD);

	dig_t->big_jump_step1 = (u8)(ret_value & 0xe) >> 1;
	dig_t->big_jump_step2 = (u8)(ret_value & 0x30) >> 4;
	dig_t->big_jump_step3 = (u8)(ret_value & 0xc0) >> 6;

	if (dm->support_ic_type & (ODM_RTL8822B | ODM_RTL8197F)) {
		for (i = 0; i < sizeof(dig_t->big_jump_lmt); i++) {
			if (dig_t->big_jump_lmt[i] == 0)
				dig_t->big_jump_lmt[i] = 0x64;		/* Set -10dBm as default value */
		}
	}
#endif

	dm->pre_rssi_min = 0;

#ifdef PHYDM_TDMA_DIG_SUPPORT
	dm->original_dig_restore = 1;
#endif
}

boolean phydm_dig_performance_mode_decision(
	struct dm_struct		*dm
)
{
	boolean	is_performance = true;

#ifdef PHYDM_DIG_MODE_DECISION_SUPPORT
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;

	switch (dig_t->dig_mode_decision) {
	case PHYDM_DIG_PERFORAMNCE_MODE:
		is_performance = true;
		break;
	case PHYDM_DIG_COVERAGE_MODE:
		is_performance = false;
		break;
	default:
		is_performance = true;
		break;
	}
#endif

	return is_performance;
}

void phydm_dig_abs_boundary_decision(
	struct dm_struct		*dm,
	boolean	is_performance,
	boolean	is_dfs_band
)
{
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;

	if (!dm->is_linked) {
		dig_t->dm_dig_max = DIG_MAX_COVERAGR;
		dig_t->dm_dig_min = DIG_MIN_COVERAGE;
	} else if (is_dfs_band == true) {
		if (*dm->band_width == CHANNEL_WIDTH_20)
			dig_t->dm_dig_min = DIG_MIN_DFS + 2;
		else
			dig_t->dm_dig_min = DIG_MIN_DFS;

		dig_t->dig_max_of_min = DIG_MAX_OF_MIN_BALANCE_MODE;
		dig_t->dm_dig_max = DIG_MAX_BALANCE_MODE;

	} else if (!is_performance) {
		dig_t->dm_dig_max = DIG_MAX_COVERAGR;
		dig_t->dm_dig_min = DIG_MIN_COVERAGE;
#if (DIG_HW == 1)
		dig_t->dig_max_of_min = DIG_MIN_COVERAGE;
#else
		dig_t->dig_max_of_min = DIG_MAX_OF_MIN_COVERAGE;
#endif
	} else {
		if (*dm->bb_op_mode == PHYDM_BALANCE_MODE) {	/*service > 2 devices*/
			dig_t->dm_dig_max = DIG_MAX_BALANCE_MODE;
#if (DIG_HW == 1)
			dig_t->dig_max_of_min = DIG_MIN_COVERAGE;
#else
			dig_t->dig_max_of_min = DIG_MAX_OF_MIN_BALANCE_MODE;
#endif
		} else if (*dm->bb_op_mode ==
			   PHYDM_PERFORMANCE_MODE) {	/*service 1 devices*/
			dig_t->dm_dig_max = DIG_MAX_PERFORMANCE_MODE;
			dig_t->dig_max_of_min = DIG_MAX_OF_MIN_PERFORMANCE_MODE;
		}

		if (dm->support_ic_type &
		    (ODM_RTL8814A | ODM_RTL8812 | ODM_RTL8821 | ODM_RTL8822B))
			dig_t->dm_dig_min = 0x1c;
		else if (dm->support_ic_type & ODM_RTL8197F)
			dig_t->dm_dig_min = 0x1e;		/*For HW setting*/
		else
			dig_t->dm_dig_min = DIG_MIN_PERFORMANCE;
	}

	PHYDM_DBG(dm, DBG_DIG,
		  "Abs-bound{Max, Min}={0x%x, 0x%x}, Max_of_min =  0x%x\n",
		  dig_t->dm_dig_max,
		  dig_t->dm_dig_min,
		  dig_t->dig_max_of_min);

}

void phydm_dig_dym_boundary_decision(
	struct dm_struct		*dm,
	boolean	is_performance
)
{
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	u8 offset = 15, tmp_max = 0;
	u8 max_of_rssi_min = 0;

	PHYDM_DBG(dm, DBG_DIG,
		  "Offset=((%d))\n", offset);

	/* DIG lower bound */
	if (dm->rssi_min > dig_t->dig_max_of_min)
		dig_t->rx_gain_range_min = dig_t->dig_max_of_min;
	else if (dm->rssi_min < dig_t->dm_dig_min)
		dig_t->rx_gain_range_min = dig_t->dm_dig_min;
	else
		dig_t->rx_gain_range_min = dm->rssi_min;

	/* DIG upper bound */
	tmp_max = dig_t->rx_gain_range_min + offset;
	if (dig_t->rx_gain_range_min != dm->rssi_min) {
		max_of_rssi_min = dm->rssi_min + offset;
		if (tmp_max > max_of_rssi_min)
			tmp_max = max_of_rssi_min;
	}

	if (tmp_max > dig_t->dm_dig_max)
		dig_t->rx_gain_range_max = dig_t->dm_dig_max;
	else
		dig_t->rx_gain_range_max = tmp_max;

	/* 1 Force Lower Bound for AntDiv */
	if (dm->is_one_entry_only != 0)
		goto out;

	if ((dm->support_ic_type & ODM_ANTDIV_SUPPORT)
	    && (dm->support_ability & ODM_BB_ANT_DIV)) {
		if (dm->ant_div_type == CG_TRX_HW_ANTDIV
		    || dm->ant_div_type == CG_TRX_SMART_ANTDIV) {
			if (dig_t->ant_div_rssi_max > dig_t->dig_max_of_min)
				dig_t->rx_gain_range_min = dig_t->dig_max_of_min;
			else
				dig_t->rx_gain_range_min = (u8)dig_t->ant_div_rssi_max;

			PHYDM_DBG(dm, DBG_DIG,
				  "AntDiv: Force Dyn-Min = 0x%x, RSSI_max = 0x%x\n",
				  dig_t->rx_gain_range_min, dig_t->ant_div_rssi_max);
		}
	}

out:
	PHYDM_DBG(dm, DBG_DIG,
		  "Dym-bound{Max, Min}={0x%x, 0x%x}\n",
		  dig_t->rx_gain_range_max, dig_t->rx_gain_range_min);
}

void phydm_dig_abnormal_case(
	struct dm_struct		*dm,
	u8	current_igi,
	boolean	is_performance,
	boolean	is_dfs_band
)
{
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	boolean	first_connect = false, first_dis_connect = false;

	first_connect = (dm->is_linked) && !dig_t->is_media_connect;
	first_dis_connect = (!dm->is_linked) && dig_t->is_media_connect;

	/* Modify DIG lower bound, deal with abnormal case */
	if (!dm->is_linked && is_dfs_band && is_performance) {
		dig_t->rx_gain_range_max = DIG_MAX_DFS;
		PHYDM_DBG(dm, DBG_DIG,
			  "DFS band: Force max to 0x%x before link\n", dig_t->rx_gain_range_max);
	}

	if (is_dfs_band)
		dig_t->rx_gain_range_min = dig_t->dm_dig_min;

	/* Abnormal lower bound case */
	if (dig_t->rx_gain_range_min > dig_t->rx_gain_range_max)
		dig_t->rx_gain_range_min = dig_t->rx_gain_range_max;

	PHYDM_DBG(dm, DBG_DIG,
		  "Abnoraml checked {Max, Min}={0x%x, 0x%x}\n",
		  dig_t->rx_gain_range_max, dig_t->rx_gain_range_min);

}

u8 phydm_dig_current_igi_by_fa_th(
	struct dm_struct		*dm,
	u8			current_igi,
	u32			false_alm_cnt,
	u8			*step_size
)
{
	boolean	dig_go_up_check = true;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;

	dig_go_up_check = phydm_dig_go_up_check(dm);

	if ((false_alm_cnt > dig_t->fa_th[2]) && dig_go_up_check)
		current_igi = current_igi + step_size[0];
	else if ((false_alm_cnt > dig_t->fa_th[1]) && dig_go_up_check)
		current_igi = current_igi + step_size[1];
	else if (false_alm_cnt < dig_t->fa_th[0])
		current_igi = current_igi - step_size[2];

	return current_igi;

}

u8 phydm_dig_igi_start_value(
	struct dm_struct		*dm,
	boolean	is_performance,
	u8		current_igi,
	u32		false_alm_cnt,
	boolean	is_dfs_band
)
{
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	u8		step_size[3] = {0};
	boolean	first_connect = false, first_dis_connect = false;

	first_connect = (dm->is_linked) && !dig_t->is_media_connect;
	first_dis_connect = (!dm->is_linked) && dig_t->is_media_connect;

	if (dm->is_linked) {
		if (dm->pre_rssi_min <= dm->rssi_min) {
			step_size[0] = 2;
			step_size[1] = 1;
			step_size[2] = 2;
		} else {
			step_size[0] = 4;
			step_size[1] = 2;
			step_size[2] = 2;
		}
		dm->pre_rssi_min = dm->rssi_min;
	} else {
		step_size[0] = 2;
		step_size[1] = 1;
		step_size[2] = 2;
	}

	PHYDM_DBG(dm, DBG_DIG,
		  "step_size = {-%d,  +%d, +%d}\n", step_size[2], step_size[1],
		  step_size[0]);

	PHYDM_DBG(dm, DBG_DIG,
		  "rssi_min = %d, pre_rssi_min = %d\n", dm->rssi_min, dm->pre_rssi_min);

	if (dm->is_linked && is_performance) {
		/* 2 After link */
		PHYDM_DBG(dm, DBG_DIG, "Adjust IGI after link\n");

		if (first_connect && is_performance) {
			if (is_dfs_band) {
				if (dm->rssi_min > DIG_MAX_DFS)
					current_igi = DIG_MAX_DFS;
				else
					current_igi = dm->rssi_min;
				PHYDM_DBG(dm, DBG_DIG,
					  "DFS band: one shot IGI to 0x%x most\n", dig_t->rx_gain_range_max);
			} else
				current_igi = dig_t->rx_gain_range_min;
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
#if (RTL8812A_SUPPORT == 1)
			if (dm->support_ic_type == ODM_RTL8812)
				odm_config_bb_with_header_file(dm, CONFIG_BB_AGC_TAB_DIFF);
#endif
#endif
			PHYDM_DBG(dm, DBG_DIG,
				  "First connect case: IGI does on-shot to 0x%x\n", current_igi);
		} else {
			/* 4 Abnormal # beacon case */
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
			if ((dm->phy_dbg_info.num_qry_beacon_pkt < 5) &&
			    (false_alm_cnt < DM_DIG_FA_TH1) && (dm->bsta_state)) {
				if (dm->support_ic_type != ODM_RTL8723D) {
					dig_t->rx_gain_range_min = 0x1c;
					current_igi = dig_t->rx_gain_range_min;
					PHYDM_DBG(dm, DBG_DIG,
						  "Abnormal #beacon (%d) case: IGI does one-shot to 0x%x\n",
						  dm->phy_dbg_info.num_qry_beacon_pkt, current_igi);
				}
			} else
#endif
				current_igi = phydm_dig_current_igi_by_fa_th(dm,
						current_igi, false_alm_cnt, step_size);
		}
	} else {
		/* 2 Before link */
		PHYDM_DBG(dm, DBG_DIG, "Adjust IGI before link\n");

		if (first_dis_connect) {
			current_igi = dig_t->dm_dig_min;
			PHYDM_DBG(dm, DBG_DIG,
				  "First disconnect case: IGI does on-shot to lower bound\n");
		} else {
			PHYDM_DBG(dm, DBG_DIG,
				  "Pre_IGI=((0x%x)), FA=((%d))\n", current_igi, false_alm_cnt);

			current_igi = phydm_dig_current_igi_by_fa_th(dm,
					current_igi, false_alm_cnt, step_size);
		}
	}

	return current_igi;

}

void phydm_dig(
	void		*dm_void
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	struct phydm_fa_struct		*falm_cnt = &dm->false_alm_cnt;
#ifdef PHYDM_TDMA_DIG_SUPPORT
	struct phydm_fa_acc_struct *falm_cnt_acc = &dm->false_alm_cnt_acc;
#endif
	boolean		first_connect, first_dis_connect;
	u8			current_igi = dig_t->cur_ig_value;
	u32			false_alm_cnt = falm_cnt->cnt_all;
	boolean		is_dfs_band = false, is_performance = true;

#ifdef PHYDM_TDMA_DIG_SUPPORT
	if (dm->original_dig_restore == 0) {
		if (dig_t->cur_ig_value_tdma == 0)
			dig_t->cur_ig_value_tdma = dig_t->cur_ig_value;

		current_igi = dig_t->cur_ig_value_tdma;
		false_alm_cnt = falm_cnt_acc->cnt_all_1sec;
	}
#endif

	if (odm_dig_abort(dm) == true) {
		dig_t->cur_ig_value = (u8)odm_get_bb_reg(dm, 0xc50, 0x7f);
		return;
	}

	PHYDM_DBG(dm, DBG_DIG, "%s Start===>\n", __func__);

	/* 1 Update status */
	first_connect = (dm->is_linked) && !dig_t->is_media_connect;
	first_dis_connect = (!dm->is_linked) && dig_t->is_media_connect;

	PHYDM_DBG(dm, DBG_DIG,
		  "is_linked = %d, RSSI = %d, 1stConnect = %d, 1stDisconnect = %d\n",
		  dm->is_linked, dm->rssi_min, first_connect, first_dis_connect);

#if (DM_ODM_SUPPORT_TYPE & (ODM_AP | ODM_CE))
	/* Modify lower bound for DFS band */
	if (dm->is_dfs_band) {
#if (DM_ODM_SUPPORT_TYPE & (ODM_CE))
		if (phydm_dfs_master_enabled(dm))
#endif
			is_dfs_band = true;

		PHYDM_DBG(dm, DBG_DIG, "In DFS band\n");
	}
#endif

	is_performance = phydm_dig_performance_mode_decision(dm);
	PHYDM_DBG(dm, DBG_DIG,
		  "DIG ((%s)) mode\n", (is_performance ? "Performance" : "Coverage"));

	/* Boundary Decision */
	phydm_dig_abs_boundary_decision(dm, is_performance, is_dfs_band);

	/*init dym boundary*/
	dig_t->rx_gain_range_max =
		dig_t->dig_max_of_min;	/*if no link, always stay at lower bound*/
	dig_t->rx_gain_range_min = dig_t->dm_dig_min;

	/* Adjust boundary by RSSI */
	if (dm->is_linked)
		phydm_dig_dym_boundary_decision(dm, is_performance);

	/*Abnormal case check*/
	phydm_dig_abnormal_case(dm, current_igi, is_performance, is_dfs_band);

	/* False alarm threshold decision */
	odm_fa_threshold_check(dm, is_dfs_band, is_performance);

	/* 1 Adjust initial gain by false alarm */
	current_igi = phydm_dig_igi_start_value(dm,
						is_performance, current_igi, false_alm_cnt, is_dfs_band);

	/* 1 Check initial gain by upper/lower bound */
	if (current_igi < dig_t->rx_gain_range_min)
		current_igi = dig_t->rx_gain_range_min;

	if (current_igi > dig_t->rx_gain_range_max)
		current_igi = dig_t->rx_gain_range_max;

	PHYDM_DBG(dm, DBG_DIG, "New_IGI=((0x%x))\n", current_igi);

	/* 1 Update status */
#ifdef PHYDM_TDMA_DIG_SUPPORT
	if (dm->original_dig_restore == 0) {
		dig_t->cur_ig_value_tdma = current_igi;
		/*It is possible fa_acc_1sec_tsf >= */
		/*1sec while tdma_dig_state == 0*/
		if (dig_t->tdma_dig_state != 0)
			odm_write_dig(dm, dig_t->cur_ig_value_tdma);
	} else
#endif
		odm_write_dig(dm, current_igi);

	dig_t->is_media_connect = dm->is_linked;

	PHYDM_DBG(dm, DBG_DIG, "DIG end\n");
}

void phydm_dig_lps_32k(
	void		*dm_void
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	u8	current_igi = dm->rssi_min;


	odm_write_dig(dm, current_igi);
}

void phydm_dig_by_rssi_lps(
	void		*dm_void
)
{
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_fa_struct	*falm_cnt;

	u8	rssi_lower = DIG_MIN_LPS; /* 0x1E or 0x1C */
	u8	current_igi = dm->rssi_min;

	falm_cnt = &dm->false_alm_cnt;
	if (odm_dig_abort(dm) == true)
		return;

	current_igi = current_igi + RSSI_OFFSET_DIG_LPS;
	PHYDM_DBG(dm, DBG_DIG, "%s==>\n", __func__);

	/* Using FW PS mode to make IGI */
	/* Adjust by  FA in LPS MODE */
	if (falm_cnt->cnt_all > DM_DIG_FA_TH2_LPS)
		current_igi = current_igi + 4;
	else if (falm_cnt->cnt_all > DM_DIG_FA_TH1_LPS)
		current_igi = current_igi + 2;
	else if (falm_cnt->cnt_all < DM_DIG_FA_TH0_LPS)
		current_igi = current_igi - 2;


	/* Lower bound checking */

	/* RSSI Lower bound check */
	if ((dm->rssi_min - 10) > DIG_MIN_LPS)
		rssi_lower = (dm->rssi_min - 10);
	else
		rssi_lower = DIG_MIN_LPS;

	/* Upper and Lower Bound checking */
	if (current_igi > DIG_MAX_LPS)
		current_igi = DIG_MAX_LPS;
	else if (current_igi < rssi_lower)
		current_igi = rssi_lower;

	PHYDM_DBG(dm, DBG_DIG,
		  "%s falm_cnt->cnt_all = %d\n", __func__,
		  falm_cnt->cnt_all);
	PHYDM_DBG(dm, DBG_DIG,
		  "%s dm->rssi_min = %d\n", __func__,
		  dm->rssi_min);
	PHYDM_DBG(dm, DBG_DIG,
		  "%s current_igi = 0x%x\n", __func__,
		  current_igi);

	/* odm_write_dig(dm, dig_t->cur_ig_value); */
	odm_write_dig(dm, current_igi);
#endif
}

/* 3============================================================
 * 3 FASLE ALARM CHECK
 * 3============================================================ */
void phydm_false_alarm_counter_reg_reset(
	void					*dm_void
)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	struct phydm_fa_struct *falm_cnt = &dm->false_alm_cnt;
#ifdef PHYDM_TDMA_DIG_SUPPORT
	struct phydm_fa_acc_struct *falm_cnt_acc = &dm->false_alm_cnt_acc;
#endif
	u32	false_alm_cnt;

#ifdef PHYDM_TDMA_DIG_SUPPORT
	if (dm->original_dig_restore == 0) {
		if (dig_t->cur_ig_value_tdma == 0)
			dig_t->cur_ig_value_tdma = dig_t->cur_ig_value;

		false_alm_cnt = falm_cnt_acc->cnt_all_1sec;
	} else
#endif
	{
		false_alm_cnt = falm_cnt->cnt_all;
	}

#if (ODM_IC_11N_SERIES_SUPPORT == 1)
	if (dm->support_ic_type & ODM_IC_11N_SERIES) {
		/*reset false alarm counter registers*/
		odm_set_bb_reg(dm, 0xC0C, BIT(31), 1);
		odm_set_bb_reg(dm, 0xC0C, BIT(31), 0);
		odm_set_bb_reg(dm, 0xD00, BIT(27), 1);
		odm_set_bb_reg(dm, 0xD00, BIT(27), 0);

		/*update ofdm counter*/
		/*update page C counter*/
		odm_set_bb_reg(dm, 0xD00, BIT(31), 0);
		/*update page D counter*/
		odm_set_bb_reg(dm, 0xD00, BIT(31), 0);

		/*reset CCK CCA counter*/
		odm_set_bb_reg(dm, 0xA2C, BIT(13) | BIT(12), 0);
		odm_set_bb_reg(dm, 0xA2C, BIT(13) | BIT(12), 2);

		/*reset CCK FA counter*/
		odm_set_bb_reg(dm, 0xA2C, BIT(15) | BIT(14), 0);
		odm_set_bb_reg(dm, 0xA2C, BIT(15) | BIT(14), 2);

		/*reset CRC32 counter*/
		odm_set_bb_reg(dm, 0xF14, BIT(16), 1);
		odm_set_bb_reg(dm, 0xF14, BIT(16), 0);
	}
#endif	/* #if (ODM_IC_11N_SERIES_SUPPORT == 1) */

#if (ODM_IC_11AC_SERIES_SUPPORT == 1)
	if (dm->support_ic_type & ODM_IC_11AC_SERIES) {
#if (RTL8881A_SUPPORT == 1)
		/* Reset FA counter by enable/disable OFDM */
		if (false_alm_cnt->cnt_ofdm_fail_pre >= 0x7fff) {
			/* reset OFDM */
			odm_set_bb_reg(dm, 0x808, BIT(29), 0);
			odm_set_bb_reg(dm, 0x808, BIT(29), 1);
			false_alm_cnt->cnt_ofdm_fail_pre = 0;
			PHYDM_DBG(dm, DBG_FA_CNT, "Reset FA_cnt\n");
		}
#endif	/* #if (RTL8881A_SUPPORT == 1) */
		/* reset OFDM FA countner */
		odm_set_bb_reg(dm, 0x9A4, BIT(17), 1);
		odm_set_bb_reg(dm, 0x9A4, BIT(17), 0);

		/* reset CCK FA counter */
		odm_set_bb_reg(dm, 0xA2C, BIT(15), 0);
		odm_set_bb_reg(dm, 0xA2C, BIT(15), 1);

		/* reset CCA counter */
		odm_set_bb_reg(dm, 0xB58, BIT(0), 1);
		odm_set_bb_reg(dm, 0xB58, BIT(0), 0);
	}
#endif	/* #if (ODM_IC_11AC_SERIES_SUPPORT == 1) */
}

void phydm_false_alarm_counter_reg_hold(
	void					*dm_void
)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;

	if (dm->support_ic_type & ODM_IC_11N_SERIES) {
		/*hold ofdm counter*/
		/*hold page C counter*/
		odm_set_bb_reg(dm, 0xC00, BIT(31), 1);
		/*hold page D counter*/
		odm_set_bb_reg(dm, 0xD00, BIT(31), 1);

		//hold cck counter
		odm_set_bb_reg(dm, 0xA2C, BIT(12), 1);
		odm_set_bb_reg(dm, 0xA2C, BIT(14), 1);
	}
}

void odm_false_alarm_counter_statistics(
	void		*dm_void
)
{
	struct dm_struct					*dm = (struct dm_struct *)dm_void;
	struct phydm_fa_struct	*false_alm_cnt = (struct phydm_fa_struct *)
			phydm_get_structure(dm, PHYDM_FALSEALMCNT);
	struct phydm_adaptivity_struct	*adaptivity = (struct
			phydm_adaptivity_struct *)phydm_get_structure(dm, PHYDM_ADAPTIVITY);
	u32						ret_value;

	if (!(dm->support_ability & ODM_BB_FA_CNT))
		return;

	PHYDM_DBG(dm, DBG_FA_CNT, "FA_Counter()======>\n");

#if (ODM_IC_11N_SERIES_SUPPORT == 1)
	if (dm->support_ic_type & ODM_IC_11N_SERIES) {
		/* hold ofdm & cck counter */
		phydm_false_alarm_counter_reg_hold(dm);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE1_11N, MASKDWORD);
		false_alm_cnt->cnt_fast_fsync = (ret_value & 0xffff);
		false_alm_cnt->cnt_sb_search_fail = ((ret_value & 0xffff0000) >> 16);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE2_11N, MASKDWORD);
		false_alm_cnt->cnt_ofdm_cca = (ret_value & 0xffff);
		false_alm_cnt->cnt_parity_fail = ((ret_value & 0xffff0000) >> 16);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE3_11N, MASKDWORD);
		false_alm_cnt->cnt_rate_illegal = (ret_value & 0xffff);
		false_alm_cnt->cnt_crc8_fail = ((ret_value & 0xffff0000) >> 16);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE4_11N, MASKDWORD);
		false_alm_cnt->cnt_mcs_fail = (ret_value & 0xffff);

		false_alm_cnt->cnt_ofdm_fail =
			false_alm_cnt->cnt_parity_fail + false_alm_cnt->cnt_rate_illegal +
			false_alm_cnt->cnt_crc8_fail + false_alm_cnt->cnt_mcs_fail +
			false_alm_cnt->cnt_fast_fsync + false_alm_cnt->cnt_sb_search_fail;

		/* read CCK CRC32 counter */
		false_alm_cnt->cnt_cck_crc32_error = odm_get_bb_reg(dm,
						     ODM_REG_CCK_CRC32_ERROR_CNT_11N, MASKDWORD);
		false_alm_cnt->cnt_cck_crc32_ok = odm_get_bb_reg(dm,
						  ODM_REG_CCK_CRC32_OK_CNT_11N, MASKDWORD);

		/* read OFDM CRC32 counter */
		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_CRC32_CNT_11N, MASKDWORD);
		false_alm_cnt->cnt_ofdm_crc32_error = (ret_value & 0xffff0000) >> 16;
		false_alm_cnt->cnt_ofdm_crc32_ok = ret_value & 0xffff;

		/* read HT CRC32 counter */
		ret_value = odm_get_bb_reg(dm, ODM_REG_HT_CRC32_CNT_11N, MASKDWORD);
		false_alm_cnt->cnt_ht_crc32_error = (ret_value & 0xffff0000) >> 16;
		false_alm_cnt->cnt_ht_crc32_ok = ret_value & 0xffff;

		/* read VHT CRC32 counter */
		false_alm_cnt->cnt_vht_crc32_error = 0;
		false_alm_cnt->cnt_vht_crc32_ok = 0;

#if (RTL8723D_SUPPORT == 1)
		if (dm->support_ic_type == ODM_RTL8723D) {
			/* read HT CRC32 agg counter */
			ret_value = odm_get_bb_reg(dm, ODM_REG_HT_CRC32_CNT_11N_AGG, MASKDWORD);
			false_alm_cnt->cnt_ht_crc32_error_agg = (ret_value & 0xffff0000) >> 16;
			false_alm_cnt->cnt_ht_crc32_ok_agg = ret_value & 0xffff;
		}
#endif

#if (RTL8188E_SUPPORT == 1)
		if (dm->support_ic_type == ODM_RTL8188E) {
			ret_value = odm_get_bb_reg(dm, ODM_REG_SC_CNT_11N, MASKDWORD);
			false_alm_cnt->cnt_bw_lsc = (ret_value & 0xffff);
			false_alm_cnt->cnt_bw_usc = ((ret_value & 0xffff0000) >> 16);
		}
#endif

		{
			ret_value = odm_get_bb_reg(dm, ODM_REG_CCK_FA_LSB_11N, MASKBYTE0);
			false_alm_cnt->cnt_cck_fail = ret_value;

			ret_value = odm_get_bb_reg(dm, ODM_REG_CCK_FA_MSB_11N, MASKBYTE3);
			false_alm_cnt->cnt_cck_fail += (ret_value & 0xff) << 8;

			ret_value = odm_get_bb_reg(dm, ODM_REG_CCK_CCA_CNT_11N, MASKDWORD);
			false_alm_cnt->cnt_cck_cca = ((ret_value & 0xFF) << 8) | ((
							     ret_value & 0xFF00) >> 8);
		}

		false_alm_cnt->cnt_all_pre = false_alm_cnt->cnt_all;

		false_alm_cnt->time_fa_all = (false_alm_cnt->cnt_fast_fsync +
					      false_alm_cnt->cnt_sb_search_fail) * 12 +
					     (false_alm_cnt->cnt_parity_fail + false_alm_cnt->cnt_rate_illegal) * 28 +
					     false_alm_cnt->cnt_crc8_fail * 36 +
					     false_alm_cnt->cnt_mcs_fail * 32 +
					     false_alm_cnt->cnt_cck_fail * 80;

		false_alm_cnt->cnt_all = (false_alm_cnt->cnt_fast_fsync +
					  false_alm_cnt->cnt_sb_search_fail +
					  false_alm_cnt->cnt_parity_fail +
					  false_alm_cnt->cnt_rate_illegal +
					  false_alm_cnt->cnt_crc8_fail +
					  false_alm_cnt->cnt_mcs_fail +
					  false_alm_cnt->cnt_cck_fail);

		false_alm_cnt->cnt_cca_all = false_alm_cnt->cnt_ofdm_cca +
					     false_alm_cnt->cnt_cck_cca;

		PHYDM_DBG(dm, DBG_FA_CNT,
			  "[OFDM FA Detail] Parity_Fail = (( %d )), Rate_Illegal = (( %d )), CRC8_fail = (( %d )), Mcs_fail = (( %d )), Fast_Fsync = (( %d )), SB_Search_fail = (( %d ))\n",
			  false_alm_cnt->cnt_parity_fail, false_alm_cnt->cnt_rate_illegal,
			  false_alm_cnt->cnt_crc8_fail, false_alm_cnt->cnt_mcs_fail,
			  false_alm_cnt->cnt_fast_fsync, false_alm_cnt->cnt_sb_search_fail);

	}
#endif

#if (ODM_IC_11AC_SERIES_SUPPORT == 1)
	if (dm->support_ic_type & ODM_IC_11AC_SERIES) {
		u32 cck_enable;

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE1_11AC, MASKDWORD);
		false_alm_cnt->cnt_fast_fsync = ((ret_value & 0xffff0000) >> 16);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE2_11AC, MASKDWORD);
		false_alm_cnt->cnt_sb_search_fail = (ret_value & 0xffff);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE3_11AC, MASKDWORD);
		false_alm_cnt->cnt_parity_fail = (ret_value & 0xffff);
		false_alm_cnt->cnt_rate_illegal = ((ret_value & 0xffff0000) >> 16);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE4_11AC, MASKDWORD);
		false_alm_cnt->cnt_crc8_fail = (ret_value & 0xffff);
		false_alm_cnt->cnt_mcs_fail = ((ret_value & 0xffff0000) >> 16);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE5_11AC, MASKDWORD);
		false_alm_cnt->cnt_crc8_fail_vht = (ret_value & 0xffff);

		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_TYPE6_11AC, MASKDWORD);
		false_alm_cnt->cnt_mcs_fail_vht = (ret_value & 0xffff);

		/* read OFDM FA counter */
		false_alm_cnt->cnt_ofdm_fail = odm_get_bb_reg(dm, ODM_REG_OFDM_FA_11AC,
					       MASKLWORD);

		/* Read CCK FA counter */
		false_alm_cnt->cnt_cck_fail = odm_get_bb_reg(dm, ODM_REG_CCK_FA_11AC,
					      MASKLWORD);

		/* read CCK/OFDM CCA counter */
		ret_value = odm_get_bb_reg(dm, ODM_REG_CCK_CCA_CNT_11AC, MASKDWORD);
		false_alm_cnt->cnt_ofdm_cca = (ret_value & 0xffff0000) >> 16;
		false_alm_cnt->cnt_cck_cca = ret_value & 0xffff;

		/* read CCK CRC32 counter */
		ret_value = odm_get_bb_reg(dm, ODM_REG_CCK_CRC32_CNT_11AC, MASKDWORD);
		false_alm_cnt->cnt_cck_crc32_error = (ret_value & 0xffff0000) >> 16;
		false_alm_cnt->cnt_cck_crc32_ok = ret_value & 0xffff;

		/* read OFDM CRC32 counter */
		ret_value = odm_get_bb_reg(dm, ODM_REG_OFDM_CRC32_CNT_11AC, MASKDWORD);
		false_alm_cnt->cnt_ofdm_crc32_error = (ret_value & 0xffff0000) >> 16;
		false_alm_cnt->cnt_ofdm_crc32_ok = ret_value & 0xffff;

		/* read HT CRC32 counter */
		ret_value = odm_get_bb_reg(dm, ODM_REG_HT_CRC32_CNT_11AC, MASKDWORD);
		false_alm_cnt->cnt_ht_crc32_error = (ret_value & 0xffff0000) >> 16;
		false_alm_cnt->cnt_ht_crc32_ok = ret_value & 0xffff;

		/* read VHT CRC32 counter */
		ret_value = odm_get_bb_reg(dm, ODM_REG_VHT_CRC32_CNT_11AC, MASKDWORD);
		false_alm_cnt->cnt_vht_crc32_error = (ret_value & 0xffff0000) >> 16;
		false_alm_cnt->cnt_vht_crc32_ok = ret_value & 0xffff;

#if (RTL8881A_SUPPORT == 1)
		/* For 8881A */
		if (dm->support_ic_type == ODM_RTL8881A) {
			u32 cnt_ofdm_fail_temp = 0;

			if (false_alm_cnt->cnt_ofdm_fail >= false_alm_cnt->cnt_ofdm_fail_pre) {
				cnt_ofdm_fail_temp = false_alm_cnt->cnt_ofdm_fail_pre;
				false_alm_cnt->cnt_ofdm_fail_pre = false_alm_cnt->cnt_ofdm_fail;
				false_alm_cnt->cnt_ofdm_fail = false_alm_cnt->cnt_ofdm_fail -
							       cnt_ofdm_fail_temp;
			} else
				false_alm_cnt->cnt_ofdm_fail_pre = false_alm_cnt->cnt_ofdm_fail;
			PHYDM_DBG(dm, DBG_FA_CNT,
				  "odm_false_alarm_counter_statistics(): cnt_ofdm_fail=%d\n",
				  false_alm_cnt->cnt_ofdm_fail_pre);
			PHYDM_DBG(dm, DBG_FA_CNT,
				  "odm_false_alarm_counter_statistics(): cnt_ofdm_fail_pre=%d\n",
				  cnt_ofdm_fail_temp);
		}
#endif
		cck_enable =  odm_get_bb_reg(dm, ODM_REG_BB_RX_PATH_11AC, BIT(28));
		if (cck_enable) { /* if(*dm->band_type == ODM_BAND_2_4G) */
			false_alm_cnt->cnt_all = false_alm_cnt->cnt_ofdm_fail +
						 false_alm_cnt->cnt_cck_fail;
			false_alm_cnt->cnt_cca_all = false_alm_cnt->cnt_cck_cca +
						     false_alm_cnt->cnt_ofdm_cca;
		} else {
			false_alm_cnt->cnt_all = false_alm_cnt->cnt_ofdm_fail;
			false_alm_cnt->cnt_cca_all = false_alm_cnt->cnt_ofdm_cca;
		}
	}
#endif

	if (dm->support_ic_type != ODM_RTL8723D) {
		if (phydm_set_bb_dbg_port(dm, BB_DBGPORT_PRIORITY_1,
					  0x0)) {/*set debug port to 0x0*/
			false_alm_cnt->dbg_port0 = phydm_get_bb_dbg_port_value(dm);
			phydm_release_bb_dbg_port(dm);
		}

		if (phydm_set_bb_dbg_port(dm, BB_DBGPORT_PRIORITY_1,
					  adaptivity->adaptivity_dbg_port)) {
			if (dm->support_ic_type & (ODM_RTL8723B | ODM_RTL8188E))
				false_alm_cnt->edcca_flag = (boolean)((phydm_get_bb_dbg_port_value(
						dm) & BIT(30)) >> 30);
			else
				false_alm_cnt->edcca_flag = (boolean)((phydm_get_bb_dbg_port_value(
						dm) & BIT(29)) >> 29);
			phydm_release_bb_dbg_port(dm);
		}
	} else {
		false_alm_cnt->edcca_flag = (boolean)(odm_get_bb_reg(dm, 0x9a0, BIT(29)));
	}

	phydm_false_alarm_counter_reg_reset(dm_void);

	false_alm_cnt->time_fa_all = (false_alm_cnt->cnt_fast_fsync +
				      false_alm_cnt->cnt_sb_search_fail) * 12 +
				     (false_alm_cnt->cnt_parity_fail + false_alm_cnt->cnt_rate_illegal) * 28 +
				     (false_alm_cnt->cnt_crc8_fail + false_alm_cnt->cnt_crc8_fail_vht +
				      false_alm_cnt->cnt_mcs_fail_vht) * 36 +
				     false_alm_cnt->cnt_mcs_fail * 32 +
				     false_alm_cnt->cnt_cck_fail * 80;

	false_alm_cnt->cnt_crc32_error_all = false_alm_cnt->cnt_vht_crc32_error +
					     false_alm_cnt->cnt_ht_crc32_error + false_alm_cnt->cnt_ofdm_crc32_error +
					     false_alm_cnt->cnt_cck_crc32_error;
	false_alm_cnt->cnt_crc32_ok_all = false_alm_cnt->cnt_vht_crc32_ok +
					  false_alm_cnt->cnt_ht_crc32_ok + false_alm_cnt->cnt_ofdm_crc32_ok +
					  false_alm_cnt->cnt_cck_crc32_ok;

	PHYDM_DBG(dm, DBG_FA_CNT,
		  "[OFDM FA Detail] Parity_Fail = (( %d )), Rate_Illegal = (( %d )), HT_CRC8_fail = (( %d )), HT_Mcs_fail = (( %d )), Fast_Fsync = (( %d )), SB_Search_fail = (( %d )), VHT_CRC8_fail = (( %d )), VHT_Mcs_fail = (( %d ))\n",
		  false_alm_cnt->cnt_parity_fail, false_alm_cnt->cnt_rate_illegal,
		  false_alm_cnt->cnt_crc8_fail, false_alm_cnt->cnt_mcs_fail,
		  false_alm_cnt->cnt_fast_fsync, false_alm_cnt->cnt_sb_search_fail,
		  false_alm_cnt->cnt_crc8_fail_vht, false_alm_cnt->cnt_mcs_fail_vht);
	PHYDM_DBG(dm, DBG_FA_CNT, "[CCA Cnt] {CCK, OFDM, Total} = {%d, %d, %d}\n",
		  false_alm_cnt->cnt_cck_cca, false_alm_cnt->cnt_ofdm_cca,
		  false_alm_cnt->cnt_cca_all);

	PHYDM_DBG(dm, DBG_FA_CNT, "[FA Cnt] {CCK, OFDM, Total} = {%d, %d, %d}\n",
		  false_alm_cnt->cnt_cck_fail, false_alm_cnt->cnt_ofdm_fail,
		  false_alm_cnt->cnt_all);

	PHYDM_DBG(dm, DBG_FA_CNT, "[CCK]  CRC32 {error, ok}= {%d, %d}\n",
		  false_alm_cnt->cnt_cck_crc32_error, false_alm_cnt->cnt_cck_crc32_ok);
	PHYDM_DBG(dm, DBG_FA_CNT, "[OFDM]CRC32 {error, ok}= {%d, %d}\n",
		  false_alm_cnt->cnt_ofdm_crc32_error, false_alm_cnt->cnt_ofdm_crc32_ok);
	PHYDM_DBG(dm, DBG_FA_CNT, "[ HT ]  CRC32 {error, ok}= {%d, %d}\n",
		  false_alm_cnt->cnt_ht_crc32_error, false_alm_cnt->cnt_ht_crc32_ok);
	PHYDM_DBG(dm, DBG_FA_CNT, "[VHT]  CRC32 {error, ok}= {%d, %d}\n",
		  false_alm_cnt->cnt_vht_crc32_error, false_alm_cnt->cnt_vht_crc32_ok);
	PHYDM_DBG(dm, DBG_FA_CNT, "[TOTAL]  CRC32 {error, ok}= {%d, %d}\n",
		  false_alm_cnt->cnt_crc32_error_all, false_alm_cnt->cnt_crc32_ok_all);
	PHYDM_DBG(dm, DBG_FA_CNT, "FA_Cnt: Dbg port 0x0 = 0x%x, EDCCA = %d\n\n",
		  false_alm_cnt->dbg_port0, false_alm_cnt->edcca_flag);
}

#ifdef PHYDM_TDMA_DIG_SUPPORT
void phydm_set_tdma_dig_timer(
	void		*dm_void
)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	u32	delta_time_us = dm->tdma_dig_timer_ms * 1000;
	struct phydm_dig_struct	*dig_t;
	u32	timeout;
	u32	current_time_stamp, diff_time_stamp, regb0;

	dig_t = &dm->dm_dig_table;
	/*some IC has no FREERUN_CUNT register, like 92E*/
	if (dm->support_ic_type & ODM_RTL8197F)
		current_time_stamp = odm_get_bb_reg(dm, 0x568, bMaskDWord);
	else
		return;

	timeout = current_time_stamp + delta_time_us;

	diff_time_stamp = current_time_stamp - dig_t->cur_timestamp;
	dig_t->pre_timestamp = dig_t->cur_timestamp;
	dig_t->cur_timestamp = current_time_stamp;

	/*HIMR0, it shows HW interrupt mask*/
	regb0 = odm_get_bb_reg(dm, 0xb0, bMaskDWord);

	PHYDM_DBG(dm, DBG_DIG,
		  "Set next tdma_dig_timer\n");
	PHYDM_DBG(dm, DBG_DIG,
		  "current_time_stamp=%d, delta_time_us=%d, timeout=%d, diff_time_stamp=%d, Reg0xb0 = 0x%x\n",
		  current_time_stamp,
		  delta_time_us,
		  timeout,
		  diff_time_stamp,
		  regb0);

	if (dm->support_ic_type & ODM_RTL8197F)		/*REG_PS_TIMER2*/
		odm_set_bb_reg(dm, 0x588, bMaskDWord, timeout);
	else {
		PHYDM_DBG(dm, DBG_DIG,
			  "NOT 97F, TDMA-DIG timer does NOT start!\n");
		return;
	}
}

void phydm_tdma_dig_timer_check(
	void		*dm_void
)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t;

	dig_t = &dm->dm_dig_table;

	PHYDM_DBG(dm, DBG_DIG,
		  "tdma_dig_cnt=%d, pre_tdma_dig_cnt=%d\n",
		  dig_t->tdma_dig_cnt,
		  dig_t->pre_tdma_dig_cnt);

	if ((dig_t->tdma_dig_cnt == 0) ||
	    (dig_t->tdma_dig_cnt == dig_t->pre_tdma_dig_cnt)) {
		if (dm->support_ability & ODM_BB_DIG) {
			/*if interrupt mask info is got.*/
			/*Reg0xb0 is no longer needed*/
			/*regb0 = odm_get_bb_reg(dm, 0xb0, bMaskDWord);*/
			PHYDM_DBG(dm, DBG_DIG,
				  "Check fail, IntMask[0]=0x%x, restart tdma_dig_timer !!!\n",
				  *dm->interrupt_mask);

			phydm_tdma_dig_add_interrupt_mask_handler(dm);
			phydm_enable_rx_related_interrupt_handler(dm);
			phydm_set_tdma_dig_timer(dm);
		}
	} else
		PHYDM_DBG(dm, DBG_DIG,
			  "Check pass, update pre_tdma_dig_cnt\n");

	dig_t->pre_tdma_dig_cnt = dig_t->tdma_dig_cnt;
}

/*different IC/team may use different timer for tdma-dig*/
void phydm_tdma_dig_add_interrupt_mask_handler(
	void		*dm_void
)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;

#if (DM_ODM_SUPPORT_TYPE == (ODM_AP))
	if (dm->support_ic_type & ODM_RTL8197F)
		phydm_add_interrupt_mask_handler(dm,
						 HAL_INT_TYPE_PSTIMEOUT2);	/*HAL_INT_TYPE_PSTIMEOUT2*/
#elif (DM_ODM_SUPPORT_TYPE == (ODM_WIN))
#elif (DM_ODM_SUPPORT_TYPE == (ODM_CE))
#endif
}

void phydm_tdma_dig(
	void		*dm_void
)
{
	struct dm_struct *dm;
	struct phydm_dig_struct	*dig_t;
	struct phydm_fa_struct *falm_cnt;
	u32	reg_c50;

	dm = (struct dm_struct *)dm_void;
	dig_t = &dm->dm_dig_table;
	falm_cnt = &dm->false_alm_cnt;
	reg_c50 = odm_get_bb_reg(dm, 0xc50, MASKBYTE0);

	dig_t->tdma_dig_state =
		dig_t->tdma_dig_cnt % dm->tdma_dig_state_number;

	PHYDM_DBG(dm, DBG_DIG,
		  "tdma_dig_state=%d, regc50=0x%x\n",
		  dig_t->tdma_dig_state,
		  reg_c50);

	dig_t->tdma_dig_cnt++;

	if (dig_t->tdma_dig_state == 1) {
		// update IGI from tdma_dig_state == 0
		if (dig_t->cur_ig_value_tdma == 0)
			dig_t->cur_ig_value_tdma = dig_t->cur_ig_value;

		odm_write_dig(dm, dig_t->cur_ig_value_tdma);
		phydm_tdma_false_alarm_counter_check(dm);
		PHYDM_DBG(dm, DBG_DIG,
			  "tdma_dig_state=%d, reset FA counter !!!\n",
			  dig_t->tdma_dig_state);

	} else if (dig_t->tdma_dig_state == 0) {
		/* update dig_t->CurIGValue,*/
		/* it may different from dig_t->cur_ig_value_tdma */
		/* TDMA IGI upperbond @ L-state = */
		/* rf_ft_var.tdma_dig_low_upper_bond = 0x26 */

		if (dig_t->cur_ig_value >= dm->tdma_dig_low_upper_bond)
			dig_t->low_ig_value = dm->tdma_dig_low_upper_bond;
		else
			dig_t->low_ig_value = dig_t->cur_ig_value;

		odm_write_dig(dm, dig_t->low_ig_value);
		phydm_tdma_false_alarm_counter_check(dm);
	} else
		phydm_tdma_false_alarm_counter_check(dm);
}

/*============================================================*/
/*FASLE ALARM CHECK*/
/*============================================================*/

void phydm_tdma_false_alarm_counter_check(
	void		*dm_void
)
{
	struct dm_struct	*dm;
	struct phydm_fa_struct	*falm_cnt;
	struct phydm_fa_acc_struct	*falm_cnt_acc;
	struct phydm_dig_struct	*dig_t;
	boolean	rssi_dump_en = 0;
	u32 timestamp;
	u8 tdma_dig_state_number;

	dm = (struct dm_struct *)dm_void;
	falm_cnt = &dm->false_alm_cnt;
	falm_cnt_acc = &dm->false_alm_cnt_acc;
	dig_t = &dm->dm_dig_table;

	if (dig_t->tdma_dig_state == 1)
		phydm_false_alarm_counter_reset(dm);
	/* Reset FalseAlarmCounterStatistics */
	/* fa_acc_1sec_tsf = fa_acc_1sec_tsf, keep */
	/* fa_end_tsf = fa_start_tsf = TSF */
	else {
		odm_false_alarm_counter_statistics(dm);
		if (dm->support_ic_type & ODM_RTL8197F)		/*REG_FREERUN_CNT*/
			timestamp = odm_get_bb_reg(dm, 0x568, bMaskDWord);
		else {
			PHYDM_DBG(dm, DBG_DIG,
				  "Caution! NOT 97F! TDMA-DIG timer does NOT start!!!\n");
			return;
		}
		dig_t->fa_end_timestamp = timestamp;
		dig_t->fa_acc_1sec_timestamp +=
			(dig_t->fa_end_timestamp - dig_t->fa_start_timestamp);

		/*prevent dumb*/
		if (dm->tdma_dig_state_number == 1)
			dm->tdma_dig_state_number = 2;

		tdma_dig_state_number = dm->tdma_dig_state_number;
		dig_t->sec_factor =
			tdma_dig_state_number / (tdma_dig_state_number - 1);

		/*1sec = 1000000us*/
		if (dig_t->fa_acc_1sec_timestamp >= (u32)(1000000 / dig_t->sec_factor)) {
			rssi_dump_en = 1;
			phydm_false_alarm_counter_acc(dm, rssi_dump_en);
			PHYDM_DBG(dm, DBG_DIG,
				  "sec_factor = %u, total FA = %u, is_linked=%u\n",
				  dig_t->sec_factor,
				  falm_cnt_acc->cnt_all,
				  dm->is_linked);

			phydm_noisy_detection(dm);
			phydm_cck_pd_th(dm);
			phydm_dig(dm);
			phydm_false_alarm_counter_acc_reset(dm);

			/* Reset FalseAlarmCounterStatistics */
			/* fa_end_tsf = fa_start_tsf = TSF, keep */
			/* fa_acc_1sec_tsf = 0 */
			phydm_false_alarm_counter_reset(dm);
		} else
			phydm_false_alarm_counter_acc(dm, rssi_dump_en);
	}
}

void phydm_false_alarm_counter_acc(
	void		*dm_void,
	boolean		rssi_dump_en
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_fa_struct			*falm_cnt;
	struct phydm_fa_acc_struct		*falm_cnt_acc;
	struct phydm_dig_struct	*dig_t;

	falm_cnt = &dm->false_alm_cnt;
	falm_cnt_acc = &dm->false_alm_cnt_acc;
	dig_t = &dm->dm_dig_table;

	falm_cnt_acc->cnt_parity_fail += falm_cnt->cnt_parity_fail;
	falm_cnt_acc->cnt_rate_illegal += falm_cnt->cnt_rate_illegal;
	falm_cnt_acc->cnt_crc8_fail += falm_cnt->cnt_crc8_fail;
	falm_cnt_acc->cnt_mcs_fail += falm_cnt->cnt_mcs_fail;
	falm_cnt_acc->cnt_ofdm_fail += falm_cnt->cnt_ofdm_fail;
	falm_cnt_acc->cnt_cck_fail += falm_cnt->cnt_cck_fail;
	falm_cnt_acc->cnt_all += falm_cnt->cnt_all;
	falm_cnt_acc->cnt_fast_fsync += falm_cnt->cnt_fast_fsync;
	falm_cnt_acc->cnt_sb_search_fail += falm_cnt->cnt_sb_search_fail;
	falm_cnt_acc->cnt_ofdm_cca += falm_cnt->cnt_ofdm_cca;
	falm_cnt_acc->cnt_cck_cca += falm_cnt->cnt_cck_cca;
	falm_cnt_acc->cnt_cca_all += falm_cnt->cnt_cca_all;
	falm_cnt_acc->cnt_cck_crc32_error += falm_cnt->cnt_cck_crc32_error;
	falm_cnt_acc->cnt_cck_crc32_ok += falm_cnt->cnt_cck_crc32_ok;
	falm_cnt_acc->cnt_ofdm_crc32_error += falm_cnt->cnt_ofdm_crc32_error;
	falm_cnt_acc->cnt_ofdm_crc32_ok += falm_cnt->cnt_ofdm_crc32_ok;
	falm_cnt_acc->cnt_ht_crc32_error += falm_cnt->cnt_ht_crc32_error;
	falm_cnt_acc->cnt_ht_crc32_ok += falm_cnt->cnt_ht_crc32_ok;
	falm_cnt_acc->cnt_vht_crc32_error += falm_cnt->cnt_vht_crc32_error;
	falm_cnt_acc->cnt_vht_crc32_ok += falm_cnt->cnt_vht_crc32_ok;
	falm_cnt_acc->cnt_crc32_error_all += falm_cnt->cnt_crc32_error_all;
	falm_cnt_acc->cnt_crc32_ok_all += falm_cnt->cnt_crc32_ok_all;

	if (rssi_dump_en == 1) {
		falm_cnt_acc->cnt_all_1sec =
			falm_cnt_acc->cnt_all * dig_t->sec_factor;
		falm_cnt_acc->cnt_cca_all_1sec =
			falm_cnt_acc->cnt_cca_all * dig_t->sec_factor;
		falm_cnt_acc->cnt_cck_fail_1sec =
			falm_cnt_acc->cnt_cck_fail * dig_t->sec_factor;
	}
}

void phydm_false_alarm_counter_acc_reset(
	void		*dm_void
)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_fa_acc_struct *falm_cnt_acc;

	falm_cnt_acc = &dm->false_alm_cnt_acc;

	/* Cnt_all_for_rssi_dump & Cnt_CCA_all_for_rssi_dump */
	/* do NOT need to be reset */
	odm_memory_set(dm, falm_cnt_acc, 0, sizeof(falm_cnt_acc));
}

void phydm_false_alarm_counter_reset(
	void		*dm_void
)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_fa_struct *falm_cnt;
	struct phydm_dig_struct	*dig_t;
	u32	timestamp;

	falm_cnt = &dm->false_alm_cnt;
	dig_t = &dm->dm_dig_table;

	memset(falm_cnt, 0, sizeof(dm->false_alm_cnt));
	phydm_false_alarm_counter_reg_reset(dm);

	if (dig_t->tdma_dig_state != 1)
		dig_t->fa_acc_1sec_timestamp = 0;
	else
		dig_t->fa_acc_1sec_timestamp = dig_t->fa_acc_1sec_timestamp;

	/*REG_FREERUN_CNT*/
	timestamp = odm_get_bb_reg(dm, 0x568, bMaskDWord);
	dig_t->fa_start_timestamp = timestamp;
	dig_t->fa_end_timestamp = timestamp;
}

#endif	/*#ifdef PHYDM_TDMA_DIG_SUPPORT*/

#ifdef PHYDM_LNA_SAT_CHK_SUPPORT
void phydm_lna_sat_chk_init(
	void		*dm_void
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;

	struct phydm_lna_sat_info_struct *lna_info = &dm->dm_lna_sat_info;

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK, "%s ==>\n", __FUNCTION__);

	lna_info->check_time = 0;
	lna_info->sat_cnt_acc_patha = 0;
	lna_info->sat_cnt_acc_pathb = 0;
	lna_info->cur_sat_status = 0;
	lna_info->pre_sat_status = 0;
	lna_info->cur_timer_check_cnt = 0;
	lna_info->pre_timer_check_cnt = 0;
}

void phydm_set_ofdm_agc_tab(
	void	*dm_void,
	u8		tab_sel
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;

	/* table sel:0/2, 1 is used for CCK */
	if (tab_sel == OFDM_AGC_TAB_0)
		odm_set_bb_reg(dm, 0xc70, 0x1e00, OFDM_AGC_TAB_0);
	else if (tab_sel == OFDM_AGC_TAB_2)
		odm_set_bb_reg(dm, 0xc70, 0x1e00, OFDM_AGC_TAB_2);
	else
		odm_set_bb_reg(dm, 0xc70, 0x1e00, OFDM_AGC_TAB_0);
}

u8 phydm_get_ofdm_agc_tab(
	void	*dm_void
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;

	return (u1Byte)odm_get_bb_reg(dm, 0xc70, 0x1e00);
}

void phydm_lna_sat_chk(
	void		*dm_void
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	struct phydm_lna_sat_info_struct *lna_info = &dm->dm_lna_sat_info;

	u1Byte			igi_rssi_min, rssi_min = dm->rssi_min;
	u4Byte			sat_status_patha, sat_status_pathb;
	u1Byte			igi_restore = dig_t->cur_ig_value;
	u1Byte			i, lna_sat_chk_cnt = dm->lna_sat_chk_cnt;
	u4Byte			lna_sat_cnt_thd = 0;
	u1Byte			agc_tab;
	u4Byte			max_check_time = 0;

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK, "\n%s ==>\n", __FUNCTION__);

	if (!(dm->support_ability & ODM_BB_LNA_SAT_CHK)) {
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
			  "support ability is disabled, return.\n");
		return;
	}

	if (dm->is_disable_lna_sat_chk) {
		phydm_lna_sat_chk_init(dm);
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
			  "is_disable_lna_sat_chk=%d, return.\n", dm->is_disable_lna_sat_chk);
		return;
	}

	//func_start = ODM_GetBBReg(pDM_Odm, 0x560, bMaskDWord);

	// move igi to target pin of rssi_min
	if ((rssi_min == 0) || (rssi_min == 0xff)) {
		// adapt agc table 0
		phydm_set_ofdm_agc_tab(dm, OFDM_AGC_TAB_0);
		phydm_lna_sat_chk_init(dm);
		return;
	} else if (rssi_min % 2 != 0)
		igi_rssi_min = rssi_min + DIFF_RSSI_TO_IGI - 1;
	else
		igi_rssi_min = rssi_min + DIFF_RSSI_TO_IGI;

	if ((dm->lna_sat_chk_period_ms > 0)
	    && (dm->lna_sat_chk_period_ms <= ONE_SEC_MS))
		max_check_time = lna_sat_chk_cnt * (ONE_SEC_MS /
						    (dm->lna_sat_chk_period_ms)) * 5;
	else
		max_check_time = lna_sat_chk_cnt * 5;

	lna_sat_cnt_thd = (max_check_time * dm->lna_sat_chk_duty_cycle) / 100;

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
		  "check_time=%d, rssi_min=%d, igi_rssi_min=0x%x\nlna_sat_chk_cnt=%d, lna_sat_chk_period_ms=%d, max_check_time=%d, lna_sat_cnt_thd=%d\n",
		  lna_info->check_time,
		  rssi_min,
		  igi_rssi_min,
		  lna_sat_chk_cnt,
		  dm->lna_sat_chk_period_ms,
		  max_check_time,
		  lna_sat_cnt_thd);

	odm_write_dig(dm, igi_rssi_min);

	// adapt agc table 0 check saturation status
	phydm_set_ofdm_agc_tab(dm, OFDM_AGC_TAB_0);
	// open rf power detection ckt & set detection range
	odm_set_rf_reg(dm, RF_PATH_A, 0x86, 0x1f, 0x10);
	odm_set_rf_reg(dm, RF_PATH_B, 0x86, 0x1f, 0x10);

	// check saturation status
	for (i = 0; i < lna_sat_chk_cnt; i++) {
		sat_status_patha = odm_get_rf_reg(dm, RF_PATH_A, 0xae, 0xc0000);
		sat_status_pathb = odm_get_rf_reg(dm, RF_PATH_B, 0xae, 0xc0000);
		if (sat_status_patha != 0)
			lna_info->sat_cnt_acc_patha++;
		if (sat_status_pathb != 0)
			lna_info->sat_cnt_acc_pathb++;

		if ((lna_info->sat_cnt_acc_patha >= lna_sat_cnt_thd) ||
		    (lna_info->sat_cnt_acc_pathb >= lna_sat_cnt_thd)) {
			lna_info->cur_sat_status = 1;
			PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
				  "cur_sat_status=%d, check_time=%d\n",
				  lna_info->cur_sat_status,
				  lna_info->check_time);
			break;
		} else
			lna_info->cur_sat_status = 0;
	}

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
		  "cur_sat_status=%d, pre_sat_status=%d, sat_cnt_acc_patha=%d, sat_cnt_acc_pathb=%d\n",
		  lna_info->cur_sat_status,
		  lna_info->pre_sat_status,
		  lna_info->sat_cnt_acc_patha,
		  lna_info->sat_cnt_acc_pathb);

	// agc table decision
	if (lna_info->cur_sat_status) {
		if (!dm->is_disable_gain_table_switch)
			phydm_set_ofdm_agc_tab(dm, OFDM_AGC_TAB_2);
		lna_info->check_time = 0;
		lna_info->sat_cnt_acc_patha = 0;
		lna_info->sat_cnt_acc_pathb = 0;
		lna_info->pre_sat_status = lna_info->cur_sat_status;

	} else if (lna_info->check_time <= (max_check_time - 1)) {
		if (lna_info->pre_sat_status && (!dm->is_disable_gain_table_switch))
			phydm_set_ofdm_agc_tab(dm, OFDM_AGC_TAB_2);
		lna_info->check_time++;

	} else if (lna_info->check_time == max_check_time) {
		if (!dm->is_disable_gain_table_switch && (lna_info->pre_sat_status == 1))
			phydm_set_ofdm_agc_tab(dm, OFDM_AGC_TAB_0);
		lna_info->check_time = 0;
		lna_info->sat_cnt_acc_patha = 0;
		lna_info->sat_cnt_acc_pathb = 0;
		lna_info->pre_sat_status = lna_info->cur_sat_status;
	}

	agc_tab = phydm_get_ofdm_agc_tab(dm);

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK, "use AGC tab %d\n", agc_tab);
	//func_end = ODM_GetBBReg(pDM_Odm, 0x560, bMaskDWord);

	//PHYDM_DBG(pDM_Odm, ODM_COMP_COMMON, ODM_DBG_LOUD, ("function process time=%d\n",
	//	func_end - func_start));

	// restore previous igi
	odm_write_dig(dm, igi_restore);
	lna_info->cur_timer_check_cnt++;
	odm_set_timer(dm, &lna_info->phydm_lna_sat_chk_timer,
		      dm->lna_sat_chk_period_ms);
}

void phydm_lna_sat_chk_callback(
	void		*dm_void

)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK, "\n%s ==>\n", __FUNCTION__);
	phydm_lna_sat_chk(dm);
}

void phydm_lna_sat_chk_timers(
	void		*dm_void,
	u8			state
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_lna_sat_info_struct *lna_info = &dm->dm_lna_sat_info;

	if (state == INIT_LNA_SAT_CHK_TIMMER) {
		odm_initialize_timer(dm,
				     &lna_info->phydm_lna_sat_chk_timer,
				     (void *)phydm_lna_sat_chk_callback, NULL,
				     "phydm_lna_sat_chk_timer");
	} else if (state == CANCEL_LNA_SAT_CHK_TIMMER) {
		odm_cancel_timer(dm, &lna_info->phydm_lna_sat_chk_timer);
	} else if (state == RELEASE_LNA_SAT_CHK_TIMMER) {
		odm_release_timer(dm, &lna_info->phydm_lna_sat_chk_timer);
	}
}

void phydm_lna_sat_chk_watchdog(
	void		*dm_void
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_lna_sat_info_struct *lna_info = &dm->dm_lna_sat_info;

	u1Byte rssi_min = dm->rssi_min;

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK, "\n%s ==>\n", __FUNCTION__);

	if (!(dm->support_ability & ODM_BB_LNA_SAT_CHK)) {
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
			  "support ability is disabled, return.\n");
		return;
	}

	PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
		  "pre_timer_check_cnt=%d, cur_timer_check_cnt=%d\n",
		  lna_info->pre_timer_check_cnt,
		  lna_info->cur_timer_check_cnt);

	if (dm->is_disable_lna_sat_chk) {
		phydm_lna_sat_chk_init(dm);
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
			  "is_disable_lna_sat_chk=%d, return.\n", dm->is_disable_lna_sat_chk);
		return;
	}

	if ((dm->support_ic_type & ODM_RTL8197F) == 0) {
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
			  "SupportICType != ODM_RTL8197F, return.\n");
		return;
	}

	if ((rssi_min == 0) || (rssi_min == 0xff)) {
		// adapt agc table 0
		phydm_set_ofdm_agc_tab(dm, OFDM_AGC_TAB_0);
		phydm_lna_sat_chk_init(dm);
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK,
			  "rssi_min=%d, return.\n", rssi_min);
		return;
	}

	if (lna_info->cur_timer_check_cnt == lna_info->pre_timer_check_cnt) {
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK, "Timer check fail, restart timer.\n");
		phydm_lna_sat_chk(dm);
	} else {
		PHYDM_DBG(dm, DBG_LNA_SAT_CHK, "Timer check pass.\n");
	}
	lna_info->pre_timer_check_cnt = lna_info->cur_timer_check_cnt;
}
#endif	/*#if (PHYDM_LNA_SAT_CHK_SUPPORT == 1)*/

void phydm_dig_debug(
	void		*dm_void,
	char		input[][16],
	u32		*_used,
	char		*output,
	u32		*_out_len,
	u32		input_num
)
{
	struct dm_struct	*dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct	*dig_t = &dm->dm_dig_table;
	char		help[] = "-h";
	char		monitor[] = "-m";
	u32		var1[10] = {0};
	u32		used = *_used;
	u32		out_len = *_out_len;
	u8		i;

	if ((strcmp(input[1], help) == 0))
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "{0} fa[0] fa[1] fa[2]\n");
	else if ((strcmp(input[1], monitor) == 0)) {
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "Read DIG fa_th[0:2]= {%d, %d, %d}\n",
			 dig_t->fa_th[0], dig_t->fa_th[1],
			 dig_t->fa_th[2]);

	} else {
		PHYDM_SSCANF(input[1], DCMD_DECIMAL, &var1[0]);

		for (i = 1; i < 10; i++) {
			if (input[i + 1])
				PHYDM_SSCANF(input[i + 1], DCMD_DECIMAL, &var1[i]);
		}

		if (var1[0] == 0) {
			dig_t->is_dbg_fa_th = true;
			dig_t->fa_th[0] = (u16)var1[1];
			dig_t->fa_th[1] = (u16)var1[2];
			dig_t->fa_th[2] = (u16)var1[3];

			PDM_SNPF(out_len, used, output + used,
				 out_len - used,
				 "Set DIG fa_th[0:2]= {%d, %d, %d}\n",
				 dig_t->fa_th[0], dig_t->fa_th[1],
				 dig_t->fa_th[2]);
		} else
			dig_t->is_dbg_fa_th = false;
	}
	*_used = used;
	*_out_len = out_len;
}

