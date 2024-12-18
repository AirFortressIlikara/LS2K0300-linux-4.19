/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define _RTL8812A_CMD_C_

//#include <drv_types.h>
#include <rtl8812a_hal.h>

#define CONFIG_H2C_EF

#define RTL8812_MAX_H2C_BOX_NUMS	4
#define RTL8812_MAX_CMD_LEN	7
#define RTL8812_MESSAGE_BOX_SIZE		4
#define RTL8812_EX_MESSAGE_BOX_SIZE	4


static u8 _is_fw_read_cmd_down(_adapter *padapter, u8 msgbox_num)
{
	u8	read_down = _FALSE;
	int 	retry_cnts = 100;

	u8 valid;

	//DBG_8192C(" _is_fw_read_cmd_down ,reg_1cc(%x),msg_box(%d)...\n",rtw_read8(padapter,REG_HMETFR),msgbox_num);

	do {
		valid = rtw_read8(padapter, REG_HMETFR) & BIT(msgbox_num);
		if (0 == valid) {
			read_down = _TRUE;
		}
#ifdef CONFIG_WOWLAN
		rtw_msleep_os(2);
#endif
	} while ((!read_down) && (retry_cnts--));

	return read_down;

}


/*****************************************
* H2C Msg format :
* 0x1DF - 0x1D0
*| 31 - 8	| 7-5 	 4 - 0	|
*| h2c_msg 	|Class_ID CMD_ID	|
*
* Extend 0x1FF - 0x1F0
*|31 - 0	  |
*|ext_msg|
******************************************/
static s32 FillH2CCmd_8812(PADAPTER padapter, u8 ElementID, u32 CmdLen,
			   u8 *pCmdBuffer)
{
	u8 bcmd_down = _FALSE;
	s32 retry_cnts = 100;
	u8 h2c_box_num;
	u32	msgbox_addr;
	u32 msgbox_ex_addr;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u8 cmd_idx, ext_cmd_len;
	u32	h2c_cmd = 0;
	u32	h2c_cmd_ex = 0;
	s32 ret = _FAIL;

	_func_enter_;

	padapter = GET_PRIMARY_ADAPTER(padapter);
	pHalData = GET_HAL_DATA(padapter);


	if (padapter->bFWReady == _FALSE) {
		//DBG_8192C("FillH2CCmd_8812(): return H2C cmd because fw is not ready\n");
		return ret;
	}

	_enter_critical_mutex(&(adapter_to_dvobj(padapter)->h2c_fwcmd_mutex),
			      NULL);


	if (!pCmdBuffer) {
		goto exit;
	}
	if (CmdLen > RTL8812_MAX_CMD_LEN) {
		goto exit;
	}
	if (padapter->bSurpriseRemoved == _TRUE)
		goto exit;

	//pay attention to if  race condition happened in  H2C cmd setting.
	do {
		h2c_box_num = pHalData->LastHMEBoxNum;

		if (!_is_fw_read_cmd_down(padapter, h2c_box_num)) {
			DBG_8192C(" fw read cmd failed...\n");
			goto exit;
		}

		*(u8 *)(&h2c_cmd) = ElementID;

		if (CmdLen <= 3) {
			_rtw_memcpy((u8 *)(&h2c_cmd) + 1, pCmdBuffer, CmdLen);
		} else {
			_rtw_memcpy((u8 *)(&h2c_cmd) + 1, pCmdBuffer, 3);
			ext_cmd_len = CmdLen - 3;
			_rtw_memcpy((u8 *)(&h2c_cmd_ex), pCmdBuffer + 3, ext_cmd_len);

			//Write Ext command
			msgbox_ex_addr = REG_HMEBOX_EXT0_8812 + (h2c_box_num *
					 RTL8812_EX_MESSAGE_BOX_SIZE);
#ifdef CONFIG_H2C_EF
			for (cmd_idx = 0; cmd_idx < ext_cmd_len; cmd_idx++) {
				rtw_write8(padapter, msgbox_ex_addr + cmd_idx,
					   *((u8 *)(&h2c_cmd_ex) + cmd_idx));
			}
#else
			h2c_cmd_ex = le32_to_cpu(h2c_cmd_ex);
			rtw_write32(padapter, msgbox_ex_addr, h2c_cmd_ex);
#endif
		}
		// Write command
		msgbox_addr = REG_HMEBOX_0 + (h2c_box_num * RTL8812_MESSAGE_BOX_SIZE);
#ifdef CONFIG_H2C_EF
		for (cmd_idx = 0; cmd_idx < RTL8812_MESSAGE_BOX_SIZE; cmd_idx++) {
			rtw_write8(padapter, msgbox_addr + cmd_idx, *((u8 *)(&h2c_cmd) + cmd_idx));
		}
#else
		h2c_cmd = le32_to_cpu(h2c_cmd);
		rtw_write32(padapter, msgbox_addr, h2c_cmd);
#endif

		bcmd_down = _TRUE;

		//	DBG_8192C("MSG_BOX:%d,CmdLen(%d), reg:0x%x =>h2c_cmd:0x%x, reg:0x%x =>h2c_cmd_ex:0x%x ..\n"
		//	 	,pHalData->LastHMEBoxNum ,CmdLen,msgbox_addr,h2c_cmd,msgbox_ex_addr,h2c_cmd_ex);

		pHalData->LastHMEBoxNum = (h2c_box_num + 1) % RTL8812_MAX_H2C_BOX_NUMS;

	} while ((!bcmd_down) && (retry_cnts--));

	ret = _SUCCESS;

exit:

	_exit_critical_mutex(&(adapter_to_dvobj(padapter)->h2c_fwcmd_mutex), NULL);

	_func_exit_;

	return ret;
}

u8 rtl8812_h2c_msg_hdl(_adapter *padapter, unsigned char *pbuf)
{
	u8 ElementID, CmdLen;
	u8 *pCmdBuffer;
	struct cmd_msg_parm  *pcmdmsg;

	if (!pbuf)
		return H2C_PARAMETERS_ERROR;

	pcmdmsg = (struct cmd_msg_parm *)pbuf;
	ElementID = pcmdmsg->eid;
	CmdLen = pcmdmsg->sz;
	pCmdBuffer = pcmdmsg->buf;

	FillH2CCmd_8812(padapter, ElementID, CmdLen, pCmdBuffer);

	return H2C_SUCCESS;
}

u8 rtl8812_set_rssi_cmd(_adapter *padapter, u8 *param)
{
	u8	res = _SUCCESS;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	_func_enter_;

	*((u32 *) param) = cpu_to_le32(*((u32 *) param));

	FillH2CCmd_8812(padapter, H2C_8812_RSSI_REPORT, 4, param);

	_func_exit_;

	return res;
}

u8	Get_VHT_ENI(
	u32		IOTAction,
	u32		WirelessMode,
	u32		ratr_bitmap
)
{
	u8	Ret = 0;

	if (WirelessMode == WIRELESS_11_24AC) {
		if (ratr_bitmap & 0xfff00000)	// Mix , 2SS
			Ret = 3;
		else 					// Mix, 1SS
			Ret = 2;
	} else if (WirelessMode == WIRELESS_11_5AC) {
		Ret = 1;					// VHT
	}

	return (Ret << 4);
}

BOOLEAN Get_RA_ShortGI(
	PADAPTER			Adapter,
	struct sta_info		*psta,
	u8					shortGIrate
)
{
	BOOLEAN		bShortGI;
	struct mlme_ext_priv	*pmlmeext = &Adapter->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	bShortGI = shortGIrate;

#ifdef CONFIG_80211AC_VHT
	if (bShortGI &&
	    IsSupportedVHT(psta->wireless_mode) &&
	    (pmlmeinfo->assoc_AP_vendor == HT_IOT_PEER_REALTEK_JAGUAR_CCUTAP) &&
	    TEST_FLAG(psta->vhtpriv.ldpc_cap, LDPC_VHT_ENABLE_TX)
	   ) {
		if (psta->vhtpriv.vht_highest_rate >= MGN_VHT2SS_MCS8)
			bShortGI = _FALSE;
	}
#endif

	return bShortGI;
}


void Set_RA_LDPC_8812(
	struct sta_info	*psta,
	BOOLEAN			bLDPC
)
{
	if (psta == NULL)
		return;
#ifdef CONFIG_80211AC_VHT
	if (psta->wireless_mode == WIRELESS_11_5AC) {
		if (bLDPC && TEST_FLAG(psta->vhtpriv.ldpc_cap, LDPC_VHT_CAP_TX))
			SET_FLAG(psta->vhtpriv.ldpc_cap, LDPC_VHT_ENABLE_TX);
		else
			CLEAR_FLAG(psta->vhtpriv.ldpc_cap, LDPC_VHT_ENABLE_TX);
	} else if (IsSupportedTxHT(psta->wireless_mode)
		   || IsSupportedVHT(psta->wireless_mode)) {
		if (bLDPC && TEST_FLAG(psta->htpriv.ldpc_cap, LDPC_HT_CAP_TX))
			SET_FLAG(psta->htpriv.ldpc_cap, LDPC_HT_ENABLE_TX);
		else
			CLEAR_FLAG(psta->htpriv.ldpc_cap, LDPC_HT_ENABLE_TX);
	}
#endif

	//DBG_871X("MacId %d bLDPC %d\n", psta->mac_id, bLDPC);
}


u8 Get_RA_LDPC_8812(
	struct sta_info		*psta
)
{
	u8	bLDPC = 0;

	if (psta->mac_id == 1)
		bLDPC = 0;
	else if (psta != NULL) {
#ifdef CONFIG_80211AC_VHT
		if (IsSupportedVHT(psta->wireless_mode)) {
			if (TEST_FLAG(psta->vhtpriv.ldpc_cap, LDPC_VHT_CAP_TX))
				bLDPC = 1;
			else
				bLDPC = 0;
		} else if (IsSupportedTxHT(psta->wireless_mode)) {
			if (TEST_FLAG(psta->htpriv.ldpc_cap, LDPC_HT_CAP_TX))
				bLDPC = 1;
			else
				bLDPC = 0;
		} else
#endif
			bLDPC = 0;
	}

	return (bLDPC << 2);
}

void rtl8812_set_raid_cmd(PADAPTER padapter, u32 bitmap, u8 *arg)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;
	struct mlme_ext_priv	*pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct sta_info	*psta;
	u8 macid, init_rate, raid, shortGIrate = _FALSE;

	_func_enter_;

	macid = arg[0];
	raid = arg[1];
	shortGIrate = arg[2];
	init_rate = arg[3];

	psta = pmlmeinfo->FW_sta_info[macid].psta;
	if (psta == NULL) {
		return;
	}

	if (pHalData->fw_ractrl == _TRUE) {
		u8	H2CCommand[7] = {0};

		shortGIrate = Get_RA_ShortGI(padapter, psta, shortGIrate);

		H2CCommand[0] = macid;
		H2CCommand[1] = (raid & 0x1F) | (shortGIrate ? 0x80 : 0x00) ;
		H2CCommand[2] = (pmlmeext->cur_bwmode & 0x3) | Get_RA_LDPC_8812(
					psta) | Get_VHT_ENI(0, psta->wireless_mode, bitmap);

		H2CCommand[3] = (u8)(bitmap & 0x000000ff);
		H2CCommand[4] = (u8)((bitmap & 0x0000ff00) >> 8);
		H2CCommand[5] = (u8)((bitmap & 0x00ff0000) >> 16);
		H2CCommand[6] = (u8)((bitmap & 0xff000000) >> 24);

		DBG_871X("rtl8812_set_raid_cmd, bitmap=0x%x, mac_id=0x%x, raid=0x%x, shortGIrate=%x\n",
			 bitmap, macid, raid, shortGIrate);

		FillH2CCmd_8812(padapter, H2C_8812_RA_MASK, 7, H2CCommand);
	}

	if (shortGIrate == _TRUE)
		init_rate |= BIT(7);

	pdmpriv->INIDATA_RATE[macid] = init_rate;

	_func_exit_;

}

void rtl8812_Add_RateATid(PADAPTER pAdapter, u32 bitmap, u8 *arg,
			  u8 rssi_level)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	u8	macid;

	macid = arg[0];

#ifdef CONFIG_ODM_REFRESH_RAMASK
	if (rssi_level != DM_RATR_STA_INIT)
		bitmap = ODM_Get_Rate_Bitmap(&pHalData->odmpriv, macid, bitmap,
					     rssi_level);
#endif //CONFIG_ODM_REFRESH_RAMASK

	rtl8812_set_raid_cmd(pAdapter, bitmap, arg);
}

void rtl8812_set_FwPwrMode_cmd(PADAPTER padapter, u8 PSMode)
{
	u8	u1H2CSetPwrMode[5] = {0};
	struct pwrctrl_priv *pwrpriv = &padapter->pwrctrlpriv;
	u8	Mode = 0, RLBM = 0, PowerState = 0, LPSAwakeIntvl = 1;

	_func_enter_;

	DBG_871X("%s: Mode=%d SmartPS=%d UAPSD=%d\n", __FUNCTION__,
		 PSMode, pwrpriv->smart_ps, padapter->registrypriv.uapsd_enable);

	switch (PSMode) {
	case PS_MODE_ACTIVE:
		Mode = 0;
		break;
	case PS_MODE_MIN:
		Mode = 1;
		break;
	case PS_MODE_MAX:
		RLBM = 1;
		Mode = 1;
		break;
	case PS_MODE_DTIM:
		RLBM = 2;
		Mode = 1;
		break;
	case PS_MODE_UAPSD_WMM:
		Mode = 2;
		break;
	default:
		Mode = 0;
		break;
	}

	if (Mode > PS_MODE_ACTIVE) {
		PowerState = 0x00;// AllON(0x0C), RFON(0x04), RFOFF(0x00)
#ifdef CONFIG_EXT_CLK
		Mode |= BIT(7);//supporting 26M XTAL CLK_Request feature.
#endif //CONFIG_EXT_CLK
	} else {
		PowerState = 0x0C;// AllON(0x0C), RFON(0x04), RFOFF(0x00)
	}

	// 0: Active, 1: LPS, 2: WMMPS
	SET_8812_H2CCMD_PWRMODE_PARM_MODE(u1H2CSetPwrMode, Mode);

	// 0:Min, 1:Max , 2:User define
	SET_8812_H2CCMD_PWRMODE_PARM_RLBM(u1H2CSetPwrMode, RLBM);

	// (LPS) smart_ps:  0: PS_Poll, 1: PS_Poll , 2: NullData
	// (WMM)smart_ps: 0:PS_Poll, 1:NullData
	SET_8812_H2CCMD_PWRMODE_PARM_SMART_PS(u1H2CSetPwrMode, pwrpriv->smart_ps);

	// AwakeInterval: Unit is beacon interval, this field is only valid in PS_DTIM mode
	SET_8812_H2CCMD_PWRMODE_PARM_BCN_PASS_TIME(u1H2CSetPwrMode, LPSAwakeIntvl);

	// (WMM only)bAllQueueUAPSD
	SET_8812_H2CCMD_PWRMODE_PARM_ALL_QUEUE_UAPSD(u1H2CSetPwrMode,
			padapter->registrypriv.uapsd_enable);

	// AllON(0x0C), RFON(0x04), RFOFF(0x00)
	SET_8812_H2CCMD_PWRMODE_PARM_PWR_STATE(u1H2CSetPwrMode, PowerState);

	FillH2CCmd_8812(padapter, H2C_8812_SETPWRMODE, sizeof(u1H2CSetPwrMode),
			(u8 *)&u1H2CSetPwrMode);

	_func_exit_;
}

void rtl8812_set_FwMediaStatus_cmd(PADAPTER padapter, u16 mstatus_rpt)
{
	u8	u1JoinBssRptParm[3] = {0};
	u8	mstatus, macId, macId_Ind = 0, macId_End = 0;

	mstatus = (u8)(mstatus_rpt & 0xFF);
	macId = (u8)(mstatus_rpt >> 8)  ;

	SET_8812_H2CCMD_MSRRPT_PARM_OPMODE(u1JoinBssRptParm, mstatus);
	SET_8812_H2CCMD_MSRRPT_PARM_MACID_IND(u1JoinBssRptParm, macId_Ind);

	SET_8812_H2CCMD_MSRRPT_PARM_MACID(u1JoinBssRptParm, macId);
	SET_8812_H2CCMD_MSRRPT_PARM_MACID_END(u1JoinBssRptParm, macId_End);

	DBG_871X("[MacId],  Set MacId Ctrl(original) = 0x%x \n",
		 u1JoinBssRptParm[0] << 16 | u1JoinBssRptParm[1] << 8 |
		 u1JoinBssRptParm[2]);

	FillH2CCmd_8812(padapter, H2C_8812_MSRRPT, 3, u1JoinBssRptParm);
}

void ConstructBeacon(_adapter *padapter, u8 *pframe, u32 *pLength)
{
	struct rtw_ieee80211_hdr	*pwlanhdr;
	u16					*fctrl;
	u32					rate_len, pktlen;
	struct mlme_ext_priv	*pmlmeext = &(padapter->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);
	u8	bc_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};


	//DBG_871X("%s\n", __FUNCTION__);

	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	_rtw_memcpy(pwlanhdr->addr1, bc_addr, ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr2, myid(&(padapter->eeprompriv)), ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr3, get_my_bssid(cur_network), ETH_ALEN);

	SetSeqNum(pwlanhdr, 0/*pmlmeext->mgnt_seq*/);
	//pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_BEACON);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	//timestamp will be inserted by hardware
	pframe += 8;
	pktlen += 8;

	// beacon interval: 2 bytes
	_rtw_memcpy(pframe, (unsigned char *)(rtw_get_beacon_interval_from_ie(
			cur_network->IEs)), 2);

	pframe += 2;
	pktlen += 2;

	// capability info: 2 bytes
	_rtw_memcpy(pframe, (unsigned char *)(rtw_get_capability_from_ie(
			cur_network->IEs)), 2);

	pframe += 2;
	pktlen += 2;

	if ((pmlmeinfo->state & 0x03) == WIFI_FW_AP_STATE) {
		//DBG_871X("ie len=%d\n", cur_network->IELength);
		pktlen += cur_network->IELength - sizeof(NDIS_802_11_FIXED_IEs);
		_rtw_memcpy(pframe, cur_network->IEs + sizeof(NDIS_802_11_FIXED_IEs),
			    pktlen);

		goto _ConstructBeacon;
	}

	//below for ad-hoc mode

	// SSID
	pframe = rtw_set_ie(pframe, _SSID_IE_, cur_network->Ssid.SsidLength,
			    cur_network->Ssid.Ssid, &pktlen);

	// supported rates...
	rate_len = rtw_get_rateset_len(cur_network->SupportedRates);
	pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_,
			    ((rate_len > 8) ? 8 : rate_len), cur_network->SupportedRates, &pktlen);

	// DS parameter set
	pframe = rtw_set_ie(pframe, _DSSET_IE_, 1,
			    (unsigned char *) & (cur_network->Configuration.DSConfig), &pktlen);

	if ((pmlmeinfo->state & 0x03) == WIFI_FW_ADHOC_STATE) {
		u32 ATIMWindow;
		// IBSS Parameter Set...
		//ATIMWindow = cur->Configuration.ATIMWindow;
		ATIMWindow = 0;
		pframe = rtw_set_ie(pframe, _IBSS_PARA_IE_, 2,
				    (unsigned char *)(&ATIMWindow), &pktlen);
	}


	//todo: ERP IE


	// EXTERNDED SUPPORTED RATE
	if (rate_len > 8) {
		pframe = rtw_set_ie(pframe, _EXT_SUPPORTEDRATES_IE_, (rate_len - 8),
				    (cur_network->SupportedRates + 8), &pktlen);
	}


	//todo:HT for adhoc

_ConstructBeacon:

	if ((pktlen + TXDESC_SIZE) > 512) {
		DBG_871X("beacon frame too large\n");
		return;
	}

	*pLength = pktlen;

	//DBG_871X("%s bcn_sz=%d\n", __FUNCTION__, pktlen);

}

void ConstructPSPoll(_adapter *padapter, u8 *pframe, u32 *pLength)
{
	struct rtw_ieee80211_hdr	*pwlanhdr;
	u16					*fctrl;
	u32					pktlen;
	struct mlme_ext_priv	*pmlmeext = &(padapter->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	//DBG_871X("%s\n", __FUNCTION__);

	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	// Frame control.
	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;
	SetPwrMgt(fctrl);
	SetFrameSubType(pframe, WIFI_PSPOLL);

	// AID.
	SetDuration(pframe, (pmlmeinfo->aid | 0xc000));

	// BSSID.
	_rtw_memcpy(pwlanhdr->addr1, get_my_bssid(&(pmlmeinfo->network)),
		    ETH_ALEN);

	// TA.
	_rtw_memcpy(pwlanhdr->addr2, myid(&(padapter->eeprompriv)), ETH_ALEN);

	*pLength = 16;
}

void ConstructNullFunctionData(
	PADAPTER padapter,
	u8		*pframe,
	u32		*pLength,
	u8		*StaAddr,
	u8		bQoS,
	u8		AC,
	u8		bEosp,
	u8		bForcePowerSave)
{
	struct rtw_ieee80211_hdr	*pwlanhdr;
	u16						*fctrl;
	u32						pktlen;
	struct mlme_priv		*pmlmepriv = &padapter->mlmepriv;
	struct wlan_network		*cur_network = &pmlmepriv->cur_network;
	struct mlme_ext_priv	*pmlmeext = &(padapter->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);


	//DBG_871X("%s:%d\n", __FUNCTION__, bForcePowerSave);

	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &pwlanhdr->frame_ctl;
	*(fctrl) = 0;
	if (bForcePowerSave) {
		SetPwrMgt(fctrl);
	}

	switch (cur_network->network.InfrastructureMode) {
	case Ndis802_11Infrastructure:
		SetToDs(fctrl);
		_rtw_memcpy(pwlanhdr->addr1, get_my_bssid(&(pmlmeinfo->network)),
			    ETH_ALEN);
		_rtw_memcpy(pwlanhdr->addr2, myid(&(padapter->eeprompriv)), ETH_ALEN);
		_rtw_memcpy(pwlanhdr->addr3, StaAddr, ETH_ALEN);
		break;
	case Ndis802_11APMode:
		SetFrDs(fctrl);
		_rtw_memcpy(pwlanhdr->addr1, StaAddr, ETH_ALEN);
		_rtw_memcpy(pwlanhdr->addr2, get_my_bssid(&(pmlmeinfo->network)),
			    ETH_ALEN);
		_rtw_memcpy(pwlanhdr->addr3, myid(&(padapter->eeprompriv)), ETH_ALEN);
		break;
	case Ndis802_11IBSS:
	default:
		_rtw_memcpy(pwlanhdr->addr1, StaAddr, ETH_ALEN);
		_rtw_memcpy(pwlanhdr->addr2, myid(&(padapter->eeprompriv)), ETH_ALEN);
		_rtw_memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)),
			    ETH_ALEN);
		break;
	}

	SetSeqNum(pwlanhdr, 0);

	if (bQoS == _TRUE) {
		struct rtw_ieee80211_hdr_3addr_qos *pwlanqoshdr;

		SetFrameSubType(pframe, WIFI_QOS_DATA_NULL);

		pwlanqoshdr = (struct rtw_ieee80211_hdr_3addr_qos *)pframe;
		SetPriority(&pwlanqoshdr->qc, AC);
		SetEOSP(&pwlanqoshdr->qc, bEosp);

		pktlen = sizeof(struct rtw_ieee80211_hdr_3addr_qos);
	} else {
		SetFrameSubType(pframe, WIFI_DATA_NULL);

		pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);
	}

	*pLength = pktlen;
}

void ConstructProbeRsp(_adapter *padapter, u8 *pframe, u32 *pLength,
		       u8 *StaAddr, BOOLEAN bHideSSID)
{
	struct rtw_ieee80211_hdr	*pwlanhdr;
	u16					*fctrl;
	u8					*mac, *bssid;
	u32					pktlen;
	struct mlme_ext_priv	*pmlmeext = &(padapter->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);


	//DBG_871X("%s\n", __FUNCTION__);

	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	mac = myid(&(padapter->eeprompriv));
	bssid = cur_network->MacAddress;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;
	_rtw_memcpy(pwlanhdr->addr1, StaAddr, ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr2, mac, ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr3, bssid, ETH_ALEN);

	SetSeqNum(pwlanhdr, 0);
	SetFrameSubType(fctrl, WIFI_PROBERSP);

	pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);
	pframe += pktlen;

	if (cur_network->IELength > MAX_IE_SZ)
		return;

	_rtw_memcpy(pframe, cur_network->IEs, cur_network->IELength);
	pframe += cur_network->IELength;
	pktlen += cur_network->IELength;

	*pLength = pktlen;
}

// To check if reserved page content is destroyed by beacon beacuse beacon is too large.
// 2010.06.23. Added by tynli.
VOID CheckFwRsvdPageContent(
	IN	PADAPTER		Adapter
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u32	MaxBcnPageNum;

	if (pHalData->FwRsvdPageStartOffset != 0) {
		/*MaxBcnPageNum = PageNum_128(pMgntInfo->MaxBeaconSize);
		RT_ASSERT((MaxBcnPageNum <= pHalData->FwRsvdPageStartOffset),
			("CheckFwRsvdPageContent(): The reserved page content has been"\
			"destroyed by beacon!!! MaxBcnPageNum(%d) FwRsvdPageStartOffset(%d)\n!",
			MaxBcnPageNum, pHalData->FwRsvdPageStartOffset));*/
	}
}

//
// Description: Get the reserved page number in Tx packet buffer.
// Retrun value: the page number.
// 2012.08.09, by tynli.
//
u8 GetTxBufferRsvdPageNum8812(
	IN	PADAPTER	Adapter,
	IN	BOOLEAN		bWoWLANBoundary
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u8	RsvdPageNum = 0;
	u8	TxPageBndy =
		LAST_ENTRY_OF_TX_PKT_BUFFER_8812; // default reseved 1 page for the IC type which is undefined.

	if (bWoWLANBoundary) {
		rtw_hal_get_def_var(Adapter, HAL_DEF_TX_PAGE_BOUNDARY_WOWLAN,
				    (u8 *)&TxPageBndy);
	} else {
		rtw_hal_get_def_var(Adapter, HAL_DEF_TX_PAGE_BOUNDARY, (u8 *)&TxPageBndy);
	}

	RsvdPageNum = LAST_ENTRY_OF_TX_PKT_BUFFER_8812 - TxPageBndy + 1;

	return RsvdPageNum;
}

//
// Description: Fill the reserved packets that FW will use to RSVD page.
//			Now we just send 4 types packet to rsvd page.
//			(1)Beacon, (2)Ps-poll, (3)Null data, (4)ProbeRsp.
//	Input:
//	    bDLFinished - FALSE: At the first time we will send all the packets as a large packet to Hw,
//				 		so we need to set the packet length to total lengh.
//			      TRUE: At the second time, we should send the first packet (default:beacon)
//						to Hw again and set the lengh in descriptor to the real beacon lengh.
// 2009.10.15 by tynli.
static void SetFwRsvdPagePkt_8812(PADAPTER padapter, BOOLEAN bDLFinished)
{
	PHAL_DATA_TYPE pHalData;
	struct xmit_frame	*pcmdframe;
	struct pkt_attrib	*pattrib;
	struct xmit_priv	*pxmitpriv;
	struct mlme_ext_priv	*pmlmeext;
	struct mlme_ext_info	*pmlmeinfo;
	u32	PSPollLength, NullFunctionDataLength, QosNullLength;
	u32	BcnLen;
	u8	TotalPageNum = 0, CurtPktPageNum = 0, TxDescLen = 0, RsvdPageNum = 0;
	u8	*ReservedPagePacket;
	u8	RsvdPageLoc[5] = {0};
	u16	BufIndex = 0, PageSize = 256;
	u32	TotalPacketLen, MaxRsvdPageBufSize = 0;;


	//DBG_871X("%s\n", __FUNCTION__);

	pHalData = GET_HAL_DATA(padapter);
	pxmitpriv = &padapter->xmitpriv;
	pmlmeext = &padapter->mlmeextpriv;
	pmlmeinfo = &pmlmeext->mlmext_info;

	if (IS_HARDWARE_TYPE_8812(padapter))
		PageSize = 512;
	else if (IS_HARDWARE_TYPE_8821(padapter))
		PageSize = PAGE_SIZE_TX_8821A;

	// <tynli_note> The function SetFwRsvdPagePkt_8812() input must be added a value "bDLWholePackets" to
	// decide if download wowlan packets, and use "bDLWholePackets" to be GetTxBufferRsvdPageNum8812() 2nd input value.
	RsvdPageNum = GetTxBufferRsvdPageNum8812(padapter, _FALSE);
	MaxRsvdPageBufSize = RsvdPageNum * PageSize;

	pcmdframe = rtw_alloc_cmdxmitframe(pxmitpriv, MaxRsvdPageBufSize);
	if (pcmdframe == NULL) {
		return;
	}

	ReservedPagePacket = pcmdframe->buf_addr;

	TxDescLen =
		TXDESC_SIZE;//The desc lengh in Tx packet buffer of 8812A is 40 bytes.

	//(1) beacon
	BufIndex = TXDESC_OFFSET;
	ConstructBeacon(padapter, &ReservedPagePacket[BufIndex], &BcnLen);

	// When we count the first page size, we need to reserve description size for the RSVD
	// packet, it will be filled in front of the packet in TXPKTBUF.
	CurtPktPageNum = (u8)PageNum(BcnLen + TxDescLen, PageSize);

	if (bDLFinished) {
		TotalPageNum += CurtPktPageNum;
		TotalPacketLen = (TotalPageNum * PageSize);
		DBG_871X("%s(): Beacon page size = %d\n", __FUNCTION__, TotalPageNum);
	} else {
		TotalPageNum += CurtPktPageNum;

		pHalData->FwRsvdPageStartOffset = TotalPageNum;

		BufIndex += (CurtPktPageNum * PageSize);

		if (BufIndex > MaxRsvdPageBufSize) {
			DBG_871X("%s(): Beacon: The rsvd page size is not enough!!BufIndex %d, MaxRsvdPageBufSize %d\n",
				 __FUNCTION__,
				 BufIndex, MaxRsvdPageBufSize);
			goto error;
		}

		//(2) ps-poll
		ConstructPSPoll(padapter, &ReservedPagePacket[BufIndex], &PSPollLength);
		rtl8812a_fill_fake_txdesc(padapter,
					  &ReservedPagePacket[BufIndex - TxDescLen], PSPollLength, _TRUE, _FALSE);

		SET_8812_H2CCMD_RSVDPAGE_LOC_PSPOLL(RsvdPageLoc, TotalPageNum);

		//DBG_871X("SetFwRsvdPagePkt_8812(): HW_VAR_SET_TX_CMD: PS-POLL %p %d\n",
		//	&ReservedPagePacket[BufIndex-TxDescLen], (PSPollLength+TxDescLen));

		CurtPktPageNum = (u8)PageNum(PSPollLength + TxDescLen, PageSize);

		BufIndex += (CurtPktPageNum * PageSize);

		TotalPageNum += CurtPktPageNum;

		if (BufIndex > MaxRsvdPageBufSize) {
			DBG_871X("%s(): ps-poll: The rsvd page size is not enough!!BufIndex %d, MaxRsvdPageBufSize %d\n",
				 __FUNCTION__,
				 BufIndex, MaxRsvdPageBufSize);
			goto error;
		}

		//(3) null data
		ConstructNullFunctionData(
			padapter,
			&ReservedPagePacket[BufIndex],
			&NullFunctionDataLength,
			get_my_bssid(&pmlmeinfo->network),
			_FALSE, 0, 0, _FALSE);
		rtl8812a_fill_fake_txdesc(padapter,
					  &ReservedPagePacket[BufIndex - TxDescLen], NullFunctionDataLength, _FALSE,
					  _FALSE);

		SET_8812_H2CCMD_RSVDPAGE_LOC_NULL_DATA(RsvdPageLoc, TotalPageNum);

		//DBG_871X("SetFwRsvdPagePkt_8812(): HW_VAR_SET_TX_CMD: NULL DATA %p %d\n",
		//	&ReservedPagePacket[BufIndex-TxDescLen], (NullFunctionDataLength+TxDescLen));

		CurtPktPageNum = (u8)PageNum(NullFunctionDataLength + TxDescLen, PageSize);

		BufIndex += (CurtPktPageNum * PageSize);

		TotalPageNum += CurtPktPageNum;

		if (BufIndex > MaxRsvdPageBufSize) {
			DBG_871X("%s(): Null-data: The rsvd page size is not enough!!BufIndex %d, MaxRsvdPageBufSize %d\n",
				 __FUNCTION__,
				 BufIndex, MaxRsvdPageBufSize);
			goto error;
		}

		//(5) Qos null data
		ConstructNullFunctionData(
			padapter,
			&ReservedPagePacket[BufIndex],
			&QosNullLength,
			get_my_bssid(&pmlmeinfo->network),
			_TRUE, 0, 0, _FALSE);
		rtl8812a_fill_fake_txdesc(padapter,
					  &ReservedPagePacket[BufIndex - TxDescLen], QosNullLength, _FALSE, _FALSE);

		SET_8812_H2CCMD_RSVDPAGE_LOC_QOS_NULL_DATA(RsvdPageLoc, TotalPageNum);

		//DBG_871X("SetFwRsvdPagePkt_8812(): HW_VAR_SET_TX_CMD: QOS NULL DATA %p %d\n",
		//	&ReservedPagePacket[BufIndex-TxDescLen], (QosNullLength+TxDescLen));

		CurtPktPageNum = (u8)PageNum(QosNullLength + TxDescLen, PageSize);

		BufIndex += (CurtPktPageNum * PageSize);

		TotalPageNum += CurtPktPageNum;

		TotalPacketLen = (TotalPageNum * PageSize);
	}


	if (TotalPacketLen > MaxRsvdPageBufSize) {
		DBG_871X("%s(): ERROR: The rsvd page size is not enough!!TotalPacketLen %d, MaxRsvdPageBufSize %d\n",
			 __FUNCTION__,
			 TotalPacketLen, MaxRsvdPageBufSize);
		goto error;
	} else {
		// update attribute
		pattrib = &pcmdframe->attrib;
		update_mgntframe_attrib(padapter, pattrib);
		pattrib->qsel = 0x10;
		pattrib->pktlen = pattrib->last_txcmdsz = TotalPacketLen - TxDescLen;

		dump_mgntframe_and_wait(padapter, pcmdframe, 100);
	}

	if (!bDLFinished) {
		DBG_871X("%s: Set RSVD page location to Fw ,TotalPacketLen(%d), TotalPageNum(%d)\n",
			 __FUNCTION__, TotalPacketLen, TotalPageNum);
		FillH2CCmd_8812(padapter, H2C_8812_RSVDPAGE, 5, RsvdPageLoc);
	}

	rtw_free_cmd_xmitbuf(pxmitpriv);

	return;

error:
	rtw_free_cmdxmitframe(pxmitpriv, pcmdframe);
}

void rtl8812_set_FwJoinBssReport_cmd(PADAPTER padapter, u8 mstatus)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct mlme_ext_priv	*pmlmeext = &(padapter->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	BOOLEAN		bSendBeacon = _FALSE;
	BOOLEAN		bcn_valid = _FALSE;
	u8	DLBcnCount = 0;
	u32 poll = 0;

	_func_enter_;

	DBG_871X("%s mstatus(%x)\n", __FUNCTION__, mstatus);

	if (mstatus == 1) {
		// We should set AID, correct TSF, HW seq enable before set JoinBssReport to Fw in 88/92C.
		// Suggested by filen. Added by tynli.
		rtw_write16(padapter, REG_BCN_PSR_RPT, (0xC000 | pmlmeinfo->aid));
		// Do not set TSF again here or vWiFi beacon DMA INT will not work.
		//correct_TSF(padapter, pmlmeext);
		// Hw sequende enable by dedault. 2010.06.23. by tynli.
		//rtw_write16(padapter, REG_NQOS_SEQ, ((pmlmeext->mgnt_seq+100)&0xFFF));
		//rtw_write8(padapter, REG_HWSEQ_CTRL, 0xFF);

		//Set REG_CR bit 8. DMA beacon by SW.
		pHalData->RegCR_1 |= BIT0;
		rtw_write8(padapter,  REG_CR + 1, pHalData->RegCR_1);

		// Disable Hw protection for a time which revserd for Hw sending beacon.
		// Fix download reserved page packet fail that access collision with the protection time.
		// 2010.05.11. Added by tynli.
		//SetBcnCtrlReg(padapter, 0, BIT3);
		//SetBcnCtrlReg(padapter, BIT4, 0);
		rtw_write8(padapter, REG_BCN_CTRL, rtw_read8(padapter,
				REG_BCN_CTRL) & (~BIT(3)));
		rtw_write8(padapter, REG_BCN_CTRL, rtw_read8(padapter,
				REG_BCN_CTRL) | BIT(4));

		if (pHalData->RegFwHwTxQCtrl & BIT6) {
			DBG_871X("HalDownloadRSVDPage(): There is an Adapter is sending beacon.\n");
			bSendBeacon = _TRUE;
		}

		// Set FWHW_TXQ_CTRL 0x422[6]=0 to tell Hw the packet is not a real beacon frame.
		rtw_write8(padapter, REG_FWHW_TXQ_CTRL + 2,
			   (pHalData->RegFwHwTxQCtrl & (~BIT6)));
		pHalData->RegFwHwTxQCtrl &= (~BIT6);

		// Clear beacon valid check bit.
		rtw_hal_set_hwreg(padapter, HW_VAR_BCN_VALID, NULL);
		DLBcnCount = 0;
		poll = 0;
		do {
			// download rsvd page.
			SetFwRsvdPagePkt_8812(padapter, _FALSE);
			DLBcnCount++;
			do {
				rtw_yield_os();
				//rtw_mdelay_os(10);
				// check rsvd page download OK.
				rtw_hal_get_hwreg(padapter, HW_VAR_BCN_VALID, (u8 *)(&bcn_valid));
				poll++;
			} while (!bcn_valid && (poll % 10) != 0 && !padapter->bSurpriseRemoved
				 && !padapter->bDriverStopped);

		} while (!bcn_valid && DLBcnCount <= 100 && !padapter->bSurpriseRemoved
			 && !padapter->bDriverStopped);

		//RT_ASSERT(bcn_valid, ("HalDownloadRSVDPage88ES(): 1 Download RSVD page failed!\n"));
		if (padapter->bSurpriseRemoved || padapter->bDriverStopped) {
		} else if (!bcn_valid)
			DBG_871X("%s: 1 Download RSVD page failed! DLBcnCount:%u, poll:%u\n",
				 __FUNCTION__, DLBcnCount, poll);
		else
			DBG_871X("%s: 1 Download RSVD success! DLBcnCount:%u, poll:%u\n",
				 __FUNCTION__, DLBcnCount, poll);
		//
		// We just can send the reserved page twice during the time that Tx thread is stopped (e.g. pnpsetpower)
		// becuase we need to free the Tx BCN Desc which is used by the first reserved page packet.
		// At run time, we cannot get the Tx Desc until it is released in TxHandleInterrupt() so we will return
		// the beacon TCB in the following code. 2011.11.23. by tynli.
		//
		//if(bcn_valid && padapter->bEnterPnpSleep)
		if (0) {
			if (bSendBeacon) {
				rtw_hal_set_hwreg(padapter, HW_VAR_BCN_VALID, NULL);
				DLBcnCount = 0;
				poll = 0;
				do {
					SetFwRsvdPagePkt_8812(padapter, _TRUE);
					DLBcnCount++;

					do {
						rtw_yield_os();
						//rtw_mdelay_os(10);
						// check rsvd page download OK.
						rtw_hal_get_hwreg(padapter, HW_VAR_BCN_VALID, (u8 *)(&bcn_valid));
						poll++;
					} while (!bcn_valid && (poll % 10) != 0 && !padapter->bSurpriseRemoved
						 && !padapter->bDriverStopped);
				} while (!bcn_valid && DLBcnCount <= 100 && !padapter->bSurpriseRemoved
					 && !padapter->bDriverStopped);

				//RT_ASSERT(bcn_valid, ("HalDownloadRSVDPage(): 2 Download RSVD page failed!\n"));
				if (padapter->bSurpriseRemoved || padapter->bDriverStopped) {
				} else if (!bcn_valid)
					DBG_871X("%s: 2 Download RSVD page failed! DLBcnCount:%u, poll:%u\n",
						 __FUNCTION__, DLBcnCount, poll);
				else
					DBG_871X("%s: 2 Download RSVD success! DLBcnCount:%u, poll:%u\n",
						 __FUNCTION__, DLBcnCount, poll);
			}
		}

		// Enable Bcn
		//SetBcnCtrlReg(padapter, BIT3, 0);
		//SetBcnCtrlReg(padapter, 0, BIT4);
		rtw_write8(padapter, REG_BCN_CTRL, rtw_read8(padapter,
				REG_BCN_CTRL) | BIT(3));
		rtw_write8(padapter, REG_BCN_CTRL, rtw_read8(padapter,
				REG_BCN_CTRL) & (~BIT(4)));

		// To make sure that if there exists an adapter which would like to send beacon.
		// If exists, the origianl value of 0x422[6] will be 1, we should check this to
		// prevent from setting 0x422[6] to 0 after download reserved page, or it will cause
		// the beacon cannot be sent by HW.
		// 2010.06.23. Added by tynli.
		if (bSendBeacon) {
			rtw_write8(padapter, REG_FWHW_TXQ_CTRL + 2,
				   (pHalData->RegFwHwTxQCtrl | BIT6));
			pHalData->RegFwHwTxQCtrl |= BIT6;
		}

		//
		// Update RSVD page location H2C to Fw.
		//
		if (bcn_valid) {
			rtw_hal_set_hwreg(padapter, HW_VAR_BCN_VALID, NULL);
			DBG_871X("Set RSVD page location to Fw.\n");
			//FillH2CCmd88E(Adapter, H2C_88E_RSVDPAGE, H2C_RSVDPAGE_LOC_LENGTH, pMgntInfo->u1RsvdPageLoc);
		}

		// Do not enable HW DMA BCN or it will cause Pcie interface hang by timing issue. 2011.11.24. by tynli.
		//if(!padapter->bEnterPnpSleep)
		{
			// Clear CR[8] or beacon packet will not be send to TxBuf anymore.
			pHalData->RegCR_1 &= (~BIT0);
			rtw_write8(padapter,  REG_CR + 1, pHalData->RegCR_1);
		}
	}
#ifdef CONFIG_WOWLAN
	if (padapter->pwrctrlpriv.wowlan_mode) {
		u16	media_status;

		media_status = mstatus;
		rtl8812_set_FwMediaStatus_cmd(padapter, media_status);
		DBG_871X_LEVEL(_drv_info_, "%s wowlan_mode is on\n", __func__);
	} else {
		DBG_871X_LEVEL(_drv_info_, "%s wowlan_mode is off\n", __func__);
	}
#endif //CONFIG_WOWLAN
	_func_exit_;
}

#ifdef CONFIG_P2P_PS
void rtl8812_set_p2p_ps_offload_cmd(_adapter *padapter, u8 p2p_ps_state)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct pwrctrl_priv		*pwrpriv = &padapter->pwrctrlpriv;
	struct wifidirect_info	*pwdinfo = &(padapter->wdinfo);
	u8	*p2p_ps_offload = (u8 *)&pHalData->p2p_ps_offload;
	u8	i;

	_func_enter_;

#if 1
	switch (p2p_ps_state) {
	case P2P_PS_DISABLE:
		DBG_8192C("P2P_PS_DISABLE \n");
		_rtw_memset(p2p_ps_offload, 0, 1);
		break;
	case P2P_PS_ENABLE:
		DBG_8192C("P2P_PS_ENABLE \n");
		// update CTWindow value.
		if (pwdinfo->ctwindow > 0) {
			SET_8812_H2CCMD_P2P_PS_OFFLOAD_CTWINDOW_EN(p2p_ps_offload, 1);
			rtw_write8(padapter, REG_P2P_CTWIN, pwdinfo->ctwindow);
		}

		// hw only support 2 set of NoA
		for (i = 0 ; i < pwdinfo->noa_num ; i++) {
			// To control the register setting for which NOA
			rtw_write8(padapter, REG_NOA_DESC_SEL, (i << 4));
			if (i == 0) {
				SET_8812_H2CCMD_P2P_PS_OFFLOAD_NOA0_EN(p2p_ps_offload, 1);
			} else {
				SET_8812_H2CCMD_P2P_PS_OFFLOAD_NOA1_EN(p2p_ps_offload, 1);
			}

			// config P2P NoA Descriptor Register
			//DBG_8192C("%s(): noa_duration = %x\n",__FUNCTION__,pwdinfo->noa_duration[i]);
			rtw_write32(padapter, REG_NOA_DESC_DURATION, pwdinfo->noa_duration[i]);

			//DBG_8192C("%s(): noa_interval = %x\n",__FUNCTION__,pwdinfo->noa_interval[i]);
			rtw_write32(padapter, REG_NOA_DESC_INTERVAL, pwdinfo->noa_interval[i]);

			//DBG_8192C("%s(): start_time = %x\n",__FUNCTION__,pwdinfo->noa_start_time[i]);
			rtw_write32(padapter, REG_NOA_DESC_START, pwdinfo->noa_start_time[i]);

			//DBG_8192C("%s(): noa_count = %x\n",__FUNCTION__,pwdinfo->noa_count[i]);
			rtw_write8(padapter, REG_NOA_DESC_COUNT, pwdinfo->noa_count[i]);
		}

		if ((pwdinfo->opp_ps == 1) || (pwdinfo->noa_num > 0)) {
			// rst p2p circuit
			rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(4));

			SET_8812_H2CCMD_P2P_PS_OFFLOAD_ENABLE(p2p_ps_offload, 1);

			if (pwdinfo->role == P2P_ROLE_GO) {
				// 1: Owner, 0: Client
				SET_8812_H2CCMD_P2P_PS_OFFLOAD_ROLE(p2p_ps_offload, 1);
				SET_8812_H2CCMD_P2P_PS_OFFLOAD_ALLSTASLEEP(p2p_ps_offload, 0);
			} else {
				// 1: Owner, 0: Client
				SET_8812_H2CCMD_P2P_PS_OFFLOAD_ROLE(p2p_ps_offload, 0);
			}

			SET_8812_H2CCMD_P2P_PS_OFFLOAD_DISCOVERY(p2p_ps_offload, 0);
		}
		break;
	case P2P_PS_SCAN:
		DBG_8192C("P2P_PS_SCAN \n");
		SET_8812_H2CCMD_P2P_PS_OFFLOAD_DISCOVERY(p2p_ps_offload, 1);
		break;
	case P2P_PS_SCAN_DONE:
		DBG_8192C("P2P_PS_SCAN_DONE \n");
		SET_8812_H2CCMD_P2P_PS_OFFLOAD_DISCOVERY(p2p_ps_offload, 0);
		pwdinfo->p2p_ps_state = P2P_PS_ENABLE;
		break;
	default:
		break;
	}

	DBG_871X("P2P_PS_OFFLOAD : %x\n", p2p_ps_offload[0]);
	FillH2CCmd_8812(padapter, H2C_8812_P2P_PS_OFFLOAD, 1, p2p_ps_offload);
#endif

	_func_exit_;

}
#endif //CONFIG_P2P

#ifdef CONFIG_TSF_RESET_OFFLOAD
/*
	ask FW to Reset sync register at Beacon early interrupt
*/
u8 rtl8812_reset_tsf(_adapter *padapter, u8 reset_port)
{
	u8	buf[2];
	u8	res = _SUCCESS;

	s32 ret;
	_func_enter_;
	if (IFACE_PORT0 == reset_port) {
		buf[0] = 0x1;
		buf[1] = 0;
	} else {
		buf[0] = 0x0;
		buf[1] = 0x1;
	}

	ret = FillH2CCmd_8812(padapter, H2C_8812_TSF_RESET, 2, buf);

	_func_exit_;

	return res;
}

int reset_tsf(PADAPTER Adapter, u8 reset_port)
{
	u8 reset_cnt_before = 0, reset_cnt_after = 0, loop_cnt = 0;
	u32 reg_reset_tsf_cnt = (IFACE_PORT0 == reset_port) ?
				REG_FW_RESET_TSF_CNT_0 : REG_FW_RESET_TSF_CNT_1;
	u32 reg_bcncrtl = (IFACE_PORT0 == reset_port) ?
			  REG_BCN_CTRL_1 : REG_BCN_CTRL;

	rtw_scan_abort(
		Adapter->pbuddy_adapter);	/*	site survey will cause reset_tsf fail	*/
	reset_cnt_after = reset_cnt_before = rtw_read8(Adapter, reg_reset_tsf_cnt);
	rtl8812_reset_tsf(Adapter, reset_port);

	while ((reset_cnt_after == reset_cnt_before) && (loop_cnt < 10)) {
		rtw_msleep_os(100);
		loop_cnt++;
		reset_cnt_after = rtw_read8(Adapter, reg_reset_tsf_cnt);
	}

	return (loop_cnt >= 10) ? _FAIL : _TRUE;
}


#endif	// CONFIG_TSF_RESET_OFFLOAD

#ifdef CONFIG_WOWLAN
void rtl8812_set_wowlan_cmd(_adapter *padapter, u8 enable)
{
	u8		res = _SUCCESS;
	u32		test = 0;
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	SETWOWLAN_PARM		pwowlan_parm;
	struct pwrctrl_priv	*pwrpriv = &padapter->pwrctrlpriv;

	_func_enter_;
	DBG_871X_LEVEL(_drv_always_, "+%s+\n", __func__);

	pwowlan_parm.mode = 0;
	pwowlan_parm.gpio_index = 0;
	pwowlan_parm.gpio_duration = 0;
	pwowlan_parm.second_mode = 0;
	pwowlan_parm.reserve = 0;

	if (enable) {

		pwowlan_parm.mode |= FW_WOWLAN_FUN_EN;
		pwrpriv->wowlan_magic = _TRUE;
		pwrpriv->wowlan_unicast = _TRUE;

		if (pwrpriv->wowlan_pattern == _TRUE) {
			pwowlan_parm.mode |= FW_WOWLAN_PATTERN_MATCH;
			DBG_871X_LEVEL(_drv_info_, "%s 2.pwowlan_parm.mode=0x%x \n", __FUNCTION__,
				       pwowlan_parm.mode);
		}
		if (pwrpriv->wowlan_magic == _TRUE) {
			pwowlan_parm.mode |= FW_WOWLAN_MAGIC_PKT;
			DBG_871X_LEVEL(_drv_info_, "%s 3.pwowlan_parm.mode=0x%x \n", __FUNCTION__,
				       pwowlan_parm.mode);
		}
		if (pwrpriv->wowlan_unicast == _TRUE) {
			pwowlan_parm.mode |= FW_WOWLAN_UNICAST;
			DBG_871X_LEVEL(_drv_info_, "%s 4.pwowlan_parm.mode=0x%x \n", __FUNCTION__,
				       pwowlan_parm.mode);
		}

		if (!(padapter->pwrctrlpriv.wowlan_wake_reason & FWDecisionDisconnect))
			rtl8812a_set_FwJoinBssReport_cmd(padapter, 1);
		else
			DBG_871X_LEVEL(_drv_always_, "%s, disconnected, no FwJoinBssReport\n",
				       __FUNCTION__);
		rtw_msleep_os(2);

		//WOWLAN_GPIO_ACTIVE means GPIO high active
		//pwowlan_parm.mode |=FW_WOWLAN_GPIO_ACTIVE;
		//pwowlan_parm.mode |=FW_WOWLAN_REKEY_WAKEUP;
		pwowlan_parm.mode |= FW_WOWLAN_DEAUTH_WAKEUP;
		//pwowlan_parm.mode |=FW_WOWLAN_ALL_PKT_DROP;

		//DataPinWakeUp
		pwowlan_parm.gpio_index = 0x80;

		DBG_871X_LEVEL(_drv_info_, "%s 5.pwowlan_parm.mode=0x%x \n", __FUNCTION__,
			       pwowlan_parm.mode);
		DBG_871X_LEVEL(_drv_info_, "%s 6.pwowlan_parm.index=0x%x \n", __FUNCTION__,
			       pwowlan_parm.gpio_index);

		res = FillH2CCmd_8812(padapter, H2C_8812_WO_WLAN, 2, (u8 *)&pwowlan_parm);

		rtw_msleep_os(2);

		//disconnect decision
		pwowlan_parm.mode = 1;
		pwowlan_parm.gpio_index = 0;
		pwowlan_parm.gpio_duration = 0;
		FillH2CCmd_8812(padapter, H2C_8812_DISCONNECT_DECISION, 3,
				(u8 *)&pwowlan_parm);

		//keep alive period = 10 * 10 BCN interval
		pwowlan_parm.mode = 1;
		pwowlan_parm.gpio_index = 10;

		res = FillH2CCmd_8812(padapter, H2C_8812_KEEP_ALIVE_CTRL, 2,
				      (u8 *)&pwowlan_parm);

		rtw_msleep_os(2);
		//enable Remote wake ctrl
		pwowlan_parm.mode = 1;
		pwowlan_parm.gpio_index = 0;
		pwowlan_parm.gpio_duration = 0;

		res = FillH2CCmd_8812(padapter, H2C_8812_REMOTE_WAKE_CTRL, 3,
				      (u8 *)&pwowlan_parm);
	} else {
		pwrpriv->wowlan_magic = _FALSE;
		res = FillH2CCmd_8812(padapter, H2C_8812_WO_WLAN, 2, (u8 *)&pwowlan_parm);
		rtw_msleep_os(2);
		res = FillH2CCmd_8812(padapter, H2C_8812_REMOTE_WAKE_CTRL, 3,
				      (u8 *)&pwowlan_parm);
	}
	_func_exit_;
	DBG_871X_LEVEL(_drv_always_, "-%s res:%d-\n", __func__, res);
	return ;
}
#endif  //CONFIG_WOWLAN

