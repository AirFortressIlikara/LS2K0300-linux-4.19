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

//============================================================
// include files
//============================================================

#include "Mp_Precomp.h"
#include "phydm_precomp.h"


const u2Byte dB_Invert_Table[12][8] = {
	{	1,		1,		1,		2,		2,		2,		2,		3},
	{	3,		3,		4,		4,		4,		5,		6,		6},
	{	7,		8,		9,		10,		11,		13,		14,		16},
	{	18,		20,		22,		25,		28,		32,		35,		40},
	{	45,		50,		56,		63,		71,		79,		89,		100},
	{	112,		126,		141,		158,		178,		200,		224,		251},
	{	282,		316,		355,		398,		447,		501,		562,		631},
	{	708,		794,		891,		1000,	1122,	1259,	1413,	1585},
	{	1778,	1995,	2239,	2512,	2818,	3162,	3548,	3981},
	{	4467,	5012,	5623,	6310,	7079,	7943,	8913,	10000},
	{	11220,	12589,	14125,	15849,	17783,	19953,	22387,	25119},
	{	28184,	31623,	35481,	39811,	44668,	50119,	56234,	65535}
};


//============================================================
// Local Function predefine.
//============================================================

/* START------------COMMON INFO RELATED--------------- */

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
VOID ODM_UpdateInitRateWorkItemCallback(
	IN PVOID            pContext
);
#endif

VOID odm_GlobalAdapterCheck(
	IN		VOID
);

//move to odm_PowerTacking.h by YuChen


VOID odm_IQCalibrate(
	IN	PDM_ODM_T	pDM_Odm
);

//remove by YuChen for PT

VOID odm_UpdatePowerTrainingState(
	IN	PDM_ODM_T	pDM_Odm
);

//============================================================
//3 Export Interface
//============================================================

/*Y = 10*log(X)*/
s4Byte ODM_PWdB_Conversion(
	IN  s4Byte X,
	IN  u4Byte TotalBit,
	IN  u4Byte DecimalBit
)
{
	s4Byte Y, integer = 0, decimal = 0;
	u4Byte i;

	if (X == 0)
		X = 1; // log2(x), x can't be 0

	for (i = (TotalBit - 1); i > 0; i--) {
		if (X & BIT(i)) {
			integer = i;
			if (i > 0)
				decimal = (X & BIT(i - 1)) ? 2 : 0; //decimal is 0.5dB*3=1.5dB~=2dB
			break;
		}
	}

	Y = 3 * (integer - DecimalBit) + decimal; //10*log(x)=3*log2(x),

	return Y;
}

s4Byte ODM_SignConversion(
	IN  s4Byte value,
	IN  u4Byte TotalBit
)
{
	if (value & BIT(TotalBit - 1))
		value -= BIT(TotalBit);
	return value;
}

VOID ODM_InitMpDriverStatus(
	IN		PDM_ODM_T		pDM_Odm
)
{
#if(DM_ODM_SUPPORT_TYPE & ODM_WIN)

	// Decide when compile time
#if(MP_DRIVER == 1)
	pDM_Odm->mp_mode = TRUE;
#else
	pDM_Odm->mp_mode = FALSE;
#endif

#elif(DM_ODM_SUPPORT_TYPE & ODM_CE)

	PADAPTER	Adapter =  pDM_Odm->Adapter;

	// Update information every period
	pDM_Odm->mp_mode = (BOOLEAN)Adapter->registrypriv.mp_mode;

#else

	// MP mode is always false at AP side
	pDM_Odm->mp_mode = FALSE;

#endif
}

VOID ODM_UpdateMpDriverStatus(
	IN		PDM_ODM_T		pDM_Odm
)
{
#if(DM_ODM_SUPPORT_TYPE & ODM_WIN)

	// Do nothing.

#elif(DM_ODM_SUPPORT_TYPE & ODM_CE)
	PADAPTER	Adapter =  pDM_Odm->Adapter;

	// Update information erery period
	pDM_Odm->mp_mode = (BOOLEAN)Adapter->registrypriv.mp_mode;

#else

	// Do nothing.

#endif
}

VOID odm_CommonInfoSelfInit(
	IN		PDM_ODM_T		pDM_Odm
)
{
	pDM_Odm->bCckHighPower = (BOOLEAN) ODM_GetBBReg(pDM_Odm,
				 ODM_REG(CCK_RPT_FORMAT, pDM_Odm), ODM_BIT(CCK_RPT_FORMAT, pDM_Odm));
	pDM_Odm->RFPathRxEnable = (u1Byte) ODM_GetBBReg(pDM_Odm,
				  ODM_REG(BB_RX_PATH, pDM_Odm), ODM_BIT(BB_RX_PATH, pDM_Odm));
#if (DM_ODM_SUPPORT_TYPE != ODM_CE)
	pDM_Odm->pbNet_closed = &pDM_Odm->BOOLEAN_temp;
#endif

	PHYDM_InitDebugSetting(pDM_Odm);
	ODM_InitMpDriverStatus(pDM_Odm);

	pDM_Odm->TxRate = 0xFF;

}

VOID odm_CommonInfoSelfUpdate(
	IN		PDM_ODM_T		pDM_Odm
)
{
	u1Byte	EntryCnt = 0;
	u4Byte	i, OneEntry_MACID = 0;
	PSTA_INFO_T   	pEntry;

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)

	PADAPTER	Adapter =  pDM_Odm->Adapter;
	PMGNT_INFO	pMgntInfo = &Adapter->MgntInfo;

	pEntry = pDM_Odm->pODM_StaInfo[0];
	if (pMgntInfo->mAssoc) {
		pEntry->bUsed = TRUE;
		for (i = 0; i < 6; i++)
			pEntry->MacAddr[i] = pMgntInfo->Bssid[i];
	} else {
		pEntry->bUsed = FALSE;
		for (i = 0; i < 6; i++)
			pEntry->MacAddr[i] = 0;
	}

	//STA mode is linked to AP
	if (IS_STA_VALID(pDM_Odm->pODM_StaInfo[0]) && !ACTING_AS_AP(Adapter))
		pDM_Odm->bsta_state = TRUE;
	else
		pDM_Odm->bsta_state = FALSE;
#endif


	if (*(pDM_Odm->pBandWidth) == ODM_BW40M) {
		if (*(pDM_Odm->pSecChOffset) == 1)
			pDM_Odm->ControlChannel = *(pDM_Odm->pChannel) - 2;
		else if (*(pDM_Odm->pSecChOffset) == 2)
			pDM_Odm->ControlChannel = *(pDM_Odm->pChannel) + 2;
	} else
		pDM_Odm->ControlChannel = *(pDM_Odm->pChannel);

	for (i = 0; i < ODM_ASSOCIATE_ENTRY_NUM; i++) {
		pEntry = pDM_Odm->pODM_StaInfo[i];
		if (IS_STA_VALID(pEntry)) {
			EntryCnt++;
			if (EntryCnt == 1) {
				OneEntry_MACID = i;
			}
		}
	}

	if (EntryCnt == 1) {
		pDM_Odm->bOneEntryOnly = TRUE;
		pDM_Odm->OneEntry_MACID = OneEntry_MACID;
	} else
		pDM_Odm->bOneEntryOnly = FALSE;

	// Update MP driver status
	ODM_UpdateMpDriverStatus(pDM_Odm);
}

VOID odm_CommonInfoSelfReset(
	IN		PDM_ODM_T		pDM_Odm
)
{
#if( DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE))
	pDM_Odm->PhyDbgInfo.NumQryBeaconPkt = 0;
#endif
}

PVOID PhyDM_Get_Structure(
	IN		PDM_ODM_T		pDM_Odm,
	IN		u1Byte			Structure_Type
)

{
	PVOID	pStruct = NULL;
#if RTL8195A_SUPPORT
	switch (Structure_Type) {
	case	PHYDM_FALSEALMCNT:
		pStruct = &FalseAlmCnt;
		break;

	case	PHYDM_CFOTRACK:
		pStruct = &DM_CfoTrack;
		break;

	default:
		break;
	}

#else
	switch (Structure_Type) {
	case	PHYDM_FALSEALMCNT:
		pStruct = &(pDM_Odm->FalseAlmCnt);
		break;

	case	PHYDM_CFOTRACK:
		pStruct = &(pDM_Odm->DM_CfoTrack);
		break;

	default:
		break;
	}

#endif
	return	pStruct;
}

VOID odm_HWSetting(
	IN		PDM_ODM_T		pDM_Odm
)
{
#if (RTL8821A_SUPPORT == 1)
	if (pDM_Odm->SupportICType & ODM_RTL8821)
		odm_HWSetting_8821A(pDM_Odm);
#endif

}

//
// 2011/09/21 MH Add to describe different team necessary resource allocate??
//
VOID ODM_DMInit(
	IN		PDM_ODM_T		pDM_Odm
)
{

	odm_CommonInfoSelfInit(pDM_Odm);
	odm_DIGInit(pDM_Odm);
	Phydm_NHMCounterStatisticsInit(pDM_Odm);
	Phydm_AdaptivityInit(pDM_Odm);
	odm_RateAdaptiveMaskInit(pDM_Odm);
	odm_RA_ParaAdjust_init(pDM_Odm);
	ODM_CfoTrackingInit(pDM_Odm);
	ODM_EdcaTurboInit(pDM_Odm);
	odm_RSSIMonitorInit(pDM_Odm);
	odm_TXPowerTrackingInit(pDM_Odm);
	odm_AntennaDiversityInit(pDM_Odm);
	odm_AutoChannelSelectInit(pDM_Odm);
	odm_PathDiversityInit(pDM_Odm);

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE))
	ODM_ClearTxPowerTrackingState(pDM_Odm);
#endif

	if (pDM_Odm->SupportICType & ODM_IC_11N_SERIES) {
		odm_DynamicBBPowerSavingInit(pDM_Odm);
		odm_DynamicTxPowerInit(pDM_Odm);

#if (RTL8188E_SUPPORT == 1)
		if (pDM_Odm->SupportICType == ODM_RTL8188E) {
			odm_PrimaryCCA_Init(pDM_Odm);
			ODM_RAInfo_Init_all(pDM_Odm);
		}
#endif

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE))

#if (RTL8723B_SUPPORT == 1)
		if (pDM_Odm->SupportICType == ODM_RTL8723B)
			odm_SwAntDetectInit(pDM_Odm);
#endif

#if (RTL8192E_SUPPORT == 1)
		if (pDM_Odm->SupportICType == ODM_RTL8192E)
			odm_PrimaryCCA_Check_Init(pDM_Odm);
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
#if (RTL8723A_SUPPORT == 1)
		if (pDM_Odm->SupportICType == ODM_RTL8723A)
			odm_PSDMonitorInit(pDM_Odm);
#endif

#if (RTL8192D_SUPPORT == 1)
		if (pDM_Odm->SupportICType == ODM_RTL8192D)
			odm_PathDivInit_92D(pDM_Odm);
#endif

#if ((RTL8192C_SUPPORT == 1) || (RTL8192D_SUPPORT == 1))
		if (pDM_Odm->SupportICType & (ODM_RTL8192C | ODM_RTL8192D))
			odm_RXHPInit(pDM_Odm);
#endif
#endif
#endif

	}

}

VOID ODM_DMReset(
	IN		PDM_ODM_T		pDM_Odm
)
{
	ODM_AntDivReset(pDM_Odm);
}

#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
//
//tmp modify for LC Only
//
VOID ODM_DMWatchdog_LPS(
	IN		PDM_ODM_T		pDM_Odm
)
{
	odm_CommonInfoSelfUpdate(pDM_Odm);
	odm_FalseAlarmCounterStatistics(pDM_Odm);
	odm_RSSIMonitorCheck(pDM_Odm);
	odm_DIGbyRSSI_LPS(pDM_Odm);
	odm_CCKPacketDetectionThresh(pDM_Odm);
	odm_CommonInfoSelfReset(pDM_Odm);

	if (*(pDM_Odm->pbPowerSaving) == TRUE)
		return;
}
#endif
//
// 2011/09/20 MH This is the entry pointer for all team to execute HW out source DM.
// You can not add any dummy function here, be care, you can only use DM structure
// to perform any new ODM_DM.
//
VOID ODM_DMWatchdog(
	IN		PDM_ODM_T		pDM_Odm
)
{
	ODM_AsocEntry_Init(pDM_Odm);
	odm_CommonInfoSelfUpdate(pDM_Odm);
	phydm_BasicDbgMessage(pDM_Odm);
	odm_HWSetting(pDM_Odm);

#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
	{
		prtl8192cd_priv priv		= pDM_Odm->priv;
		if ((priv->auto_channel != 0)
		    && (priv->auto_channel != 2)) //if ACS running, do not do FA/CCA counter read
			return;
	}
#endif
	odm_FalseAlarmCounterStatistics(pDM_Odm);
	odm_RSSIMonitorCheck(pDM_Odm);

	if (*(pDM_Odm->pbPowerSaving) == TRUE) {
		odm_DIGbyRSSI_LPS(pDM_Odm);
		{
			pDIG_T	pDM_DigTable = &pDM_Odm->DM_DigTable;
			Phydm_Adaptivity(pDM_Odm, pDM_DigTable->CurIGValue);
		}
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_COMMON, ODM_DBG_LOUD,
			     ("DMWatchdog in power saving mode\n"));
		return;
	}

	Phydm_CheckAdaptivity(pDM_Odm);
	odm_UpdatePowerTrainingState(pDM_Odm);
	odm_DIG(pDM_Odm);
	{
		pDIG_T	pDM_DigTable = &pDM_Odm->DM_DigTable;
		Phydm_Adaptivity(pDM_Odm, pDM_DigTable->CurIGValue);
	}
	odm_CCKPacketDetectionThresh(pDM_Odm);
	odm_RefreshRateAdaptiveMask(pDM_Odm);
	odm_RefreshBasicRateMask(pDM_Odm);
	odm_DynamicBBPowerSaving(pDM_Odm);
	odm_EdcaTurboCheck(pDM_Odm);
	odm_PathDiversity(pDM_Odm);
	ODM_CfoTracking(pDM_Odm);
	odm_DynamicTxPower(pDM_Odm);
	odm_AntennaDiversity(pDM_Odm);

#if( DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE))

	ODM_TXPowerTrackingCheck(pDM_Odm);

	if (pDM_Odm->SupportICType & ODM_IC_11AC_SERIES)
		odm_IQCalibrate(pDM_Odm);
	else
#endif
		if (pDM_Odm->SupportICType & ODM_IC_11N_SERIES) {
#if (RTL8192D_SUPPORT == 1)
			if (pDM_Odm->SupportICType == ODM_RTL8192D)
				ODM_DynamicEarlyMode(pDM_Odm);
#endif

#if (RTL8188E_SUPPORT == 1)
			if (pDM_Odm->SupportICType == ODM_RTL8188E)
				odm_DynamicPrimaryCCA(pDM_Odm);
#endif

#if( DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE))

#if (RTL8192E_SUPPORT == 1)
			if (pDM_Odm->SupportICType == ODM_RTL8192E)
				odm_DynamicPrimaryCCA_Check(pDM_Odm);
#endif

#if( DM_ODM_SUPPORT_TYPE == ODM_WIN)
#if ((RTL8192C_SUPPORT == 1) || (RTL8192D_SUPPORT == 1))
			if (pDM_Odm->SupportICType & (ODM_RTL8192C | ODM_RTL8192D))
				odm_RXHP(pDM_Odm);
#endif
#endif
#endif
		}

#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	odm_dtc(pDM_Odm);
#endif

	odm_CommonInfoSelfReset(pDM_Odm);

}


//
// Init /.. Fixed HW value. Only init time.
//
VOID ODM_CmnInfoInit(
	IN		PDM_ODM_T		pDM_Odm,
	IN		ODM_CMNINFO_E	CmnInfo,
	IN		u4Byte			Value
)
{
	//
	// This section is used for init value
	//
	switch	(CmnInfo) {
	//
	// Fixed ODM value.
	//
	case	ODM_CMNINFO_ABILITY:
		pDM_Odm->SupportAbility = (u4Byte)Value;
		break;

	case	ODM_CMNINFO_RF_TYPE:
		pDM_Odm->RFType = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_PLATFORM:
		pDM_Odm->SupportPlatform = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_INTERFACE:
		pDM_Odm->SupportInterface = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_MP_TEST_CHIP:
		pDM_Odm->bIsMPChip = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_IC_TYPE:
		pDM_Odm->SupportICType = Value;
		break;

	case	ODM_CMNINFO_CUT_VER:
		pDM_Odm->CutVersion = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_FAB_VER:
		pDM_Odm->FabVersion = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_RFE_TYPE:
		pDM_Odm->RFEType = (u1Byte)Value;
		break;

	case    ODM_CMNINFO_RF_ANTENNA_TYPE:
		pDM_Odm->AntDivType = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_BOARD_TYPE:
		pDM_Odm->BoardType = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_PACKAGE_TYPE:
		pDM_Odm->PackageType = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_EXT_LNA:
		pDM_Odm->ExtLNA = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_5G_EXT_LNA:
		pDM_Odm->ExtLNA5G = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_EXT_PA:
		pDM_Odm->ExtPA = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_5G_EXT_PA:
		pDM_Odm->ExtPA5G = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_GPA:
		pDM_Odm->TypeGPA = (ODM_TYPE_GPA_E)Value;
		break;
	case	ODM_CMNINFO_APA:
		pDM_Odm->TypeAPA = (ODM_TYPE_APA_E)Value;
		break;
	case	ODM_CMNINFO_GLNA:
		pDM_Odm->TypeGLNA = (ODM_TYPE_GLNA_E)Value;
		break;
	case	ODM_CMNINFO_ALNA:
		pDM_Odm->TypeALNA = (ODM_TYPE_ALNA_E)Value;
		break;

	case	ODM_CMNINFO_EXT_TRSW:
		pDM_Odm->ExtTRSW = (u1Byte)Value;
		break;
	case 	ODM_CMNINFO_PATCH_ID:
		pDM_Odm->PatchID = (u1Byte)Value;
		break;
	case 	ODM_CMNINFO_BINHCT_TEST:
		pDM_Odm->bInHctTest = (BOOLEAN)Value;
		break;
	case 	ODM_CMNINFO_BWIFI_TEST:
		pDM_Odm->bWIFITest = (BOOLEAN)Value;
		break;
	case	ODM_CMNINFO_SMART_CONCURRENT:
		pDM_Odm->bDualMacSmartConcurrent = (BOOLEAN)Value;
		break;
	case	ODM_CMNINFO_DOMAIN_CODE_2G:
		pDM_Odm->odm_Regulation2_4G = (u1Byte)Value;
		break;
	case	ODM_CMNINFO_DOMAIN_CODE_5G:
		pDM_Odm->odm_Regulation5G = (u1Byte)Value;
		break;
	case	ODM_CMNINFO_CONFIG_BB_RF:
		pDM_Odm->ConfigBBRF = (BOOLEAN)Value;
		break;
	case	ODM_CMNINFO_IQKFWOFFLOAD:
		pDM_Odm->IQKFWOffload = (u1Byte)Value;
		break;
	//To remove the compiler warning, must add an empty default statement to handle the other values.
	default:
		//do nothing
		break;

	}

}


VOID ODM_CmnInfoHook(
	IN		PDM_ODM_T		pDM_Odm,
	IN		ODM_CMNINFO_E	CmnInfo,
	IN		PVOID			pValue
)
{
	//
	// Hook call by reference pointer.
	//
	switch	(CmnInfo) {
	//
	// Dynamic call by reference pointer.
	//
	case	ODM_CMNINFO_MAC_PHY_MODE:
		pDM_Odm->pMacPhyMode = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_TX_UNI:
		pDM_Odm->pNumTxBytesUnicast = (u8Byte *)pValue;
		break;

	case	ODM_CMNINFO_RX_UNI:
		pDM_Odm->pNumRxBytesUnicast = (u8Byte *)pValue;
		break;

	case	ODM_CMNINFO_WM_MODE:
		pDM_Odm->pWirelessMode = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_BAND:
		pDM_Odm->pBandType = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_SEC_CHNL_OFFSET:
		pDM_Odm->pSecChOffset = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_SEC_MODE:
		pDM_Odm->pSecurity = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_BW:
		pDM_Odm->pBandWidth = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_CHNL:
		pDM_Odm->pChannel = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_DMSP_GET_VALUE:
		pDM_Odm->pbGetValueFromOtherMac = (BOOLEAN *)pValue;
		break;

	case	ODM_CMNINFO_BUDDY_ADAPTOR:
		pDM_Odm->pBuddyAdapter = (PADAPTER *)pValue;
		break;

	case	ODM_CMNINFO_DMSP_IS_MASTER:
		pDM_Odm->pbMasterOfDMSP = (BOOLEAN *)pValue;
		break;

	case	ODM_CMNINFO_SCAN:
		pDM_Odm->pbScanInProcess = (BOOLEAN *)pValue;
		break;

	case	ODM_CMNINFO_POWER_SAVING:
		pDM_Odm->pbPowerSaving = (BOOLEAN *)pValue;
		break;

	case	ODM_CMNINFO_ONE_PATH_CCA:
		pDM_Odm->pOnePathCCA = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_DRV_STOP:
		pDM_Odm->pbDriverStopped = (BOOLEAN *)pValue;
		break;

	case	ODM_CMNINFO_PNP_IN:
		pDM_Odm->pbDriverIsGoingToPnpSetPowerSleep = (BOOLEAN *)pValue;
		break;

	case	ODM_CMNINFO_INIT_ON:
		pDM_Odm->pinit_adpt_in_progress = (BOOLEAN *)pValue;
		break;

	case	ODM_CMNINFO_ANT_TEST:
		pDM_Odm->pAntennaTest = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_NET_CLOSED:
		pDM_Odm->pbNet_closed = (BOOLEAN *)pValue;
		break;

	case 	ODM_CMNINFO_FORCED_RATE:
		pDM_Odm->pForcedDataRate = (pu2Byte)pValue;
		break;

	case  ODM_CMNINFO_FORCED_IGI_LB:
		pDM_Odm->pu1ForcedIgiLb = (u1Byte *)pValue;
		break;

	case	ODM_CMNINFO_P2P_LINK:
		pDM_Odm->DM_DigTable.pbP2pLinkInProgress = (u1Byte *)pValue;
		break;

	case 	ODM_CMNINFO_IS1ANTENNA:
		pDM_Odm->pIs1Antenna = (BOOLEAN *)pValue;
		break;

	case 	ODM_CMNINFO_RFDEFAULTPATH:
		pDM_Odm->pRFDefaultPath = (u1Byte *)pValue;
		break;

	case ODM_CMNINFO_FCS_MODE:
		pDM_Odm->pIsFcsModeEnable = (BOOLEAN *)pValue;
		break;

	//case	ODM_CMNINFO_RTSTA_AID:
	//	pDM_Odm->pAidMap =  (u1Byte *)pValue;
	//	break;

	//case	ODM_CMNINFO_BT_COEXIST:
	//	pDM_Odm->BTCoexist = (BOOLEAN *)pValue;

	//case	ODM_CMNINFO_STA_STATUS:
	//pDM_Odm->pODM_StaInfo[] = (PSTA_INFO_T)pValue;
	//break;

	//case	ODM_CMNINFO_PHY_STATUS:
	//	pDM_Odm->pPhyInfo = (ODM_PHY_INFO *)pValue;
	//	break;

	//case	ODM_CMNINFO_MAC_STATUS:
	//	pDM_Odm->pMacInfo = (ODM_MAC_INFO *)pValue;
	//	break;
	//To remove the compiler warning, must add an empty default statement to handle the other values.
	default:
		//do nothing
		break;

	}

}


VOID ODM_CmnInfoPtrArrayHook(
	IN		PDM_ODM_T		pDM_Odm,
	IN		ODM_CMNINFO_E	CmnInfo,
	IN		u2Byte			Index,
	IN		PVOID			pValue
)
{
	//
	// Hook call by reference pointer.
	//
	switch	(CmnInfo) {
	//
	// Dynamic call by reference pointer.
	//
	case	ODM_CMNINFO_STA_STATUS:
		pDM_Odm->pODM_StaInfo[Index] = (PSTA_INFO_T)pValue;
		break;
	//To remove the compiler warning, must add an empty default statement to handle the other values.
	default:
		//do nothing
		break;
	}

}


//
// Update Band/CHannel/.. The values are dynamic but non-per-packet.
//
VOID ODM_CmnInfoUpdate(
	IN		PDM_ODM_T		pDM_Odm,
	IN		u4Byte			CmnInfo,
	IN		u8Byte			Value
)
{
	//
	// This init variable may be changed in run time.
	//
	switch	(CmnInfo) {
	case ODM_CMNINFO_LINK_IN_PROGRESS:
		pDM_Odm->bLinkInProcess = (BOOLEAN)Value;
		break;

	case	ODM_CMNINFO_ABILITY:
		pDM_Odm->SupportAbility = (u4Byte)Value;
		break;

	case	ODM_CMNINFO_RF_TYPE:
		pDM_Odm->RFType = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_WIFI_DIRECT:
		pDM_Odm->bWIFI_Direct = (BOOLEAN)Value;
		break;

	case	ODM_CMNINFO_WIFI_DISPLAY:
		pDM_Odm->bWIFI_Display = (BOOLEAN)Value;
		break;

	case	ODM_CMNINFO_LINK:
		pDM_Odm->bLinked = (BOOLEAN)Value;
		break;

	case	ODM_CMNINFO_STATION_STATE:
		pDM_Odm->bsta_state = (BOOLEAN)Value;
		break;

	case	ODM_CMNINFO_RSSI_MIN:
		pDM_Odm->RSSI_Min = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_DBG_COMP:
		pDM_Odm->DebugComponents = Value;
		break;

	case	ODM_CMNINFO_DBG_LEVEL:
		pDM_Odm->DebugLevel = (u4Byte)Value;
		break;
	case	ODM_CMNINFO_RA_THRESHOLD_HIGH:
		pDM_Odm->RateAdaptive.HighRSSIThresh = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_RA_THRESHOLD_LOW:
		pDM_Odm->RateAdaptive.LowRSSIThresh = (u1Byte)Value;
		break;
#if defined(BT_30_SUPPORT) && (BT_30_SUPPORT == 1)
	// The following is for BT HS mode and BT coexist mechanism.
	case ODM_CMNINFO_BT_ENABLED:
		pDM_Odm->bBtEnabled = (BOOLEAN)Value;
		break;

	case ODM_CMNINFO_BT_HS_CONNECT_PROCESS:
		pDM_Odm->bBtConnectProcess = (BOOLEAN)Value;
		break;

	case ODM_CMNINFO_BT_HS_RSSI:
		pDM_Odm->btHsRssi = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_BT_OPERATION:
		pDM_Odm->bBtHsOperation = (BOOLEAN)Value;
		break;

	case	ODM_CMNINFO_BT_LIMITED_DIG:
		pDM_Odm->bBtLimitedDig = (BOOLEAN)Value;
		break;

	case ODM_CMNINFO_BT_DIG:
		pDM_Odm->btHsDigVal = (u1Byte)Value;
		break;

	case	ODM_CMNINFO_BT_BUSY:
		pDM_Odm->bBtBusy = (BOOLEAN)Value;
		break;

	case	ODM_CMNINFO_BT_DISABLE_EDCA:
		pDM_Odm->bBtDisableEdcaTurbo = (BOOLEAN)Value;
		break;
#endif

#if(DM_ODM_SUPPORT_TYPE & ODM_AP)		// for repeater mode add by YuChen 2014.06.23
#ifdef UNIVERSAL_REPEATER
	case	ODM_CMNINFO_VXD_LINK:
		pDM_Odm->VXD_bLinked = (BOOLEAN)Value;
		break;
#endif
#endif
	/*
			case	ODM_CMNINFO_OP_MODE:
				pDM_Odm->OPMode = (u1Byte)Value;
				break;

			case	ODM_CMNINFO_WM_MODE:
				pDM_Odm->WirelessMode = (u1Byte)Value;
				break;

			case	ODM_CMNINFO_BAND:
				pDM_Odm->BandType = (u1Byte)Value;
				break;

			case	ODM_CMNINFO_SEC_CHNL_OFFSET:
				pDM_Odm->SecChOffset = (u1Byte)Value;
				break;

			case	ODM_CMNINFO_SEC_MODE:
				pDM_Odm->Security = (u1Byte)Value;
				break;

			case	ODM_CMNINFO_BW:
				pDM_Odm->BandWidth = (u1Byte)Value;
				break;

			case	ODM_CMNINFO_CHNL:
				pDM_Odm->Channel = (u1Byte)Value;
				break;
	*/
	default:
		//do nothing
		break;
	}


}


#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
VOID ODM_InitAllWorkItems(IN PDM_ODM_T	pDM_Odm)
{
#if USE_WORKITEM
	PADAPTER		pAdapter = pDM_Odm->Adapter;

#if (RTL8723B_SUPPORT == 1)||(RTL8821A_SUPPORT == 1)
	ODM_InitializeWorkItem(pDM_Odm,
			       &pDM_Odm->DM_SWAT_Table.SwAntennaSwitchWorkitem_8723B,
			       (RT_WORKITEM_CALL_BACK)ODM_SW_AntDiv_WorkitemCallback,
			       (PVOID)pAdapter,
			       "AntennaSwitchWorkitem");
#endif

	ODM_InitializeWorkItem(pDM_Odm,
			       &pDM_Odm->DM_SWAT_Table.SwAntennaSwitchWorkitem,
			       (RT_WORKITEM_CALL_BACK)odm_SwAntDivChkAntSwitchWorkitemCallback,
			       (PVOID)pAdapter,
			       "AntennaSwitchWorkitem");


	ODM_InitializeWorkItem(
		pDM_Odm,
		&(pDM_Odm->PathDivSwitchWorkitem),
		(RT_WORKITEM_CALL_BACK)odm_PathDivChkAntSwitchWorkitemCallback,
		(PVOID)pAdapter,
		"SWAS_WorkItem");

	ODM_InitializeWorkItem(
		pDM_Odm,
		&(pDM_Odm->CCKPathDiversityWorkitem),
		(RT_WORKITEM_CALL_BACK)odm_CCKTXPathDiversityWorkItemCallback,
		(PVOID)pAdapter,
		"CCKTXPathDiversityWorkItem");

	ODM_InitializeWorkItem(
		pDM_Odm,
		&(pDM_Odm->MPT_DIGWorkitem),
		(RT_WORKITEM_CALL_BACK)odm_MPT_DIGWorkItemCallback,
		(PVOID)pAdapter,
		"MPT_DIGWorkitem");

	ODM_InitializeWorkItem(
		pDM_Odm,
		&(pDM_Odm->RaRptWorkitem),
		(RT_WORKITEM_CALL_BACK)ODM_UpdateInitRateWorkItemCallback,
		(PVOID)pAdapter,
		"RaRptWorkitem");

	ODM_InitializeWorkItem(
		pDM_Odm,
		&(pDM_Odm->sbdcnt_workitem),
		(RT_WORKITEM_CALL_BACK)phydm_sbd_workitem_callback,
		(PVOID)pAdapter,
		"SbdCntWorkitem");

#if( defined(CONFIG_5G_CG_SMART_ANT_DIVERSITY) ) ||( defined(CONFIG_2G_CG_SMART_ANT_DIVERSITY) )
	ODM_InitializeWorkItem(
		pDM_Odm,
		&(pDM_Odm->FastAntTrainingWorkitem),
		(RT_WORKITEM_CALL_BACK)odm_FastAntTrainingWorkItemCallback,
		(PVOID)pAdapter,
		"FastAntTrainingWorkitem");
#endif
	ODM_InitializeWorkItem(
		pDM_Odm,
		&(pDM_Odm->DM_RXHP_Table.PSDTimeWorkitem),
		(RT_WORKITEM_CALL_BACK)odm_PSD_RXHPWorkitemCallback,
		(PVOID)pAdapter,
		"PSDRXHP_WorkItem");
#endif
}

VOID ODM_FreeAllWorkItems(IN PDM_ODM_T	pDM_Odm)
{
#if USE_WORKITEM
#if (RTL8723B_SUPPORT == 1)||(RTL8821A_SUPPORT == 1)
	ODM_FreeWorkItem(&(pDM_Odm->DM_SWAT_Table.SwAntennaSwitchWorkitem_8723B));
#endif
	ODM_FreeWorkItem(&(pDM_Odm->DM_SWAT_Table.SwAntennaSwitchWorkitem));
	ODM_FreeWorkItem(&(pDM_Odm->PathDivSwitchWorkitem));
	ODM_FreeWorkItem(&(pDM_Odm->CCKPathDiversityWorkitem));
	ODM_FreeWorkItem(&(pDM_Odm->FastAntTrainingWorkitem));
	ODM_FreeWorkItem(&(pDM_Odm->MPT_DIGWorkitem));
	ODM_FreeWorkItem(&(pDM_Odm->RaRptWorkitem));
	ODM_FreeWorkItem((&pDM_Odm->DM_RXHP_Table.PSDTimeWorkitem));
	ODM_FreeWorkItem((&pDM_Odm->sbdcnt_workitem));
#endif
}
#endif

/*
VOID
odm_FindMinimumRSSI(
	IN		PDM_ODM_T		pDM_Odm
	)
{
	u4Byte	i;
	u1Byte	RSSI_Min = 0xFF;

	for(i=0; i<ODM_ASSOCIATE_ENTRY_NUM; i++)
	{
//		if(pDM_Odm->pODM_StaInfo[i] != NULL)
		if(IS_STA_VALID(pDM_Odm->pODM_StaInfo[i]) )
		{
			if(pDM_Odm->pODM_StaInfo[i]->RSSI_Ave < RSSI_Min)
			{
				RSSI_Min = pDM_Odm->pODM_StaInfo[i]->RSSI_Ave;
			}
		}
	}

	pDM_Odm->RSSI_Min = RSSI_Min;

}

VOID
odm_IsLinked(
	IN		PDM_ODM_T		pDM_Odm
	)
{
	u4Byte i;
	BOOLEAN Linked = FALSE;

	for(i=0; i<ODM_ASSOCIATE_ENTRY_NUM; i++)
	{
			if(IS_STA_VALID(pDM_Odm->pODM_StaInfo[i]) )
			{
				Linked = TRUE;
				break;
			}

	}

	pDM_Odm->bLinked = Linked;
}
*/

VOID ODM_InitAllTimers(
	IN PDM_ODM_T	pDM_Odm
)
{
#if(defined(CONFIG_HW_ANTENNA_DIVERSITY))
	ODM_AntDivTimers(pDM_Odm, INIT_ANTDIV_TIMMER);
#elif(defined(CONFIG_SW_ANTENNA_DIVERSITY))
	ODM_InitializeTimer(pDM_Odm, &pDM_Odm->DM_SWAT_Table.SwAntennaSwitchTimer,
			    (RT_TIMER_CALL_BACK)odm_SwAntDivChkAntSwitchCallback, NULL,
			    "SwAntennaSwitchTimer");
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
#ifdef MP_TEST
	if (pDM_Odm->priv->pshare->rf_ft_var.mp_specific)
		ODM_InitializeTimer(pDM_Odm, &pDM_Odm->MPT_DIGTimer,
				    (RT_TIMER_CALL_BACK)odm_MPT_DIGCallback, NULL, "MPT_DIGTimer");
#endif
#elif(DM_ODM_SUPPORT_TYPE == ODM_WIN)
	ODM_InitializeTimer(pDM_Odm, &pDM_Odm->MPT_DIGTimer,
			    (RT_TIMER_CALL_BACK)odm_MPT_DIGCallback, NULL, "MPT_DIGTimer");
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	ODM_InitializeTimer(pDM_Odm, &pDM_Odm->PSDTimer,
			    (RT_TIMER_CALL_BACK)dm_PSDMonitorCallback, NULL, "PSDTimer");
	ODM_InitializeTimer(pDM_Odm, &pDM_Odm->PathDivSwitchTimer,
			    (RT_TIMER_CALL_BACK)odm_PathDivChkAntSwitchCallback, NULL, "PathDivTimer");
	ODM_InitializeTimer(pDM_Odm, &pDM_Odm->CCKPathDiversityTimer,
			    (RT_TIMER_CALL_BACK)odm_CCKTXPathDiversityCallback, NULL,
			    "CCKPathDiversityTimer");
	ODM_InitializeTimer(pDM_Odm, &pDM_Odm->DM_RXHP_Table.PSDTimer,
			    (RT_TIMER_CALL_BACK)odm_PSD_RXHPCallback, NULL, "PSDRXHPTimer");
	ODM_InitializeTimer(pDM_Odm, &pDM_Odm->sbdcnt_timer,
			    (RT_TIMER_CALL_BACK)phydm_sbd_callback, NULL, "SbdTimer");

#endif
}

VOID ODM_CancelAllTimers(
	IN PDM_ODM_T	pDM_Odm
)
{
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	//
	// 2012/01/12 MH Temp BSOD fix. We need to find NIC allocate mem fail reason in
	// win7 platform.
	//
	HAL_ADAPTER_STS_CHK(pDM_Odm)
#endif

#if(defined(CONFIG_HW_ANTENNA_DIVERSITY))
	ODM_AntDivTimers(pDM_Odm, CANCEL_ANTDIV_TIMMER);
#elif(defined(CONFIG_SW_ANTENNA_DIVERSITY))
	ODM_CancelTimer(pDM_Odm, &pDM_Odm->DM_SWAT_Table.SwAntennaSwitchTimer);
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
#ifdef MP_TEST
	if (pDM_Odm->priv->pshare->rf_ft_var.mp_specific)
		ODM_CancelTimer(pDM_Odm, &pDM_Odm->MPT_DIGTimer);
#endif
#elif (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	ODM_CancelTimer(pDM_Odm, &pDM_Odm->MPT_DIGTimer);
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	ODM_CancelTimer(pDM_Odm, &pDM_Odm->PSDTimer);
	ODM_CancelTimer(pDM_Odm, &pDM_Odm->PathDivSwitchTimer);
	ODM_CancelTimer(pDM_Odm, &pDM_Odm->CCKPathDiversityTimer);
	ODM_CancelTimer(pDM_Odm, &pDM_Odm->DM_RXHP_Table.PSDTimer);
	ODM_CancelTimer(pDM_Odm, &pDM_Odm->sbdcnt_timer);
#endif
}


VOID ODM_ReleaseAllTimers(
	IN PDM_ODM_T	pDM_Odm
)
{
#if(defined(CONFIG_HW_ANTENNA_DIVERSITY))
	ODM_AntDivTimers(pDM_Odm, RELEASE_ANTDIV_TIMMER);
#elif(defined(CONFIG_SW_ANTENNA_DIVERSITY))
	ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->DM_SWAT_Table.SwAntennaSwitchTimer);
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
#ifdef MP_TEST
	if (pDM_Odm->priv->pshare->rf_ft_var.mp_specific)
		ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->MPT_DIGTimer);
#endif
#elif(DM_ODM_SUPPORT_TYPE == ODM_WIN)
	ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->MPT_DIGTimer);
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->PSDTimer);
	ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->PathDivSwitchTimer);
	ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->CCKPathDiversityTimer);
	ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->DM_RXHP_Table.PSDTimer);
	ODM_ReleaseTimer(pDM_Odm, &pDM_Odm->sbdcnt_timer);
#endif
}


//3============================================================
//3 Tx Power Tracking
//3============================================================

VOID odm_IQCalibrate(
	IN	PDM_ODM_T	pDM_Odm
)
{
	PADAPTER	Adapter = pDM_Odm->Adapter;

#if( DM_ODM_SUPPORT_TYPE == ODM_WIN)
	if (*pDM_Odm->pIsFcsModeEnable)
		return;
#endif

	if (!IS_HARDWARE_TYPE_JAGUAR(Adapter))
		return;
	else if (IS_HARDWARE_TYPE_8812AU(Adapter))
		return;
#if (RTL8821A_SUPPORT == 1)
	if (pDM_Odm->bLinked) {
		if ((*pDM_Odm->pChannel != pDM_Odm->preChannel)
		    && (!*pDM_Odm->pbScanInProcess)) {
			pDM_Odm->preChannel = *pDM_Odm->pChannel;
			pDM_Odm->LinkedInterval = 0;
		}

		if (pDM_Odm->LinkedInterval < 3)
			pDM_Odm->LinkedInterval++;

		if (pDM_Odm->LinkedInterval == 2) {
			// Mark out IQK flow to prevent tx stuck. by Maddest 20130306
			// Open it verified by James 20130715
			PHY_IQCalibrate_8821A(pDM_Odm, FALSE);
		}
	} else
		pDM_Odm->LinkedInterval = 0;
#endif
}


#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
VOID ODM_InitAllThreads(
	IN PDM_ODM_T	pDM_Odm
)
{
#ifdef TPT_THREAD
	kTPT_task_init(pDM_Odm->priv);
#endif
}

VOID ODM_StopAllThreads(
	IN PDM_ODM_T	pDM_Odm
)
{
#ifdef TPT_THREAD
	kTPT_task_stop(pDM_Odm->priv);
#endif
}
#endif


#if( DM_ODM_SUPPORT_TYPE == ODM_WIN)
//
// 2011/07/26 MH Add an API for testing IQK fail case.
//
BOOLEAN ODM_CheckPowerStatus(
	IN	PADAPTER		Adapter)
{

	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	PDM_ODM_T			pDM_Odm = &pHalData->DM_OutSrc;
	RT_RF_POWER_STATE 	rtState;
	PMGNT_INFO			pMgntInfo	= &(Adapter->MgntInfo);

	// 2011/07/27 MH We are not testing ready~~!! We may fail to get correct value when init sequence.
	if (pMgntInfo->init_adpt_in_progress == TRUE) {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
			     ("ODM_CheckPowerStatus Return TRUE, due to initadapter\n"));
		return	TRUE;
	}

	//
	//	2011/07/19 MH We can not execute tx pwoer tracking/ LLC calibrate or IQK.
	//
	Adapter->HalFunc.GetHwRegHandler(Adapter, HW_VAR_RF_STATE,
					 (pu1Byte)(&rtState));
	if (Adapter->bDriverStopped || Adapter->bDriverIsGoingToPnpSetPowerSleep
	    || rtState == eRfOff) {
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
			     ("ODM_CheckPowerStatus Return FALSE, due to %d/%d/%d\n",
			      Adapter->bDriverStopped, Adapter->bDriverIsGoingToPnpSetPowerSleep,
			      rtState));
		return	FALSE;
	}
	return	TRUE;
}
#elif( DM_ODM_SUPPORT_TYPE == ODM_AP)
BOOLEAN ODM_CheckPowerStatus(
	IN	PADAPTER		Adapter)
{
	/*
	   HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	   PDM_ODM_T			pDM_Odm = &pHalData->DM_OutSrc;
	   RT_RF_POWER_STATE 	rtState;
	   PMGNT_INFO			pMgntInfo	= &(Adapter->MgntInfo);

	// 2011/07/27 MH We are not testing ready~~!! We may fail to get correct value when init sequence.
	if (pMgntInfo->init_adpt_in_progress == TRUE)
	{
	ODM_RT_TRACE(pDM_Odm,COMP_INIT, DBG_LOUD, ("ODM_CheckPowerStatus Return TRUE, due to initadapter"));
	return	TRUE;
	}

	//
	//	2011/07/19 MH We can not execute tx pwoer tracking/ LLC calibrate or IQK.
	//
	Adapter->HalFunc.GetHwRegHandler(Adapter, HW_VAR_RF_STATE, (pu1Byte)(&rtState));
	if(Adapter->bDriverStopped || Adapter->bDriverIsGoingToPnpSetPowerSleep || rtState == eRfOff)
	{
	ODM_RT_TRACE(pDM_Odm,COMP_INIT, DBG_LOUD, ("ODM_CheckPowerStatus Return FALSE, due to %d/%d/%d\n",
	Adapter->bDriverStopped, Adapter->bDriverIsGoingToPnpSetPowerSleep, rtState));
	return	FALSE;
	}
	 */
	return	TRUE;
}
#endif

// need to ODM CE Platform
//move to here for ANT detection mechanism using

#if ((DM_ODM_SUPPORT_TYPE == ODM_WIN)||(DM_ODM_SUPPORT_TYPE == ODM_CE))
u4Byte GetPSDData(
	IN PDM_ODM_T	pDM_Odm,
	unsigned int 	point,
	u1Byte initial_gain_psd)
{
	//unsigned int	val, rfval;
	//int	psd_report;
	u4Byte	psd_report;

	//HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	//Debug Message
	//val = PHY_QueryBBReg(Adapter,0x908, bMaskDWord);
	//DbgPrint("Reg908 = 0x%x\n",val);
	//val = PHY_QueryBBReg(Adapter,0xDF4, bMaskDWord);
	//rfval = PHY_QueryRFReg(Adapter, ODM_RF_PATH_A, 0x00, bRFRegOffsetMask);
	//DbgPrint("RegDF4 = 0x%x, RFReg00 = 0x%x\n",val, rfval);
	//DbgPrint("PHYTXON = %x, OFDMCCA_PP = %x, CCKCCA_PP = %x, RFReg00 = %x\n",
	//(val&BIT25)>>25, (val&BIT14)>>14, (val&BIT15)>>15, rfval);

	//Set DCO frequency index, offset=(40MHz/SamplePts)*point
	ODM_SetBBReg(pDM_Odm, 0x808, 0x3FF, point);

	//Start PSD calculation, Reg808[22]=0->1
	ODM_SetBBReg(pDM_Odm, 0x808, BIT22, 1);
	//Need to wait for HW PSD report
	ODM_StallExecution(1000);
	ODM_SetBBReg(pDM_Odm, 0x808, BIT22, 0);
	//Read PSD report, Reg8B4[15:0]
	psd_report = ODM_GetBBReg(pDM_Odm, 0x8B4, bMaskDWord) & 0x0000FFFF;

#if 1//(DEV_BUS_TYPE == RT_PCI_INTERFACE) && ( (RT_PLATFORM == PLATFORM_LINUX) || (RT_PLATFORM == PLATFORM_MACOSX))
	psd_report = (u4Byte)(odm_ConvertTo_dB(psd_report)) + (u4Byte)(
			     initial_gain_psd - 0x1c);
#else
	psd_report = (int)(20 * log10((double)psd_report)) + (int)(
			     initial_gain_psd - 0x1c);
#endif

	return psd_report;

}
#endif

u4Byte odm_ConvertTo_dB(
	u4Byte 	Value)
{
	u1Byte i;
	u1Byte j;
	u4Byte dB;

	Value = Value & 0xFFFF;

	for (i = 0; i < 12; i++) {
		if (Value <= dB_Invert_Table[i][7]) {
			break;
		}
	}

	if (i >= 12) {
		return (96);	// maximum 96 dB
	}

	for (j = 0; j < 8; j++) {
		if (Value <= dB_Invert_Table[i][j]) {
			break;
		}
	}

	dB = (i << 3) + j + 1;

	return (dB);
}

u4Byte odm_ConvertTo_linear(
	u4Byte 	Value)
{
	u1Byte i;
	u1Byte j;
	u4Byte linear;

	Value = Value & 0xFF;

	i = (u1Byte)((Value - 1) >> 3);
	j = (u1Byte)(Value - 1) - (i << 3);

	linear = dB_Invert_Table[i][j];

	return (linear);
}

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
VOID ODM_UpdateInitRateWorkItemCallback(
	IN PVOID            pContext
)
{
	PADAPTER	Adapter = (PADAPTER)pContext;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	PDM_ODM_T		pDM_Odm = &pHalData->DM_OutSrc;

	u1Byte			p = 0;

	if (pDM_Odm->SupportICType == ODM_RTL8821) {
		ODM_TxPwrTrackSetPwr8821A(pDM_Odm, MIX_MODE, ODM_RF_PATH_A, 0);
	} else if (pDM_Odm->SupportICType == ODM_RTL8812) {
		for (p = ODM_RF_PATH_A; p < MAX_PATH_NUM_8812A;
		     p++) {  //DOn't know how to include &c
			ODM_TxPwrTrackSetPwr8812A(pDM_Odm, MIX_MODE, p, 0);
		}
	} else if (pDM_Odm->SupportICType == ODM_RTL8723B) {
		ODM_TxPwrTrackSetPwr_8723B(pDM_Odm, MIX_MODE, ODM_RF_PATH_A, 0);
	} else if (pDM_Odm->SupportICType == ODM_RTL8192E) {
		for (p = ODM_RF_PATH_A; p < MAX_PATH_NUM_8192E;
		     p++) {  //DOn't know how to include &c
			ODM_TxPwrTrackSetPwr92E(pDM_Odm, MIX_MODE, p, 0);
		}
	} else if (pDM_Odm->SupportICType == ODM_RTL8188E) {
		ODM_TxPwrTrackSetPwr88E(pDM_Odm, MIX_MODE, ODM_RF_PATH_A, 0);
	}
}
#endif

//
// ODM multi-port consideration, added by Roger, 2013.10.01.
//
VOID ODM_AsocEntry_Init(
	IN	PDM_ODM_T	pDM_Odm
)
{
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	PADAPTER pLoopAdapter = GetDefaultAdapter(pDM_Odm->Adapter);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pLoopAdapter);
	PDM_ODM_T		 pDM_OutSrc = &pHalData->DM_OutSrc;
	u1Byte	TotalAssocEntryNum = 0;
	u1Byte	index = 0;


	ODM_CmnInfoPtrArrayHook(pDM_OutSrc, ODM_CMNINFO_STA_STATUS, 0,
				&pLoopAdapter->MgntInfo.DefaultPort[0]);
	pLoopAdapter->MgntInfo.DefaultPort[0].MultiPortStationIdx =
		TotalAssocEntryNum;

	pLoopAdapter = GetNextExtAdapter(pLoopAdapter);
	TotalAssocEntryNum += 1;

	while (pLoopAdapter) {
		for (index = 0; index < ASSOCIATE_ENTRY_NUM; index++) {
			ODM_CmnInfoPtrArrayHook(pDM_OutSrc, ODM_CMNINFO_STA_STATUS,
						TotalAssocEntryNum + index, &pLoopAdapter->MgntInfo.AsocEntry[index]);
			pLoopAdapter->MgntInfo.AsocEntry[index].MultiPortStationIdx =
				TotalAssocEntryNum + index;
		}

		TotalAssocEntryNum += index;
		if (IS_HARDWARE_TYPE_8188E((pDM_Odm->Adapter)))
			pLoopAdapter->RASupport = TRUE;
		pLoopAdapter = GetNextExtAdapter(pLoopAdapter);
	}
#endif
}

#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
/* Justin: According to the current RRSI to adjust Response Frame TX power, 2012/11/05 */
void odm_dtc(PDM_ODM_T pDM_Odm)
{
#ifdef CONFIG_DM_RESP_TXAGC
#define DTC_BASE            35	/* RSSI higher than this value, start to decade TX power */
#define DTC_DWN_BASE       (DTC_BASE-5)	/* RSSI lower than this value, start to increase TX power */

	/* RSSI vs TX power step mapping: decade TX power */
	static const u8 dtc_table_down[] = {
		DTC_BASE,
		(DTC_BASE + 5),
		(DTC_BASE + 10),
		(DTC_BASE + 15),
		(DTC_BASE + 20),
		(DTC_BASE + 25)
	};

	/* RSSI vs TX power step mapping: increase TX power */
	static const u8 dtc_table_up[] = {
		DTC_DWN_BASE,
		(DTC_DWN_BASE - 5),
		(DTC_DWN_BASE - 10),
		(DTC_DWN_BASE - 15),
		(DTC_DWN_BASE - 15),
		(DTC_DWN_BASE - 20),
		(DTC_DWN_BASE - 20),
		(DTC_DWN_BASE - 25),
		(DTC_DWN_BASE - 25),
		(DTC_DWN_BASE - 30),
		(DTC_DWN_BASE - 35)
	};

	u8 i;
	u8 dtc_steps = 0;
	u8 sign;
	u8 resp_txagc = 0;

#if 0
	/* As DIG is disabled, DTC is also disable */
	if (!(pDM_Odm->SupportAbility & ODM_XXXXXX))
		return;
#endif

	if (DTC_BASE < pDM_Odm->RSSI_Min) {
		/* need to decade the CTS TX power */
		sign = 1;
		for (i = 0; i < ARRAY_SIZE(dtc_table_down); i++) {
			if ((dtc_table_down[i] >= pDM_Odm->RSSI_Min) || (dtc_steps >= 6))
				break;
			else
				dtc_steps++;
		}
	}
#if 0
	else if (DTC_DWN_BASE > pDM_Odm->RSSI_Min) {
		/* needs to increase the CTS TX power */
		sign = 0;
		dtc_steps = 1;
		for (i = 0; i < ARRAY_SIZE(dtc_table_up); i++) {
			if ((dtc_table_up[i] <= pDM_Odm->RSSI_Min) || (dtc_steps >= 10))
				break;
			else
				dtc_steps++;
		}
	}
#endif
	else {
		sign = 0;
		dtc_steps = 0;
	}

	resp_txagc = dtc_steps | (sign << 4);
	resp_txagc = resp_txagc | (resp_txagc << 5);
	ODM_Write1Byte(pDM_Odm, 0x06d9, resp_txagc);

	DBG_871X("%s RSSI_Min:%u, set RESP_TXAGC to %s %u\n",
		 __func__, pDM_Odm->RSSI_Min, sign ? "minus" : "plus", dtc_steps);
#endif /* CONFIG_RESP_TXAGC_ADJUST */
}

#endif /* #if (DM_ODM_SUPPORT_TYPE == ODM_CE) */

VOID odm_UpdatePowerTrainingState(
	IN	PDM_ODM_T	pDM_Odm
)
{
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN|ODM_CE))
	PFALSE_ALARM_STATISTICS 	FalseAlmCnt = (PFALSE_ALARM_STATISTICS)
			PhyDM_Get_Structure(pDM_Odm, PHYDM_FALSEALMCNT);
	pDIG_T						pDM_DigTable = &pDM_Odm->DM_DigTable;
	u4Byte						score = 0;

	if (!(pDM_Odm->SupportAbility & ODM_BB_PWR_TRAIN))
		return;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
		     ("odm_UpdatePowerTrainingState()============>\n"));
	pDM_Odm->bChangeState = FALSE;

	// Debug command
	if (pDM_Odm->ForcePowerTrainingState) {
		if (pDM_Odm->ForcePowerTrainingState == 1
		    && !pDM_Odm->bDisablePowerTraining) {
			pDM_Odm->bChangeState = TRUE;
			pDM_Odm->bDisablePowerTraining = TRUE;
		} else if (pDM_Odm->ForcePowerTrainingState == 2
			   && pDM_Odm->bDisablePowerTraining) {
			pDM_Odm->bChangeState = TRUE;
			pDM_Odm->bDisablePowerTraining = FALSE;
		}

		pDM_Odm->PT_score = 0;
		pDM_Odm->PhyDbgInfo.NumQryPhyStatusOFDM = 0;
		pDM_Odm->PhyDbgInfo.NumQryPhyStatusCCK = 0;
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
			     ("odm_UpdatePowerTrainingState(): ForcePowerTrainingState = %d\n",
			      pDM_Odm->ForcePowerTrainingState));
		return;
	}

	if (!pDM_Odm->bLinked)
		return;

	// First connect
	if ((pDM_Odm->bLinked) && (pDM_DigTable->bMediaConnect_0 == FALSE)) {
		pDM_Odm->PT_score = 0;
		pDM_Odm->bChangeState = TRUE;
		pDM_Odm->PhyDbgInfo.NumQryPhyStatusOFDM = 0;
		pDM_Odm->PhyDbgInfo.NumQryPhyStatusCCK = 0;
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
			     ("odm_UpdatePowerTrainingState(): First Connect\n"));
		return;
	}

	// Compute score
	if (pDM_Odm->NHM_cnt_0 >= 215)
		score = 2;
	else if (pDM_Odm->NHM_cnt_0 >= 190)
		score = 1;							// unknow state
	else {
		u4Byte	RX_Pkt_Cnt;

		RX_Pkt_Cnt = (u4Byte)(pDM_Odm->PhyDbgInfo.NumQryPhyStatusOFDM) + (u4Byte)(
				     pDM_Odm->PhyDbgInfo.NumQryPhyStatusCCK);

		if ((FalseAlmCnt->Cnt_CCA_all > 31 && RX_Pkt_Cnt > 31)
		    && (FalseAlmCnt->Cnt_CCA_all >= RX_Pkt_Cnt)) {
			if ((RX_Pkt_Cnt + (RX_Pkt_Cnt >> 1)) <= FalseAlmCnt->Cnt_CCA_all)
				score = 0;
			else if ((RX_Pkt_Cnt + (RX_Pkt_Cnt >> 2)) <= FalseAlmCnt->Cnt_CCA_all)
				score = 1;
			else
				score = 2;
		}
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
			     ("odm_UpdatePowerTrainingState(): RX_Pkt_Cnt = %d, Cnt_CCA_all = %d\n",
			      RX_Pkt_Cnt, FalseAlmCnt->Cnt_CCA_all));
	}
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
		     ("odm_UpdatePowerTrainingState(): NumQryPhyStatusOFDM = %d, NumQryPhyStatusCCK = %d\n",
		      (u4Byte)(pDM_Odm->PhyDbgInfo.NumQryPhyStatusOFDM),
		      (u4Byte)(pDM_Odm->PhyDbgInfo.NumQryPhyStatusCCK)));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
		     ("odm_UpdatePowerTrainingState(): NHM_cnt_0 = %d, score = %d\n",
		      pDM_Odm->NHM_cnt_0, score));

	// smoothing
	pDM_Odm->PT_score = (score << 4) + (pDM_Odm->PT_score >> 1) +
			    (pDM_Odm->PT_score >> 2);
	score = (pDM_Odm->PT_score + 32) >> 6;
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
		     ("odm_UpdatePowerTrainingState(): PT_score = %d, score after smoothing = %d\n",
		      pDM_Odm->PT_score, score));

	// Mode decision
	if (score == 2) {
		if (pDM_Odm->bDisablePowerTraining) {
			pDM_Odm->bChangeState = TRUE;
			pDM_Odm->bDisablePowerTraining = FALSE;
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
				     ("odm_UpdatePowerTrainingState(): Change state\n"));
		}
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
			     ("odm_UpdatePowerTrainingState(): Enable Power Training\n"));
	} else if (score == 0) {
		if (!pDM_Odm->bDisablePowerTraining) {
			pDM_Odm->bChangeState = TRUE;
			pDM_Odm->bDisablePowerTraining = TRUE;
			ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
				     ("odm_UpdatePowerTrainingState(): Change state\n"));
		}
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_RA_MASK, ODM_DBG_LOUD,
			     ("odm_UpdatePowerTrainingState(): Disable Power Training\n"));
	}

	pDM_Odm->PhyDbgInfo.NumQryPhyStatusOFDM = 0;
	pDM_Odm->PhyDbgInfo.NumQryPhyStatusCCK = 0;
#endif
}
