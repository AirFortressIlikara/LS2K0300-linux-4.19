//===========================================
// The following is for 8821A_CSR 2Ant BT Co-exist definition
//===========================================
#define	BT_INFO_8821A_CSR_2ANT_B_FTP						BIT7
#define	BT_INFO_8821A_CSR_2ANT_B_A2DP					BIT6
#define	BT_INFO_8821A_CSR_2ANT_B_HID						BIT5
#define	BT_INFO_8821A_CSR_2ANT_B_SCO_BUSY				BIT4
#define	BT_INFO_8821A_CSR_2ANT_B_ACL_BUSY				BIT3
#define	BT_INFO_8821A_CSR_2ANT_B_INQ_PAGE				BIT2
#define	BT_INFO_8821A_CSR_2ANT_B_SCO_ESCO				BIT1
#define	BT_INFO_8821A_CSR_2ANT_B_CONNECTION				BIT0

#define		BTC_RSSI_COEX_THRESH_TOL_8821A_CSR_2ANT		2

typedef enum _BT_INFO_SRC_8821A_CSR_2ANT {
	BT_INFO_SRC_8821A_CSR_2ANT_WIFI_FW			= 0x0,
	BT_INFO_SRC_8821A_CSR_2ANT_BT_RSP				= 0x1,
	BT_INFO_SRC_8821A_CSR_2ANT_BT_ACTIVE_SEND		= 0x2,
	BT_INFO_SRC_8821A_CSR_2ANT_MAX
} BT_INFO_SRC_8821A_CSR_2ANT, *PBT_INFO_SRC_8821A_CSR_2ANT;

typedef enum _BT_8821A_CSR_2ANT_BT_STATUS {
	BT_8821A_CSR_2ANT_BT_STATUS_IDLE				= 0x0,
	BT_8821A_CSR_2ANT_BT_STATUS_CONNECTED_IDLE	= 0x1,
	BT_8821A_CSR_2ANT_BT_STATUS_NON_IDLE			= 0x2,
	BT_8821A_CSR_2ANT_BT_STATUS_MAX
} BT_8821A_CSR_2ANT_BT_STATUS, *PBT_8821A_CSR_2ANT_BT_STATUS;

typedef enum _BT_8821A_CSR_2ANT_COEX_ALGO {
	BT_8821A_CSR_2ANT_COEX_ALGO_UNDEFINED			= 0x0,
	BT_8821A_CSR_2ANT_COEX_ALGO_SCO				= 0x1,
	BT_8821A_CSR_2ANT_COEX_ALGO_HID				= 0x2,
	BT_8821A_CSR_2ANT_COEX_ALGO_A2DP				= 0x3,
	BT_8821A_CSR_2ANT_COEX_ALGO_A2DP_PANHS		= 0x4,
	BT_8821A_CSR_2ANT_COEX_ALGO_PANEDR			= 0x5,
	BT_8821A_CSR_2ANT_COEX_ALGO_PANHS			= 0x6,
	BT_8821A_CSR_2ANT_COEX_ALGO_PANEDR_A2DP		= 0x7,
	BT_8821A_CSR_2ANT_COEX_ALGO_PANEDR_HID		= 0x8,
	BT_8821A_CSR_2ANT_COEX_ALGO_HID_A2DP_PANEDR	= 0x9,
	BT_8821A_CSR_2ANT_COEX_ALGO_HID_A2DP			= 0xa,
	BT_8821A_CSR_2ANT_COEX_ALGO_MAX				= 0xb,
} BT_8821A_CSR_2ANT_COEX_ALGO, *PBT_8821A_CSR_2ANT_COEX_ALGO;

typedef struct _COEX_DM_8821A_CSR_2ANT {
	// fw mechanism
	BOOLEAN		bPreDecBtPwr;
	BOOLEAN		bCurDecBtPwr;
	u1Byte		preFwDacSwingLvl;
	u1Byte		curFwDacSwingLvl;
	BOOLEAN		bCurIgnoreWlanAct;
	BOOLEAN		bPreIgnoreWlanAct;
	u1Byte		prePsTdma;
	u1Byte		curPsTdma;
	u1Byte		psTdmaPara[6];
	u1Byte		psTdmaDuAdjType;
	BOOLEAN		bResetTdmaAdjust;
	BOOLEAN		bPrePsTdmaOn;
	BOOLEAN		bCurPsTdmaOn;
	BOOLEAN		bPreBtAutoReport;
	BOOLEAN		bCurBtAutoReport;

	// sw mechanism
	BOOLEAN		bPreRfRxLpfShrink;
	BOOLEAN		bCurRfRxLpfShrink;
	u4Byte		btRf0x1eBackup;
	BOOLEAN 	bPreLowPenaltyRa;
	BOOLEAN		bCurLowPenaltyRa;
	BOOLEAN		bPreDacSwingOn;
	u4Byte		preDacSwingLvl;
	BOOLEAN		bCurDacSwingOn;
	u4Byte		curDacSwingLvl;
	BOOLEAN		bPreAdcBackOff;
	BOOLEAN		bCurAdcBackOff;
	BOOLEAN 	bPreAgcTableEn;
	BOOLEAN		bCurAgcTableEn;
	u4Byte		preVal0x6c0;
	u4Byte		curVal0x6c0;
	u4Byte		preVal0x6c4;
	u4Byte		curVal0x6c4;
	u4Byte		preVal0x6c8;
	u4Byte		curVal0x6c8;
	u1Byte		preVal0x6cc;
	u1Byte		curVal0x6cc;
	BOOLEAN		bLimitedDig;

	u4Byte		preRaMask;
	u4Byte		curRaMask;

	u1Byte curAmpduNumType;
	u1Byte preAmpduNumType;
	u2Byte backupAmpduMaxNum;

	u1Byte curAmpduTimeType;
	u1Byte preAmpduTimeType;
	u1Byte backupAmpduMaxTime;

	u1Byte 		curArfrType;
	u1Byte 		preArfrType;
	u4Byte		backupArfrCnt1;
	u4Byte		backupArfrCnt2;

	u1Byte		curRetryLimitType;
	u1Byte		preRetryLimitType;
	u2Byte		backupRetryLimit;

	// algorithm related
	u1Byte		preAlgorithm;
	u1Byte		curAlgorithm;
	u1Byte		btStatus;
	u1Byte		wifiChnlInfo[3];
} COEX_DM_8821A_CSR_2ANT, *PCOEX_DM_8821A_CSR_2ANT;

typedef struct _COEX_STA_8821A_CSR_2ANT {
	BOOLEAN					bBtLinkExist;
	BOOLEAN					bScoExist;
	BOOLEAN					bA2dpExist;
	BOOLEAN					bSlave;
	BOOLEAN					bHidExist;
	BOOLEAN					bPanExist;

	BOOLEAN					bUnderLps;
	BOOLEAN					bUnderIps;
	u4Byte					highPriorityTx;
	u4Byte					highPriorityRx;
	u4Byte					lowPriorityTx;
	u4Byte					lowPriorityRx;
	u1Byte					btRssi;
	u1Byte					preBtRssiState;
	u1Byte					preWifiRssiState[4];
	BOOLEAN					bC2hBtInfoReqSent;
	u1Byte					btInfoC2h[BT_INFO_SRC_8821A_CSR_2ANT_MAX][10];
	u4Byte					btInfoC2hCnt[BT_INFO_SRC_8821A_CSR_2ANT_MAX];
	BOOLEAN					bC2hBtInquiryPage;
	u1Byte					btRetryCnt;
	u1Byte					btInfoExt;
} COEX_STA_8821A_CSR_2ANT, *PCOEX_STA_8821A_CSR_2ANT;

//===========================================
// The following is interface which will notify coex module.
//===========================================
VOID EXhalbtc8821aCsr2ant_PowerOnSetting(
	IN	PBTC_COEXIST		pBtCoexist
);
VOID EXhalbtc8821aCsr2ant_InitHwConfig(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	BOOLEAN				bWifiOnly
);
VOID EXhalbtc8821aCsr2ant_InitCoexDm(
	IN	PBTC_COEXIST		pBtCoexist
);
VOID EXhalbtc8821aCsr2ant_IpsNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
);
VOID EXhalbtc8821aCsr2ant_LpsNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
);
VOID EXhalbtc8821aCsr2ant_ScanNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
);
VOID EXhalbtc8821aCsr2ant_ConnectNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	u1Byte			type
);
VOID EXhalbtc8821aCsr2ant_MediaStatusNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
);
VOID EXhalbtc8821aCsr2ant_SpecialPacketNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				type
);
VOID EXhalbtc8821aCsr2ant_BtInfoNotify(
	IN	PBTC_COEXIST		pBtCoexist,
	IN	pu1Byte			tmpBuf,
	IN	u1Byte			length
);
VOID EXhalbtc8821aCsr2ant_HaltNotify(
	IN	PBTC_COEXIST			pBtCoexist
);
VOID EXhalbtc8821aCsr2ant_PnpNotify(
	IN	PBTC_COEXIST			pBtCoexist,
	IN	u1Byte				pnpState
);
VOID EXhalbtc8821aCsr2ant_Periodical(
	IN	PBTC_COEXIST			pBtCoexist
);
VOID EXhalbtc8821aCsr2ant_DisplayCoexInfo(
	IN	PBTC_COEXIST		pBtCoexist
);
