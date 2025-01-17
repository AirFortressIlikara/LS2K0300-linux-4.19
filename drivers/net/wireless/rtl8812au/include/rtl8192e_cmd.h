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
#ifndef __RTL8192E_CMD_H__
#define __RTL8192E_CMD_H__

typedef enum _RTL8192E_H2C_CMD {
	H2C_8192E_RSVDPAGE = 0,
	H2C_8192E_MSRRPT = 1,
	H2C_8192E_SCAN = 2,
	H2C_8192E_KEEP_ALIVE_CTRL = 3,
	H2C_8192E_DISCONNECT_DECISION = 4,

	H2C_8192E_INIT_OFFLOAD = 6,
	H2C_8192E_AP_OFFLOAD = 8,
	H2C_8192E_BCN_RSVDPAGE = 9,
	H2C_8192E_PROBERSP_RSVDPAGE = 10,

	H2C_8192E_SETPWRMODE = 0x20,
	H2C_8192E_PS_TUNING_PARA = 0x21,
	H2C_8192E_PS_TUNING_PARA2 = 0x22,
	H2C_8192E_PS_LPS_PARA = 0x23,
	H2C_8192E_P2P_PS_OFFLOAD = 0x24,
	H2C_8192E_RA_MASK = 0x40,
	H2C_8192E_RSSI_REPORT = 0x42,

	H2C_8192E_WO_WLAN = 0x80,
	H2C_8192E_REMOTE_WAKE_CTRL = 0x81,
	H2C_8192E_AOAC_GLOBAL_INFO = 0x82,
	H2C_8192E_AOAC_RSVDPAGE = 0x83,

	//Not defined in new 88E H2C CMD Format
	H2C_8192E_SELECTIVE_SUSPEND_ROF_CMD,
	H2C_8192E_P2P_PS_MODE,
	H2C_8192E_PSD_RESULT,
	MAX_8192E_H2CCMD
} RTL8192E_H2C_CMD;

typedef enum _RTL8192E_C2H_EVT {
	C2H_8192E_DBG = 0,
	C2H_8192E_LB = 1,
	C2H_8192E_TXBF = 2,
	C2H_8192E_TX_REPORT = 3,
	C2H_8192E_BT_INFO = 9,
	C2H_8192E_FW_SWCHNL = 0x10,
	C2H_8192E_BT_MP = 11,
	C2H_8192E_RA_RPT = 12,

	MAX_8192E_C2HEVENT
} RTL8192E_C2H_EVT;


struct cmd_msg_parm {
	u8 eid; //element id
	u8 sz; // sz
	u8 buf[6];
};

enum {
	PWRS
};

typedef struct _SETPWRMODE_PARM {
	u8 Mode;//0:Active,1:LPS,2:WMMPS
	//u8 RLBM:4;//0:Min,1:Max,2: User define
	u8 SmartPS_RLBM;//LPS=0:PS_Poll,1:PS_Poll,2:NullData,WMM=0:PS_Poll,1:NullData
	u8 AwakeInterval;	// unit: beacon interval
	u8 bAllQueueUAPSD;
	u8 PwrState;//AllON(0x0c),RFON(0x04),RFOFF(0x00)
} SETPWRMODE_PARM, *PSETPWRMODE_PARM;

struct H2C_SS_RFOFF_PARAM {
	u8 ROFOn; // 1: on, 0:off
	u16 gpio_period; // unit: 1024 us
} __attribute__((packed));


typedef struct JOINBSSRPT_PARM_92E {
	u8 OpMode;	// RT_MEDIA_STATUS
#ifdef CONFIG_WOWLAN
	u8 MacID;       // MACID
#endif //CONFIG_WOWLAN
} JOINBSSRPT_PARM_92E, *PJOINBSSRPT_PARM_92E;

typedef struct _RSVDPAGE_LOC_92E {
	u8 LocProbeRsp;
	u8 LocPsPoll;
	u8 LocNullData;
	u8 LocQosNull;
	u8 LocBTQosNull;
} RSVDPAGE_LOC_92E, *PRSVDPAGE_LOC_92E;



//_SETPWRMODE_PARM
#define SET_8192E_H2CCMD_PWRMODE_PARM_MODE(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd, 0, 8, __Value)
#define SET_8192E_H2CCMD_PWRMODE_PARM_RLBM(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE((__pH2CCmd)+1, 0, 4, __Value)
#define SET_8192E_H2CCMD_PWRMODE_PARM_SMART_PS(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE((__pH2CCmd)+1, 4, 4, __Value)
#define SET_8192E_H2CCMD_PWRMODE_PARM_BCN_PASS_TIME(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE((__pH2CCmd)+2, 0, 8, __Value)
#define SET_8192E_H2CCMD_PWRMODE_PARM_ALL_QUEUE_UAPSD(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE((__pH2CCmd)+3, 0, 8, __Value)
#define SET_8192E_H2CCMD_PWRMODE_PARM_PWR_STATE(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE((__pH2CCmd)+4, 0, 8, __Value)
#define SET_8192E_H2CCMD_PWRMODE_PARM_BYTE5(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE((__pH2CCmd)+5, 0, 8, __Value)
#define GET_8192E_H2CCMD_PWRMODE_PARM_MODE(__pH2CCmd)					LE_BITS_TO_1BYTE(__pH2CCmd, 0, 8)

//_P2P_PS_OFFLOAD
#define SET_8192E_H2CCMD_P2P_PS_OFFLOAD_ENABLE(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd, 0, 1, __Value)
#define SET_8192E_H2CCMD_P2P_PS_OFFLOAD_ROLE(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd, 1, 1, __Value)
#define SET_8192E_H2CCMD_P2P_PS_OFFLOAD_CTWINDOW_EN(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE(__pH2CCmd, 2, 1, __Value)
#define SET_8192E_H2CCMD_P2P_PS_OFFLOAD_NOA0_EN(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE(__pH2CCmd, 3, 1, __Value)
#define SET_8192E_H2CCMD_P2P_PS_OFFLOAD_NOA1_EN(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE(__pH2CCmd, 4, 1, __Value)
#define SET_8192E_H2CCMD_P2P_PS_OFFLOAD_ALLSTASLEEP(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE(__pH2CCmd, 5, 1, __Value)


// host message to firmware cmd
void rtl8192e_set_FwPwrMode_cmd(PADAPTER padapter, u8 Mode);
void rtl8192e_set_FwJoinBssReport_cmd(PADAPTER padapter, u8 mstatus);
u8 rtl8192e_set_rssi_cmd(PADAPTER padapter, u8 *param);
void rtl8192e_set_raid_cmd(PADAPTER padapter, u32 bitmap, u8 *arg);
void rtl8192e_Add_RateATid(PADAPTER padapter, u32 bitmap, u8 *arg,
			   u8 rssi_level);
//u8 rtl8192c_set_FwSelectSuspend_cmd(PADAPTER padapter, u8 bfwpoll, u16 period);


#ifdef CONFIG_P2P_PS
void rtl8192e_set_p2p_ps_offload_cmd(PADAPTER padapter, u8 p2p_ps_state);
//void rtl8723a_set_p2p_ps_offload_cmd(PADAPTER padapter, u8 p2p_ps_state);
#endif //CONFIG_P2P

void CheckFwRsvdPageContent(PADAPTER padapter);
void rtl8192e_set_FwMediaStatus_cmd(PADAPTER padapter, u16 mstatus_rpt);

#ifdef CONFIG_TSF_RESET_OFFLOAD
//u8 rtl8188e_reset_tsf(_adapter *padapter, u8 reset_port);
int reset_tsf(PADAPTER Adapter, u8 reset_port);
#endif	// CONFIG_TSF_RESET_OFFLOAD

#ifdef CONFIG_WOWLAN
typedef struct _SETWOWLAN_PARM {
	u8		mode;
	u8		gpio_index;
	u8		gpio_duration;
	u8		second_mode;
	u8		reserve;
} SETWOWLAN_PARM, *PSETWOWLAN_PARM;

#define FW_WOWLAN_FUN_EN				BIT(0)
#define FW_WOWLAN_PATTERN_MATCH			BIT(1)
#define FW_WOWLAN_MAGIC_PKT				BIT(2)
#define FW_WOWLAN_UNICAST				BIT(3)
#define FW_WOWLAN_ALL_PKT_DROP			BIT(4)
#define FW_WOWLAN_GPIO_ACTIVE			BIT(5)
#define FW_WOWLAN_REKEY_WAKEUP			BIT(6)
#define FW_WOWLAN_DEAUTH_WAKEUP			BIT(7)

#define FW_WOWLAN_GPIO_WAKEUP_EN		BIT(0)
#define FW_FW_PARSE_MAGIC_PKT			BIT(1)

#define FW_REMOTE_WAKE_CTRL_EN			BIT(0)
#define FW_REALWOWLAN_EN				BIT(5)
void rtl8192e_set_wowlan_cmd(_adapter *padapter, u8 enable);
void SetFwRelatedForWoWLAN8192E(_adapter *padapter,
				u8 bHostIsGoingtoSleep);
#endif//CONFIG_WOWLAN


void C2HContentParsing8192E(
	IN	PADAPTER		Adapter,
	IN	u1Byte			c2hCmdId,
	IN	u1Byte			c2hCmdLen,
	IN	pu1Byte 			tmpBuf
);
VOID C2HPacketHandler_8192E(
	IN	PADAPTER		Adapter,
	IN	pu1Byte			Buffer,
	IN	u1Byte			Length
);

#endif//__RTL8188E_CMD_H__
