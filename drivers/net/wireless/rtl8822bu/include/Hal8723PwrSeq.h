/******************************************************************************
 *
 * Copyright(c) 2016 - 2017 Realtek Corporation.
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
#ifndef __HAL8723PWRSEQ_H__
#define __HAL8723PWRSEQ_H__
/*
	Check document WM-20110607-Paul-RTL8723A_Power_Architecture-R02.vsd
	There are 6 HW Power States:
	0: POFF--Power Off
	1: PDN--Power Down
	2: CARDEMU--Card Emulation
	3: ACT--Active Mode
	4: LPS--Low Power State
	5: SUS--Suspend

	The transision from different states are defined below
	TRANS_CARDEMU_TO_ACT
	TRANS_ACT_TO_CARDEMU
	TRANS_CARDEMU_TO_SUS
	TRANS_SUS_TO_CARDEMU
	TRANS_CARDEMU_TO_PDN
	TRANS_ACT_TO_LPS
	TRANS_LPS_TO_ACT

	TRANS_END
*/
#include "HalPwrSeqCmd.h"

#define	RTL8723A_TRANS_CARDEMU_TO_ACT_STEPS	15
#define	RTL8723A_TRANS_ACT_TO_CARDEMU_STEPS	15
#define	RTL8723A_TRANS_CARDEMU_TO_SUS_STEPS	15
#define	RTL8723A_TRANS_SUS_TO_CARDEMU_STEPS	15
#define	RTL8723A_TRANS_CARDEMU_TO_PDN_STEPS	15
#define	RTL8723A_TRANS_PDN_TO_CARDEMU_STEPS	15
#define	RTL8723A_TRANS_ACT_TO_LPS_STEPS	15
#define	RTL8723A_TRANS_LPS_TO_ACT_STEPS	15
#define	RTL8723A_TRANS_END_STEPS	1


#define RTL8723A_TRANS_CARDEMU_TO_ACT														\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0020, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0}, /*0x20[0] = 1b'1 enable LDOA12 MACRO block for all interface*/   \
	{0x0067, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, 0}, /*0x67[0] = 0 to disable BT_GPS_SEL pins*/	\
	{0x0001, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_DELAY, 1, PWRSEQ_DELAY_MS},/*Delay 1ms*/   \
	{0x0000, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT5, 0}, /*0x00[5] = 1b'0 release analog Ips to digital ,1:isolation*/   \
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT2, 0},/* disable SW LPS 0x04[10]=0*/	\
	{0x0006, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, BIT1, BIT1},/* wait till 0x04[17] = 1    power ready*/	\
	{0x0006, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0},/* release WLON reset  0x04[16]=1*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT7, 0},/* disable HWPDN 0x04[15]=0*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, (BIT4 | BIT3), 0},/* disable WL suspend*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0},/* polling until return 0*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, BIT0, 0},/**/	\
	{0x004E, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT7, 1},/*0x4C[23] = 0x4E[7] = 1, switch DPDT_SEL_P output from WL BB */\

#define RTL8723A_TRANS_ACT_TO_CARDEMU													\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x001F, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0},/*0x1F[7:0] = 0 turn off RF*/	\
	{0x004E, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT7, 0},/*0x4C[23] = 0x4E[7] = 0, switch DPDT_SEL_P output from register 0x65[2] */\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, BIT1}, /*0x04[9] = 1 turn off MAC by HW state machine*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, BIT1, 0}, /*wait till 0x04[9] = 0 polling until return 0 to disable*/	\
	{0x0000, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT5, BIT5}, /*0x00[5] = 1b'1 analog Ips to digital ,1:isolation*/   \
	{0x0020, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 0}, /*0x20[0] = 1b'0 disable LDOA12 MACRO block*/   \


#define RTL8723A_TRANS_CARDEMU_TO_SUS													\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4 | BIT3, (BIT4 | BIT3)}, /*0x04[12:11] = 2b'11 enable WL suspend for PCIe*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT4, BIT3}, /*0x04[12:11] = 2b'01 enable WL suspend*/	\
	{0x0023, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, BIT4}, /*0x23[4] = 1b'1 12H LDO enter sleep mode*/   \
	{0x0007, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x20}, /*0x07[7:0] = 0x20 SDIO SOP option to disable BG/MB/ACK/SWR*/   \
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT4, BIT3 | BIT4}, /*0x04[12:11] = 2b'11 enable WL suspend for PCIe*/	\
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_WRITE, BIT0, BIT0}, /*Set SDIO suspend local register*/	\
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_POLLING, BIT1, 0}, /*wait power state to suspend*/

#define RTL8723A_TRANS_SUS_TO_CARDEMU													\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT7, 0}, /*clear suspend enable and power down enable*/	\
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_WRITE, BIT0, 0}, /*Set SDIO suspend local register*/	\
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_POLLING, BIT1, BIT1}, /*wait power state to suspend*/\
	{0x0023, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, 0}, /*0x23[4] = 1b'0 12H LDO enter normal mode*/   \
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT4, 0}, /*0x04[12:11] = 2b'01enable WL suspend*/

#define RTL8723A_TRANS_CARDEMU_TO_CARDDIS													\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0007, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x20}, /*0x07 = 0x20 , SOP option to disable BG/MB*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT4, BIT3}, /*0x04[12:11] = 2b'01 enable WL suspend*/	\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT2, BIT2}, /*0x04[10] = 1, enable SW LPS*/	\
	{0x004A, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 1}, /*0x48[16] = 1 to enable GPIO9 as EXT WAKEUP*/   \
	{0x0023, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, BIT4}, /*0x23[4] = 1b'1 12H LDO enter sleep mode*/   \
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_WRITE, BIT0, BIT0}, /*Set SDIO suspend local register*/	\
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_POLLING, BIT1, 0}, /*wait power state to suspend*/

#define RTL8723A_TRANS_CARDDIS_TO_CARDEMU													\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT7, 0}, /*clear suspend enable and power down enable*/	\
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_WRITE, BIT0, 0}, /*Set SDIO suspend local register*/	\
	{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_POLLING, BIT1, BIT1}, /*wait power state to suspend*/\
	{0x004A, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 0}, /*0x48[16] = 0 to disable GPIO9 as EXT WAKEUP*/   \
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT4, 0}, /*0x04[12:11] = 2b'01enable WL suspend*/\
	{0x0023, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, 0}, /*0x23[4] = 1b'0 12H LDO enter normal mode*/   \
	{0x0301, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0},/*PCIe DMA start*/


#define RTL8723A_TRANS_CARDEMU_TO_PDN												\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0023, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, BIT4}, /*0x23[4] = 1b'1 12H LDO enter sleep mode*/   \
	{0x0007, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK | PWR_INTF_USB_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x20}, /*0x07[7:0] = 0x20 SOP option to disable BG/MB/ACK/SWR*/   \
	{0x0006, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 0},/* 0x04[16] = 0*/\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT7, BIT7},/* 0x04[15] = 1*/

#define RTL8723A_TRANS_PDN_TO_CARDEMU												\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT7, 0},/* 0x04[15] = 0*/

#define RTL8723A_TRANS_ACT_TO_LPS														\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0301, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0xFF},/*PCIe DMA stop*/	\
	{0x0522, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0xFF},/*Tx Pause*/	\
	{0x05F8, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/	\
	{0x05F9, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/	\
	{0x05FA, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/	\
	{0x05FB, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/	\
	{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 0},/*CCK and OFDM are disabled, and clock are gated*/	\
	{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_DELAY, 0, PWRSEQ_DELAY_US},/*Delay 1us*/	\
	{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, 0},/*Whole BB is reset*/	\
	{0x0100, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x03},/*Reset MAC TRX*/	\
	{0x0101, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, 0},/*check if removed later*/	\
	{0x0093, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x00},/*When driver enter Sus/ Disable, enable LOP for BT*/	\
	{0x0553, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT5, BIT5},/*Respond TxOK to scheduler*/	\


#define RTL8723A_TRANS_LPS_TO_ACT															\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0x0080, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_WRITE, 0xFF, 0x84}, /*SDIO RPWM*/\
	{0xFE58, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x84}, /*USB RPWM*/\
	{0x0361, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x84}, /*PCIe RPWM*/\
	{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_DELAY, 0, PWRSEQ_DELAY_MS}, /*Delay*/\
	{0x0008, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, 0}, /*.	0x08[4] = 0		 switch TSF to 40M*/\
	{0x0109, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, BIT7, 0}, /*Polling 0x109[7]=0  TSF in 40M*/\
	{0x0029, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT6 | BIT7, 0}, /*.	0x29[7:6] = 2b'00	 enable BB clock*/\
	{0x0101, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, BIT1}, /*.	0x101[1] = 1*/\
	{0x0100, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0xFF}, /*.	0x100[7:0] = 0xFF	 enable WMAC TRX*/\
	{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1 | BIT0, BIT1 | BIT0}, /*.	0x02[1:0] = 2b'11	 enable BB macro*/\
	{0x0522, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0}, /*.	0x522 = 0*/

#define RTL8723A_TRANS_END															\
	/* format */																\
	/* { offset, cut_msk, fab_msk|interface_msk, base|cmd, msk, value }, // comments here*/								\
	{0xFFFF, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, 0, PWR_CMD_END, 0, 0},


extern WLAN_PWR_CFG
rtl8723A_power_on_flow[RTL8723A_TRANS_CARDEMU_TO_ACT_STEPS +
							   RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG
rtl8723A_radio_off_flow[RTL8723A_TRANS_ACT_TO_CARDEMU_STEPS +
							    RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG
rtl8723A_card_disable_flow[RTL8723A_TRANS_ACT_TO_CARDEMU_STEPS +
							       RTL8723A_TRANS_CARDEMU_TO_PDN_STEPS + RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG
rtl8723A_card_enable_flow[RTL8723A_TRANS_ACT_TO_CARDEMU_STEPS +
							      RTL8723A_TRANS_CARDEMU_TO_PDN_STEPS + RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG
rtl8723A_suspend_flow[RTL8723A_TRANS_ACT_TO_CARDEMU_STEPS +
							  RTL8723A_TRANS_CARDEMU_TO_SUS_STEPS + RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG
rtl8723A_resume_flow[RTL8723A_TRANS_ACT_TO_CARDEMU_STEPS +
							 RTL8723A_TRANS_CARDEMU_TO_SUS_STEPS + RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG rtl8723A_hwpdn_flow[RTL8723A_TRANS_ACT_TO_CARDEMU_STEPS
					+ RTL8723A_TRANS_CARDEMU_TO_PDN_STEPS + RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG rtl8723A_enter_lps_flow[RTL8723A_TRANS_ACT_TO_LPS_STEPS
		+ RTL8723A_TRANS_END_STEPS];
extern WLAN_PWR_CFG rtl8723A_leave_lps_flow[RTL8723A_TRANS_LPS_TO_ACT_STEPS
		+ RTL8723A_TRANS_END_STEPS];

#endif
