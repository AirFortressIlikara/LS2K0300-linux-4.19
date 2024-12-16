/*
 * EC (Embedded Controller) NPCE985x device driver header for Linux
 *
 * EC relative header file. All the EC registers should be defined here.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at you option) and later version.
 */

#ifndef __EC_NPCE985X_H__
#define __EC_NPCE985X_H__

#define EC_VERSION		"1.11"

/*
 * The following registers are determined by the EC index configureation.
 * 1. fill the PORT_INDEX as EC register.
 * 2. fill the PORT_DATA as EC register write data or get the data from it.
 */
/* base address for io access for Loongson3A+rs780e Notebook platform */
#define EC_BASE_ADDR_PORT	(0xb8000000)
#define SIO_INDEX_PORT		0x2E
#define SIO_DATA_PORT		0x2F

/*
 * EC delay time 500us for register and status access
 * Unit : us
 */
#define EC_REG_DELAY		30000
#define EC_CMD_TIMEOUT		0x1000
#define EC_SEND_TIMEOUT		0xffff
#define EC_RECV_TIMEOUT		0xffff

/*
 * EC access port for with Host communication.
 */
#define EC_CMD_PORT			0x66
#define EC_STS_PORT			0x66
#define EC_DAT_PORT			0x62

/*
 * ACPI legacy commands.
 */
#define CMD_READ_EC			0x80	/* Read EC command. */
#define CMD_WRITE_EC		0x81	/* Write EC command. */
#define CMD_GET_EVENT_NUM	0x84	/* Query EC command, for get SCI event number. */

/*
 * ACPI OEM commands.
 */
#define CMD_RESET			0x4E	/* Reset and poweroff the machine auto-clear: rd/wr */
enum
{
	RESET_OFF = 0,
	RESET_ON,
	PWROFF_ON,
	STR_ON,
	STANDBY_ON
};

#define CMD_EC_VERSION		0x4F	/* EC Version OEM command: 36 Bytes */

/*
 * Used ACPI legacy command 80h to do active.
 */
/* >>> Read/Write temperature & fan index for ACPI 80h/81h command. */
#define INDEX_TEMPERATURE_VALUE		0x1B	/* Current CPU temperature value, Read and Write(81h command). */
#define INDEX_FAN_MAXSPEED_LEVEL	0x5B	/* Fan speed maxinum levels supported. Defaut is 6. */
#define INDEX_FAN_SPEED_LEVEL		0x5C	/* FAn speed level. [0,5] or [0x06, 0x38]*/
#define INDEX_FAN_CTRLMOD			0x5D	/* Fan control mode, 0 = by EC, 1 = by Host.*/
enum
{
	FAN_CTRL_BYEC = 0,
	FAN_CTRL_BYHOST
};
#define INDEX_FAN_STSCTRL			0x5E	/* Fan status/control, 0 = stop, 1 = run. */
enum
{
	FAN_STSCTRL_OFF = 0,
	FAN_STSCTRL_ON
};
#define INDEX_FAN_ERRSTS			0x5F	/* Fan error status, 0 = no error, 1 = has error. */
enum
{
	FAN_ERRSTS_NO = 0,
	FAN_ERRSTS_HAS
};
#define INDEX_FAN_SPEED_LOW			0x08	/* Fan speed low byte.*/
#define INDEX_FAN_SPEED_HIGH		0x09	/* Fan speed high byte. */
/* <<< End Temp & Fan */

/* >>> Read/Write LCD backlight information/control index for ACPI 80h/81h command. */
#define INDEX_BACKLIGHT_CTRLMODE	0x57	/* LCD backlight control mode: 0 = by EC, 1 = by HOST */
enum
{
	BACKLIGHT_CTRL_BYEC = 0,
	BACKLIGHT_CTRL_BYHOST
};
#define INDEX_BACKLIGHT_STSCTRL		0x58	/* LCD backlight status or control: 0 = turn off, 1 = turn on */
enum
{
	BACKLIGHT_OFF = 0,
	BACKLIGHT_ON
};
#define	INDEX_DISPLAY_MAXBRIGHTNESS_LEVEL	0x59	/* LCD backlight brightness max level */
#define	INDEX_DISPLAY_BRIGHTNESS	0x5A	/* 10 stages (0~9) LCD backlight brightness adjust */
enum
{
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_0	= 0,	/* This level is backlight turn off. */
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_1,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_2,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_3,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_4,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_5,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_6,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_7,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_8,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_9,
	FLAG_DISPLAY_BRIGHTNESS_LEVEL_10
};
/* <<< End Backlight */

/* >>> Read battery index for ACPI 80h command */
/*
 * The reported battery die temperature.
 * The temperature is expressed in units of 0.25 seconds and is updated every 2.56 seconds.
 * The equation to calculate reported pack temperature is:
 * Temperature = 0.1 * (256 * TEMPH + TEMPL) Kelvin
 * Temperature -= 273 Degrees Celsius
 * The host sytem has read-only access to this register pair.
 */
#define INDEX_BATTERY_VOL_LOW		0x10	/* Battery Voltage Low byte. */
#define INDEX_BATTERY_VOL_HIGH		0x11	/* Battery Voltage High byte. */
#define INDEX_BATTERY_CURRENT_LOW	0x12	/* Battery Current Low byte. */
#define INDEX_BATTERY_CURRENT_HIGH	0x13	/* Battery Current High byte. */
#define INDEX_BATTERY_AC_LOW		0x18	/* Battery AverageCurrent Low byte. */
#define INDEX_BATTERY_AC_HIGH		0x19	/* Battery AverageCurrent High byte. */
#define INDEX_BATTERY_CAPACITY		0x14	/* Battery RemainingCapacity percent. */
#define INDEX_BATTERY_STATUS_LOW	0x1A	/* Battery Status low byte. */
enum
{
	BIT_BATTERY_STATUS_FD = 4,	/* Battery Fully Discharged Notify. 1 = Fully Discharged */
	BIT_BATTERY_STATUS_FC,		/* Battery Fully Charged Notify. 1 = Fully Charged. */
	BIT_BATTERY_STATUS_DSG,		/* Battery Discharging mode. 0 = in charging mode, 1 = in discharging mode,
	                               relaxation mode, or valid charge termination has occurred. */
	BIT_BATTERY_STATUS_INIT		/* Battery Initialization. 1 = Initialization */
};
#define INDEX_BATTERY_STATUS_HIGH	0x1B	/* Battery Status high byte. */
enum
{
	BIT_BATTERT_STATUS_RTA = 0,	/* Battery Remaining Time Alarm. <= 10min */
	BIT_BATTERY_STATUS_RCA,		/* Battery Remaining Capacity Alarm. <= 430mAh */
	BIT_BATTERY_STATUS_TDA = 3,	/* Battery Terminate Discharge Alarm. */
	BIT_BATTERY_STATUS_OTA,		/* Battery Over Temperature Alarm. */
	BIT_BATTERY_STATUS_TCA = 6,	/* Battery Terminate Charge Alarm. */
	BIT_BATTERY_STATUS_OCA		/* Battery Over Charged Alarm. */
};
#define INDEX_BATTERY_RC_LOW		0x16	/* Battery RemainingCapacity Low byte. */
#define INDEX_BATTERY_RC_HIGH		0x17	/* Battery RemainingCapacity High byte. */
#define INDEX_BATTERY_ATTE_LOW		0x24	/* Battery AverageTimeToEmpty Low byte. */
#define INDEX_BATTERY_ATTE_HIGH		0x25	/* Battery AverageTimeToEmpty High byte. */
#define INDEX_BATTERY_ATTF_LOW		0x26	/* Battery AverageTimeToFull Low byte. */
#define INDEX_BATTERY_ATTF_HIGH		0x27	/* Battery AverageTimeToFull High byte. */
#define INDEX_BATTERY_FCC_LOW		0x1C	/* Battery FullChargeCapacity Low byte. */
#define INDEX_BATTERY_FCC_HIGH		0x1D	/* Battery FullChargeCapacity High byte. */
#define INDEX_BATTERY_CC_LOW		0x2C	/* Battery ChargingCurrent Low byte. */
#define INDEX_BATTERY_CC_HIGH		0x2D	/* Battery ChargingCurrent High byte. */
#define INDEX_BATTERY_CV_LOW		0x84	/* Battery ChargingVoltage Low byte. */
#define INDEX_BATTERY_CV_HIGH		0x85	/* Battery ChargingVoltage High byte. */
#define INDEX_BATTERY_TEMP_LOW		0x30	/* Battery temperature low byte. */
#define INDEX_BATTERY_TEMP_HIGH		0x31	/* Battery temperature high byte. */
#define INDEX_BATTERY_CYCLECNT_LOW	0x20	/* Battery CycleCount Low byte. */
#define INDEX_BATTERY_CYCLECNT_HIGH	0x21	/* Battery CycleCount High byte. */

/* Battery static information. */
#define INDEX_BATTERY_DC_LOW		0x1E	/* Battery DesignCapacity Low byte. */
#define INDEX_BATTERY_DC_HIGH		0x1F	/* Battery DesignCapacity High byte. */
#define INDEX_BATTERY_DV_LOW		0x2E	/* Battery DesignVoltage Low byte. */
#define INDEX_BATTERY_DV_HIGH		0x2F	/* Battery DesignVoltage High byte. */
#define INDEX_BATTERY_SN_LOW		0x2A	/* Battery SerialNumber Low byte. */
#define INDEX_BATTERY_SN_HIGH		0x2B	/* Battery SerialNumber High byte. */
/* <<< End Battery */

#define MASK(x)	(1 << x)

#define INDEX_POWER_STATUS		0xB2	/* Read current power status. */
enum
{
	BIT_POWER_BATPRES,		/* ZW000B Master Battery present. */
	BIT_POWER_SBATFCHG,		/* Slave Battery in fully charging status. */
	BIT_POWER_SBATCHG,		/* Slave Battery in charging status. */
	BIT_POWER_BATFCHG,		/* Battery in fully charging status. */
	BIT_POWER_BATCHG,		/* Battery in charging status. */
	BIT_LIDSTS,				/* Lid status 0: Closed, 1: Open */
	BIT_POWER_SBBATPRES,	/* Slave Battery present. */
	BIT_POWER_ACPRES		/* AC present. */
};

#define	INDEX_DEVICE_STATUS		0xB3	/* Read Current Device Status */
enum
{
	BIT_DEVICE_TP = 0,	/* TouchPad status: 0 = close, 1 = open */
	BIT_DEVICE_WLAN,	/* WLAN status: 0 = close, 1 = open */
	BIT_DEVICE_3G,		/* 3G status: 0 = close, 1 = open */
	BIT_DEVICE_CAM,		/* Camera status: 0 = close, 1 = open */
	BIT_DEVICE_MUTE,	/* Mute status: 0 = close, 1 = open */
	BIT_DEVICE_LID,		/* LID status: 0 = close, 1 = open */
	BIT_DEVICE_BKLIGHT,	/* BackLight status: 0 = close, 1 = open */
	BIT_DEVICE_SIM		/* SIM Card status: 0 = pull out, 1 = insert */
};

#define	INDEX_SHUTDOWN_ID		0xA4	/* Read Shutdown ID */
enum
{
	BIT_SHUTDNID_S45 = 0,	/* in S4 or S5 */
	BIT_SHUTDNID_BATDEAD,	/* Battery Dead */
	BIT_SHUTDNID_OVERHEAT,	/* Over Heat */
	BIT_SHUTDNID_SYSCMD,	/* System command */
	BIT_SHUTDNID_LPRESSPWN,	/* Long press power button */
	BIT_SHUTDNID_PWRUNDER9V,/* Batery voltage low under 9V */
	BIT_SHUTDNID_S3,		/* Entry S3 state */
	BIT_SHUTDNID_S1			/* Entry S1 state */
};

#define	INDEX_SYSTEM_CFG		0xA5		/* Read System config */
#define BIT_SYSCFG_TPSWITCH		(1 << 0)	/* TouchPad switch */
#define BIT_SYSCFG_WLANPRES		(1 << 1)	/* WLAN present */
#define BIT_SYSCFG_NB3GPRES		(1 << 2)	/* 3G present */
#define BIT_SYSCFG_CAMERAPRES	(1 << 3)	/* Camera Present */
#define BIT_SYSCFG_VOLCTRLEC	(1 << 4)	/* Volume control by EC */
#define BIT_SYSCFG_BLCTRLEC		(1 << 5)	/* Backlight control by EC */
#define BIT_SYSCFG_AUTOBRIGHT	(1 << 7)	/* Auto brightness */

#define	INDEX_VOLUME_LEVEL		0xA6		/* Read Volume Level command */
#define	INDEX_VOLUME_MAXLEVEL	0xA7		/* Volume MaxLevel */
#define	VOLUME_MAX_LEVEL		0x0A		/* Volume level max is 11 */
enum
{
	FLAG_VOLUME_LEVEL_0 = 0,
	FLAG_VOLUME_LEVEL_1,
	FLAG_VOLUME_LEVEL_2,
	FLAG_VOLUME_LEVEL_3,
	FLAG_VOLUME_LEVEL_4,
	FLAG_VOLUME_LEVEL_5,
	FLAG_VOLUME_LEVEL_6,
	FLAG_VOLUME_LEVEL_7,
	FLAG_VOLUME_LEVEL_8,
	FLAG_VOLUME_LEVEL_9,
	FLAG_VOLUME_LEVEL_10
};

/* Camera control */
#define INDEX_CAM_STSCTRL			0xAA
enum
{
	CAM_STSCTRL_OFF = 0,
	CAM_STSCTRL_ON
};

/* EC_SC input */
/* EC Status query, by direct read 66h port. */
#define EC_SMI_EVT		(1 << 6)	/* 1 = SMI event padding */
#define EC_SCI_EVT		(1 << 5)	/* 1 = SCI event padding */
#define EC_BURST		(1 << 4)	/* 1 = Controller is in burst mode */
#define EC_CMD			(1 << 3)	/* 1 = Byte in data register is command */

#define EC_IBF			(1 << 1)	/* 1 = Input buffer full (data ready for ec) */
#define EC_OBF			(1 << 0)	/* 1 = Output buffer full (data ready for host) */

/* SCI Event Number from EC */
enum
{
	SCI_EVENT_NUM_AC = 0x30,			/* 0x30, AC in/out */
	SCI_EVENT_NUM_BAT = 0x32,			/* 0x32, BAT in/out */
	SCI_EVENT_NUM_LID =	0x35,			/* 0x35, press the lid or not */
	SCI_EVENT_NUM_SLEEP = 0x40,			/* 0x40, Fn+F4 for entering sleep mode */
	SCI_EVENT_NUM_WLAN,					/* 0x41, Fn+F6, Wlan is on or off */
	SCI_EVENT_NUM_TP = 0x43,			/* 0x43, Fn+F5, TouchPad is on */
	SCI_EVENT_NUM_BRIGHTNESS_OFF,		/* 0x44, Fn+F3, LCD backlight brightness on or off */
	SCI_EVENT_NUM_BRIGHTNESS_DN,		/* 0x45, Fn+F1, LCD backlight brightness down adjust */
	SCI_EVENT_NUM_BRIGHTNESS_UP,		/* 0x46, Fn+F2, LCD backlight brightness up adjust */
	SCI_EVENT_NUM_DISPLAY_TOGGLE,		/* 0x47, Fn+F7, Video switch(LCD/HDMI...)*/
	SCI_EVENT_NUM_POWERBTN = 0x50		/* 0x50, power button */
};

#define SCI_EVENT_NUM_START		SCI_EVENT_NUM_WLAN
#define SCI_EVENT_NUM_END		SCI_EVENT_NUM_POWER

#define EC_TouchPad_STSCTRL          0x78
#define BIT_ECCFG_TPSWITCH_ON      (0 << 1)    /* Fn+F5 TouchPad switch ON*/
#define BIT_ECCFG_TPSWITCH_OFF     (1 << 1)    /* Fn+F5 TouchPad switch OFF*/
enum
{
    BIT_ECCFG_TPSWITCH = 2, /* Fn+F5 TouchPad status: 0 = close, 1 = open */
};

#define EC_BackLight_STSCTRL          0x52
#define BIT_ECCFG_BRIGHTNESS_ON       (1 << 3)    /* Fn+F3, LCD backlight brightness on */
#define BIT_ECCFG_BRIGHTNESS_OFF      (0 << 3)    /* Fn+F3, LCD backlight brightness off*/
#define BIT_ECCFG_BRIGHKEYBOARD_ON    (1 << 4)    /* Fn+F8, KEYBOARD backlight on */
#define BIT_ECCFG_BRIGHKEYBOARD_OFF   (0 << 4)    /* Fn+F8, KEYBOARD backlight off*/
enum
{
    BIT_ECCFG_BRIGHTNESS = 3,       /* Fn+F3, LCD backlight brightness status: 0 = close, 1 = open */
    BIT_ECCFG_BRIGHKEYBOARD = 4,    /* Fn+F8, KEYBOARD backlight status:  0 = close, 1 = open */
};


#define EC_WLAN_STSCTRL          0xD1
#define BIT_ECCFG_WLAN_ON		(1 << 3)	/* Fn+F6 WLAN switch ON */
#define BIT_ECCFG_WLAN_OFF		(0 << 3)	/* Fn+F6 WLAN switch OFF */
enum
{
    BIT_ECCFG_WLAN = 3,            /* Fn+F6, WLAN status: 0 = close , 1 = open  */
};

extern unsigned char app_access_ec_flag;

typedef int (*sci_handler)(int status);

/* The general ec index-io port read action */
extern unsigned char ec985x_read(unsigned char index);
extern unsigned char ec985x_read_all(unsigned char command, unsigned char index);
extern unsigned char ec985x_read_noindex(unsigned char command);

/* The general ec index-io port write action */
extern int ec985x_write(unsigned char index, unsigned char data);
extern int ec985x_write_all(unsigned char command, unsigned char index, unsigned char data);
extern int ec985x_write_noindex(unsigned char command, unsigned char data);

/* Query sequence of 62/66 port access routine. */
extern int ec_query_seq(unsigned char command);
extern int ec985x_get_event_num(void);

extern void clean_ec985x_event_status(void);

#endif /* __EC_NPCE985X_H__ */
