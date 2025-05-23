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

#ifndef	__ODM_DBG_H__
#define __ODM_DBG_H__

/*#define DEBUG_VERSION	"1.1"*/  /*2015.07.29 YuChen*/
/*#define DEBUG_VERSION	"1.2"*/  /*2015.08.28 Dino*/
/*#define DEBUG_VERSION	"1.3"*/  /*2016.04.28 YuChen*/
#define DEBUG_VERSION	"1.4"  /*2017.03.13 Dino*/

/* -----------------------------------------------------------------------------
 *	Define the debug levels
 *
 *	1.	DBG_TRACE and DBG_LOUD are used for normal cases.
 *	So that, they can help SW engineer to develope or trace states changed
 *	and also help HW enginner to trace every operation to and from HW,
 *	e.g IO, Tx, Rx.
 *
 *	2.	DBG_WARNNING and DBG_SERIOUS are used for unusual or error cases,
 *	which help us to debug SW or HW.
 *
 * -----------------------------------------------------------------------------
 *
 *	Never used in a call to ODM_RT_TRACE()!
 *   */
#define ODM_DBG_OFF					1

/*
 *	Fatal bug.
 *	For example, Tx/Rx/IO locked up, OS hangs, memory access violation,
 *	resource allocation failed, unexpected HW behavior, HW BUG and so on.
 *   */
#define ODM_DBG_SERIOUS				2

/*
 *	Abnormal, rare, or unexpeted cases.
 *	For example, IRP/Packet/OID canceled, device suprisely unremoved and so on.
 *   */
#define ODM_DBG_WARNING				3

/*
 *	Normal case with useful information about current SW or HW state.
 *	For example, Tx/Rx descriptor to fill, Tx/Rx descriptor completed status,
 *	SW protocol state change, dynamic mechanism state change and so on.
 *   */
#define ODM_DBG_LOUD					4

/*
 *	Normal case with detail execution flow or information.
 *   */
#define ODM_DBG_TRACE					5

/*FW DBG MSG*/
#define	RATE_DECISION	1
#define	INIT_RA_TABLE	2
#define	RATE_UP		4
#define	RATE_DOWN		8
#define	TRY_DONE		16
#define	RA_H2C			32
#define	F_RATE_AP_RPT	64
#define	DBC_FW_CLM	9

/* -----------------------------------------------------------------------------
 * Define the tracing components
 *
 * -----------------------------------------------------------------------------
 *BB FW Functions*/
#define	PHYDM_FW_COMP_RA			BIT(0)
#define	PHYDM_FW_COMP_MU			BIT(1)
#define	PHYDM_FW_COMP_PATH_DIV		BIT(2)
#define	PHYDM_FW_COMP_PT				BIT(3)

/*------------------------Export Marco Definition---------------------------*/

#define	config_phydm_read_txagc_check(data)		(data != INVALID_TXAGC_DATA)

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
extern	VOID DCMD_Printf(const char *pMsg);

#define	pr_debug				DbgPrint
#define	dcmd_printf				DCMD_Printf
#define	dcmd_scanf				DCMD_Scanf
#define	RT_PRINTK				pr_debug
#define	PRINT_MAX_SIZE		512
#elif (DM_ODM_SUPPORT_TYPE == ODM_CE) && defined(DM_ODM_CE_MAC80211)

#elif (DM_ODM_SUPPORT_TYPE == ODM_CE)
#undef pr_debug
#define pr_debug	printk
#define RT_PRINTK(fmt, args...)	pr_debug(fmt, ## args)
#define	RT_DISP(dbgtype, dbgflag, printstr)
#define RT_TRACE(adapter, comp, drv_level, fmt, args...)	\
		RTW_INFO(fmt, ## args)
#else
#define pr_debug	panic_printk
/*#define RT_PRINTK(fmt, args...)	pr_debug("%s(): " fmt, __FUNCTION__, ## args);*/
#define RT_PRINTK(fmt, args...)	pr_debug(fmt, ## args)
#endif

#ifndef ASSERT
#define ASSERT(expr)
#endif

#if DBG
#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
#define PHYDM_DBG(dm, comp, fmt, args...)			\
	do {							\
		if ((comp) & (dm->debug_components)) {          \
			pr_debug("[PHYDM] ");			\
			RT_PRINTK(fmt, ## args);				\
		}						\
	} while (0)

#define PHYDM_DBG_F(dm, comp, fmt, args...)			\
	do {							\
		if ((comp) & dm->debug_components) {		\
			RT_PRINTK(fmt, ## args);				\
		}						\
	} while (0)

#define PHYDM_PRINT_ADDR(dm, comp, title_str, addr)		\
	do {							\
		if ((comp) & dm->debug_components) {		\
			int __i;				\
			u8 *__ptr = (u8 *)addr;			\
			pr_debug("[PHYDM] ");			\
			pr_debug(title_str);			\
			pr_debug(" ");				\
			for (__i = 0; __i < 6; __i++)		\
				pr_debug("%02X%s", __ptr[__i], (__i == 5) ? "" : "-");\
			pr_debug("\n");				\
		}						\
	} while (0)
#elif (DM_ODM_SUPPORT_TYPE == ODM_WIN)

static __inline void PHYDM_DBG(PDM_ODM_T dm, int comp, char *fmt, ...)
{

	RT_STATUS rt_status;
	va_list args;
	char buf[PRINT_MAX_SIZE] = {0};

	if ((comp & dm->debug_components) == 0)
		return;

	if (fmt == NULL)
		return;

	va_start(args, fmt);
	rt_status = (RT_STATUS)RtlStringCbVPrintfA(buf, PRINT_MAX_SIZE, fmt, args);
	va_end(args);

	if (rt_status != RT_STATUS_SUCCESS) {
		DbgPrint("Failed (%d) to print message to buffer\n", rt_status);
		return;
	}

	DbgPrint("[PHYDM] %s", buf);
}

static __inline void PHYDM_DBG_F(PDM_ODM_T dm, int comp, char *fmt, ...)
{

	RT_STATUS rt_status;
	va_list args;
	char buf[PRINT_MAX_SIZE] = {0};

	if ((comp & dm->debug_components) == 0)
		return;

	if (fmt == NULL)
		return;

	va_start(args, fmt);
	rt_status = (RT_STATUS)RtlStringCbVPrintfA(buf, PRINT_MAX_SIZE, fmt, args);
	va_end(args);

	if (rt_status != RT_STATUS_SUCCESS) {
		/*DbgPrint("DM Print Fail\n");*/
		return;
	}

	DbgPrint("%s", buf);
}

#define PHYDM_PRINT_ADDR(p_dm, comp, title_str, ptr)							 do {\
		if ((comp) & p_dm->debug_components) { \
			\
			int __i;																\
			u8 *__ptr = (u8 *)ptr;											\
			pr_debug("[PHYDM] ");													\
			pr_debug(title_str);													\
			pr_debug(" ");														\
			for (__i = 0; __i < 6; __i++)												\
				pr_debug("%02X%s", __ptr[__i], (__i == 5) ? "" : "-");						\
			pr_debug("\n");														\
		}	\
	} while (0)

#else
#define PHYDM_DBG(dm, comp, fmt, args...)                                      \
	do {								\
		if ((comp) & (dm->debug_components)) {                         \
			RT_TRACE(((struct rtl_priv *)dm->adapter), COMP_PHYDM, \
				 DBG_DMESG, "[PHYDM] " fmt, ##args);           \
		}												\
	} while (0)

#define PHYDM_DBG_F(dm, comp, fmt, args...)                                    \
	do {								\
		if ((comp) & dm->debug_components) {                           \
			RT_TRACE(((struct rtl_priv *)dm->adapter), COMP_PHYDM, \
				 DBG_DMESG, fmt, ##args);                      \
		}	\
	} while (0)

#define PHYDM_PRINT_ADDR(dm, comp, title_str, addr)                            \
	do {								\
		if ((comp) & dm->debug_components) {                           \
			RT_TRACE(((struct rtl_priv *)dm->adapter), COMP_PHYDM, \
				 DBG_DMESG, "[PHYDM] " title_str "%pM\n",      \
				 addr);                                        \
		}							\
	} while (0)
#endif

#define ODM_RT_TRACE(dm, comp, level, fmt)

#else
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
static	__inline	void	PHYDM_DBG(struct dm_struct	*dm, int comp, char *fmt,
		...)
{}
static __inline void PHYDM_DBG_F(struct dm_struct	*dm, int comp, char *fmt,
				 ...)
{}
#else
#define PHYDM_DBG(dm, comp, fmt)
#define PHYDM_DBG_F(dm, comp, fmt)
#endif
#define PHYDM_PRINT_ADDR(dm, comp, title_str, ptr)
#define ODM_RT_TRACE(dm, comp, level, fmt)
#endif

#define	BB_DBGPORT_PRIORITY_3	3	/*Debug function (the highest priority)*/
#define	BB_DBGPORT_PRIORITY_2	2	/*Check hang function & Strong function*/
#define	BB_DBGPORT_PRIORITY_1	1	/*Watch dog function*/
#define	BB_DBGPORT_RELEASE		0	/*Init value (the lowest priority)*/

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
#define	PHYDM_DBGPRINT		0
#define	PHYDM_SSCANF(x, y, z)	dcmd_scanf(x, y, z)
#define	PHYDM_VAST_INFO_SNPRINTF	PDM_SNPF
#if (PHYDM_DBGPRINT == 1)
#define	PDM_SNPF(msg)	\
	do {\
		rsprintf msg;\
		pr_debug("%s", output);\
	} while (0)
#else

static __inline void PDM_SNPF(u32	out_len, u32 used, char *buff, int len,
			      char *fmt, ...)
{
	RT_STATUS rt_status;
	va_list args;
	char buf[PRINT_MAX_SIZE] = {0};

	if (fmt == NULL)
		return;

	va_start(args, fmt);
	rt_status = (RT_STATUS)RtlStringCbVPrintfA(buf, PRINT_MAX_SIZE, fmt, args);
	va_end(args);

	if (rt_status != RT_STATUS_SUCCESS) {
		/*DbgPrint("DM Print Fail\n");*/
		return;
	}

	DCMD_Printf(buf);
}

#endif	/*#if (PHYDM_DBGPRINT == 1)*/
#else	/*(DM_ODM_SUPPORT_TYPE & (ODM_CE | ODM_AP))*/
#if (DM_ODM_SUPPORT_TYPE == ODM_CE) || defined(__OSK__)
#define	PHYDM_DBGPRINT		0
#else
#define	PHYDM_DBGPRINT		1
#endif
#define	MAX_ARGC				20
#define	MAX_ARGV				16
#define	DCMD_DECIMAL			"%d"
#define	DCMD_CHAR				"%c"
#define	DCMD_HEX				"%x"

#define	PHYDM_SSCANF(x, y, z)	sscanf(x, y, z)

#define	PHYDM_VAST_INFO_SNPRINTF(out_len, used, buff, len, fmt, args...)	\
	do {								\
		RT_TRACE(((struct rtl_priv *)dm->adapter), COMP_PHYDM, \
			DBG_DMESG, fmt, ##args);		\
	} while (0)

#if (PHYDM_DBGPRINT == 1)
#define	PDM_SNPF(out_len, used, buff, len, fmt, args...)		\
	do {								\
		snprintf(buff, len, fmt, ##args);			\
		pr_debug("%s", output);					\
	} while (0)
#else
#define	PDM_SNPF(out_len, used, buff, len, fmt, args...)		\
	do {								\
		if (out_len > used)					\
			used += snprintf(buff, len, fmt, ##args);	\
	} while (0)
#endif
#endif

void phydm_init_debug_setting(struct dm_struct *dm);

void phydm_bb_dbg_port_header_sel(void *dm_void, u32 header_idx);

u8 phydm_set_bb_dbg_port(void *dm_void, u8 curr_dbg_priority,
			 u32 debug_port);

void phydm_release_bb_dbg_port(void *dm_void);

u32 phydm_get_bb_dbg_port_value(void *dm_void);

void phydm_reset_rx_rate_distribution(struct dm_struct *dm);

void phydm_rx_rate_distribution(void *dm_void);

void phydm_get_avg_phystatus_val(void *dm_void);

void phydm_get_phy_statistic(void *dm_void);

void phydm_basic_dbg_message(void *dm_void);

void phydm_basic_profile(void *dm_void, u32 *_used, char *output,
			 u32 *_out_len);
#if (DM_ODM_SUPPORT_TYPE & (ODM_CE | ODM_AP))
s32 phydm_cmd(struct dm_struct *dm, char *input, u32 in_len, u8 flag,
	      char *output, u32 out_len);
#endif
void phydm_cmd_parser(struct dm_struct *dm, char input[][16],
		      u32 input_num,
		      u8 flag, char *output, u32 out_len);

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
void phydm_sbd_check(
	struct	dm_struct	*dm
);

void phydm_sbd_callback(
	struct phydm_timer_list		*timer
);

void phydm_sbd_workitem_callback(
	void	*context
);
#endif

void phydm_fw_trace_en_h2c(void *dm_void, boolean enable,
			   u32 fw_debug_component, u32 monitor_mode, u32 macid);

void phydm_fw_trace_handler(void *dm_void, u8 *cmd_buf, u8 cmd_len);

void phydm_fw_trace_handler_code(void *dm_void, u8 *buffer, u8 cmd_len);

void phydm_fw_trace_handler_8051(void *dm_void, u8 *cmd_buf, u8 cmd_len);

#endif /* __ODM_DBG_H__ */
