/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 1999 - 2021 Intel Corporation. */

#ifndef _KCOMPAT_RHEL_DEFS_H_
#define _KCOMPAT_RHEL_DEFS_H_

/* This is the RedHat Enterprise Linux distribution specific definitions file.
 * It defines what features need backports for a given version of the RHEL
 * kernel.
 *
 * It checks the RHEL_RELEASE_CODE and RHEL_RELEASE_VERSION macros to decide
 * what support the target kernel has.
 *
 * It assumes that kcompat_std_defs.h has already been processed, and will
 * #define or #undef any flags that have changed based on backports done by
 * RHEL.
 */

#if !RHEL_RELEASE_CODE
#error "RHEL_RELEASE_CODE is 0 or undefined"
#endif

#ifndef RHEL_RELEASE_VERSION
#error "RHEL_RELEASE_VERSION is undefined"
#endif

#if (RHEL_RELEASE_CODE <= RHEL_RELEASE_VERSION(7,0))
#define NO_BIT_ATTRS
#define NO_ETHDEV_PORT
#define NO_SKB_DUMP
#define NO_REAL_QUEUE_NUM
#define NO_PCI_MSIX_COUNT
#define COMPAT_PTP_NO_PINS
#define NO_NETDEV_PORT
#else /* > 6.8 */
#endif /* 6.8 */

#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(7,2))
#else /* >= 7.2 */
#define HAVE_NDO_FEATURES_CHECK
#endif /* 7.2 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(7,3))
#else /* >= 7.3 */
#undef NEED_DEV_PRINTK_ONCE
#endif /* 7.3 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(7,5))
#define NO_TX_MAXRATE
#else /* >= 7.5 */
#define HAVE_TCF_EXTS_TO_LIST
#endif /* 7.5 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(7,6))
#else /* >= 7.6 */
#undef NEED_TC_CLS_CAN_OFFLOAD_AND_CHAIN0
#undef NEED_TC_SETUP_QDISC_MQPRIO
#endif /* 7.6 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(7,7))
#else /* >= 7.7 */
#define HAVE_DEVLINK_PORT_ATTRS_SET_PORT_FLAVOUR
#define HAVE_NDO_FEATURES_CHECK
#endif /* 7.7 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(8,0))
#else /* >= 8.0 */
#undef HAVE_TCF_EXTS_TO_LIST
#define HAVE_TCF_EXTS_FOR_EACH_ACTION
#endif /* 7.5 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(8,2))
#else /* >= 8.2 */
#undef NEED_DEVLINK_FLASH_UPDATE_STATUS_NOTIFY
#undef NEED_SKB_FRAG_OFF_ACCESSORS
#undef NEED_FLOW_INDR_BLOCK_CB_REGISTER
#define HAVE_DEVLINK_PORT_ATTRS_SET_SWITCH_ID
#endif /* 8.2 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(8,4))
#else /* >= 8.4 */
#undef NEED_DEVLINK_PORT_ATTRS_SET_STRUCT
#undef NEED_NET_PREFETCH
#undef NEED_DEVLINK_FLASH_UPDATE_TIMEOUT_NOTIFY
#undef HAVE_XDP_QUERY_PROG
#endif /* 8.4 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(8,5))
#else /* >= 8.5 */
#define HAVE_DEVLINK_FLASH_UPDATE_PARAMS
#define HAVE_DEVLINK_FLASH_UPDATE_PARAMS_FW
#define HAVE_DEVLINK_REGION_OPS_SNAPSHOT_OPS
#undef HAVE_DEVLINK_FLASH_UPDATE_BEGIN_END_NOTIFY
#undef HAVE_XDP_BUFF_RXQ
#undef HAVE_NAPI_BUSY_LOOP
#endif /* 8.5 */

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(8,6))
#else /* >= 8.6 */
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(9,0))
#define HAVE_ETHTOOL_COALESCE_EXTACK
#endif /* 9.0 */
#undef NEED_ETH_HW_ADDR_SET
#endif

/*****************************************************************************/
#if (RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(9,0))
#else /* >= 9.0 */
#define HAVE_ETHTOOL_EXTENDED_RINGPARAMS
#define HAVE_ETHTOOL_COALESCE_EXTACK
#define HAVE_XDP_BUFF_RXQ
#undef NEED_DEVLINK_ALLOC_SETS_DEV
#undef HAVE_DEVLINK_PARAMS_PUBLISH
#undef HAVE_DEVLINK_RELOAD_ACTION_AND_LIMIT
#undef HAVE_DEVLINK_REGISTER_SETS_DEV
#endif /* 9.0 */


#endif /* _KCOMPAT_RHEL_DEFS_H_ */
