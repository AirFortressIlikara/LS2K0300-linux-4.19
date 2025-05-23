/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 1999 - 2021 Intel Corporation. */

#ifndef _KCOMPAT_STD_DEFS_H_
#define _KCOMPAT_STD_DEFS_H_

/* This file contains the definitions for what kernel features need backports
 * for a given kernel. It targets only the standard stable kernel releases.
 * It must check only LINUX_VERSION_CODE and assume the kernel is a standard
 * release, and not a custom distribution.
 *
 * It must define HAVE_<FLAG> and NEED_<FLAG> for features. It must not
 * implement any backports, instead leaving the implementation to the
 * kcompat_impl.h header.
 *
 * If a feature can be easily implemented as a replacement macro or fully
 * backported, use a NEED_<FLAG> to indicate that the feature needs
 * a backport. (If NEED_<FLAG> is undefined, then no backport for that feature
 * is needed).
 *
 * If a feature cannot be easily implemented in kcompat directly, but
 * requires drivers to make specific changes such as stripping out an entire
 * feature or modifying a function pointer prototype, use a HAVE_<FLAG>.
 */

#ifndef LINUX_VERSION_CODE
#error "LINUX_VERSION_CODE is undefined"
#endif

#ifndef KERNEL_VERSION
#error "KERNEL_VERSION is undefined"
#endif

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0))
#define NO_TX_MAXRATE
#define NEED_DEV_PRINTK_ONCE
#else /* >= 3,19,0 */
#endif /* 3,19,0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,8,0))
#else /* >= 4,8,0 */
#define HAVE_TCF_EXTS_TO_LIST
#endif /* 4,8,0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0))
#define NEED_TC_SETUP_QDISC_MQPRIO
#else /* >= 4,15,0 */
#define HAVE_TC_CB_AND_SETUP_QDISC_MQPRIO
#endif /* 4,15,0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,16,0))
#define NEED_TC_CLS_CAN_OFFLOAD_AND_CHAIN0
#else /* >= 4,16,0 */
#endif /* 4,16,0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,18,0))
#define NEED_MACVLAN_ACCEL_PRIV
#define NEED_MACVLAN_RELEASE_L2FW_OFFLOAD
#define NEED_MACVLAN_SUPPORTS_DEST_FILTER
#else /* >= 4,18,0 */
#define HAVE_DEVLINK_PORT_ATTRS_SET_PORT_FLAVOUR
#endif /* 4,18,0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0))
#else /* >= 4,19,0 */
#undef HAVE_TCF_EXTS_TO_LIST
#define HAVE_TCF_EXTS_FOR_EACH_ACTION
#endif /* 4,19,0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,2,0))
#else /* >= 5.2.0 */
#define HAVE_DEVLINK_PORT_ATTRS_SET_SWITCH_ID
#endif /* 5.2.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,3,0))
#define NEED_DEVLINK_FLASH_UPDATE_STATUS_NOTIFY
#else /* >= 5.3.0 */
#endif /* 5.3.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,7,0))
#define NEED_DEVLINK_REGION_CREATE_OPS
#else /* >= 5.7.0 */
#endif /* 5.7.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0))
#define NEED_SKB_FRAG_OFF_ACCESSORS
#define NEED_FLOW_INDR_BLOCK_CB_REGISTER
#else /* >= 5.4.0 */
#endif /* 5.4.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,9,0))
#define NEED_DEVLINK_PORT_ATTRS_SET_STRUCT
#define HAVE_XDP_QUERY_PROG
#else /* >= 5.9.0 */
#endif /* 5.9.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,10,0))
#define NEED_NET_PREFETCH
#define NEED_DEVLINK_FLASH_UPDATE_TIMEOUT_NOTIFY
#else /* >= 5.10.0 */
#endif /* 5.10.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0))
#define HAVE_DEVLINK_FLASH_UPDATE_BEGIN_END_NOTIFY
#else /* >= 5.11.0 */
#define HAVE_DEVLINK_FLASH_UPDATE_PARAMS_FW
#define HAVE_XSK_BATCHED_DESCRIPTOR_INTERFACES
#define HAVE_PASID_SUPPORT
#endif /* 5.11.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,13,0))
/* HAVE_KOBJ_IN_MDEV_PARENT_OPS_CREATE
 *
 * create api changed as part of the commit c2ef2f50ad0c( vfio/mdev: Remove
 * kobj from mdev_parent_ops->create())
 *
 * if flag is defined use the old API else new API
 */
#define HAVE_KOBJ_IN_MDEV_PARENT_OPS_CREATE
#define HAVE_DEV_IN_MDEV_API
#else /* >= 5.13.0 */
#endif /* 5.13.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0))
#else /* >= 5.14.0 */
#define HAVE_TTY_WRITE_ROOM_UINT
#endif /* 5.14.0 */

/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0))
#define NEED_DEVLINK_ALLOC_SETS_DEV
#define HAVE_DEVLINK_REGISTER_SETS_DEV
#define NEED_ETH_HW_ADDR_SET
#else /* >= 5.15.0 */
#define HAVE_ETHTOOL_COALESCE_EXTACK
#define HAVE_NDO_ETH_IOCTL
#define HAVE_DEVICE_IN_MDEV_PARENT_OPS
#endif /* 5.15.0 */
/*****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0))
#define NEED_NO_NETDEV_PROG_XDP_WARN_ACTION
#else /* >=5.17.0*/
#define HAVE_XDP_DO_FLUSH
#define HAVE_ETHTOOL_EXTENDED_RINGPARAMS
#endif /* 5.17.0 */
#endif /* _KCOMPAT_STD_DEFS_H_ */
