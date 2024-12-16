/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2008 - 2022 Xel Technology. */

#ifndef _XLNID_DEBUG_H_
#define _XLNID_DEBUG_H_

//#define XLNID_DEBUG_LOOPBACK

/* shiyage multiqueue msi test */
//#define XLNID_MULTI_QUEUE
#define XLNID_DMACTL_DIRECT
//#define XLNID_GC_XL84101

#ifdef XLNID_GC_XL84101
#define XLNID_XL84101_MAX_QUEUES    32
#define XLNID_XL84101_BUFF_SIZE     (64 * 1024)
#endif

#ifdef __KERNEL__
#include "xlnid.h"
#endif

#define DEBUG_TX_CMD         (1 << 0)
#define DEBUG_RX_CMD         (1 << 1)

#define DEBUG_CONFIG_RX         0x01
#define DEBUG_CONFIG_TX         0x02
#define DEBUG_CONFIG_ALL        0x03

/* PTSW */
#ifndef SIOCWANDEV
#define SIOCWANDEV  0x894A
#endif

#ifndef SIOCDEVPRIVATE
#define SIOCDEVPRIVATE  0x89F0
#endif

#define NETLINK_GENERIC 16

typedef struct xel_pci_info_s
{
    unsigned short device_id;
	unsigned short vendor_id;
	unsigned int pcie_bar0;
	unsigned int pcie_bar0_end;
	unsigned int pcie_resource_length;
	uintptr_t pcie_io_addr;
	unsigned char  pcie_function_id;
	unsigned int flag;
    unsigned int ctxdesc_send;
    unsigned int inited;
    unsigned int low_power;
    unsigned long last_stats_check;
	void *dev;
	struct xlnid_hw *hw;
}xel_pci_info_t;

#ifdef __KERNEL__
#define XEL_PCI_DEV_NAME        "xlnid_kernel"
#else
#define XEL_PCI_DEV_NAME        "/dev/xlnid_kernel"
static unsigned int debug_cmd = 0;
#endif

typedef struct xel_pipe
{
    unsigned int    addr;
    unsigned int    len;
    unsigned int    data;
	unsigned int    unit;
	unsigned int    data1;
	unsigned int    data2;
	unsigned int    data3;
	unsigned int    data4;
	unsigned int    data5;
	unsigned int    data6;
} xel_pipe_t;

enum 
{
    ioctl_xel_pci_read              = 0x1000,
    ioctl_xel_pci_write,
    ioctl_xel_pci_config_read,
    ioctl_xel_pci_config_write,
    ioctl_xel_dma_get,
    ioctl_xel_pci_base_addr_get,
    ioctl_xel_get_pci_num,
    ioctl_xel_copy_pci_info,
    ioctl_xel_debug_cmd,
    ioctl_xel_init_nic,
    ioctl_xel_msix_table,

    ioctl_xel_rxdesc_read,
    ioctl_xel_txdesc_read,
    ioctl_xel_dmactl_config,
    ioctl_xel_clear_stats,

    ioctl_xel_rx_stats,
    ioctl_xel_rx_err_frame,
    ioctl_xel_rx_buff_offset,
    ioctl_xel_rx_ring_set,
    ioctl_xel_rx_data_check,
    ioctl_xel_rx_itr_set,
    ioctl_xel_rx_alloc_buffer,
    ioctl_xel_rx_rss_set,

    ioctl_xel_tx_stats,
    ioctl_xel_tx_vlan,
    ioctl_xel_tx_ip_csum,
    ioctl_xel_tx_l4_csum,
    ioctl_xel_tx_tcp_seg,
    ioctl_xel_tx_macsec,
    ioctl_xel_tx_double_ctxdesc,
    ioctl_xel_tx_datalen_zero,
    ioctl_xel_tx_pad,
    ioctl_xel_tx_ctxdesc,

    ioctl_xel_stats_fdir,

    ioctl_xel_maxdata_per_txd_set,
    ioctl_xel_maxdata_per_txd_get,

    ioctl_xel_interrupt_count,
    ioctl_xel_interrupt_debug,

    ioctl_xel_ptp_debug,
    ioctl_xel_checksum,

    ioctl_xel_speed_set,

#ifdef XLNID_GC_XL84101
    ioctl_xel_magic_packet,
    ioctl_xel_vlan_prio,
    ioctl_xel_read_data,
    ioctl_xel_write_data,
    ioctl_xel_int_mode_msix,
#endif

#ifdef XLNID_DEBUG_ITR
    ioctl_xel_intr_rate_get,
#endif

#ifdef XLNID_DEBUG_LOOPBACK
    ioctl_xel_debug_loopback,
#endif

    ioctl_xel_ptsw,
};

#ifdef __KERNEL__
extern void xlnid_test_alloc_rx_buffers(struct xlnid_ring *rx_ring, u16 cleaned_count);
extern void xlnid_test_write_eitr(struct xlnid_q_vector *q_vector);
extern int xlnid_test_poll(struct napi_struct *napi, int budget);
extern void xlnid_test_down(struct xlnid_adapter *adapter);
extern void xlnid_test_up(struct xlnid_adapter *adapter);
extern void xlnid_test_free_tx_resources(struct xlnid_ring *tx_ring);
extern void xlnid_test_free_rx_resources(struct xlnid_ring *rx_ring);
extern netdev_tx_t xlnid_test_xmit_frame_ring(struct sk_buff *skb,
				  struct xlnid_adapter __maybe_unused *adapter,
				  struct xlnid_ring *tx_ring);

extern int xlnid_test_init_interrupt_scheme(struct xlnid_adapter *adapter);
extern void xlnid_test_clear_interrupt_scheme(struct xlnid_adapter *adapter);
extern void xlnid_test_tx_ctxtdesc(struct xlnid_ring *tx_ring, u32 vlan_macip_lens,
		       u32 fcoe_sof_eof, u32 type_tucmd, u32 mss_l4len_idx);

#ifdef XLNID_DEBUG_LOOPBACK
extern int xlnid_test_loopback_start(struct xlnid_adapter *adapter);
extern int xlnid_test_loopback_stop(void);
extern int xlnid_test_loopback_init(struct xlnid_adapter *adapter);
extern void xlnid_test_loopback_exit(struct xlnid_adapter *adapter);
extern void xlnid_test_loopback_intr(struct xlnid_adapter *adapter, u32 eicr);
#endif

#endif /* __KERNEL__ */


#endif /* _XLNID_DEBUG_H_ */


