/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2022 - 2023 Mucse Corporation. */

#ifndef _RNPGBE_COMMON_H_
#define _RNPGBE_COMMON_H_

#include <linux/skbuff.h>
#include <linux/highmem.h>
#include "rnpgbe_type.h"
#include "rnpgbe.h"
#include "rnpgbe_regs.h"
#include "rnp_compat.h"

struct rnpgbe_adapter;

#define TRACE() printk(KERN_DEBUG "==[ %s %d ] ==\n", __func__, __LINE__)

#ifdef CONFIG_RNP_RX_DEBUG
#define rx_debug_printk printk
#define rx_buf_dump buf_dump
#define rx_dbg(fmt, args...)                                                   \
	printk(KERN_DEBUG "[ %s:%d ] " fmt, __func__, __LINE__, ##args)
#else
#define rx_debug_printk(fmt, args...)
#define rx_buf_dump(a, b, c)
#define rx_dbg(fmt, args...)
#endif //CONFIG_RNP_RX_DEBUG

#ifdef CONFIG_RNP_TX_DEBUG
#define desc_hex_dump(msg, buf, len)                                           \
	print_hex_dump(KERN_WARNING, msg, DUMP_PREFIX_OFFSET, 16, 1, (buf),    \
		       (len), false)
#define rnpgbe_skb_dump _rnpgbe_skb_dump

#define tx_dbg(fmt, args...)                                                   \
	printk(KERN_DEBUG "[ %s:%d ] " fmt, __func__, __LINE__, ##args)
#else
#define desc_hex_dump(msg, buf, len)
#define rnpgbe_skb_dump(skb, full_pkt)
#define tx_dbg(fmt, args...)
#endif //CONFIG_RNP_TX_DEBUG

#ifdef DEBUG
#define dbg(fmt, args...)                                                      \
	printk(KERN_DEBUG "[ %s:%d ] " fmt, __func__, __LINE__, ##args)
#else
#define dbg(fmt, args...)
#endif

#ifdef CONFIG_RNP_VF_DEBUG
#define vf_dbg(fmt, args...)                                                   \
	printk(KERN_DEBUG "[ %s:%d ] " fmt, __func__, __LINE__, ##args)
#else
#define vf_dbg(fmt, args...)
#endif

u16 rnpgbe_get_pcie_msix_count_generic(struct rnpgbe_hw *hw);
void rnpgbe_free_msix_vectors(struct rnpgbe_adapter *adapter);
int rnpgbe_acquire_msix_vectors(struct rnpgbe_adapter *adapter, int vectors);

s32 rnpgbe_init_eeprom_params_generic(struct rnpgbe_hw *hw);

u16 rnpgbe_get_pcie_msix_count_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_init_ops_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_init_hw_generic(struct rnpgbe_hw *hw);
void rnpgbe_reset_msix_table_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_start_hw_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_start_hw_gen2(struct rnpgbe_hw *hw);
s32 rnpgbe_clear_hw_cntrs_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_read_pba_string_generic(struct rnpgbe_hw *hw, u8 *pba_num,
				   u32 pba_num_size);
s32 rnpgbe_get_mac_addr_generic(struct rnpgbe_hw *hw, u8 *mac_addr);

s32 rnpgbe_get_permtion_mac_addr(struct rnpgbe_hw *hw, u8 *mac_addr);
enum rnpgbe_bus_width rnpgbe_convert_bus_width(u16 link_status);
enum rnpgbe_bus_speed rnpgbe_convert_bus_speed(u16 link_status);
s32 rnpgbe_get_bus_info_generic(struct rnpgbe_hw *hw);
void rnpgbe_set_lan_id_multi_port_pcie(struct rnpgbe_hw *hw);
s32 rnpgbe_stop_adapter_generic(struct rnpgbe_hw *hw);

s32 rnpgbe_led_on_generic(struct rnpgbe_hw *hw, u32 index);
s32 rnpgbe_led_off_generic(struct rnpgbe_hw *hw, u32 index);

s32 rnpgbe_init_eeprom_params_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_write_eeprom_generic(struct rnpgbe_hw *hw, u16 offset, u16 data);
s32 rnpgbe_write_eeprom_buffer_bit_bang_generic(struct rnpgbe_hw *hw,
						u16 offset, u16 words,
						u16 *data);
s32 rnpgbe_read_eerd_generic(struct rnpgbe_hw *hw, u16 offset, u16 *data);
s32 rnpgbe_read_eerd_buffer_generic(struct rnpgbe_hw *hw, u16 offset, u16 words,
				    u16 *data);
s32 rnpgbe_write_eewr_generic(struct rnpgbe_hw *hw, u16 offset, u16 data);
s32 rnpgbe_write_eewr_buffer_generic(struct rnpgbe_hw *hw, u16 offset,
				     u16 words, u16 *data);
s32 rnpgbe_read_eeprom_bit_bang_generic(struct rnpgbe_hw *hw, u16 offset,
					u16 *data);
s32 rnpgbe_read_eeprom_buffer_bit_bang_generic(struct rnpgbe_hw *hw, u16 offset,
					       u16 words, u16 *data);
u16 rnpgbe_calc_eeprom_checksum_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_validate_eeprom_checksum_generic(struct rnpgbe_hw *hw,
					    u16 *checksum_val);
s32 rnpgbe_update_eeprom_checksum_generic(struct rnpgbe_hw *hw);

s32 rnpgbe_set_rar_generic(struct rnpgbe_hw *hw, u32 index, u8 *addr, u32 vmdq,
			   u32 enable_addr);
s32 rnpgbe_clear_rar_generic(struct rnpgbe_hw *hw, u32 index);
s32 rnpgbe_init_rx_addrs_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_update_mc_addr_list_generic(struct rnpgbe_hw *hw,
				       struct net_device *netdev);
s32 rnpgbe_enable_mc_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_disable_mc_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_disable_rx_buff_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_enable_rx_buff_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_enable_rx_dma_generic(struct rnpgbe_hw *hw, u32 regval);
s32 rnpgbe_fc_enable_generic(struct rnpgbe_hw *hw);
void rnpgbe_fc_autoneg(struct rnpgbe_hw *hw);

s32 rnpgbe_acquire_swfw_sync(struct rnpgbe_hw *hw, u16 mask);
void rnpgbe_release_swfw_sync(struct rnpgbe_hw *hw, u16 mask);
s32 rnpgbe_get_san_mac_addr_generic(struct rnpgbe_hw *hw, u8 *san_mac_addr);
s32 rnpgbe_set_vmdq_generic(struct rnpgbe_hw *hw, u32 rar, u32 vmdq);
s32 rnpgbe_set_vmdq_san_mac_generic(struct rnpgbe_hw *hw, u32 vmdq);
s32 rnpgbe_clear_vmdq_generic(struct rnpgbe_hw *hw, u32 rar, u32 vmdq);
s32 rnpgbe_init_uta_tables_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_set_vfta_generic(struct rnpgbe_hw *hw, u32 vlan, u32 vind,
			    bool vlan_on);
s32 rnpgbe_clear_vfta_generic(struct rnpgbe_hw *hw);
s32 rnpgbe_check_mac_link_generic(struct rnpgbe_hw *hw,
				  rnpgbe_link_speed *speed, bool *link_up,
				  bool link_up_wait_to_complete);
s32 rnpgbe_get_wwn_prefix_generic(struct rnpgbe_hw *hw, u16 *wwnn_prefix,
				  u16 *wwpn_prefix);
s32 rnpgbe_blink_led_start_generic(struct rnpgbe_hw *hw, u32 index);
s32 rnpgbe_blink_led_stop_generic(struct rnpgbe_hw *hw, u32 index);
void rnpgbe_set_mac_anti_spoofing(struct rnpgbe_hw *hw, bool enable, int pf);
void rnpgbe_set_vlan_anti_spoofing(struct rnpgbe_hw *hw, bool enable, int vf);
s32 rnpgbe_get_device_caps_generic(struct rnpgbe_hw *hw, u16 *device_caps);
s32 rnpgbe_set_fw_drv_ver_generic(struct rnpgbe_hw *hw, u8 maj, u8 min,
				  u8 build, u8 ver);
void rnpgbe_clear_tx_pending(struct rnpgbe_hw *hw);

void rnpgbe_set_rxpba_generic(struct rnpgbe_hw *hw, int num_pb, u32 headroom,
			      int strategy);
u8 *rnpgbe_addr_list_itr(struct rnpgbe_hw __maybe_unused *hw, u8 **mc_addr_ptr);

s32 rnpgbe_reset_pipeline_82599(struct rnpgbe_hw *hw);

//================= registers  read/write helper =====
#define p_rnpgbe_wr_reg(reg, val)                                              \
	do {                                                                   \
		printk(KERN_DEBUG " wr-reg: %p <== 0x%08x \t#%-4d %s\n",       \
		       (reg), (val), __LINE__, __FILE__);                      \
		iowrite32((val), (void *)(reg));                               \
	} while (0)

static inline unsigned int prnpgbe_rd_reg(void *reg)
{
	unsigned int v = ioread32((void *)(reg));

	printk(KERN_DEBUG "  %p => 0x%08x\n", reg, v);
	return v;
}

#ifdef IO_PRINT
static inline unsigned int rnpgbe_rd_reg(void *reg)
{
	unsigned int v = ioread32((void *)(reg));

	dbg(" rd-reg: %p <== 0x%08x\n", reg, v);
	return v;
}
#define rnpgbe_wr_reg(reg, val)                                                \
	do {                                                                   \
		dbg(" wr-reg: %p <== 0x%08x \t#%-4d %s\n", (reg), (val),       \
		    __LINE__, __FILE__);                                       \
		iowrite32((val), (void *)(reg));                               \
	} while (0)
#else
#define rnpgbe_rd_reg(reg) readl((void *)(reg))
#define rnpgbe_wr_reg(reg, val) writel((val), (void *)(reg))
#endif

#define rd32(hw, off) rnpgbe_rd_reg((hw)->hw_addr + (off))
#define wr32(hw, off, val) rnpgbe_wr_reg((hw)->hw_addr + (off), (val))

#define nic_rd32(nic, off) rnpgbe_rd_reg((nic)->nic_base_addr + (off))
#define nic_wr32(nic, off, val)                                                \
	rnpgbe_wr_reg((nic)->nic_base_addr + (off), (val))

#define dma_rd32(dma, off) rnpgbe_rd_reg((dma)->dma_base_addr + (off))
#define dma_wr32(dma, off, val)                                                \
	rnpgbe_wr_reg((dma)->dma_base_addr + (off), (val))

#define dma_ring_rd32(dma, off) rnpgbe_rd_reg((dma)->dma_ring_addr + (off))
#define dma_ring_wr32(dma, off, val)                                           \
	rnpgbe_wr_reg((dma)->dma_ring_addr + (off), (val))

#define eth_rd32(eth, off) rnpgbe_rd_reg((eth)->eth_base_addr + (off))
#define eth_wr32(eth, off, val)                                                \
	rnpgbe_wr_reg((eth)->eth_base_addr + (off), (val))

#define mac_rd32(mac, off) rnpgbe_rd_reg((mac)->mac_addr + (off))
#define mac_wr32(mac, off, val) rnpgbe_wr_reg((mac)->mac_addr + (off), (val))
#ifdef debug_ring
static inline unsigned int rnpgbe_rd_reg_1(int ring, u32 off, void *reg)
{
	unsigned int v = ioread32((void *)(reg));

	printk(KERN_DEBUG "%d rd-reg: %x <== 0x%08x\n", ring, off, v);
	return v;
}

#define ring_rd32(ring, off)                                                   \
	rnpgbe_rd_reg_1(ring->rnpgbe_queue_idx, off, (ring)->ring_addr + (off))
#define ring_wr32(ring, off, val)                                              \
	rnpgbe_wr_reg((ring)->ring_addr + (off), (val))
#else
#define ring_rd32(ring, off) rnpgbe_rd_reg((ring)->ring_addr + (off))
#define ring_wr32(ring, off, val)                                              \
	rnpgbe_wr_reg((ring)->ring_addr + (off), (val))
#endif

#define pwr32(hw, off, val) p_rnpgbe_wr_reg((hw)->hw_addr + (off), (val))

#define rnpgbe_mbx_rd(hw, off) rnpgbe_rd_reg((hw)->ring_msix_base + (off))
#define rnpgbe_mbx_wr(hw, off, val)                                            \
	rnpgbe_wr_reg((hw)->ring_msix_base + (off), val)

static inline void hw_queue_strip_rx_vlan(struct rnpgbe_hw *hw, u8 ring_num,
					  bool enable)
{
	u32 reg = RNP_ETH_VLAN_VME_REG(ring_num / 32);
	u32 offset = ring_num % 32;
	u32 data = rd32(hw, reg);

	if (enable == true)
		data |= (1 << offset);
	else
		data &= ~(1 << offset);
	wr32(hw, reg, data);
}

#define rnpgbe_set_reg_bit(hw, reg_def, bit)                                   \
	do {                                                                   \
		u32 reg = reg_def;                                             \
		u32 value = rd32(hw, reg);                                     \
		dbg("before set  %x %x\n", reg, value);                        \
		value |= (0x01 << bit);                                        \
		dbg("after set %x %x\n", reg, value);                          \
		wr32(hw, reg, value);                                          \
	} while (0)

#define rnpgbe_clr_reg_bit(hw, reg_def, bit)                                   \
	do {                                                                   \
		u32 reg = reg_def;                                             \
		u32 value = rd32(hw, reg);                                     \
		dbg("before clr %x %x\n", reg, value);                         \
		value &= (~(0x01 << bit));                                     \
		dbg("after clr %x %x\n", reg, value);                          \
		wr32(hw, reg, value);                                          \
	} while (0)

#define rnpgbe_vlan_filter_on(hw)                                              \
	rnpgbe_set_reg_bit(hw, RNP_ETH_VLAN_FILTER_ENABLE, 30)
#define rnpgbe_vlan_filter_off(hw)                                             \
	rnpgbe_clr_reg_bit(hw, RNP_ETH_VLAN_FILTER_ENABLE, 30)

#define DPRINTK(nlevel, klevel, fmt, args...)                                  \
	((NETIF_MSG_##nlevel & adapter->msg_enable) ?                          \
		 (void)(netdev_printk(KERN_##klevel, adapter->netdev, fmt,     \
				      ##args)) :                               \
		 NULL)

//==== log helper ===
#ifdef HW_DEBUG
#define hw_dbg(hw, fmt, args...) printk(KERN_DEBUG "hw-dbg : " fmt, ##args)
#define eth_dbg(eth, fmt, args...) printk(KERN_DEBUG "hw-dbg : " fmt, ##args)
#else
#define hw_dbg(hw, fmt, args...)
#define eth_dbg(hw, fmt, args...)
#endif

//#define RNP_DEBUG_OPEN
#ifdef RNP_DEBUG_OPEN
#define rnpgbe_dbg(fmt, args...) printk(KERN_DEBUG fmt, ##args)
#else
#define rnpgbe_dbg(fmt, args...)
#endif
#define rnpgbe_info(fmt, args...) printk(KERN_DEBUG "rnp-info: " fmt, ##args)
#define rnpgbe_warn(fmt, args...) printk(KERN_DEBUG "rnp-warn: " fmt, ##args)
#define rnpgbe_err(fmt, args...) printk(KERN_ERR "rnp-err : " fmt, ##args)

#define e_info(msglvl, format, arg...)                                         \
	netif_info(adapter, msglvl, adapter->netdev, format, ##arg)
#define e_err(msglvl, format, arg...)                                          \
	netif_err(adapter, msglvl, adapter->netdev, format, ##arg)
#define e_warn(msglvl, format, arg...)                                         \
	netif_warn(adapter, msglvl, adapter->netdev, format, ##arg)
#define e_crit(msglvl, format, arg...)                                         \
	netif_crit(adapter, msglvl, adapter->netdev, format, ##arg)

#define e_dev_info(format, arg...) dev_info(&adapter->pdev->dev, format, ##arg)
#define e_dev_warn(format, arg...) dev_warn(&adapter->pdev->dev, format, ##arg)
#define e_dev_err(format, arg...) dev_err(&adapter->pdev->dev, format, ##arg)

#ifdef CONFIG_RNP_TX_DEBUG
static inline void buf_dump_line(const char *msg, int line, void *buf, int len)
{
	int i, offset = 0;
	int msg_len = 1024;
	u8 msg_buf[1024];
	u8 *ptr = (u8 *)buf;

	offset += snprintf(msg_buf + offset, msg_len,
			   "=== %s #%d line:%d buf:%p==\n000: ", msg, len, line,
			   buf);

	for (i = 0; i < len; ++i) {
		if ((i != 0) && (i % 16) == 0 && (offset >= (1024 - 10 * 16))) {
			printk(KERN_DEBUG "%s\n", msg_buf);
			offset = 0;
		}

		if ((i != 0) && (i % 16) == 0) {
			offset += snprintf(msg_buf + offset, msg_len,
					   "\n%03x: ", i);
		}
		offset += snprintf(msg_buf + offset, msg_len, "%02x ", ptr[i]);
	}

	offset += snprintf(msg_buf + offset, msg_len, "\n");
	printk(KERN_DEBUG "%s\n", msg_buf);
}
#else
#define buf_dump_line(msg, line, buf, len)
#endif

static inline __le64 build_ctob(u32 vlan_cmd, u32 mac_ip_len, u32 size)
{
	return cpu_to_le64(((u64)vlan_cmd << 32) | ((u64)mac_ip_len << 16) |
			   ((u64)size));
}

static inline void buf_dump(const char *msg, void *buf, int len)
{
	int i, offset = 0;
	int msg_len = 1024;
	char msg_buf[1024];
	u8 *ptr = (u8 *)buf;

	offset += snprintf(msg_buf + offset, msg_len,
			   "=== %s #%d ==\n000: ", msg, len);

	for (i = 0; i < len; ++i) {
		if ((i != 0) && (i % 16) == 0 && (offset >= (1024 - 10 * 16))) {
			printk(KERN_DEBUG "%s\n", msg_buf);
			offset = 0;
		}

		if ((i != 0) && (i % 16) == 0) {
			offset += snprintf(msg_buf + offset, msg_len,
					   "\n%03x: ", i);
		}
		offset += snprintf(msg_buf + offset, msg_len, "%02x ", ptr[i]);
	}

	offset += snprintf(msg_buf + offset, msg_len, "\n=== done ==\n");
	printk(KERN_DEBUG "%s\n", msg_buf);
}

#ifndef NO_SKB_DUMP
static inline void _rnpgbe_skb_dump(const struct sk_buff *skb, bool full_pkt)
{
	static atomic_t can_dump_full = ATOMIC_INIT(5);
#ifdef DEBUG
	struct skb_shared_info *sh = skb_shinfo(skb);
#endif
	struct net_device *dev = skb->dev;
	//struct sock *sk = skb->sk;
	struct sk_buff *list_skb;
	bool has_mac, has_trans;
	int headroom, tailroom;
	int i, len, seg_len;
	const char *level = KERN_WARNING;

	if (full_pkt)
		full_pkt = atomic_dec_if_positive(&can_dump_full) >= 0;

	if (full_pkt)
		len = skb->len;
	else
		len = min_t(int, skb->len, MAX_HEADER + 128);

	headroom = skb_headroom(skb);
	tailroom = skb_tailroom(skb);

	has_mac = skb_mac_header_was_set(skb);
	has_trans = skb_transport_header_was_set(skb);

	dbg("%sskb len=%u headroom=%u headlen=%u tailroom=%u\n"
	    "mac=(%d,%d) net=(%d,%d) trans=%d\n"
	    "shinfo(txflags=%u nr_frags=%u gso(size=%hu type=%u segs=%hu))\n"
	    "csum(0x%x ip_summed=%u complete_sw=%u valid=%u level=%u)\n"
	    "hash(0x%x sw=%u l4=%u) proto=0x%04x pkttype=%u iif=%d\n",
	    level, skb->len, headroom, skb_headlen(skb), tailroom,
	    has_mac ? skb->mac_header : -1,
	    has_mac ? (skb->network_header - skb->mac_header) : -1,
	    skb->network_header, has_trans ? skb_network_header_len(skb) : -1,
	    has_trans ? skb->transport_header : -1, sh->tx_flags, sh->nr_frags,
	    sh->gso_size, sh->gso_type, sh->gso_segs, skb->csum, skb->ip_summed,
	    skb->csum_complete_sw, skb->csum_valid, skb->csum_level, skb->hash,
	    skb->sw_hash, skb->l4_hash, ntohs(skb->protocol), skb->pkt_type,
	    skb->skb_iif);

	if (dev)
		dbg("%sdev name=%s feat=0x%pNF\n", level, dev->name,
		    &dev->features);

	//if (full_pkt && headroom)
	//		print_hex_dump(level, "skb headroom: ",
	//		DUMP_PREFIX_OFFSET,
	//		16, 1, skb->head, headroom, false);

	seg_len = min_t(int, skb_headlen(skb), len);
	if (seg_len)
		print_hex_dump(level, "skb linear:   ", DUMP_PREFIX_OFFSET, 16,
			       1, skb->data, seg_len, false);
	len -= seg_len;

	//	if (full_pkt && tailroom)
	//		print_hex_dump(level, "skb tailroom: ",
	//		DUMP_PREFIX_OFFSET,
	//		16, 1, skb_tail_pointer(skb), tailroom, false);

	for (i = 0; len && i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		//u32 p_off, p_len, copied;
		u32 p_len;
		struct page *p;
		u8 *vaddr;

		p = skb_frag_address(frag);
		p_len = skb_frag_size(frag);
		seg_len = min_t(int, p_len, len);
		vaddr = kmap_atomic(p);
		print_hex_dump(level, "skb frag:     ", DUMP_PREFIX_OFFSET, 16,
			       1, vaddr, seg_len, false);
		kunmap_atomic(vaddr);
		len -= seg_len;
		if (!len)
			break;
	}

	if (full_pkt && skb_has_frag_list(skb)) {
		dbg("skb fraglist:\n");
		skb_walk_frags(skb, list_skb) _rnpgbe_skb_dump(list_skb, true);
	}
}
#endif

enum RNP_LOG_EVT {
	LOG_MBX_IN,
	LOG_MBX_OUT,
	LOG_MBX_MSG_IN,
	LOG_MBX_MSG_OUT,
	LOG_LINK_EVENT,
	LOG_ADPT_STAT,
	LOG_MBX_ABLI,
	LOG_MBX_LINK_STAT,
	LOG_MBX_IFUP_DOWN,
	LOG_MBX_LOCK,
	LOG_ETHTOOL,
	LOG_PHY,

};

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002
#define MII_DATA_MASK GENMASK(15, 0)

extern unsigned int rnpgbe_loglevel;

#define rnpgbe_logd(evt, fmt, args...)                                         \
	do {                                                                   \
		if (BIT(evt) & rnpgbe_loglevel) {                              \
			printk(KERN_DEBUG fmt, ##args);                        \
		}                                                              \
	} while (0)

#endif /* RNP_COMMON */
