#ifndef _RNPVF_REGS_H_
#define _RNPVF_REGS_H_

enum NIC_MODE {
	MODE_NIC_MODE_2PORT_40G = 0,
	MODE_NIC_MODE_2PORT_10G = 1,
	MODE_NIC_MODE_4PORT_10G = 2,
	MODE_NIC_MODE_8PORT_10G = 3,
};

//RNP-Ring Registers
#define RNP_DMA_RING_BASE 0x8000
#define RNP_DMA_RX_DESC_TIMEOUT_TH 0x8000
#define RNP_DMA_TX_DESC_FETCH_CTL 0x8004
#define RNP_DMA_TX_FLOW_CTRL_TM 0x8008
//DMA-ENABLE-IRQ
#define RNP_DMA_RX_START(queue_idx) (0x8010 + 0x100 * (queue_idx))
#define RNP_DMA_RX_READY(queue_idx) (0x8014 + 0x100 * (queue_idx))
#define RNP_DMA_TX_START(queue_idx) (0x8018 + 0x100 * (queue_idx))
#define RNP_DMA_TX_READY(queue_idx) (0x801c + 0x100 * (queue_idx))
#define RNP_DMA_INT_STAT(queue_idx) (0x8020 + 0x100 * (queue_idx))
#define RNP_DMA_INT_MASK(queue_idx) (0x8024 + 0x100 * (queue_idx))
#define TX_INT_MASK (1 << 1)
#define RX_INT_MASK (1 << 0)
#define RNP_DMA_INT_CLR(queue_idx) (0x8028 + 0x100 * (queue_idx))
//RX-Queue Registers
#define RNP_DMA_REG_RX_DESC_BUF_BASE_ADDR_HI(queue_idx)                        \
	(0x8030 + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_DESC_BUF_BASE_ADDR_LO(queue_idx)                        \
	(0x8034 + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_DESC_BUF_LEN(queue_idx) (0x8038 + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_DESC_BUF_HEAD(queue_idx) (0x803c + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_DESC_BUF_TAIL(queue_idx) (0x8040 + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_DESC_FETCH_CTRL(queue_idx) (0x8044 + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_INT_DELAY_TIMER(queue_idx) (0x8048 + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_INT_DELAY_PKTCNT(queue_idx)                             \
	(0x804c + 0x100 * (queue_idx))
#define RNP_DMA_REG_RX_ARB_DEF_LVL(queue_idx) (0x8050 + 0x100 * (queue_idx))
#define PCI_DMA_REG_RX_DESC_TIMEOUT_TH(queue_idx) (0x8054 + 0x100 * (queue_idx))
// TX-Queue Registers
#define RNP_DMA_REG_TX_DESC_BUF_BASE_ADDR_HI(queue_idx)                        \
	(0x8060 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_DESC_BUF_BASE_ADDR_LO(queue_idx)                        \
	(0x8064 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_DESC_BUF_LEN(queue_idx) (0x8068 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_DESC_BUF_HEAD(queue_idx) (0x806c + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_DESC_BUF_TAIL(queue_idx) (0x8070 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_DESC_FETCH_CTRL(queue_idx) (0x8074 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_INT_DELAY_TIMER(queue_idx) (0x8078 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_INT_DELAY_PKTCNT(queue_idx)                             \
	(0x807c + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_ARB_DEF_LVL(queue_idx) (0x8080 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_FLOW_CTRL_TH(queue_idx) (0x8084 + 0x100 * (queue_idx))
#define RNP_DMA_REG_TX_FLOW_CTRL_TM(queue_idx) (0x8088 + 0x100 * (queue_idx))

// VEB Registers
#define VEB_TBL_CNTS 64
#define RNP_DMA_PORT_VBE_MAC_LO_TBL(port, vf)                                  \
	(0x80A0 + 4 * (port) + 0x100 * (vf))
#define RNP_DMA_PORT_VBE_MAC_HI_TBL(port, vf)                                  \
	(0x80B0 + 4 * (port) + 0x100 * (vf))
#define RNP_DMA_PORT_VEB_VID_TBL(port, vf) (0x80C0 + 4 * (port) + 0x100 * (vf))
#define RNP_DMA_PORT_VEB_VF_RING_TBL(port, vf)                                 \
	(0x80D0 + 4 * (port) +                                                 \
	 0x100 * (vf)) //[0:7]:Ring_id,[8:15]:vf_num,vf_num[7]=1=vf valid

#define RNP_DMA_STATS_DMA_TO_MAC (0x1a0)
#define RNP_DMA_STATS_DMA_TO_SWITCH (0x1a4)
#define RNP_DMA_STATS_MAC_TO_MAC (0x1b0)
#define RNP_DMA_STATS_SWITCH_TO_SWITCH (0x1a4)
#define RNP_DMA_STATS_MAC_TO_DMA (0x1a8)
#define RNP_DMA_STATS_SWITCH_TO_DMA (0x1ac)

//=====  PF-VF Functions ====
#define VF_NUM_REG 0xa3000
#define VF_NUM_REG_N10 0x75f000
// 8bit: 7:vf_actiove 6:fun0/fun1 [5:0]:vf_num
#define VF_NUM(vfnum, fun) ((1 << 7) | (((fun) & 0x1) << 6) | ((vfnum)&0x3f))
#define PF_NUM(fun) (((fun)&0x1) << 6)

//==== Ring-MSIX Registers (MSI-X_module_design.docs) ===
#define RING_VECTOR(n) (0x4000 + 0x04 * (n))

static inline unsigned int p_rnpvf_rd_reg(void *reg)
{
	unsigned int v = ioread32((void *)(reg));

	printk(" rd-reg: %p ==> 0x%08x\n", reg, v);
	return v;
}
#define p_rnpvf_wr_reg(reg, val)                                               \
	do {                                                                   \
		printk(" wr-reg: %p <== 0x%08x \t#%-4d %s\n", (reg), (val),    \
		       __LINE__, __FILE__);                                    \
		iowrite32((val), (void *)(reg));                               \
	} while (0)

#ifdef IO_PRINT
#define rnpvf_rd_reg(reg) p_rnpvf_rd_reg(reg)
#define rnpvf_wr_reg(reg, val) p_rnpvf_wr_reg(reg, val)
#else
#define rnpvf_rd_reg(reg) readl((void *)(reg))
#define rnpvf_wr_reg(reg, val) writel((val), (void *)(reg))
#endif

#ifdef CONFIG_RNP_MBX_DEBUG
#define mbx_rd32(hw, reg) p_rnpvf_rd_reg((hw)->hw_addr + (reg))
#define mbx_wr32(hw, reg, val) p_rnpvf_wr_reg((hw)->hw_addr + (reg), (val))
#else
#define mbx_rd32(hw, reg) rnpvf_rd_reg((hw)->hw_addr + (reg))
#define mbx_wr32(hw, reg, val) rnpvf_wr_reg((hw)->hw_addr + (reg), (val))
#endif

#define rd32(hw, off) rnpvf_rd_reg((hw)->hw_addr + (off))
#define wr32(hw, off, val) rnpvf_wr_reg((hw)->hw_addr + (off), (val))

#define pwr32(hw, reg, val)                                                    \
	do {                                                                   \
		printk(" wr-reg: %p <== 0x%08x \t#%-4d %s\n",                  \
		       (hw)->hw_addr + (reg), (val), __LINE__, __FILE__);      \
		iowrite32((val), (hw)->hw_addr + (reg));                       \
	} while (0)

//==== log helper ===
#ifdef DEBUG
#define hw_dbg(hw, fmt, args...) printk("hw-dbg : " fmt, ##args)
#else
#define hw_dbg(hw, fmt, args...)
#endif

#endif /* _RNPVF_REGS_H_ */
