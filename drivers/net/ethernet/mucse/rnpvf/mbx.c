#include "mbx.h"
#include "rnpvf.h"

struct counter {
	union {
		struct {
			unsigned short pf_req;
			unsigned short pf_ack;
		};
		struct {
			unsigned short cpu_req;
			unsigned short cpu_ack;
		};
	};
	unsigned short vf_req;
	unsigned short vf_ack;
} __packed;

static s32 rnpvf_poll_for_msg(struct rnpvf_hw *hw, bool to_cm3);
static s32 rnpvf_poll_for_ack(struct rnpvf_hw *hw, bool to_cm3);

//== VEC ==
#define PF2VF_MBOX_VEC(VF) (0xa5000 + 4 * (VF))
#define CPU2VF_MBOX_VEC(vf) (0xa5200 + 4 * (vf))

//== PF <--> VF mailbox ====
//#define SHARE_MEM_BYTES 64         //64bytes
#define PF_VF_SHM(vf)                                                          \
	((0xa6000) + (64 * (vf))) //for PF1 rtl will remap 6000 to 0xb000
#define PF2VF_COUNTER(vf) (PF_VF_SHM(vf) + 0)
#define VF2PF_COUNTER(vf) (PF_VF_SHM(vf) + 4)
#define PF_VF_SHM_DATA(vf) (PF_VF_SHM(vf) + 8)
#define VF2PF_MBOX_CTRL(vf) ((0xa7000) + (4 * (vf)))

//=== CPU <--> VF ===
#define CPU_VF_SHM(vf) (0xa8000 + (64 * (vf)))
#define CPU2VF_COUNTER(vf) (CPU_VF_SHM(vf) + 0)
#define VF2CPU_COUNTER(vf) (CPU_VF_SHM(vf) + 4)
#define CPU_VF_SHM_DATA(vf) (CPU_VF_SHM(vf) + 8)
#define VF2CPU_MBOX_CTRL(vf) (0xa9000 + 64 * (vf))
#define CPU_VF_MBOX_MASK_LO(vf) (0xa9200 + 64 * (vf))
#define CPU_VF_MBOX_MASK_HI(vf) (0xa9300 + 64 * (vf))

#define MBOX_CTRL_REQ (1 << 0) // WO
#define MBOX_CTRL_VF_HOLD_SHM (1 << 2) // VF:WR, PF:RO
//#define MBOX_CTRL_PF_HOLD_SHM       (1<<3) // VF:RO, PF:WR
//#define MBOX_CTRL_VF_CPU_HOLD_SHM   (1<<3) // for pf <--> cpu

#define MBOX_IRQ_EN 0
#define MBOX_IRQ_DISABLE 1

/**
 *  rnpvf_read_posted_mbx - Wait for message notification and receive message
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *
 *  returns 0 if it successfully received a message notification and
 *  copied it into the receive buffer.
 **/
static s32
rnpvf_read_posted_mbx(struct rnpvf_hw *hw, u32 *msg, u16 size, bool to_cm3)
{
	struct rnp_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -RNP_ERR_MBX;

	if (!mbx->ops.read)
		goto out;

	ret_val = rnpvf_poll_for_msg(hw, to_cm3);

	/* if ack received read message, otherwise we timed out */
	if (!ret_val)
		ret_val = mbx->ops.read(hw, msg, size, to_cm3);
out:
	return ret_val;
}

/**
 *  rnpvf_write_posted_mbx - Write a message to the mailbox, wait for ack
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *
 *  returns 0 if it successfully copied message into the buffer and
 *  received an ack to that message within delay * timeout period
 **/
static s32
rnpvf_write_posted_mbx(struct rnpvf_hw *hw, u32 *msg, u16 size, bool to_cm3)
{
	struct rnp_mbx_info *mbx = &hw->mbx;
	s32 ret_val = -RNP_ERR_MBX;

	/* exit if either we can't write or there isn't a defined timeout */
	if (!mbx->ops.write || !mbx->timeout)
		goto out;

	/* send msg */
	ret_val = mbx->ops.write(hw, msg, size, to_cm3);

	/* if msg sent wait until we receive an ack */
	if (!ret_val)
		ret_val = rnpvf_poll_for_ack(hw, to_cm3);
out:
	return ret_val;
}

static inline u16 rnpvf_mbx_get_req(struct rnpvf_hw *hw, int reg)
{
	mb();
	return mbx_rd32(hw, reg) & 0xffff;
}

static inline u16 rnpvf_mbx_get_ack(struct rnpvf_hw *hw, int reg)
{
	mb();
	return (mbx_rd32(hw, reg) >> 16) & 0xffff;
}

static inline void rnpvf_mbx_inc_vfreq(struct rnpvf_hw *hw, bool to_cm3)
{
	u16 req;
	u8 vfnum = VFNUM(hw->vfnum);
	int reg = to_cm3 ? VF2CPU_COUNTER(vfnum) : VF2PF_COUNTER(vfnum);
	u32 v = mbx_rd32(hw, reg);

	req = (v & 0xffff);
	req++;
	v &= ~(0x0000ffff);
	v |= req;
	mb();
	mbx_wr32(hw, reg, v);

	/* update stats */
	hw->mbx.stats.msgs_tx++;
}

static inline void rnpvf_mbx_inc_vfack(struct rnpvf_hw *hw, bool to_cm3)
{
	u16 ack;
	u8 vfnum = VFNUM(hw->vfnum);
	int reg = to_cm3 ? VF2CPU_COUNTER(vfnum) : VF2PF_COUNTER(vfnum);
	u32 v = mbx_rd32(hw, reg);

	ack = (v >> 16) & 0xffff;
	ack++;
	v &= ~(0xffff0000);
	v |= (ack << 16);
	mb();
	mbx_wr32(hw, reg, v);

	/* update stats */
	hw->mbx.stats.msgs_rx++;
}

/**
 *  rnpvf_check_for_msg_vf - checks to see if the PF has sent mail
 *  @hw: pointer to the HW structure
 *
 *  returns 0 if the PF has set the Status bit or else ERR_MBX
 **/
static s32 rnpvf_check_for_msg_vf(struct rnpvf_hw *hw, bool to_cm3)
{
	s32 ret_val = RNP_ERR_MBX;
	u8 vfnum = VFNUM(hw->vfnum);
	struct rnpvf_adapter *adapter = hw->back;

	u32 DATA_REG = (to_cm3) ? CPU_VF_SHM_DATA(vfnum) : PF_VF_SHM_DATA(vfnum);

	// dev_info(&hw->pdev->dev, "start\n");
	if (to_cm3 == true) {
		if (rnpvf_mbx_get_req(hw, CPU2VF_COUNTER(vfnum)) != hw->mbx.cpu_req) {
			ret_val = 0;
			hw->mbx.stats.reqs++;
		}
	} else {
		// dev_info(&hw->pdev->dev, "%s mbx.pf_req %d \n", adapter->name,
		// hw->mbx.pf_req);
		if (rnpvf_mbx_get_req(hw, PF2VF_COUNTER(vfnum)) != hw->mbx.pf_req) {
			ret_val = 0;
			hw->mbx.stats.reqs++;
			// dev_info(&hw->pdev->dev, "%s new req %d \n", adapter->name,
			// hw->mbx.stats.reqs);
		}
	}
	// dev_info(&hw->pdev->dev, "return\n");

	return ret_val;
}

/**
 *  rnpvf_poll_for_msg - Wait for message notification
 *  @hw: pointer to the HW structure
 *
 *  returns 0 if it successfully received a message notification
 **/
static s32 rnpvf_poll_for_msg(struct rnpvf_hw *hw, bool to_cm3)
{
	struct rnp_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	while (countdown && mbx->ops.check_for_msg(hw, to_cm3)) {
		countdown--;
		udelay(mbx->udelay);
	}

	/* if we failed, all future posted messages fail until reset */
	/*
	if (!countdown) {
		mbx->timeout = 0;
		dbg("%s timeout\n", __func__);
	}*/

	return countdown ? 0 : RNP_ERR_MBX;
}

/**
 *  rnpvf_poll_for_ack - Wait for message acknowledgement
 *  @hw: pointer to the HW structure
 *
 *  returns 0 if it successfully received a message acknowledgement
 **/
static s32 rnpvf_poll_for_ack(struct rnpvf_hw *hw, bool to_cm3)
{
	struct rnp_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	while (countdown && mbx->ops.check_for_ack(hw, to_cm3)) {
		countdown--;
		udelay(mbx->udelay);
	}

	/* if we failed, all future posted messages fail until reset */
	if (!countdown) {
		mbx->timeout = 0;
		dbg("%s timeout\n", __func__);
	}

	return countdown ? 0 : RNP_ERR_MBX;
}

/**
 *  rnpvf_check_for_rst_msg_vf - checks to see if the PF has ACK'd
 *  @hw: pointer to the HW structure
 *
 *  returns 0 if the PF has set the ACK bit or else ERR_MBX
 **/
static s32 rnpvf_check_for_rst_msg_vf(struct rnpvf_hw *hw, bool to_cm3)
{
	struct rnpvf_adapter *adapter = hw->back;
	s32 ret_val = RNP_ERR_MBX;
	u8 vfnum = VFNUM(hw->vfnum);
	u32 DATA_REG = (to_cm3) ? CPU_VF_SHM_DATA(vfnum) : PF_VF_SHM_DATA(vfnum);
	u32 data;
	int ret = 1;

	ret_val = rnpvf_check_for_msg_vf(hw, to_cm3);
	if (!ret_val) {
		// maybe use mbx read ?
		data = mbx_rd32(hw, DATA_REG);

		data &= ~RNP_PF_VFNUM_MASK;
		dbg("mbx %x\n", data);
		/* add other mailbox setup */
		if (((data) & (~RNP_VT_MSGTYPE_CTS)) == RNP_PF_CONTROL_PRING_MSG) {
		} else if ((data) == RNP_PF_SET_FCS) {
			// to-do
			data = mbx_rd32(hw, DATA_REG + 4);
			if (data) {
				adapter->priv_flags |= RNPVF_PRIV_FLAG_FCS_ON;
			} else {
				adapter->priv_flags &= (~RNPVF_PRIV_FLAG_FCS_ON);
			}

		} else if ((data) == RNP_PF_SET_PAUSE) {
			hw->fc.current_mode = mbx_rd32(hw, DATA_REG + 4);
		} else if ((data) == RNP_PF_SET_FT_PADDING) {
			data = mbx_rd32(hw, DATA_REG + 4);
			if (data) {
				adapter->priv_flags |= RNPVF_PRIV_FLAG_FT_PADDING;
			} else {
				adapter->priv_flags &= (~RNPVF_PRIV_FLAG_FT_PADDING);
			}
		} else if ((data) == RNP_PF_SET_VLAN_FILTER) {
			data = mbx_rd32(hw, DATA_REG + 4);
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
			if (data) {
				adapter->netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
			} else {
				adapter->netdev->features &= ~NETIF_F_HW_VLAN_CTAG_FILTER;
			}
#endif
		} else if ((data) == RNP_PF_SET_VLAN) {
			data = mbx_rd32(hw, DATA_REG + 4);
			// pf set vlan for this vf
			adapter->flags |= RNPVF_FLAG_PF_UPDATE_VLAN;
			if (data) {
				adapter->flags |= RNPVF_FLAG_PF_SET_VLAN;
				adapter->vf_vlan = data;
				// should close vlan offload
#ifdef NETIF_F_HW_VLAN_CTAG_RX
				adapter->netdev->features &= ~NETIF_F_HW_VLAN_CTAG_RX;
#endif
#ifdef NETIF_F_HW_VLAN_CTAG_TX
				adapter->netdev->features &= ~NETIF_F_HW_VLAN_CTAG_TX;
#endif
			} else {
				adapter->flags &= (~RNPVF_FLAG_PF_SET_VLAN);
				adapter->vf_vlan = 0;
			}
		} else if ((data) == RNP_PF_SET_LINK) {
			data = mbx_rd32(hw, DATA_REG + 4);
			if (data & RNP_PF_LINK_UP) {
				hw->link = true;
				hw->speed = data & 0xffff;
			} else {
				hw->link = false;
				hw->speed = 0;
			}
			//ret = 0;

		} else if ((data) == RNP_PF_SET_MTU) {
			data = mbx_rd32(hw, DATA_REG + 4);
			// update mtu
			hw->mtu = data;
			adapter->flags |= RNPVF_FLAG_PF_UPDATE_MTU;
			//ret = 0;
		} else if ((data) == RNP_PF_SET_RESET) {
			// pf call reset vf
			//adapter->flags |= RNPVF_FLAG_PF_UPDATE_MAC;
			adapter->flags |= RNPVF_FLAG_PF_RESET;
			//schedule_work(&adapter->reset_task);
			//ret = 0;
		} else {
			return RNP_ERR_MBX;
		}
	}

	return ret;
}

/**
 *  rnpvf_check_for_ack_vf - checks to see if the PF has ACK'd
 *  @hw: pointer to the HW structure
 *
 *  returns 0 if the PF has set the ACK bit or else ERR_MBX
 **/
static s32 rnpvf_check_for_ack_vf(struct rnpvf_hw *hw, bool to_cm3)
{
	s32 ret_val = RNP_ERR_MBX;
	u8 vfnum = VFNUM(hw->vfnum);

	if (to_cm3 == true) {
		if (rnpvf_mbx_get_ack(hw, CPU2VF_COUNTER(vfnum)) != hw->mbx.cpu_ack) {
			ret_val = 0;
			hw->mbx.stats.acks++;
		}
	} else {
		if (rnpvf_mbx_get_ack(hw, PF2VF_COUNTER(vfnum)) != hw->mbx.pf_ack) {
			ret_val = 0;
			hw->mbx.stats.acks++;
		}
	}

	return ret_val;
}

/**
 *  rnpvf_obtain_mbx_lock_vf - obtain mailbox lock
 *  @hw: pointer to the HW structure
 *
 *  return 0 if we obtained the mailbox lock
 **/
static s32 rnpvf_obtain_mbx_lock_vf(struct rnpvf_hw *hw, bool to_cm3)
{
	int try_cnt = 2 * 1000; // 1s
	s32 ret_val = RNP_ERR_MBX;
	u8 vfnum = VFNUM(hw->vfnum);
	struct rnpvf_adapter *adapter = hw->back;
	u32 CTRL_REG =
		(to_cm3) ? VF2CPU_MBOX_CTRL(vfnum) : VF2PF_MBOX_CTRL(vfnum);

	while (try_cnt-- > 0) {
		/* Take ownership of the buffer */
		mbx_wr32(hw, CTRL_REG, MBOX_CTRL_VF_HOLD_SHM);
		mb();
		/* reserve mailbox for vf use */
		if (mbx_rd32(hw, CTRL_REG) & MBOX_CTRL_VF_HOLD_SHM)
			return 0;
		udelay(500);
	}

	printk("[rnpvf] %s: faild to get mbx-lock \n", adapter->name);
	return RNP_ERR_MBX;
}

/**
 *  rnpvf_write_mbx_vf - Write a message to the mailbox
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *
 *  returns 0 if it successfully copied message into the buffer
 **/
static s32 rnpvf_write_mbx_vf(struct rnpvf_hw *hw, u32 *msg, u16 size,
			      bool to_cm3)
{
	s32 ret_val;
	u32 stat, i;
	u8 vfnum = VFNUM(hw->vfnum);
	u32 DATA_REG =
		(to_cm3) ? CPU_VF_SHM_DATA(vfnum) : PF_VF_SHM_DATA(vfnum);
	u32 CTRL_REG =
		(to_cm3) ? VF2CPU_MBOX_CTRL(vfnum) : VF2PF_MBOX_CTRL(vfnum);

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = rnpvf_obtain_mbx_lock_vf(hw, to_cm3);
	if (ret_val) {
		printk("%s: get mbx wlock failed. ret:%d. req:0x%08x-0x%08x\n",
			   __func__,
			   ret_val,
			   msg[0],
			   msg[1]);
		goto out_no_write;
	}

	/* add mailbox_id [27:21] */
#define VF_NUM_OFFSET (21)
	if (!to_cm3)
		msg[0] |= ((hw->vfnum & 0x3f) << VF_NUM_OFFSET);

	//dev_info(&hw->pdev->dev,"vf->pf %x-%x-%x\n", msg[0], msg[1], msg[2]);
	/* copy the caller specified message to the mailbox memory buffer */
	for (i = 0; i < size; i++) {
		mbx_wr32(hw, DATA_REG + i * 4, msg[i]);
	}

	/* update acks. used by rnpvf_check_for_ack_vf  */
	if (to_cm3 == true)
		hw->mbx.cpu_ack = rnpvf_mbx_get_ack(hw, CPU2VF_COUNTER(vfnum));
	else
		hw->mbx.pf_ack = rnpvf_mbx_get_ack(hw, PF2VF_COUNTER(vfnum));
	rnpvf_mbx_inc_vfreq(hw, to_cm3);

	/* Drop VFU and interrupt the PF/CM3 to
	 * tell it a message has been sent
	 */
	mbx_wr32(hw, CTRL_REG, MBOX_CTRL_REQ);

out_no_write:
	return ret_val;
}

/**
 *  rnpvf_read_mbx_vf - Reads a message from the inbox intended for vf
 *  @hw: pointer to the HW structure
 *  @msg: The message buffer
 *  @size: Length of buffer
 *
 *  returns 0 if it successfully read message from buffer
 **/
static s32 rnpvf_read_mbx_vf(struct rnpvf_hw *hw, u32 *msg, u16 size,
			     bool to_cm3)
{
	s32 ret_val = 0;
	u32 stat, i;
	u8 vfnum = VFNUM(hw->vfnum);
	u32 BUF_REG = (to_cm3) ? CPU_VF_SHM_DATA(vfnum) : PF_VF_SHM_DATA(vfnum);
	u32 CTRL_REG =
		(to_cm3) ? VF2CPU_MBOX_CTRL(vfnum) : VF2PF_MBOX_CTRL(vfnum);

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = rnpvf_obtain_mbx_lock_vf(hw, to_cm3);
	if (ret_val)
		goto out_no_read;

	mb();
	/* copy the message from the mailbox memory buffer */
	for (i = 0; i < size; i++)
		msg[i] = mbx_rd32(hw, BUF_REG + 4 * i);

	//dev_info(&hw->pdev->dev,"pf->vf %x_%x_%x\n", msg[0], msg[1], msg[2]);
		/* clear vf_num */
#define RNP_VF_NUM_MASK (0x7f << 21)
	msg[0] &= (~RNP_VF_NUM_MASK);

	/* update req. used by rnpvf_check_for_msg_vf  */
	if (to_cm3 == true)
		hw->mbx.cpu_req = rnpvf_mbx_get_req(hw, CPU2VF_COUNTER(vfnum));
	else
		hw->mbx.pf_req = rnpvf_mbx_get_req(hw, PF2VF_COUNTER(vfnum));
	/* Acknowledge receipt and release mailbox, then we're done */
	rnpvf_mbx_inc_vfack(hw, to_cm3);

	/* free ownership of the buffer */
	mbx_wr32(hw, CTRL_REG, 0);

out_no_read:
	return ret_val;
}

static void rnpvf_reset_mbx(struct rnpvf_hw *hw)
{
	u32 v;
	u8 vfnum = VFNUM(hw->vfnum);

	// release vfu
	mbx_wr32(hw, VF2CPU_MBOX_CTRL(vfnum), 0);
	mbx_wr32(hw, VF2PF_MBOX_CTRL(vfnum), 0);

	// fetch mbx counter values
	v = mbx_rd32(hw, PF2VF_COUNTER(vfnum));
	hw->mbx.pf_req = v & 0xffff;
	hw->mbx.pf_ack = (v >> 16) & 0xffff;
	
	//dev_info(&hw->pdev->dev, "mbx.pf_req %d mbx.pf_ack %d \n", hw->mbx.pf_req, hw->mbx.pf_ack);

	v = mbx_rd32(hw, CPU2VF_COUNTER(vfnum));
	hw->mbx.cpu_req = v & 0xffff;
	hw->mbx.cpu_ack = (v >> 16) & 0xffff;

	return;
}

static s32 rnpvf_mbx_configure_vf(struct rnpvf_hw *hw, int nr_vec, bool enable)
{
	int mbx_vec_reg, vfnum = VFNUM(hw->vfnum);

	// PF --> VF
	mbx_vec_reg = PF2VF_MBOX_VEC(vfnum);
	mbx_wr32(hw, mbx_vec_reg, nr_vec); //#2 irq for PF mailbox

	// CM3 --> VF
	// mbx_vec_reg = CPU2VF_MBOX_VEC( vfnum ) ;
	// mbx_wr32(hw, mbx_vec_reg, 3);//#3 irq for CPU mailbox

	return 0;
}

/**
 *  rnpvf_init_mbx_params_vf - set initial values for vf mailbox
 *  @hw: pointer to the HW structure
 *
 *  Initializes the hw->mbx struct to correct values for vf mailbox
 */
static s32 rnpvf_init_mbx_params_vf(struct rnpvf_hw *hw)
{
	struct rnp_mbx_info *mbx = &hw->mbx;

	/* start mailbox as timed out and let the reset_hw call set the timeout
	 * value to begin communications
	 */
	mbx->timeout = 0;
	mbx->udelay = RNP_VF_MBX_INIT_DELAY;

	mbx->stats.msgs_tx = 0;
	mbx->stats.msgs_rx = 0;
	mbx->stats.reqs = 0;
	mbx->stats.acks = 0;
	mbx->stats.rsts = 0;

	mbx->size = RNP_VFMAILBOX_SIZE;

	// mutex_init(&mbx->lock);
	rnpvf_reset_mbx(hw);
	return 0;
}

const struct rnp_mbx_operations rnpvf_mbx_ops = {
	.init_params = rnpvf_init_mbx_params_vf,
	.read = rnpvf_read_mbx_vf,
	.write = rnpvf_write_mbx_vf,
	.read_posted = rnpvf_read_posted_mbx,
	.write_posted = rnpvf_write_posted_mbx,
	.check_for_msg = rnpvf_check_for_msg_vf,
	.check_for_ack = rnpvf_check_for_ack_vf,
	.check_for_rst = rnpvf_check_for_rst_msg_vf,
	.configure = rnpvf_mbx_configure_vf,
};

