#include <linux/wait.h>
#include <linux/sem.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>

#include "rnp.h"
#include "rnp_mbx.h"
#include "rnp_mbx_fw.h"

#define RNP_FW_MAILBOX_SIZE RNP_VFMAILBOX_SIZE

#define dbg_here printk("%s %d\n", __func__, __LINE__)

struct mbx_req_cookie *mbx_cookie_zalloc(int priv_len)
{
	struct mbx_req_cookie *cookie = kzalloc(sizeof(*cookie) + priv_len, GFP_KERNEL);

	if (cookie) {
		cookie->timeout_jiffes = 30 * HZ;
		cookie->magic		   = COOKIE_MAGIC;
		cookie->priv_len	   = priv_len;

		init_waitqueue_head(&cookie->wait);
	}

	return cookie;
}

int rnp_mbx_write_posted_locked(struct rnp_hw *hw, struct mbx_fw_cmd_req *req)
{
	int err	  = 0;
	int retry = 3;

	if (mutex_lock_interruptible(&hw->mbx.lock)) {
		rnp_err("[%s] get mbx lock faild opcode:0x%x\n", __func__, req->opcode);
		return -EAGAIN;
	}

	rnp_logd(LOG_MBX_LOCK, "%s %d lock:%p hw:%p opcode:0x%x\n", __func__, hw->pfvfnum,
			 &hw->mbx.lock, hw, req->opcode);

try_again:
	retry--;
	if (retry < 0) {
		mutex_unlock(&hw->mbx.lock);
		rnp_err("%s: write_posted faild! err:0x%x opcode:0x%x\n", __func__, err, req->opcode);
		return -EIO;
	}

	err = hw->mbx.ops.write_posted(hw, (u32 *)req, (req->datalen + MBX_REQ_HDR_LEN) / 4, MBX_FW);
	if (err) {
		goto try_again;
	}
	mutex_unlock(&hw->mbx.lock);

	return err;
}
/*
	force firmware report link event to driver
*/
void rnp_link_stat_mark_reset(struct rnp_hw *hw)
{
	struct rnp_adapter *adapter = hw->back;

	spin_lock(&adapter->link_stat_lock);
	wr32(hw, RNP_DMA_DUMY, 0xa5a40000);
	spin_unlock(&adapter->link_stat_lock);
}

void rnp_link_stat_mark_disable(struct rnp_hw *hw)
{
	struct rnp_adapter *adapter = hw->back;

	spin_lock(&adapter->link_stat_lock);
	wr32(hw, RNP_DMA_DUMY, 0);
	spin_unlock(&adapter->link_stat_lock);
}

int rnp_mbx_fw_post_req(struct rnp_hw *hw, struct mbx_fw_cmd_req *req,
						struct mbx_req_cookie *cookie)
{
	int err = 0;

	cookie->errcode = 0;
	cookie->done	= 0;
	init_waitqueue_head(&cookie->wait);

	if (mutex_lock_interruptible(&hw->mbx.lock)) {
		rnp_err("[%s] wait mbx lock timeout opcode:0x%x\n", __func__, req->opcode);
		return -EAGAIN;
	}

	rnp_logd(LOG_MBX_LOCK, "%s %d lock:%p hw:%p opcode:0x%x\n", __func__, hw->pfvfnum,
			 &hw->mbx.lock, hw, req->opcode);

	if ((err = rnp_write_mbx(hw, (u32 *)req, (req->datalen + MBX_REQ_HDR_LEN) / 4, MBX_FW))) {
		rnp_err("rnp_write_mbx faild! err:%d opcode:0x%x\n", err, req->opcode);
		mutex_unlock(&hw->mbx.lock);
		return err;
	}

	if (cookie->timeout_jiffes != 0) {
		err = wait_event_interruptible_timeout(cookie->wait, cookie->done == 1,
											   cookie->timeout_jiffes);
		if (err == 0) {
			rnp_err("%s faild! timeout err:%d opcode:%x\n", __func__, err, req->opcode);
			err = -ETIME;
		} else {
			err = 0;
		}
	} else {
		wait_event_interruptible(cookie->wait, cookie->done == 1);
	}

	mutex_unlock(&hw->mbx.lock);

	if (cookie->errcode) {
		err = cookie->errcode;
	}

	return err;
}

int rnp_fw_send_cmd_wait(struct rnp_hw *hw, struct mbx_fw_cmd_req *req,
						 struct mbx_fw_cmd_reply *reply)
{
	int err;
	int retry_cnt = 3;

	if (!hw || !req || !reply || !hw->mbx.ops.read_posted) {
		printk("error: hw:%p req:%p reply:%p\n", hw, req, reply);
		return -EINVAL;
	}

	if (mutex_lock_interruptible(&hw->mbx.lock)) {
		rnp_err("[%s] get mbx lock faild opcode:0x%x\n", __func__, req->opcode);
		return -EAGAIN;
	}

	rnp_logd(LOG_MBX_LOCK, "%s %d lock:%p hw:%p opcode:0x%x\n", __func__, hw->pfvfnum,
			 &hw->mbx.lock, hw, req->opcode);
	err = hw->mbx.ops.write_posted(hw, (u32 *)req, (req->datalen + MBX_REQ_HDR_LEN) / 4, MBX_FW);
	if (err) {
		rnp_err("%s: write_posted faild! err:0x%x opcode:0x%x\n", __func__, err, req->opcode);
		mutex_unlock(&hw->mbx.lock);
		return err;
	}

// ignore link-status event
retry:
	retry_cnt--;
	if (retry_cnt < 0) {
		return -EIO;
	}
	err = hw->mbx.ops.read_posted(hw, (u32 *)reply, sizeof(*reply) / 4, MBX_FW);
	if (err) {
		rnp_err("%s: read_posted faild! err:0x%x opcode:0x%x\n", __func__, err, req->opcode);
		mutex_unlock(&hw->mbx.lock);
		return err;
	}
	if (reply->opcode != req->opcode) {
		goto retry;
	}
	mutex_unlock(&hw->mbx.lock);

#if 0
	if (req->reply_lo) {
		memcpy(reply, hw->mbx.reply_dma, sizeof(*reply));
		memset(hw->mbx.reply_dma, 0, 16);
	}
#endif

	if (reply->error_code) {
		rnp_err("%s: reply err:0x%x req:0x%x\n", __func__, reply->error_code, req->opcode);
		return -reply->error_code;
	}
	return 0;
}

int rnp_mbx_get_link(struct rnp_hw *hw)
{
	struct rnp_adapter *adpt = hw->back;
	int					v	 = rd32(hw, RNP_TOP_NIC_DUMMY);

	if ((v & 0xff000000) == 0xa5000000) {
		hw->link = (v & BIT(hw->nr_lane)) ? 1 : 0;
		adpt->flags |= RNP_FLAG_NEED_LINK_UPDATE;

		return 0;
	}
	return -1;
}

int rnp_mbx_get_lane_stat(struct rnp_hw *hw)
{
	int						err = 0;
	struct mbx_fw_cmd_req	req;
	struct rnp_adapter	   *adpt = hw->back;
	struct lane_stat_data  *st;
	struct mbx_req_cookie  *cookie = NULL;
	struct mbx_fw_cmd_reply reply;

	memset(&req, 0, sizeof(req));

	if (hw->mbx.other_irq_enabled) {
		cookie = mbx_cookie_zalloc(sizeof(struct lane_stat_data));

		if (!cookie) {
			rnp_err("%s: no memory\n", __func__);
			return -ENOMEM;
		}

		st = (struct lane_stat_data *)cookie->priv;

		build_get_lane_status_req(&req, hw->nr_lane, cookie);

		err = rnp_mbx_fw_post_req(hw, &req, cookie);
		// printk("%s: phy_type:%d\n", __func__, st->phy_type);
		// buf_dump("stat", st, sizeof(*st));
		if (err) {
			rnp_err("%s: error:%d\n", __func__, err);
			goto quit;
		}
	} else {
		memset(&reply, 0, sizeof(reply));

		build_get_lane_status_req(&req, hw->nr_lane, &req);
		err = rnp_fw_send_cmd_wait(hw, &req, &reply);
		if (err) {
			rnp_err("%s: 1 error:%d\n", __func__, err);
			goto quit;
		}
		st = (struct lane_stat_data *)&(reply.data);
	}

	hw->phy_type = st->phy_type;
	hw->speed = adpt->speed = st->speed;
	if (st->is_sgmii) {
		adpt->phy_addr = st->phy_addr;
	} else {
		adpt->sfp.fault	  = st->sfp.fault;
		adpt->sfp.los	  = st->sfp.los;
		adpt->sfp.mod_abs = st->sfp.mod_abs;
		adpt->sfp.tx_dis  = st->sfp.tx_dis;
	}
	adpt->si.main	   = st->si_main;
	adpt->si.pre	   = st->si_pre;
	adpt->si.post	   = st->si_post;
	adpt->si.tx_boost  = st->si_tx_boost;
	adpt->an		   = st->an;
	adpt->link_traing  = st->link_traing;
	adpt->fec		   = st->fec;
	hw->is_sgmii	   = st->is_sgmii;
	hw->pci_gen		   = st->pci_gen;
	hw->pci_lanes	   = st->pci_lanes;
	adpt->speed		   = st->speed;
	adpt->hw.link	   = st->linkup;
	hw->is_backplane   = st->is_backplane;
	hw->supported_link = st->supported_link;
	if (hw->fw_version >= 0x00050000) {
		hw->sfp_connector = st->sfp_connector;
	} else {
		hw->sfp_connector = 0xff;
	}
	if (hw->fw_version <= 0x00050000) {
		hw->supported_link |= RNP_LINK_SPEED_10GB_FULL | RNP_LINK_SPEED_1GB_FULL;
	}

	rnp_logd(LOG_MBX_LINK_STAT,
			 "%s:pma_type:0x%x phy_type:0x%x,linkup:%d duplex:%d auton:%d "
			 "fec:%d an:%d lt:%d is_sgmii:%d supported_link:0x%x, backplane:%d "
			 "speed:%d sfp_connector:0x%x\n",
			 adpt->name, st->pma_type, st->phy_type, st->linkup, st->duplex, st->autoneg, st->fec,
			 st->an, st->link_traing, st->is_sgmii, hw->supported_link, hw->is_backplane, st->speed,
			 st->sfp_connector);

quit:
	if (cookie)
		kfree(cookie);
	return err;
}

int rnp_mbx_get_link_stat(struct rnp_hw *hw)
{
	int						err;
	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;
	int						i;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	build_get_link_status_req(&req, hw->nr_lane, &req);
	return rnp_fw_send_cmd_wait(hw, &req, &reply);
}

int rnp_mbx_fw_reset_phy(struct rnp_hw *hw)
{
	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;
	int						ret;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	if (hw->mbx.other_irq_enabled) {
		struct mbx_req_cookie *cookie = mbx_cookie_zalloc(0);

		if (!cookie) {
			return -ENOMEM;
		}

		build_reset_phy_req(&req, cookie);

		ret = rnp_mbx_fw_post_req(hw, &req, cookie);
		kfree(cookie);
		return ret;
	} else {
		build_reset_phy_req(&req, &req);
		return rnp_fw_send_cmd_wait(hw, &req, &reply);
	}
}

#if 0
int rnp_fw_get_vf_macaddr(struct rnp_adapter *adapter, int vf_num)
{
    int                     err;
    struct mbx_fw_cmd_req   req;
    struct mbx_fw_cmd_reply reply;
    struct rnp_hw *         hw = &adapter->hw;

    memset(&req, 0, sizeof(req));
    memset(&reply, 0, sizeof(reply));

    build_get_macaddress_req(&req, &req, hw->lane_mask, VF_NUM(vf_num,0)|hw->pfvfnum));

    return rnp_fw_send_cmd_wait(hw, &req, &reply);
}
#endif

int rnp_maintain_req(struct rnp_hw *hw, int cmd, int arg0, int req_data_bytes, int reply_bytes,
					 dma_addr_t dma_phy_addr)
{
	int					   err;
	struct mbx_req_cookie *cookie = NULL;

	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;

	cookie = mbx_cookie_zalloc(0);
	if (!cookie) {
		return -ENOMEM;
	}

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));
	cookie->timeout_jiffes = 60 * HZ;  // 60s

	build_maintain_req(&req, cookie, cmd, arg0, req_data_bytes, reply_bytes,
					   dma_phy_addr & 0xffffffff, (dma_phy_addr >> 32) & 0xffffffff);

	if (hw->mbx.other_irq_enabled) {
		err = rnp_mbx_fw_post_req(hw, &req, cookie);
	} else {
		int old_mbx_timeout = hw->mbx.timeout;
		hw->mbx.timeout		= (30 * 1000 * 1000) / hw->mbx.usec_delay;	// wait 30s
		err					= rnp_fw_send_cmd_wait(hw, &req, &reply);
		hw->mbx.timeout		= old_mbx_timeout;
	}

	// quit:
	if (cookie) {
		kfree(cookie);
	}
	return (err) ? -EIO : 0;
}

int rnp_fw_get_macaddr(struct rnp_hw *hw, int pfvfnum, u8 *mac_addr, int nr_lane)
{
	int						err;
	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	rnp_dbg("%s: pfvfnum:0x%x nr_lane:%d\n", __func__, pfvfnum, nr_lane);

	if (!mac_addr) {
		rnp_err("%s: mac_addr is null\n", __func__);
		return -EINVAL;
	}

	if (hw->mbx.other_irq_enabled) {
		struct mbx_req_cookie *cookie = mbx_cookie_zalloc(sizeof(reply.mac_addr));
		struct mac_addr		  *mac	  = (struct mac_addr *)cookie->priv;

		if (!cookie) {
			return -ENOMEM;
		}

		build_get_macaddress_req(&req, 1 << nr_lane, pfvfnum, cookie);

		if ((err = rnp_mbx_fw_post_req(hw, &req, cookie))) {
			kfree(cookie);
			return err;
		}

		if ((1 << nr_lane) & mac->lanes) {
			memcpy(mac_addr, mac->addrs[nr_lane].mac, 6);
		}
		kfree(cookie);
		return 0;
	} else {
		build_get_macaddress_req(&req, 1 << nr_lane, pfvfnum, &req);

		// mbx_fw_req_set_reply(&req, hw->mbx.reply_dma_phy);
		err = rnp_fw_send_cmd_wait(hw, &req, &reply);
		if (err) {
			rnp_err("%s: faild. err:%d\n", __func__, err);
			return err;
		}

		if ((1 << nr_lane) & reply.mac_addr.lanes) {
			memcpy(mac_addr, reply.mac_addr.addrs[nr_lane].mac, 6);
			return 0;
		}
	}

	// buf_dump("reply", &reply,sizeof(reply));

	return -ENODATA;
}

static int rnp_mbx_sfp_read(struct rnp_hw *hw, int sfp_i2c_addr, int reg, int cnt, u8 *out_buf)
{
	struct mbx_fw_cmd_req req;
	int					  err	  = -EIO;
	int					  nr_lane = hw->nr_lane;

	if ((cnt > MBX_SFP_READ_MAX_CNT) || !out_buf) {
		rnp_err("%s: cnt:%d should <= %d out_buf:%p\n", __func__, cnt, MBX_SFP_READ_MAX_CNT,
				out_buf);
		return -EINVAL;
	}

	memset(&req, 0, sizeof(req));

	// printk("%s: lane%d i2c:0x%x, reg:0x%x cnt:%d\n", __func__, nr_lane,
	// sfp_i2c_addr, reg, cnt);

	if (hw->mbx.other_irq_enabled) {
		struct mbx_req_cookie *cookie = mbx_cookie_zalloc(cnt);
		if (!cookie) {
			return -ENOMEM;
		}
		build_mbx_sfp_read(&req, nr_lane, sfp_i2c_addr, reg, cnt, cookie);

		if ((err = rnp_mbx_fw_post_req(hw, &req, cookie))) {  // err
			kfree(cookie);
			return err;
		} else {
			memcpy(out_buf, cookie->priv, cnt);
			err = 0;
		}
		kfree(cookie);
	} else {
		struct mbx_fw_cmd_reply reply;

		memset(&reply, 0, sizeof(reply));
		build_mbx_sfp_read(&req, nr_lane, sfp_i2c_addr, reg, cnt, &reply);

		err = rnp_fw_send_cmd_wait(hw, &req, &reply);
		if (err == 0) {
			memcpy(out_buf, reply.sfp_read.value, cnt);
		}
	}

	return err;
}

int rnp_mbx_sfp_module_eeprom_info(struct rnp_hw *hw, int sfp_addr, int reg, int data_len, u8 *buf)
{
	int left = data_len;
	int cnt, err;

	do {
		cnt = (left > MBX_SFP_READ_MAX_CNT) ? MBX_SFP_READ_MAX_CNT : left;
		if ((err = rnp_mbx_sfp_read(hw, sfp_addr, reg, cnt, buf))) {
			rnp_err("%s: error:%d\n", __func__, err);
			return err;
		}
		reg += cnt;
		buf += cnt;
		left -= cnt;
	} while (left > 0);

	return 0;
}

int rnp_mbx_sfp_write(struct rnp_hw *hw, int sfp_addr, int reg, short v)
{
	struct mbx_fw_cmd_req req;
	int					  err;
	int					  nr_lane = hw->nr_lane;

	memset(&req, 0, sizeof(req));

	build_mbx_sfp_write(&req, nr_lane, sfp_addr, reg, v);

	err = rnp_mbx_write_posted_locked(hw, &req);
	return err;
}

int rnp_mbx_set_dump(struct rnp_hw *hw, int flag)
{
	int					  err;
	struct mbx_fw_cmd_req req;

	memset(&req, 0, sizeof(req));
	build_set_dump(&req, hw->nr_lane, flag);

	err = rnp_mbx_write_posted_locked(hw, &req);

	return err;
}

int rnp_mbx_get_dump(struct rnp_hw *hw, int flags, u8 *data_out, int bytes)
{
	int					   err;
	struct mbx_req_cookie *cookie = NULL;

	struct mbx_fw_cmd_reply reply;
	struct mbx_fw_cmd_req	req;
	struct get_dump_reply  *get_dump;

	void	  *dma_buf = NULL;
	dma_addr_t dma_phy = 0;

	cookie = mbx_cookie_zalloc(sizeof(*get_dump));
	if (!cookie) {
		return -ENOMEM;
	}
	get_dump = (struct get_dump_reply *)cookie->priv;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	if (bytes > sizeof(get_dump->data)) {
		// dma_buf = pci_alloc_consistent(hw->pdev, bytes, &dma_phy);
		dma_buf = dma_alloc_coherent(&hw->pdev->dev, bytes, &dma_phy, GFP_ATOMIC);
		if (!dma_buf) {
			dev_err(&hw->pdev->dev, "%s: no memory:%d!", __func__, bytes);
			err = -ENOMEM;
			goto quit;
		}
	}
	// printk("%s: bytes:%d %x\n", __func__, bytes, dma_phy);

	build_get_dump_req(&req, cookie, hw->nr_lane, dma_phy & 0xffffffff,
					   (dma_phy >> 32) & 0xffffffff, bytes);

	if (hw->mbx.other_irq_enabled) {
		err = rnp_mbx_fw_post_req(hw, &req, cookie);
	} else {
		err		 = rnp_fw_send_cmd_wait(hw, &req, &reply);
		get_dump = &reply.get_dump;
	}

quit:
	if (err == 0) {
		hw->dump.version = get_dump->version;
		hw->dump.flag	 = get_dump->flags;
		hw->dump.len	 = get_dump->bytes;
	}
	if (err == 0 && data_out) {
		if (dma_buf) {
			memcpy(data_out, dma_buf, bytes);
		} else {
			memcpy(data_out, get_dump->data, bytes);
		}
	}
	if (dma_buf) {
		// pci_free_consistent(hw->pdev, bytes, dma_buf, dma_phy);
		dma_free_coherent(&hw->pdev->dev, bytes, dma_buf, dma_phy);
	}
	if (cookie) {
		kfree(cookie);
	}
	return err ? -err : 0;
}

int rnp_fw_update(struct rnp_hw *hw, int partition, const u8 *fw_bin, int bytes)
{
	int					   err;
	struct mbx_req_cookie *cookie = NULL;

	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;

	void	  *dma_buf = NULL;
	dma_addr_t dma_phy;

	cookie = mbx_cookie_zalloc(0);
	if (!cookie) {
		dev_err(&hw->pdev->dev, "%s: no memory:%d!", __func__, 0);
		return -ENOMEM;
	}

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	// dma_buf = pci_alloc_consistent(hw->pdev, bytes, &dma_phy);
	dma_buf = dma_alloc_coherent(&hw->pdev->dev, bytes, &dma_phy, GFP_ATOMIC);
	if (!dma_buf) {
		dev_err(&hw->pdev->dev, "%s: no memory:%d!", __func__, bytes);
		err = -ENOMEM;
		goto quit;
	}

	memcpy(dma_buf, fw_bin, bytes);

	build_fw_update_req(&req, cookie, partition, dma_phy & 0xffffffff, (dma_phy >> 32) & 0xffffffff,
						bytes);
	if (hw->mbx.other_irq_enabled) {
		err = rnp_mbx_fw_post_req(hw, &req, cookie);
	} else {
		int old_mbx_timeout = hw->mbx.timeout;
		hw->mbx.timeout		= (20 * 1000 * 1000) / hw->mbx.usec_delay;	// wait 20s
		err					= rnp_fw_send_cmd_wait(hw, &req, &reply);
		hw->mbx.timeout		= old_mbx_timeout;
	}

quit:
	if (dma_buf) {
		// pci_free_consistent(hw->pdev, bytes, dma_buf, dma_phy);
		dma_free_coherent(&hw->pdev->dev, bytes, dma_buf, dma_phy);
	}
	if (cookie) {
		kfree(cookie);
	}
	printk("%s: %s (errcode:%d)\n", __func__, err ? " failed" : " success", err);
	return (err) ? -EIO : 0;
}

int rnp_mbx_link_event_enable(struct rnp_hw *hw, int enable)
{
	struct mbx_fw_cmd_reply reply;
	struct mbx_fw_cmd_req	req;
	int						err;
	struct rnp_adapter	   *adapter = hw->back;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	spin_lock(&adapter->link_stat_lock);
	if (enable) {
		int v = rd32(hw, RNP_DMA_DUMY);
		v &= 0x0000ffff;
		v |= 0xa5a40000;
		wr32(hw, RNP_DMA_DUMY, v);
	} else {
		wr32(hw, RNP_DMA_DUMY, 0);
	}
	spin_unlock(&adapter->link_stat_lock);

	build_link_set_event_mask(&req, BIT(EVT_LINK_UP), (enable & 1) << EVT_LINK_UP, &req);

	err = rnp_mbx_write_posted_locked(hw, &req);

	return err;
}

int rnp_fw_get_capablity(struct rnp_hw *hw, struct phy_abilities *abil)
{
	int						err;
	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	build_phy_abalities_req(&req, &req);

	err = rnp_fw_send_cmd_wait(hw, &req, &reply);

	if (err == 0)
		memcpy(abil, &reply.phy_abilities, sizeof(*abil));
	return err;
}

int to_mac_type(struct phy_abilities *ability)
{
	int lanes = hweight_long(ability->lane_mask);
	if ((ability->phy_type == PHY_TYPE_40G_BASE_KR4) ||
		(ability->phy_type == PHY_TYPE_40G_BASE_LR4) ||
		(ability->phy_type == PHY_TYPE_40G_BASE_CR4) ||
		(ability->phy_type == PHY_TYPE_40G_BASE_SR4)) {
		if (lanes == 1) {
			return rnp_mac_n10g_x8_40G;
		} else {
			return rnp_mac_n10g_x8_10G;
		}
	} else if ((ability->phy_type == PHY_TYPE_10G_BASE_KR) ||
			   (ability->phy_type == PHY_TYPE_10G_BASE_LR) ||
			   (ability->phy_type == PHY_TYPE_10G_BASE_ER) ||
			   (ability->phy_type == PHY_TYPE_10G_BASE_SR)) {
		if (lanes == 1) {
			return rnp_mac_n10g_x2_10G;
		} else if (lanes == 2) {
			return rnp_mac_n10g_x4_10G;
		} else {
			return rnp_mac_n10g_x8_10G;
		}
	} else if (ability->phy_type == PHY_TYPE_1G_BASE_KX) {
		return rnp_mac_n10l_x8_1G;
	} else if (ability->phy_type == PHY_TYPE_RGMII) {
		return rnp_mac_n10l_x8_1G;
	}
	return rnp_mac_unknown;
}

int rnp_set_lane_fun(struct rnp_hw *hw, int fun, int value0, int value1, int value2, int value3)
{
	int						err;
	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	// printk("%s: fun:%d %d-%d-%d-%d\n", __func__, fun, value0, value1, value2,
	// value3);

	build_set_lane_fun(&req, hw->nr_lane, fun, value0, value1, value2, value3);

	return rnp_mbx_write_posted_locked(hw, &req);
}

int rnp_mbx_ifup_down(struct rnp_hw *hw, int up)
{
	int						err;
	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	build_ifup_down(&req, hw->nr_lane, up);

	if (mutex_lock_interruptible(&hw->mbx.lock)) {
		return -EAGAIN;
	}
	err = hw->mbx.ops.write(hw, (u32 *)&req, (req.datalen + MBX_REQ_HDR_LEN) / 4, MBX_FW);

	mutex_unlock(&hw->mbx.lock);

	rnp_logd(LOG_MBX_IFUP_DOWN, "%s: lane:%d up:%d\n", __func__, hw->nr_lane, up);

	// force firmware report link-status
	if (up) {
		rnp_link_stat_mark_reset(hw);
	}
	return err;
}

int rnp_mbx_led_set(struct rnp_hw *hw, int value)
{
	int						err;
	struct mbx_fw_cmd_req	req;
	struct mbx_fw_cmd_reply reply;

	memset(&req, 0, sizeof(req));
	memset(&reply, 0, sizeof(reply));

	build_led_set(&req, hw->nr_lane, value, &reply);

	return rnp_mbx_write_posted_locked(hw, &req);
}

static int rnp_nic_mode_convert_to_adapter_cnt(struct phy_abilities *ability)
{
	int adapter_cnt = 0;

#if 0
	switch (ability->nic_mode) {
		case MODE_NIC_MODE_1PORT_40G:
		case MODE_NIC_MODE_1PORT:
			adapter_cnt = 1;
			break;
		case MODE_NIC_MODE_2PORT:
			adapter_cnt = 2;
			break;
		case MODE_NIC_MODE_4PORT:
			adapter_cnt = 4;
			break;
		default:
			adapter_cnt = 0;
			break;
	}
#endif

	return adapter_cnt;
}

int rnp_mbx_get_capability(struct rnp_hw *hw, struct rnp_info *info)
{
	int					 i, oldtimeout;
	struct rnp_mbx_info *mbx = &hw->mbx;
	int					 err;
	struct phy_abilities ablity;
	int					 try_cnt = 3;
	// rnp_mbx_reset(hw);

	memset(&ablity, 0, sizeof(ablity));

	rnp_link_stat_mark_disable(hw);

	// enable CM3CPU to PF MBX IRQ
	// wr32(hw, CPU_PF_MBOX_MASK, 0);

	while (try_cnt--) {
		err = rnp_fw_get_capablity(hw, &ablity);
		if (err == 0 && info) {
			hw->lane_mask	  = ablity.lane_mask & 0xf;
			info->mac		  = to_mac_type(&ablity);
			info->adapter_cnt = hweight_long(hw->lane_mask);
			hw->mode		  = ablity.nic_mode;
			hw->pfvfnum		  = ablity.pfnum;
			hw->speed		  = ablity.speed;
			hw->nr_lane		  = 0;	// PF1
			hw->fw_version	  = ablity.fw_version;
			hw->mac_type	  = info->mac;
			hw->phy_type	  = ablity.phy_type;
			hw->axi_mhz		  = ablity.axi_mhz;
			hw->port_ids	  = ablity.port_ids;
			hw->bd_uid		  = ablity.bd_uid;

			pr_info("%s: nic-mode:%d mac:%d adpt_cnt:%d lane_mask:0x%x, phy_type: "
					"0x%x, "
					"pfvfnum:0x%x, fw-version:0x%08x\n, axi:%d Mhz,"
					"port_id:%d bd_uid:0x%08x\n",
					__func__, hw->mode, info->mac, info->adapter_cnt, hw->lane_mask, hw->phy_type,
					hw->pfvfnum, ablity.fw_version, ablity.axi_mhz, ablity.port_id[0], hw->bd_uid);
			if (info->adapter_cnt != 0) {
				return 0;
			}
		}
	}

	dev_err(&hw->pdev->dev, "%s: error!\n", __func__);
	return -EIO;
}

int rnp_mbx_get_temp(struct rnp_hw *hw, int *voltage)
{
	int						err;
	struct mbx_req_cookie  *cookie = NULL;
	struct mbx_fw_cmd_reply reply;
	struct mbx_fw_cmd_req	req;
	struct get_temp		   *temp;
	int						temp_v = 0;

	cookie = mbx_cookie_zalloc(sizeof(*temp));
	if (!cookie) {
		return -ENOMEM;
	}
	temp = (struct get_temp *)cookie->priv;

	memset(&req, 0, sizeof(req));

	build_get_temp(&req, cookie);

	if (hw->mbx.other_irq_enabled) {
		err = rnp_mbx_fw_post_req(hw, &req, cookie);
	} else {
		memset(&reply, 0, sizeof(reply));
		err	 = rnp_fw_send_cmd_wait(hw, &req, &reply);
		temp = &reply.get_temp;
	}
	// printk("%s: err:%d\n", __func__, err);

	if (voltage)
		*voltage = temp->volatage;
	temp_v = temp->temp;

	if (cookie) {
		kfree(cookie);
	}
	return temp_v;
}

int rnp_fw_reg_read(struct rnp_hw *hw, int addr, int sz)
{
	struct mbx_req_cookie *cookie;
	struct mbx_fw_cmd_req  req;
	int					   value;

	cookie = mbx_cookie_zalloc(sizeof(int));
	if (!cookie) {
		return -ENOMEM;
	}
	build_readreg_req(&req, addr, cookie);

	rnp_mbx_fw_post_req(hw, &req, cookie);

	value = *((int *)cookie->priv);

	if (cookie) {
		kfree(cookie);
	}
	return 0;
}

void rnp_link_stat_mark(struct rnp_hw *hw, int up)
{
	u32					v;
	struct rnp_adapter *adapter = hw->back;

	spin_lock(&adapter->link_stat_lock);
	v = rd32(hw, RNP_DMA_DUMY);
	v &= ~(0xffff0000);
	v |= 0xa5a40000;
	if (up) {
		v |= BIT(0);
	} else {
		v &= ~BIT(0);
	}
	wr32(hw, RNP_DMA_DUMY, v);
	spin_unlock(&adapter->link_stat_lock);
}

static inline int rnp_mbx_fw_req_handler(struct rnp_adapter *adapter, struct mbx_fw_cmd_req *req)
{
	// dbg_here;
	struct rnp_hw *hw = &adapter->hw;

	switch (req->opcode) {
		case LINK_STATUS_EVENT: {
			int i;

			rnp_logd(LOG_LINK_EVENT,
					 "[LINK_STATUS_EVENT:0x%x] %s:link changed: changed_lane:0x%x, "
					 "status:0x%x, speed:%d\n",
					 req->opcode, adapter->name, req->link_stat.changed_lanes,
					 req->link_stat.lane_status, req->link_stat.st[0].speed);

			if (req->link_stat.lane_status) {  // linkup
				adapter->hw.link = 1;
				// wr32(hw, RNP_ETH_EXCEPT_DROP_PROC, 0);
				//  rnp_link_stat_mark(hw,  1);
			} else {
				adapter->hw.link = 0;
				// wr32(hw, RNP_ETH_EXCEPT_DROP_PROC, 0xf);
				//  rnp_link_stat_mark(hw,  0);
			}
			if (req->link_stat.port_st_magic == SPEED_VALID_MAGIC) {
				hw->speed = req->link_stat.st[0].speed;

				switch (hw->speed) {
					case 1000:
						adapter->speed = RNP_LINK_SPEED_1GB_FULL;
						break;
					case 10000:
						adapter->speed = RNP_LINK_SPEED_10GB_FULL;
						break;
					case 25000:
						adapter->speed = RNP_LINK_SPEED_25GB_FULL;
						break;
					case 40000:
						adapter->speed = RNP_LINK_SPEED_40GB_FULL;
						break;
				}
			}
			if (adapter->hw.link)
				rnp_link_stat_mark(hw, 1);
			else
				rnp_link_stat_mark(hw, 0);

			// printk("%s link is %x\n", adapter->name, adapter->hw.link);
			adapter->flags |= RNP_FLAG_NEED_LINK_UPDATE;
			break;
		}
	}

	return 0;
}

static inline int rnp_mbx_fw_reply_handler(struct rnp_adapter	   *adapter,
										   struct mbx_fw_cmd_reply *reply)
{
	struct mbx_req_cookie *cookie;
	// dbg_here;

	// buf_dump("reply:", reply, sizeof(*reply));

	cookie = reply->cookie;
	if (!cookie || cookie->magic != COOKIE_MAGIC) {
#if 0
		printk("[%s]%s invalid cookie:%p magic:0x%x opcode:0x%x\n",
			   __func__,
			   adapter->name,
			   cookie,
			   cookie->magic,
			   reply->opcode);

		buf_dump("reply:", reply, sizeof(*reply));
#endif
		return -EIO;
	}

	if (cookie->priv_len > 0) {
		memcpy(cookie->priv, reply->data, cookie->priv_len);
	}

	cookie->done = 1;

	if (reply->flags & FLAGS_ERR) {
		cookie->errcode = reply->error_code;
	} else {
		cookie->errcode = 0;
	}
	smp_mb();
	if (waitqueue_active(&cookie->wait)) {
		wake_up_interruptible(&cookie->wait);
	}

	return 0;
}

static inline int rnp_rcv_msg_from_fw(struct rnp_adapter *adapter)
{
	u32			   msgbuf[RNP_FW_MAILBOX_SIZE];
	struct rnp_hw *hw = &adapter->hw;
	s32			   retval;

	// dbg_here;

	retval = rnp_read_mbx(hw, msgbuf, RNP_FW_MAILBOX_SIZE, MBX_FW);
	if (retval) {
		printk("Error receiving message from FW:%d\n", retval);
		return retval;
	}
	rnp_logd(LOG_MBX_MSG_IN, "msg from fw %p,%d: msg[0]=0x%08x_0x%08x_0x%08x_0x%08x\n", hw,
			 hw->mbx.other_irq_enabled, msgbuf[0], msgbuf[1], msgbuf[2], msgbuf[3]);

	/* this is a message we already processed, do nothing */
	if (((unsigned short *)msgbuf)[0] & FLAGS_DD) {	 //
		return rnp_mbx_fw_reply_handler(adapter, (struct mbx_fw_cmd_reply *)msgbuf);
	} else {  // req
		return rnp_mbx_fw_req_handler(adapter, (struct mbx_fw_cmd_req *)msgbuf);
	}
}

static void rnp_rcv_ack_from_fw(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw  = &adapter->hw;
	u32			   msg = RNP_VT_MSGTYPE_NACK;

	// dbg_here;

	// do-nothing
}

int rnp_fw_msg_handler(struct rnp_adapter *adapter)
{
	// == check fw-req
	if (!rnp_check_for_msg(&adapter->hw, MBX_FW))
		rnp_rcv_msg_from_fw(adapter);

	/* process any acks */
	if (!rnp_check_for_ack(&adapter->hw, MBX_FW))
		rnp_rcv_ack_from_fw(adapter);

	return 0;
}
