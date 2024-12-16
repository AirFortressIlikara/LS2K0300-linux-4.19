#include <linux/dcbnl.h>

#include "rnpm.h"
#include "rnpm_dcb.h"
#include "rnpm_sriov.h"


static void rnpm_config_prio_map(struct rnpm_adapter *adapter,
		u8 pfc_map)
{
	int i, j;
	u32 prio_map = 0;
	u8 port = adapter->port;
	u8 *prio_tc = adapter->prio_tc_map;
	void __iomem *ioaddr = adapter->hw.hw_addr;
	u8 num_tc = adapter->num_tc;

	for (i = 0; i < num_tc; i++) {
		if (i > RNPM_MAX_TCS_NUM)
			break;
		for (j = 0; j < RNPM_MAX_USER_PRIO; j++) {
			dbg("prio_tc[%d]==%d tc_num[%d] pfc_map 0x%.2x\n",
				j, prio_tc[j], i, pfc_map);
			if ((prio_tc[j] == i) && (pfc_map & BIT(j))) {
				dbg("match rule tc_num %d prio_%d\n", i, j);
				prio_map |= (i << (2 * j));
				dbg("match prio_tc change to 0x%.2x\n",
					prio_map);
			}
		}
	}
	/* config untage pkt fifo */
	/* we just have four tc fifo and one fifo is must belong to untage-pkt
	 * so untage need map to the remain tc fifio
	 */
	prio_map |= i << RNPM_FC_UNCTAGS_MAP_OFFSET;
	prio_map |= (1 << 30) | (1 << 31);
	rnpm_wr_reg(ioaddr + RNPM_FC_PORT_PRIO_MAP(port), prio_map);
	dbg("tc_prio_map[%d] 0x%.2x\n", i, prio_map);

	/* enable port prio_map config */
	rnpm_wr_reg(ioaddr + RNPM_FC_EN_CONF_AVAILBLE, 1);
}

static int rnpm_dcb_hw_pfc_config(struct rnpm_adapter *adapter,
		u8 pfc_map)
{
	struct rnpm_dcb_cfg *dcb = &adapter->dcb_cfg;
	struct rnpm_pfc_cfg *pfc = &dcb->pfc_cfg;
	void __iomem *ioaddr = adapter->hw.hw_addr;
	u8 i = 0, j = 0;
	u32 reg = 0;
	u8 num_tc = adapter->num_tc;

	if (!(adapter->flags & RNPM_FLAG_DCB_ENABLED) ||
			adapter->num_rx_queues <= 1) {
		dev_warn(&adapter->pdev->dev, "%s DCB_FLAG%d",
				"don't support pfc when rx quene less"
				"than 1 or disable dcb feature \n",
				adapter->flags & RNPM_FLAG_DCB_ENABLED);
		return 0;
	}
	/* 1.Enable Receive Priority Flow Control */
	reg = RNPM_RX_RFE | RNPM_PFCE;
	rnpm_wr_reg(ioaddr + RNPM_MAC_RX_FLOW_CTRL, reg);
	/* 2.Configure which port will in pfc mode*/
	reg = rnpm_rd_reg(ioaddr + RNPM_FC_PORT_ENABLE);
	/* 3.For Now just support two port Version So just enabled
	 * PF port 0 to enable flow control
	 */
	reg |= 1 << adapter->port;
	rnpm_wr_reg(ioaddr + RNPM_FC_PORT_ENABLE, reg);

	for (i = 0; i < num_tc; i++) {
		int enabled = 0;

		for (j = 0; j < RNPM_MAX_USER_PRIO; j++) {
			if ((adapter->prio_tc_map[j] == i) &&
				(pfc_map & BIT(j))) {
				enabled = 1;
				dcb->pfc_cfg.hw_pfc_map |= BIT(j);
				dcb->pfc_cfg.pfc_num++;
				break;
			}
		}
		if (enabled) {
			/* 4.Enable Transmit Priority Flow Control */
			reg = RNPM_TX_TFE |
				(RNPM_PAUSE_28_SLOT_TIME <<
				RNPM_FC_TX_PLTH_OFFSET) |
				(RNPM_DEFAULT_PAUSE_TIME <<
				RNPM_FC_TX_PT_OFFSET);

			rnpm_wr_reg(ioaddr + RNPM_MAC_Q0_TX_FLOW_CTRL(j), reg);
		}
	}
	/* the below configure can just use default config */
	/* 5.config for pri_map */
	rnpm_config_prio_map(adapter, pfc_map);
	/* 6.Configure PFC Rx high/low thresholds per TC */

	/* 7.Configure Rx full/empty thresholds per tc*/

	/* 8.Configure pause time (3 TCs per register) */
	/* 9.Configure flow control pause low threshold value */

	return 0;
}

static int rnpm_dcb_hw_fc_enable(struct rnpm_adapter *adapter)
{
	void __iomem *ioaddr = adapter->hw.hw_addr;

	/* 1. Enabled Transmit Flow Control */
	rnpm_wr_reg(ioaddr + RNPM_MAC_Q0_TX_FLOW_CTRL(0), RNPM_TX_TFE);
	/* 2. Enabled Recvive Flow Control */
	rnpm_wr_reg(ioaddr + RNPM_MAC_RX_FLOW_CTRL, RNPM_RX_RFE);
	/* 3. Configure Fc Pause Time And Pause Low Threshold
	 * just use default value?
	 */
	return 0;
}

static int rnpm_dcbnl_getpfc(struct net_device *dev,
		struct ieee_pfc *pfc)
{
	struct rnpm_adapter *adapter = netdev_priv(dev);
	struct rnpm_dcb_cfg *dcb = &adapter->dcb_cfg;
	u8 i = 0, j = 0;

	memset(pfc, 0, sizeof(*pfc));
	pfc->pfc_cap = dcb->pfc_cfg.pfc_max;
	/* Pfc setting is based on TC */
	for (i = 0; i < adapter->num_tc; i++) {
		for (j = 0; j < RNPM_MAX_USER_PRIO; j++) {
			if ((adapter->prio_tc_map[j] == i) &&
				(dcb->pfc_cfg.hw_pfc_map & BIT(i)))
				pfc->pfc_en |= BIT(j);
		}
	}
	/* do we need to get the pfc statistic*/
	/* 1. get the tc channel send and recv pfc pkts*/
	/*
	 *for (i = 0; i < TSRN10_MAX_TC_NUM; i++) {
	 *      pfc->requests[i] = dcb->requests[i];
	 *      pfc->indications[i] = dcb->indications[i];
	 }
	 */

	return 0;
}

/* rnpm Support IEEE 802.3 flow-control and
 * Priority base flow control (PFC)
 */
static u8 rnpm_dcbnl_getcap(struct net_device *net_dev, int capid, u8 *cap)
{
	struct rnpm_adapter *priv = netdev_priv(net_dev);

	switch (capid) {
	case DCB_CAP_ATTR_PFC:
		*cap = true;
		break;
	case DCB_CAP_ATTR_PFC_TCS:
		*cap = 0x80;
		break;
	case DCB_CAP_ATTR_DCBX:
		*cap = priv->dcb_cfg.dcbx_mode;
		break;
	default:
		*cap = false;
		break;
	}

	return 0;
}

static u8 rnpm_dcbnl_getstate(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);

	return adapter->dcb_cfg.dcb_en;
}

static u8 rnpm_dcbnl_getdcbx(struct net_device *net_dev)
{
	struct rnpm_adapter *adapter = netdev_priv(net_dev);

	return adapter->dcb_cfg.dcbx_mode;
}

static u8 rnpm_dcbnl_setdcbx(struct net_device *net_dev, u8 mode)
{
	struct rnpm_adapter *adapter = netdev_priv(net_dev);

	adapter->dcb_cfg.dcbx_mode = mode;

	return 0;
	return (mode != (adapter->dcb_cfg.dcbx_mode)) ? 1 : 0;
}

static int rnpm_dcb_parse_config(struct rnpm_dcb_cfg *dcb,
		struct ieee_pfc *pfc)
{
	u8 j = 0, pfc_en_num = 0, pfc_map = 0;

	for (j = 0; j < RNPM_MAX_USER_PRIO; j++) {
		if ((pfc->pfc_en & BIT(j))) {
			pfc_map |= BIT(j);
			pfc_en_num++;
		}
	}
	dcb->pfc_cfg.pfc_num = pfc_en_num;
	dcb->pfc_cfg.hw_pfc_map = pfc_map;
	dbg("pfc_map 0x%.2x pfc->pfc_en 0x%.2x\n", pfc_map, pfc->pfc_en);
	/* tc resource rebuild */
	/* we need to decide tx_ring bind to tc 4 fifo-mac*/
	return pfc_map;
}

static int rnpm_dcbnl_setpfc(struct net_device *dev,
		struct ieee_pfc *pfc)
{
	struct rnpm_adapter *adapter = netdev_priv(dev);
	struct rnpm_dcb_cfg *dcb = &adapter->dcb_cfg;
	u8 pfc_map = 0;

	dbg("%s:%d pfc enabled %d\n", __func__, __LINE__,
		pfc->pfc_en);
	if (pfc->pfc_en) {
		/*set PFC Priority mask */
		pfc_map = rnpm_dcb_parse_config(dcb, pfc);
		rnpm_dcb_hw_pfc_config(adapter, pfc_map);
	} else {
		/*set PAUSE mode */
		rnpm_dcb_hw_fc_enable(adapter);
	}

	return 0;
}

static int rnpm_dcbnl_getnumtcs(struct net_device *netdev, int tcid, u8 *num)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	u8 rval = 0;

	if (adapter->flags & RNPM_FLAG_DCB_ENABLED) {
		switch (tcid) {
		case DCB_NUMTCS_ATTR_PFC:
			*num = adapter->dcb_cfg.pfc_cfg.pfc_num;
			break;
		default:
			rval = -EINVAL;
			break;
		}
	} else {
		rval = -EINVAL;
	}

	return rval;
}

static u8 rnpm_dcbnl_getpfcstate(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_pfc_cfg *pfc_cfg = &adapter->dcb_cfg.pfc_cfg;

	return pfc_cfg->pfc_en;
}

static void rnpm_dcbnl_setpfcstate(struct net_device *netdev, u8 state)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);

	adapter->dcb_cfg.pfc_cfg.pfc_en = state;
}

const struct dcbnl_rtnl_ops rnpm_dcbnl_ops = {
	/*DCB PFC*/
	/*IEEE*/
	.ieee_getpfc            = rnpm_dcbnl_getpfc,
	.ieee_setpfc            = rnpm_dcbnl_setpfc,
	.getcap                 = rnpm_dcbnl_getcap,
	.setdcbx                = rnpm_dcbnl_setdcbx,
	.getdcbx                = rnpm_dcbnl_getdcbx,
	/*CEE*/
	.getstate               = rnpm_dcbnl_getstate,

	.getpfcstate		= rnpm_dcbnl_getpfcstate,
	.setpfcstate		= rnpm_dcbnl_setpfcstate,
};

int rnpm_dcb_init(struct net_device *dev,
		struct rnpm_adapter *adapter)
{
	struct rnpm_dcb_cfg *dcb = &adapter->dcb_cfg;

	dcb->dcb_en = true;
	dcb->pfc_cfg.pfc_max = RNPM_MAX_TCS_NUM;
	dcb->dcbx_mode = DCB_CAP_DCBX_HOST | DCB_CAP_DCBX_VER_IEEE;
	dev->dcbnl_ops = &rnpm_dcbnl_ops;
	adapter->flags |= RNPM_FLAG_DCB_ENABLED;

	return 0;
}
