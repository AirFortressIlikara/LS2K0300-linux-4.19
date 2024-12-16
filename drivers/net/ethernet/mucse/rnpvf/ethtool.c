/* ethtool support for rnpvf */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/vmalloc.h>
#include <linux/if_vlan.h>
#include <linux/uaccess.h>

#include "rnpvf.h"
#include "rnpvf_compat.h"

#define RNP_ALL_RAR_ENTRIES 16

struct rnpvf_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
	int base_stat_offset;
	int saved_reset_offset;
};

#ifdef HAVE_TX_MQ
#ifdef HAVE_NETDEV_SELECT_QUEUE
#ifdef NO_REAL_QUEUE_NUM
#define RNPVF_NUM_RX_QUEUES netdev->num_tx_queues
#define RNPVF_NUM_TX_QUEUES netdev->num_tx_queues
#else
#define RNPVF_NUM_RX_QUEUES netdev->real_num_rx_queues
#define RNPVF_NUM_TX_QUEUES netdev->real_num_tx_queues

#endif
#else
#define RNPVF_NUM_RX_QUEUES adapter->indices
#define RNPVF_NUM_TX_QUEUES adapter->indices
#endif /* HAVE_NETDEV_SELECT_QUEUE */
#else  /* HAVE_TX_MQ */
#define RNPVF_NUM_TX_QUEUES 1
#define RNPVF_NUM_RX_QUEUES \
        (((struct rnp_adapter *)netdev_priv(netdev))->num_rx_queues)
#endif /* HAVE_TX_MQ */

#define RNP_NETDEV_STAT(_net_stat)                                             \
	{                                                                      \
		.stat_string = #_net_stat,                                     \
		.sizeof_stat =                                                 \
			sizeof_field(struct net_device_stats, _net_stat),      \
		.stat_offset = offsetof(struct net_device_stats, _net_stat)    \
	}

static const struct rnpvf_stats rnp_gstrings_net_stats[] = {
	RNP_NETDEV_STAT(rx_packets),
	RNP_NETDEV_STAT(tx_packets),
	RNP_NETDEV_STAT(rx_bytes),
	RNP_NETDEV_STAT(tx_bytes),
	RNP_NETDEV_STAT(rx_errors),
	RNP_NETDEV_STAT(tx_errors),
	RNP_NETDEV_STAT(rx_dropped),
	RNP_NETDEV_STAT(tx_dropped),
	//RNP_NETDEV_STAT(multicast),
	RNP_NETDEV_STAT(collisions),
	RNP_NETDEV_STAT(rx_over_errors),
	RNP_NETDEV_STAT(rx_crc_errors),
	RNP_NETDEV_STAT(rx_frame_errors),
	RNP_NETDEV_STAT(rx_fifo_errors),
	RNP_NETDEV_STAT(rx_missed_errors),
	RNP_NETDEV_STAT(tx_aborted_errors),
	RNP_NETDEV_STAT(tx_carrier_errors),
	RNP_NETDEV_STAT(tx_fifo_errors),
	RNP_NETDEV_STAT(tx_heartbeat_errors),
};
#define RNPVF_GLOBAL_STATS_LEN ARRAY_SIZE(rnp_gstrings_net_stats)
//#define RNPVF_GLOBAL_STATS_LEN 0

#define RNPVF_HW_STAT(_name, _stat)                                            \
	{                                                                      \
		.stat_string = _name,                                          \
		.sizeof_stat = sizeof_field(struct rnpvf_adapter, _stat),      \
		.stat_offset = offsetof(struct rnpvf_adapter, _stat)           \
	}
static struct rnpvf_stats rnpvf_hwstrings_stats[] = {
	RNPVF_HW_STAT("vlan_add_cnt", hw_stats.vlan_add_cnt),
	RNPVF_HW_STAT("vlan_strip_cnt", hw_stats.vlan_strip_cnt),
	RNPVF_HW_STAT("rx_csum_offload_errors", hw_stats.csum_err),
	RNPVF_HW_STAT("rx_csum_offload_good", hw_stats.csum_good),
};

#define RNPVF_HWSTRINGS_STATS_LEN ARRAY_SIZE(rnpvf_hwstrings_stats)

struct rnpvf_tx_queue_ring_stat {
	u64 hw_head;
	u64 hw_tail;
	u64 sw_to_clean;
};

struct rnpvf_rx_queue_ring_stat {
	u64 hw_head;
	u64 hw_tail;
	u64 sw_to_use;
};

#define RNP_QUEUE_STATS_LEN                                                    \
	(RNPVF_NUM_TX_QUEUES *                                                 \
		 (sizeof(struct rnpvf_tx_queue_stats) / sizeof(u64) +          \
		  sizeof(struct rnpvf_queue_stats) / sizeof(u64) +             \
		  sizeof(struct rnpvf_tx_queue_ring_stat) / sizeof(u64)) +     \
	 RNPVF_NUM_RX_QUEUES *                                                 \
		 (sizeof(struct rnpvf_rx_queue_stats) / sizeof(u64) +          \
		  sizeof(struct rnpvf_queue_stats) / sizeof(u64) +             \
		  sizeof(struct rnpvf_rx_queue_ring_stat) / sizeof(u64)))

#define RNPVF_STATS_LEN                                                        \
	(RNPVF_GLOBAL_STATS_LEN + RNP_QUEUE_STATS_LEN +                        \
	 RNPVF_HWSTRINGS_STATS_LEN)

static const char rnp_gstrings_test[][ETH_GSTRING_LEN] = {
	"Register test  (offline)", "Link test   (on/offline)"
};
#define RNPVF_TEST_LEN (sizeof(rnp_gstrings_test) / ETH_GSTRING_LEN)

#ifdef HAVE_ETHTOOL_GET_SSET_COUNT
enum priv_bits {
	padding_enable = 0,
};

static const char rnpvf_priv_flags_strings[][ETH_GSTRING_LEN] = {
#define RNPVF_FT_PADDING BIT(0)
#define RNPVF_FCS_ON BIT(1)
	"ft_padding", "fcs"
};
#define RNPVF_PRIV_FLAGS_STR_LEN ARRAY_SIZE(rnpvf_priv_flags_strings)

#endif
// ethtool -i rnpvf00

#ifdef HAVE_ETHTOOL_CONVERT_U32_AND_LINK_MODE

#define ADVERTISED_MASK_10G                                                    \
	(SUPPORTED_10000baseT_Full | SUPPORTED_10000baseKX4_Full |             \
	 SUPPORTED_10000baseKR_Full)
static int rnpvf_get_link_ksettings(struct net_device *netdev,
				    struct ethtool_link_ksettings *cmd)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);

	struct rnpvf_hw *hw = &adapter->hw;
	rnp_link_speed supported_link;
	bool autoneg = false;
	bool link_up;
	u32 supported, advertising;
	u32 link_speed = 0;

	ethtool_convert_link_mode_to_legacy_u32(&supported,
											cmd->link_modes.supported);

	hw->mac.ops.check_link(hw, &link_speed, &link_up, false);

	switch (link_speed) {
		case RNP_LINK_SPEED_1GB_FULL:
			supported |= SUPPORTED_1000baseT_Full;
			supported |= SUPPORTED_FIBRE;
			advertising |= ADVERTISED_FIBRE | ADVERTISED_1000baseKX_Full;
			cmd->base.port = PORT_FIBRE;
			// ecmd->transceiver = XCVR_INTERNAL;
			break;
		case RNP_LINK_SPEED_10GB_FULL:
			supported |= SUPPORTED_10000baseT_Full;
			supported |= SUPPORTED_FIBRE;
			advertising |= ADVERTISED_FIBRE | SUPPORTED_10000baseT_Full;
			cmd->base.port = PORT_FIBRE;
			// ecmd->transceiver = XCVR_INTERNAL;
			break;
		case RNP_LINK_SPEED_25GB_FULL:
			supported |= SUPPORTED_40000baseKR4_Full;
			supported |= SUPPORTED_FIBRE;
			advertising |= ADVERTISED_FIBRE | SUPPORTED_40000baseKR4_Full;
			cmd->base.port = PORT_FIBRE;
			break;
		case RNP_LINK_SPEED_40GB_FULL:
			supported |= SUPPORTED_40000baseCR4_Full |
						 SUPPORTED_40000baseSR4_Full |
						 SUPPORTED_40000baseLR4_Full;
			supported |= SUPPORTED_FIBRE;
			advertising |= ADVERTISED_FIBRE;
			cmd->base.port = PORT_FIBRE;
			// ecmd->transceiver = XCVR_INTERNAL;
			break;
	}

	if (autoneg) {
		supported |= SUPPORTED_Autoneg;
		advertising |= ADVERTISED_Autoneg;
		cmd->base.autoneg = AUTONEG_ENABLE;
	} else
		cmd->base.autoneg = AUTONEG_DISABLE;

	// set pause support
	supported |= SUPPORTED_Pause;

	switch (hw->fc.current_mode) {
		case rnp_fc_full:
			advertising |= ADVERTISED_Pause;
			break;
		case rnp_fc_rx_pause:
			advertising |= ADVERTISED_Pause | ADVERTISED_Asym_Pause;
			break;
		case rnp_fc_tx_pause:
			advertising |= ADVERTISED_Asym_Pause;
			break;
		default:
			advertising &= ~(ADVERTISED_Pause | ADVERTISED_Asym_Pause);
	}

	if (link_up) {
		switch (link_speed) {
			case RNP_LINK_SPEED_40GB_FULL:
				cmd->base.speed = SPEED_40000;
				break;
			case RNP_LINK_SPEED_25GB_FULL:
				cmd->base.speed = SPEED_25000;
				break;
			case RNP_LINK_SPEED_10GB_FULL:
				cmd->base.speed = SPEED_10000;
				break;
			case RNP_LINK_SPEED_1GB_FULL:
				cmd->base.speed = SPEED_1000;
				break;
			case RNP_LINK_SPEED_100_FULL:
				cmd->base.speed = SPEED_100;
				break;
			default:
				break;
		}
		cmd->base.duplex = DUPLEX_FULL;
	} else {
		cmd->base.speed = SPEED_UNKNOWN;
		cmd->base.duplex = DUPLEX_UNKNOWN;
	}

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
											supported);
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
											supported);
	return 0;
}
#else /* !HAVE_ETHTOOL_CONVERT_U32_AND_LINK_MODE */
static int rnpvf_get_settings(struct net_device *netdev,
			      struct ethtool_cmd *ecmd)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	struct rnpvf_hw *hw = &adapter->hw;
	rnp_link_speed supported_link;
	u32 link_speed = 0;
	bool autoneg = false;
	bool link_up;

	// hw->mac.ops.get_link_capabilities(hw, &supported_link, &autoneg,
	// &media_type);

	hw->mac.ops.check_link(hw, &link_speed, &link_up, false);

	switch (link_speed) {
		case RNP_LINK_SPEED_1GB_FULL:
			ecmd->supported |= SUPPORTED_1000baseT_Full;
			ecmd->supported |= SUPPORTED_FIBRE;
			ecmd->advertising |= ADVERTISED_FIBRE | ADVERTISED_1000baseKX_Full;
			ecmd->port = PORT_FIBRE;
			ecmd->transceiver = XCVR_INTERNAL;
			break;
		case RNP_LINK_SPEED_10GB_FULL:
			ecmd->supported |= SUPPORTED_10000baseT_Full;
			ecmd->supported |= SUPPORTED_FIBRE;
			ecmd->advertising |= ADVERTISED_FIBRE | SUPPORTED_10000baseT_Full;
			ecmd->port = PORT_FIBRE;
			ecmd->transceiver = XCVR_INTERNAL;
			break;
		case RNP_LINK_SPEED_25GB_FULL:
			ecmd->supported |= SUPPORTED_40000baseKR4_Full;
			ecmd->supported |= SUPPORTED_FIBRE;
			ecmd->advertising |= ADVERTISED_FIBRE;
			ecmd->port = PORT_FIBRE;
			ecmd->transceiver = XCVR_INTERNAL;
			break;
		case RNP_LINK_SPEED_40GB_FULL:
			ecmd->supported |= SUPPORTED_40000baseCR4_Full |
							   SUPPORTED_40000baseSR4_Full |
							   SUPPORTED_40000baseLR4_Full;
			ecmd->supported |= SUPPORTED_FIBRE;
			ecmd->advertising |= ADVERTISED_FIBRE;
			ecmd->port = PORT_FIBRE;
			ecmd->transceiver = XCVR_INTERNAL;
			break;
	}

	if (autoneg) {
		ecmd->supported |= SUPPORTED_Autoneg;
		ecmd->advertising |= ADVERTISED_Autoneg;
		ecmd->autoneg = AUTONEG_ENABLE;
	} else
		ecmd->autoneg = AUTONEG_DISABLE;

	/* Indicate pause support */
	ecmd->supported |= SUPPORTED_Pause;

	switch (hw->fc.requested_mode) {
		case rnp_fc_full:
			ecmd->advertising |= ADVERTISED_Pause;
			break;
		case rnp_fc_rx_pause:
			ecmd->advertising |= ADVERTISED_Pause | ADVERTISED_Asym_Pause;
			break;
		case rnp_fc_tx_pause:
			ecmd->advertising |= ADVERTISED_Asym_Pause;
			break;
		default:
			ecmd->advertising &= ~(ADVERTISED_Pause | ADVERTISED_Asym_Pause);
	}

	if (link_up) {
		switch (link_speed) {
			case RNP_LINK_SPEED_40GB_FULL:
				ethtool_cmd_speed_set(ecmd, SPEED_40000);
				break;
			case RNP_LINK_SPEED_25GB_FULL:
				ethtool_cmd_speed_set(ecmd, SPEED_25000);
				break;
			case RNP_LINK_SPEED_10GB_FULL:
				ethtool_cmd_speed_set(ecmd, SPEED_10000);
				break;
			case RNP_LINK_SPEED_1GB_FULL:
				ethtool_cmd_speed_set(ecmd, SPEED_1000);
				break;
			case RNP_LINK_SPEED_100_FULL:
				ethtool_cmd_speed_set(ecmd, SPEED_100);
				break;
			default:
				break;
		}
		ecmd->duplex = DUPLEX_FULL;
	} else {
		ethtool_cmd_speed_set(ecmd, -1);
		ecmd->duplex = -1;
	}

	return 0;
}
#endif

static void rnpvf_get_drvinfo(struct net_device *netdev,
			      struct ethtool_drvinfo *drvinfo)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	struct rnpvf_hw *hw = &adapter->hw;

	strlcpy(drvinfo->driver, rnpvf_driver_name, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, rnpvf_driver_version,
		sizeof(drvinfo->version));
	strlcpy(drvinfo->bus_info, pci_name(adapter->pdev),
		sizeof(drvinfo->bus_info));
	snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
			"%d.%d.%d.%d",
			((char *)&(hw->fw_version))[3],
			((char *)&(hw->fw_version))[2],
			((char *)&(hw->fw_version))[1],
			((char *)&(hw->fw_version))[0]);
#ifdef HAVE_ETHTOOL_GET_SSET_COUNT
	drvinfo->n_priv_flags = RNPVF_PRIV_FLAGS_STR_LEN;
#endif
}

// ethtool -g
static void rnpvf_get_ringparam(struct net_device *netdev,
				struct ethtool_ringparam *ring)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);

	ring->rx_max_pending = RNPVF_MAX_RXD;
	ring->tx_max_pending = RNPVF_MAX_TXD;
	ring->rx_pending = adapter->rx_ring_item_count;
	ring->tx_pending = adapter->tx_ring_item_count;
}

// ethtool -G rnpvf00 rx 1500 tx 1500
static int rnpvf_set_ringparam(struct net_device *netdev,
			       struct ethtool_ringparam *ring)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	struct rnpvf_ring *temp_ring;
	int i, err = 0;
	u32 new_rx_count, new_tx_count;

	if ((ring->rx_mini_pending) || (ring->rx_jumbo_pending))
		return -EINVAL;

	new_tx_count =
		clamp_t(u32, ring->tx_pending, RNPVF_MIN_TXD, RNPVF_MAX_TXD);
	new_tx_count = ALIGN(new_tx_count, RNP_REQ_TX_DESCRIPTOR_MULTIPLE);

	new_rx_count =
		clamp_t(u32, ring->rx_pending, RNPVF_MIN_RXD, RNPVF_MAX_RXD);
	new_rx_count = ALIGN(new_rx_count, RNP_REQ_RX_DESCRIPTOR_MULTIPLE);

	if ((new_tx_count == adapter->tx_ring_item_count) &&
	    (new_rx_count == adapter->rx_ring_item_count)) {
		/* nothing to do */
		return 0;
	}

	while (test_and_set_bit(__RNPVF_RESETTING, &adapter->state))
		usleep_range(1000, 2000);

	if (!netif_running(adapter->netdev)) {
		for (i = 0; i < adapter->num_tx_queues; i++)
			adapter->tx_ring[i]->count = new_tx_count;
		for (i = 0; i < adapter->num_rx_queues; i++)
			adapter->rx_ring[i]->count = new_rx_count;
		adapter->tx_ring_item_count = new_tx_count;
		adapter->rx_ring_item_count = new_rx_count;
		goto clear_reset;
	}

	/* allocate temporary buffer to store rings in */
	i = max_t(int, adapter->num_tx_queues, adapter->num_rx_queues);
	temp_ring = vmalloc(i * sizeof(struct rnpvf_ring));

	if (!temp_ring) {
		err = -ENOMEM;
		goto clear_reset;
	}
	memset(temp_ring, 0x00, i * sizeof(struct rnpvf_ring));

	rnpvf_down(adapter);

	/*
	 * Setup new Tx resources and free the old Tx resources in that order.
	 * We can then assign the new resources to the rings via a memcpy.
	 * The advantage to this approach is that we are guaranteed to still
	 * have resources even in the case of an allocation failure.
	 */
	if (new_tx_count != adapter->tx_ring_item_count) {
		for (i = 0; i < adapter->num_tx_queues; i++) {
			memcpy(&temp_ring[i], adapter->tx_ring[i],
			       sizeof(struct rnpvf_ring));

			temp_ring[i].count = new_tx_count;
			err = rnpvf_setup_tx_resources(adapter, &temp_ring[i]);
			if (err) {
				while (i) {
					i--;
					rnpvf_free_tx_resources(adapter,
								&temp_ring[i]);
				}
				goto err_setup;
			}
		}

		for (i = 0; i < adapter->num_tx_queues; i++) {
			rnpvf_free_tx_resources(adapter, adapter->tx_ring[i]);

			memcpy(adapter->tx_ring[i], &temp_ring[i],
			       sizeof(struct rnpvf_ring));
		}

		adapter->tx_ring_item_count = new_tx_count;
	}

	/* Repeat the process for the Rx rings if needed */
	if (new_rx_count != adapter->rx_ring_item_count) {
		for (i = 0; i < adapter->num_rx_queues; i++) {
			memcpy(&temp_ring[i], adapter->rx_ring[i],
			       sizeof(struct rnpvf_ring));

			temp_ring[i].count = new_rx_count;
			err = rnpvf_setup_rx_resources(adapter, &temp_ring[i]);
			if (err) {
				while (i) {
					i--;
					rnpvf_free_rx_resources(adapter,
								&temp_ring[i]);
				}
				goto err_setup;
			}
		}

		for (i = 0; i < adapter->num_rx_queues; i++) {
			rnpvf_free_rx_resources(adapter, adapter->rx_ring[i]);

			memcpy(adapter->rx_ring[i], &temp_ring[i],
			       sizeof(struct rnpvf_ring));
		}

		adapter->rx_ring_item_count = new_rx_count;
	}

err_setup:
	rnpvf_up(adapter);
	vfree(temp_ring);
clear_reset:
	clear_bit(__RNPVF_RESETTING, &adapter->state);
	return err;
}

static void rnpvf_get_strings(struct net_device *netdev, u32 stringset,
			      u8 *data)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	char *p = (char *)data;
	int i;
	struct rnpvf_ring *ring;
	u16 queue_idx;

	switch (stringset) {
		/*
	case ETH_SS_TEST:
		for (i = 0; i < RNPVF_TEST_LEN; i++) {
			memcpy(data, rnp_gstrings_test[i], ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
		*/
	case ETH_SS_STATS:
		for (i = 0; i < RNPVF_GLOBAL_STATS_LEN; i++) {
			memcpy(p, rnp_gstrings_net_stats[i].stat_string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}

		for (i = 0; i < RNPVF_HWSTRINGS_STATS_LEN; i++) {
			memcpy(p, rnpvf_hwstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}

		BUG_ON(RNPVF_NUM_TX_QUEUES != RNPVF_NUM_RX_QUEUES);

		for (i = 0; i < RNPVF_NUM_TX_QUEUES; i++) {
			//====  tx ========
			ring = adapter->tx_ring[i];
			queue_idx = ring->rnpvf_queue_idx;
#define SHORT_STATS
#ifdef SHORT_STATS
			sprintf(p, "\n     queue%u_tx_packets", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_bytes", i);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_tx_restart", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_busy", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_done_old", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_clean_desc", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_poll_count", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_irq_more", i);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_tx_hw_head", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_hw_tail", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_sw_next_to_clean", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_added_vlan_packets", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_irq_miss", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_next_to_clean", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_tx_equal_count", i);
			p += ETH_GSTRING_LEN;

			//====  rx ========
			ring = adapter->rx_ring[i];
			queue_idx = ring->rnpvf_queue_idx;
			sprintf(p, "\n     queue%u_rx_packets", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_bytes", i);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_rx_driver_drop_packets", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_rsc", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_rsc_flush", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_non_eop_descs", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_alloc_page_failed", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_alloc_buff_failed", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_csum_err", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_csum_good", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_poll_again_count", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_poll_count", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_rm_vlan_packets", i);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_rx_hw_head", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_hw_tail", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_sw_next_to_use", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_irq_miss", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_next_to_clean", i);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_rx_equal_count", i);
			p += ETH_GSTRING_LEN;

#else
			sprintf(p, "\n     queue%u_dma%u_tx_packets", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_bytes", i, queue_idx);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_dma%u_tx_restart", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_busy", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_done_old", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_clean_desc", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_poll_count", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_irq_more", i, queue_idx);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_dma%u_tx_hw_head", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_hw_tail", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_sw_next_to_clean", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_added_vlan_packets", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_irq_miss", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_next_to_clean", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_tx_equal_count", i,
				queue_idx);
			p += ETH_GSTRING_LEN;

			//====  rx ========
			ring = adapter->rx_ring[i];
			queue_idx = ring->rnpvf_queue_idx;
			sprintf(p, "\n     queue%u_dma%u_rx_packets", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_bytes", i, queue_idx);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_dam%u_rx_driver_drop_packets", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_rsc", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_rsc_flush", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_non_eop_descs", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_alloc_page_failed", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_alloc_buff_failed", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_csum_errs", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_csum_good", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_poll_again_count", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_poll_count", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_rm_vlan_packets", i,
				queue_idx);
			p += ETH_GSTRING_LEN;

			sprintf(p, "queue%u_dma%u_rx_hw_head", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_hw_tail", i, queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_sw_next_to_use", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_irq_miss", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_next_to_clean", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
			sprintf(p, "queue%u_dma%u_rx_equal_count", i,
				queue_idx);
			p += ETH_GSTRING_LEN;
#endif
		}
		break;
#ifdef HAVE_ETHTOOL_GET_SSET_COUNT
	case ETH_SS_PRIV_FLAGS:
		memcpy(data, rnpvf_priv_flags_strings,
		       RNPVF_PRIV_FLAGS_STR_LEN * ETH_GSTRING_LEN);
		break;
#endif /* HAVE_ETHTOOL_GET_SSET_COUNT */
	}
}

#ifndef HAVE_ETHTOOL_GET_SSET_COUNT
static int rnpvf_get_stats_count(struct net_device *netdev)
{
	return RNPVF_STATS_LEN;
}

#else

static int rnpvf_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	/* now we don't support test */
		/*
	case ETH_SS_TEST:
		return RNPVF_TEST_LEN;
		*/
	case ETH_SS_STATS:
		return RNPVF_STATS_LEN;
	case ETH_SS_PRIV_FLAGS:
		return RNPVF_PRIV_FLAGS_STR_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static u32 rnpvf_get_priv_flags(struct net_device *netdev)
{
	struct rnpvf_adapter *adapter =
		(struct rnpvf_adapter *)netdev_priv(netdev);
	u32 priv_flags = 0;

	if (adapter->priv_flags & RNPVF_PRIV_FLAG_FT_PADDING)
		priv_flags |= RNPVF_FT_PADDING;
	if (adapter->priv_flags & RNPVF_PRIV_FLAG_FCS_ON)
		priv_flags |= RNPVF_FCS_ON;

	return priv_flags;
}
#endif

static int rnpvf_get_coalesce(struct net_device *netdev,
#ifdef HAVE_ETHTOOL_COALESCE_EXTACK
                              struct ethtool_coalesce *coal,
                              struct kernel_ethtool_coalesce *kernel_coal,
                              struct netlink_ext_ack *extack)
#else
                              struct ethtool_coalesce *coal)
#endif
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);

	coal->use_adaptive_tx_coalesce = adapter->adaptive_tx_coal;
	coal->tx_coalesce_usecs = adapter->tx_usecs;
	coal->tx_coalesce_usecs_irq = 0;
	coal->tx_max_coalesced_frames = adapter->tx_frames;
	coal->tx_max_coalesced_frames_irq = adapter->tx_work_limit;

	coal->use_adaptive_rx_coalesce = adapter->adaptive_rx_coal;
	coal->rx_coalesce_usecs_irq = 0;
	coal->rx_coalesce_usecs = adapter->rx_usecs;
	coal->rx_max_coalesced_frames = adapter->rx_frames;
	coal->rx_max_coalesced_frames_irq = adapter->napi_budge;

	/* this is not support */
	coal->pkt_rate_low = 0;
	coal->pkt_rate_high = 0;
	coal->rx_coalesce_usecs_low = 0;
	coal->rx_max_coalesced_frames_low = 0;
	coal->tx_coalesce_usecs_low = 0;
	coal->tx_max_coalesced_frames_low = 0;
	coal->rx_coalesce_usecs_high = 0;
	coal->rx_max_coalesced_frames_high = 0;
	coal->tx_coalesce_usecs_high = 0;
	coal->tx_max_coalesced_frames_high = 0;
	coal->rate_sample_interval = 0;
	return 0;
}

static int rnpvf_set_coalesce(struct net_device *netdev,
#ifdef HAVE_ETHTOOL_COALESCE_EXTACK
                              struct ethtool_coalesce *ec,
                              struct kernel_ethtool_coalesce *kernel_coal,
                              struct netlink_ext_ack *extack)
#else
                              struct ethtool_coalesce *ec)
#endif
{
	int reset = 0;
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	u32 value;
	/* we don't support close tx and rx coalesce */
	if (!(ec->use_adaptive_tx_coalesce) || !(ec->use_adaptive_rx_coalesce)) {
		return -EINVAL;
	}

	if (ec->tx_max_coalesced_frames_irq) {
		value = clamp_t(u32,
				ec->tx_max_coalesced_frames_irq,
				RNPVF_MIN_TX_WORK,
				RNPVF_MAX_TX_WORK);
		value = ALIGN(value, RNPVF_WORK_ALIGN);
	
		if (adapter->tx_work_limit != value) {
			reset = 1;
			adapter->tx_work_limit =
				value;
		}
	}
	if (ec->tx_max_coalesced_frames) {
                 value = clamp_t(u32,
				 ec->tx_max_coalesced_frames,
				 RNPVF_MIN_TX_FRAME,
				 RNPVF_MAX_TX_FRAME);
		 if (adapter->tx_frames != value) {
			 reset = 1;
                         adapter->tx_frames = value;
                 }

	}

	if (ec->tx_coalesce_usecs) {
		value = clamp_t(
				u32, ec->tx_coalesce_usecs, RNPVF_MIN_TX_USEC, RNPVF_MAX_TX_USEC);
		if (adapter->tx_usecs != value) {
			reset = 1;
			adapter->tx_usecs = value;
		}

	}

	if (ec->rx_max_coalesced_frames_irq) {
		value = clamp_t(u32,
				ec->rx_max_coalesced_frames_irq,
				RNPVF_MIN_RX_WORK,
				RNPVF_MAX_RX_WORK);
		value = ALIGN(value, RNPVF_WORK_ALIGN);

		if (adapter->napi_budge != value) {
			reset = 1;
			adapter->napi_budge = value;
		}

	}
	if (ec->rx_max_coalesced_frames) {
		value = clamp_t(u32,
				ec->rx_max_coalesced_frames,
				RNPVF_MIN_RX_FRAME,
				RNPVF_MAX_RX_FRAME);
		if (adapter->rx_frames != value) {
			reset = 1;
			adapter->rx_frames = value;
		}

	}

	if (ec->rx_coalesce_usecs) {
		value = clamp_t(
				u32, ec->rx_coalesce_usecs, RNPVF_MIN_RX_USEC, RNPVF_MAX_RX_USEC);

		if (adapter->rx_usecs != value) {
			reset = 1;
			adapter->rx_usecs = value;
		}

	}
	/* other setup is not supported */
	if ((ec->pkt_rate_low) || (ec->pkt_rate_high) ||
	    (ec->rx_coalesce_usecs_low) || (ec->rx_max_coalesced_frames_low) ||
	    (ec->tx_coalesce_usecs_low) || (ec->tx_max_coalesced_frames_low) ||
	    (ec->rx_coalesce_usecs_high) ||
	    (ec->rx_max_coalesced_frames_high) ||
	    (ec->tx_coalesce_usecs_high) ||
	    (ec->tx_max_coalesced_frames_high) || (ec->rate_sample_interval) ||
	    (ec->tx_coalesce_usecs_irq) || (ec->rx_coalesce_usecs_irq))
		return -EINVAL;

	if (reset) {
		if (netif_running(netdev))
			rnpvf_close(netdev);
		remove_mbx_irq(adapter);
		rnpvf_clear_interrupt_scheme(adapter);
		rnpvf_init_interrupt_scheme(adapter);
		register_mbx_irq(adapter);
		if (netif_running(netdev))
			return rnpvf_open(netdev);
	}
	return 0;
}

//ethtool -S
static void rnpvf_get_ethtool_stats(struct net_device *netdev,
				    struct ethtool_stats *stats, u64 *data)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	struct rtnl_link_stats64 temp;
	struct rnpvf_hw *hw = &adapter->hw;
	//const struct rtnl_lihk_stats64 *net_stats;
	struct net_device_stats *net_stats = &netdev->stats;
	unsigned int start;
	struct rnpvf_ring *ring;
	int i = 0, j;
	char *p = NULL;

	rnpvf_update_stats(adapter);

	//net_stats = dev_get_stats(netdev, &temp);

	for (i = 0; i < RNPVF_GLOBAL_STATS_LEN; i++) {
		p = (char *)net_stats + rnp_gstrings_net_stats[i].stat_offset;
		data[i] =
			(rnp_gstrings_net_stats[i].sizeof_stat == sizeof(u64)) ?
				*(u64 *)p :
				*(u32 *)p;
	}
	for (j = 0; j < RNPVF_HWSTRINGS_STATS_LEN; j++, i++) {
		p = (char *)adapter + rnpvf_hwstrings_stats[j].stat_offset;
		data[i] =
			(rnpvf_hwstrings_stats[j].sizeof_stat == sizeof(u64)) ?
				*(u64 *)p :
				*(u32 *)p;
	}

	BUG_ON(RNPVF_NUM_TX_QUEUES != RNPVF_NUM_RX_QUEUES);

	for (j = 0; j < RNPVF_NUM_TX_QUEUES; j++) {
		//===== tx-ring ==
		ring = adapter->tx_ring[j];

		if (!ring) {
			data[i++] = 0;
			data[i++] = 0;

			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;

			/* rnpvf_tx_queue_ring_stat */
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;

			//===== rx-ring ==
			data[i++] = 0;
			data[i++] = 0;

			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;

			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			continue;
		}

		data[i++] = ring->stats.packets;
		data[i++] = ring->stats.bytes;

		data[i++] = ring->tx_stats.restart_queue;
		data[i++] = ring->tx_stats.tx_busy;
		data[i++] = ring->tx_stats.tx_done_old;
		data[i++] = ring->tx_stats.clean_desc;
		data[i++] = ring->tx_stats.poll_count;
		data[i++] = ring->tx_stats.irq_more_count;

		/* rnpvf_tx_queue_ring_stat */
		data[i++] = rd32(hw, RNP_DMA_REG_TX_DESC_BUF_HEAD(
					     ring->rnpvf_queue_idx));
		data[i++] = rd32(hw, RNP_DMA_REG_TX_DESC_BUF_TAIL(
					     ring->rnpvf_queue_idx));
		data[i++] = ring->next_to_clean;
		data[i++] = ring->tx_stats.vlan_add;
		data[i++] = ring->tx_stats.tx_irq_miss;
                if (ring->tx_stats.tx_next_to_clean == -1)
                        data[i++] = ring->count;
                else
                        data[i++] = ring->tx_stats.tx_next_to_clean;
		data[i++] = ring->tx_stats.tx_equal_count;

		//===== rx-ring ==
		ring = adapter->rx_ring[j];

		if (!ring) {
			//===== rx-ring ==
			data[i++] = 0;
			data[i++] = 0;

			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;

			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			continue;
		}

		data[i++] = ring->stats.packets;
		data[i++] = ring->stats.bytes;

		data[i++] = ring->rx_stats.driver_drop_packets;
		data[i++] = ring->rx_stats.rsc_count;
		data[i++] = ring->rx_stats.rsc_flush;
		data[i++] = ring->rx_stats.non_eop_descs;
		data[i++] = ring->rx_stats.alloc_rx_page_failed;
		data[i++] = ring->rx_stats.alloc_rx_buff_failed;
		data[i++] = ring->rx_stats.csum_err;
		data[i++] = ring->rx_stats.csum_good;
		data[i++] = ring->rx_stats.poll_again_count;
		data[i++] = ring->rx_stats.poll_count;
		data[i++] = ring->rx_stats.vlan_remove;
		data[i++] = rd32(hw, RNP_DMA_REG_RX_DESC_BUF_HEAD(
					     ring->rnpvf_queue_idx));
		data[i++] = rd32(hw, RNP_DMA_REG_RX_DESC_BUF_TAIL(
					     ring->rnpvf_queue_idx));
		data[i++] = ring->next_to_clean;

		data[i++] = ring->rx_stats.rx_irq_miss;
                if (ring->rx_stats.rx_next_to_clean == -1)
                        data[i++] = ring->count;
                else
                        data[i++] = ring->rx_stats.rx_next_to_clean;
		data[i++] = ring->rx_stats.rx_equal_count;
	}
}

#ifndef HAVE_NDO_SET_FEATURES
static u32 rnpvf_get_rx_csum(struct net_device *netdev)
{
	return !!(netdev->features & NETIF_F_RXCSUM);
}

static int rnpvf_set_rx_csum(struct net_device *netdev, u32 data)
{
	if (data)
		netdev->features |= NETIF_F_RXCSUM;
	else
		netdev->features &= ~NETIF_F_RXCSUM;

	return 0;
}

static int rnpvf_set_tx_csum(struct net_device *netdev, u32 data)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
#ifdef NETIF_F_IPV6_CSUM
	u32 feature_list = NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
#else
	u32 feature_list = NETIF_F_IP_CSUM;
#endif

	switch (adapter->hw.mac.type) {
	case rnp_mac_2port_10G:
#ifdef HAVE_ENCAP_TSO_OFFLOAD
		if (data)
			netdev->hw_enc_features |= NETIF_F_GSO_UDP_TUNNEL;
		else
			netdev->hw_enc_features &= ~NETIF_F_GSO_UDP_TUNNEL;
		feature_list |= NETIF_F_GSO_UDP_TUNNEL;
#endif /* HAVE_ENCAP_TSO_OFFLOAD */
		break;
	default:
		break;
	}

	if (data)
		netdev->features |= feature_list;
	else
		netdev->features &= ~feature_list;

	return 0;
}

#ifdef NETIF_F_TSO
static int rnpvf_set_tso(struct net_device *netdev, u32 data)
{
#ifdef NETIF_F_TSO6
	u32 feature_list = NETIF_F_TSO | NETIF_F_TSO6;
#else
	u32 feature_list = NETIF_F_TSO;
#endif

	if (data)
		netdev->features |= feature_list;
	else
		netdev->features &= ~feature_list;

#ifndef HAVE_NETDEV_VLAN_FEATURES
	if (!data) {
		struct rnpvf_adapter *adapter = netdev_priv(netdev);
		struct net_device *v_netdev;
		int i;

		/* disable TSO on all VLANs if they're present */
		if (!adapter->vlgrp)
			goto tso_out;

		for (i = 0; i < VLAN_GROUP_ARRAY_LEN; i++) {
			v_netdev = vlan_group_get_device(adapter->vlgrp, i);
			if (!v_netdev)
				continue;

			v_netdev->features &= ~feature_list;
			vlan_group_set_device(adapter->vlgrp, i, v_netdev);
		}
	}

tso_out:

#endif /* HAVE_NETDEV_VLAN_FEATURES */
	return 0;
}
#endif
#endif

static char *rnpvf_reg_names[] = {
	"queue%u_dma%u_rx_head",
};

static int rnpvf_get_regs_len(struct net_device *netdev)
{
#define RNPVF_REGS_LEN 1
	return RNPVF_REGS_LEN * sizeof(u32);
}

// ethtool -d rnpvf
static void rnpvf_get_regs(struct net_device *netdev, struct ethtool_regs *regs,
			   void *p)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	struct rnpvf_hw *hw = &adapter->hw;
	u32 *regs_buff = p;
	u32 regs_len = rnpvf_get_regs_len(netdev);
	u32 i = 0;
	u8 dma_ch = 2;
	u32 value;

	memset(p, 0, regs_len);

	regs->version = adapter->hw.vfnum;
	for (i = 0; i < regs_len; i++) {
		hw->mac.ops.read_eth_reg(hw, i * 4, &regs_buff[i]);
	}
	//regs_buff[i++] = rd32(hw, RNP_DMA_REG_TX_DESC_BUF_HEAD(dma_ch));

	/* Receive */
	//for (i = 0; i < ARRAY_SIZE(rnpvf_reg_names); i++)
	//	dbg("%s\t%8.8x\n", rnpvf_reg_names[i], regs_buff[i]);
}

#if 0
static int rnpvf_link_test(struct rnpvf_adapter *adapter, u64 *data)
{
	struct rnpvf_hw *hw = &adapter->hw;
	bool link_up;
	u32 link_speed = 0;
	*data = 0;

	hw->mac.ops.check_link(hw, &link_speed, &link_up, true);
	if (!link_up)
		*data = 1;

	return *data;
}

/* ethtool register test data */
struct rnpvf_reg_test {
	u16 reg;
	u8  array_len;
	u8  test_type;
	u32 mask;
	u32 write;
};

/* In the hardware, registers are laid out either singly, in arrays
 * spaced 0x40 bytes apart, or in contiguous tables.  We assume
 * most tests take place on arrays or single registers (handled
 * as a single-element array) and special-case the tables.
 * Table tests are always pattern tests.
 *
 * We also make provision for some required setup steps by specifying
 * registers to be written without any read-back testing.
 */

#define PATTERN_TEST 1
#define SET_READ_TEST 2
#define WRITE_NO_TEST 3
#define TABLE32_TEST 4
#define TABLE64_TEST_LO 5
#define TABLE64_TEST_HI 6

/* default VF register test */
static const struct rnpvf_reg_test reg_test_vf[] = {
	{ RNP_VFRDBAL(0), 2, PATTERN_TEST, 0xFFFFFF80, 0xFFFFFF80 },
	{ RNP_VFRDBAH(0), 2, PATTERN_TEST, 0xFFFFFFFF, 0xFFFFFFFF },
	{ RNP_VFRDLEN(0), 2, PATTERN_TEST, 0x000FFF80, 0x000FFFFF },
	{ RNP_VFRXDCTL(0), 2, WRITE_NO_TEST, 0, RNP_RXDCTL_ENABLE },
	{ RNP_VFRDT(0), 2, PATTERN_TEST, 0x0000FFFF, 0x0000FFFF },
	{ RNP_VFRXDCTL(0), 2, WRITE_NO_TEST, 0, 0 },
	{ RNP_VFTDBAL(0), 2, PATTERN_TEST, 0xFFFFFF80, 0xFFFFFFFF },
	{ RNP_VFTDBAH(0), 2, PATTERN_TEST, 0xFFFFFFFF, 0xFFFFFFFF },
	{ RNP_VFTDLEN(0), 2, PATTERN_TEST, 0x000FFF80, 0x000FFF80 },
	{ 0, 0, 0, 0 }
};

static const u32 register_test_patterns[] = {
	0x5A5A5A5A, 0xA5A5A5A5, 0x00000000, 0xFFFFFFFF
};

#define REG_PATTERN_TEST(R, M, W)                                              \
	do {                                                                   \
		u32 pat, val, before;                                          \
		for (pat = 0; pat < ARRAY_SIZE(register_test_patterns);        \
		     pat++) {                                                  \
			before = readl(adapter->hw.hw_addr + R);               \
			writel((register_test_patterns[pat] & W),              \
			       (adapter->hw.hw_addr + R));                     \
			val = readl(adapter->hw.hw_addr + R);                  \
			if (val != (register_test_patterns[pat] & W & M)) {    \
				hw_dbg(&adapter->hw,                           \
				       "pattern test reg %04X failed: got "    \
				       "0x%08X expected 0x%08X\n",             \
				       R, val,                                 \
				       (register_test_patterns[pat] & W & M)); \
				*data = R;                                     \
				writel(before, adapter->hw.hw_addr + R);       \
				return 1;                                      \
			}                                                      \
			writel(before, adapter->hw.hw_addr + R);               \
		}                                                              \
	} while (0)

#define REG_SET_AND_CHECK(R, M, W)                                                    \
	do {                                                                          \
		u32 val, before;                                                      \
		before = readl(adapter->hw.hw_addr + R);                              \
		writel((W & M), (adapter->hw.hw_addr + R));                           \
		val = readl(adapter->hw.hw_addr + R);                                 \
		if ((W & M) != (val & M)) {                                           \
			pr_err("set/check reg %04X test failed: got 0x%08X expected " \
			       "0x%08X\n",                                            \
			       R, (val & M), (W & M));                                \
			*data = R;                                                    \
			writel(before, (adapter->hw.hw_addr + R));                    \
			return 1;                                                     \
		}                                                                     \
		writel(before, (adapter->hw.hw_addr + R));                            \
	} while (0)

static int rnpvf_reg_test(struct rnpvf_adapter *adapter, u64 *data)
{
	const struct rnpvf_reg_test *test;
	u32 i;

	test = reg_test_vf;

	/*
	 * Perform the register test, looping through the test table
	 * until we either fail or reach the null entry.
	 */
	while (test->reg) {
		for (i = 0; i < test->array_len; i++) {
			switch (test->test_type) {
			case PATTERN_TEST:
				REG_PATTERN_TEST(test->reg + (i * 0x40),
						test->mask,
						test->write);
				break;
			case SET_READ_TEST:
				REG_SET_AND_CHECK(test->reg + (i * 0x40),
						test->mask,
						test->write);
				break;
			case WRITE_NO_TEST:
				writel(test->write,
				       (adapter->hw.hw_addr + test->reg)
				       + (i * 0x40));
				break;
			case TABLE32_TEST:
				REG_PATTERN_TEST(test->reg + (i * 4),
						test->mask,
						test->write);
				break;
			case TABLE64_TEST_LO:
				REG_PATTERN_TEST(test->reg + (i * 8),
						test->mask,
						test->write);
				break;
			case TABLE64_TEST_HI:
				REG_PATTERN_TEST((test->reg + 4) + (i * 8),
						test->mask,
						test->write);
				break;
			}
		}
		test++;
	}

	*data = 0;
	return *data;
}

static void rnpvf_diag_test(struct net_device *netdev,
			      struct ethtool_test *eth_test, u64 *data)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	bool if_running = netif_running(netdev);

	set_bit(__RNPVF_TESTING, &adapter->state);
	if (eth_test->flags == ETH_TEST_FL_OFFLINE) {
		/* Offline tests */

		hw_dbg(&adapter->hw, "offline testing starting\n");

		/* Link test performed before hardware reset so autoneg doesn't
		 * interfere with test result */
		if (rnpvf_link_test(adapter, &data[1]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		if (if_running)
			/* indicate we're in test mode */
			dev_close(netdev);
		else
			rnpvf_reset(adapter);

		hw_dbg(&adapter->hw, "register testing starting\n");
		if (rnpvf_reg_test(adapter, &data[0]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		rnpvf_reset(adapter);

		clear_bit(__RNPVF_TESTING, &adapter->state);
		if (if_running)
			dev_open(netdev);
	} else {
		hw_dbg(&adapter->hw, "online testing starting\n");
		/* Online tests */
		if (rnpvf_link_test(adapter, &data[1]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		/* Online tests aren't run; pass by default */
		data[0] = 0;

		clear_bit(__RNPVF_TESTING, &adapter->state);
	}
	msleep_interruptible(4 * 1000);
}

static int rnpvf_nway_reset(struct net_device *netdev)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);

	if (netif_running(netdev))
		rnpvf_reinit_locked(adapter);

	return 0;
}
#endif

// ethtool -l
static void rnpvf_get_channels(struct net_device *dev,
			       struct ethtool_channels *ch)
{
	struct rnpvf_adapter *adapter = netdev_priv(dev);

	/* report maximum channels */
	ch->max_combined = min_t(int, adapter->hw.mac.max_tx_queues,
				 adapter->hw.mac.max_rx_queues);

	/* report info for other vector */
	ch->max_other = NON_Q_VECTORS;
	ch->other_count = NON_Q_VECTORS;

	/* record RSS queues */
	ch->combined_count = adapter->dma_channels;
}

// ethtool -L rnpvf00 combined 2
static int rnpvf_set_channels(struct net_device *dev,
			      struct ethtool_channels *ch)
{
	struct rnpvf_adapter *adapter = netdev_priv(dev);
	unsigned int count = ch->combined_count;

	/* verify they are not requesting separate vectors */
	if (!count || ch->rx_count || ch->tx_count)
		return -EINVAL;

	/* verify other_count has not changed */
	if (ch->other_count != NON_Q_VECTORS)
		return -EINVAL;

	/* verify the number of channels does not exceed hardware limits */
	if (count > min_t(int, adapter->hw.mac.max_tx_queues,
			  adapter->hw.mac.max_rx_queues))
		return -EINVAL;

	/* update feature limits from largest to smallest supported values */
	adapter->dma_channels = count;

	/* Hardware has to reinitialize queues and interrupts to
	 * match packet buffer alignment. Unfortunately, the
	 * hardware is not flexible enough to do this dynamically.
	 */
	if (netif_running(dev))
		rnpvf_close(dev);
	rnpvf_clear_interrupt_scheme(adapter);
	rnpvf_init_interrupt_scheme(adapter);
	if (netif_running(dev))
		return rnpvf_open(dev);

	return 0;
}

static u32 rnpvf_get_msglevel(struct net_device *netdev)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);

	return adapter->msg_enable;
}

static void rnpvf_get_pauseparam(struct net_device *netdev,
				 struct ethtool_pauseparam *pause)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
	struct rnpvf_hw *hw = &adapter->hw;

	/* we don't support autoneg */
	pause->autoneg = 0;

	if (hw->fc.current_mode == rnp_fc_rx_pause) {
		pause->rx_pause = 1;
	} else if (hw->fc.current_mode == rnp_fc_tx_pause) {
		pause->tx_pause = 1;
	} else if (hw->fc.current_mode == rnp_fc_full) {
		pause->rx_pause = 1;
		pause->tx_pause = 1;
	}
}

static void rnpvf_set_msglevel(struct net_device *netdev, u32 data)
{
	struct rnpvf_adapter *adapter = netdev_priv(netdev);

	adapter->msg_enable = data;
}

static const struct ethtool_ops rnpvf_ethtool_ops = {

#ifdef HAVE_ETHTOOL_CONVERT_U32_AND_LINK_MODE
	.get_link_ksettings = rnpvf_get_link_ksettings,
#else
	.get_settings = rnpvf_get_settings,
#endif
	.get_drvinfo = rnpvf_get_drvinfo,


	.get_link = ethtool_op_get_link,
	.get_ringparam = rnpvf_get_ringparam,
	//.set_ringparam = rnpvf_set_ringparam,
	.get_strings = rnpvf_get_strings,
	// vf juset get status
	.get_pauseparam = rnpvf_get_pauseparam,
	.get_msglevel = rnpvf_get_msglevel,
	.set_msglevel = rnpvf_set_msglevel,

#ifndef HAVE_ETHTOOL_GET_SSET_COUNT
	.get_stats_count = rnpvf_get_stats_count,
#else /* HAVE_ETHTOOL_GET_SSET_COUNT */
	.get_sset_count = rnpvf_get_sset_count,
	.get_priv_flags = rnpvf_get_priv_flags, //priv flags
#endif /* HAVE_ETHTOOL_GET_SSET_COUNT */
	.get_ethtool_stats = rnpvf_get_ethtool_stats,
#ifdef HAVE_ETHTOOL_GET_PERM_ADDR
	.get_perm_addr = ethtool_op_get_perm_addr,
#endif

	.get_coalesce = rnpvf_get_coalesce,
	.set_coalesce = rnpvf_set_coalesce,
#ifdef ETHTOOL_COALESCE_USECS
        .supported_coalesce_params = ETHTOOL_COALESCE_USECS,
#endif /* ETHTOOL_COALESCE_USECS */

#ifndef HAVE_NDO_SET_FEATURES
	.get_rx_csum = rnpvf_get_rx_csum,
	.set_rx_csum = rnpvf_set_rx_csum,
	.get_tx_csum = ethtool_op_get_tx_csum,
	.set_tx_csum = rnpvf_set_tx_csum,
	.get_sg = ethtool_op_get_sg,
	.set_sg = ethtool_op_set_sg,
#ifdef NETIF_F_TSO
	.get_tso = ethtool_op_get_tso,
	.set_tso = rnpvf_set_tso,
#endif

#ifdef ETHTOOL_GFLAGS
        .get_flags              = ethtool_op_get_flags,
	//.set_flags              = rnp_set_flags,
#endif
#endif /* HAVE_NDO_SET_FEATURES */
	//.get_regs_len = rnpvf_get_regs_len,
	//.get_regs = rnpvf_get_regs,

#ifndef HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT
#ifdef ETHTOOL_SCHANNELS
        .get_channels = rnpvf_get_channels,
        //.set_channels = rnpvf_set_channels,
#endif

        //.get_dump_flag = rnp_get_dump_flag,
        //.get_dump_data = rnp_get_dump_data,
        //.set_dump = rnp_set_dump,
#endif /* HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT */

};
#ifdef HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT
static const struct ethtool_ops_ext rnpvf_ethtool_ops_ext = {
        .size                   = sizeof(struct ethtool_ops_ext),
        .get_channels           = rnpvf_get_channels,
        //.set_channels           = rnp_set_channels,
};
#endif /* HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT */


void rnpvf_set_ethtool_ops(struct net_device *netdev)
{
#ifndef ETHTOOL_OPS_COMPAT
	netdev->ethtool_ops = &rnpvf_ethtool_ops;
#else
	SET_ETHTOOL_OPS(netdev, &rnpvf_ethtool_ops);
#endif
#ifdef HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT
        set_ethtool_ops_ext(netdev, &rnpvf_ethtool_ops_ext);
#endif /* HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT */


}
