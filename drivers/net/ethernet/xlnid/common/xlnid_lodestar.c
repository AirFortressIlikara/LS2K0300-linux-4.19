// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2008 - 2023 Xel Technology. */

#include "xlnid_type.h"
#include "xlnid_lodestar.h"
#include "xlnid_api.h"
#include "xlnid_common.h"

#define XLNID_LODESTAR_MAX_TX_QUEUES    128
#define XLNID_LODESTAR_MAX_RX_QUEUES    128
#define XLNID_LODESTAR_RAR_ENTRIES      128
#define XLNID_LODESTAR_MC_TBL_SIZE      128
#define XLNID_LODESTAR_VFT_TBL_SIZE     128
#define XLNID_LODESTAR_RX_PB_SIZE	    512

STATIC s32 xlnid_read_eeprom_lodestar(struct xlnid_hw *hw,
				   u16 offset, u8 *data);
STATIC s32 xlnid_read_eeprom_buffer_lodestar(struct xlnid_hw *hw, u16 offset,
					  u16 words, u8 *data);

/**
*  xlnid_enable_rx_dma_lodestar - Enable the Rx DMA unit
*  @hw: pointer to hardware structure
*  @regval: register value to write to RXCTRL
*
*  Enables the Rx DMA unit
**/
s32 xlnid_enable_rx_dma_lodestar(struct xlnid_hw *hw, u32 regval)
{
    DEBUGFUNC("xlnid_enable_rx_dma_lodestar");
    return XLNID_SUCCESS;
}


/**
*  xlnid_start_hw_lodestar - Prepare hardware for Tx/Rx
*  @hw: pointer to hardware structure
*
*  Starts the hardware by filling the bus info structure and media type, clears
*  all on chip counters, initializes receive address registers, multicast
*  table, VLAN filter table, calls routine to set up link and flow control
*  settings, and leaves transmit and receive units disabled and uninitialized
**/
s32 xlnid_start_hw_lodestar(struct xlnid_hw *hw)
{
//    s32 ret_val;
    u16 device_caps;

    DEBUGFUNC("xlnid_start_hw_lodestar");

    /* Set the media type */
    hw->phy.media_type = hw->mac.ops.get_media_type(hw);

    /* Cache bit indicating need for crosstalk fix */
    hw->mac.ops.get_device_caps(hw, &device_caps);
    if (device_caps & XLNID_DEVICE_CAPS_NO_CROSSTALK_WR)
        hw->need_crosstalk_fix = false;
    else
        hw->need_crosstalk_fix = true;

    /* Clear adapter stopped flag */
    hw->adapter_stopped = false;

    return XLNID_SUCCESS;
}


/**
 *  xlnid_get_device_caps_lodestar - Get additional device capabilities
 *  @hw: pointer to hardware structure
 *  @device_caps: the EEPROM word with the extra device capabilities
 *
 *  This function will read the EEPROM location for the device capabilities,
 *  and return the word through device_caps.
 **/
s32 xlnid_get_device_caps_lodestar(struct xlnid_hw *hw, u16 *device_caps)
{
    *device_caps = XLNID_DEVICE_CAPS_NO_CROSSTALK_WR;

	return XLNID_SUCCESS;
}

/**
 *  xlnid_set_rar_lodestar - Set Rx address register
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *  @addr: Address to put into receive address register
 *  @vmdq: VMDq "set" or "pool" index
 *  @enable_addr: set flag that address is active
 *
 *  Puts an ethernet address into a receive address register.
 **/
s32 xlnid_set_rar_lodestar(struct xlnid_hw *hw, u32 index, u8 *addr, u32 vmdq,
			  u32 enable_addr)
{
    return XLNID_SUCCESS;
}

/**
*  xlnid_clear_rar_lodestar - Remove Rx address register
*  @hw: pointer to hardware structure
*  @index: Receive address register to write
*
*  Clears an ethernet address from a receive address register.
**/
s32 xlnid_clear_rar_lodestar(struct xlnid_hw *hw, u32 index)
{
    u32 rar_entries = hw->mac.num_rar_entries;

    DEBUGFUNC("xlnid_clear_rar_lodestar");

    /* Make sure we are using a valid rar index range */
    if (index >= rar_entries) {
      ERROR_REPORT2(XLNID_ERROR_ARGUMENT,
               "RAR index %d is out of range.\n", index);
      return XLNID_ERR_INVALID_ARGUMENT;
    }

    return XLNID_SUCCESS;
}

void xlnid_disable_rx_lodestar(struct xlnid_hw *hw)
{
    return;
}

/**
 *  xlnid_init_ops_lodestar - Inits func ptrs and MAC type
 *  @hw: pointer to hardware structure
 *
 *  Initialize the function pointers and assign the MAC type for Lodestar.
 *  Does not touch the hardware.
 **/
s32 xlnid_init_ops_lodestar(struct xlnid_hw *hw)
{
	struct xlnid_mac_info *mac = &hw->mac;
	//struct xlnid_phy_info *phy = &hw->phy;
	struct xlnid_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val;

	DEBUGFUNC("xlnid_init_ops_lodestar");

	ret_val = xlnid_init_ops_generic(hw);

	/* MAC */
    mac->ops.reset_hw = xlnid_reset_hw_lodestar;
    mac->ops.start_hw = xlnid_start_hw_lodestar;
    mac->ops.get_media_type = xlnid_get_media_type_lodestar;
    mac->ops.enable_rx_dma = xlnid_enable_rx_dma_lodestar;
	mac->ops.get_supported_physical_layer =NULL;
	mac->ops.disable_sec_rx_path = NULL;
	mac->ops.enable_sec_rx_path = NULL;
	mac->ops.read_analog_reg8 = NULL;
	mac->ops.write_analog_reg8 = NULL;
    mac->ops.get_device_caps = xlnid_get_device_caps_lodestar;
	//mac->ops.get_san_mac_addr = xlnid_get_san_mac_addr_generic;
	//mac->ops.set_san_mac_addr = xlnid_set_san_mac_addr_generic;
	mac->ops.get_wwn_prefix = xlnid_get_wwn_prefix_generic;
	mac->ops.get_fcoe_boot_status = xlnid_get_fcoe_boot_status_generic;

	/* RAR, Multicast, VLAN */
    mac->ops.set_rar = xlnid_set_rar_lodestar;
    mac->ops.clear_rar = xlnid_clear_rar_lodestar;
	mac->ops.set_vmdq = xlnid_set_vmdq_generic;
	mac->ops.set_vmdq_san_mac = xlnid_set_vmdq_san_mac_generic;
	mac->ops.clear_vmdq = xlnid_clear_vmdq_generic;
	mac->ops.insert_mac_addr = xlnid_insert_mac_addr_generic;
	mac->rar_highwater = 1;
	mac->ops.set_vfta = xlnid_set_vfta_generic;
	mac->ops.set_vlvf = xlnid_set_vlvf_generic;
	mac->ops.clear_vfta = xlnid_clear_vfta_generic;
	mac->ops.init_uta_tables = xlnid_init_uta_tables_generic;
	mac->ops.set_mac_anti_spoofing = xlnid_set_mac_anti_spoofing;
	mac->ops.set_vlan_anti_spoofing = xlnid_set_vlan_anti_spoofing;
    mac->ops.disable_rx = xlnid_disable_rx_lodestar;

	/* Link */
	mac->ops.get_link_capabilities = xlnid_get_link_capabilities_lodestar;
    mac->ops.check_link = xlnid_check_mac_link_lodestar;
	mac->ops.setup_rxpba = xlnid_set_rxpba_generic;

	mac->mcft_size		= XLNID_LODESTAR_MC_TBL_SIZE;
	mac->vft_size		= XLNID_LODESTAR_VFT_TBL_SIZE;
	mac->num_rar_entries	= XLNID_LODESTAR_RAR_ENTRIES;
	mac->rx_pb_size		= XLNID_LODESTAR_RX_PB_SIZE;
	mac->max_rx_queues	= XLNID_LODESTAR_MAX_RX_QUEUES;
	mac->max_tx_queues	= XLNID_LODESTAR_MAX_TX_QUEUES;
	mac->max_msix_vectors	= xlnid_get_pcie_msix_count_generic(hw);

	mac->arc_subsystem_valid = false;

	//hw->mbx.ops.init_params = xlnid_init_mbx_params_pf;

	/* EEPROM */
	eeprom->ops.read = xlnid_read_eeprom_lodestar;
	eeprom->ops.read_buffer = xlnid_read_eeprom_buffer_lodestar;

	/* Manageability interface */
	mac->ops.set_fw_drv_ver = xlnid_set_fw_drv_ver_generic;

	mac->ops.get_thermal_sensor_data =
					 xlnid_get_thermal_sensor_data_generic;
	mac->ops.init_thermal_sensor_thresh =
				      xlnid_init_thermal_sensor_thresh_generic;

	mac->ops.get_rtrup2tc = xlnid_dcb_get_rtrup2tc_generic;

	return ret_val;
}

/**
 *  xlnid_reset_hw_lodestar - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
s32 xlnid_reset_hw_lodestar(struct xlnid_hw *hw)
{
    return XLNID_SUCCESS;
}

/**
 *  xlnid_get_media_type_lodestar - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber)
 **/
enum xlnid_media_type xlnid_get_media_type_lodestar(struct xlnid_hw *hw)
{
    return xlnid_media_type_copper;
}

/**
 *  xlnid_read_eeprom_buffer_lodestar - Read EEPROM word(s) using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Retrieves 16 bit word(s) read from EEPROM
 **/
STATIC s32 xlnid_read_eeprom_buffer_lodestar(struct xlnid_hw *hw, u16 offset,
					  u16 words, u8 *data)
{
    return XLNID_SUCCESS;
}

/**
 *  xlnid_read_eeprom_lodestar - Read EEPROM word using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM
 **/
STATIC s32 xlnid_read_eeprom_lodestar(struct xlnid_hw *hw,
				   u16 offset, u8 *data)
{
    return XLNID_SUCCESS;
}

/**
 * xlnid_get_link_capabilities_lodestar- Determines link capabilities
 * @hw: pointer to hardware structure
 * @speed: pointer to link speed
 * @autoneg: boolean auto-negotiation value
 **/
s32 xlnid_get_link_capabilities_lodestar(struct xlnid_hw *hw,
                xlnid_link_speed *speed,
                bool *autoneg)
{
    if (hw->mac.type == xlnid_mac_lodestar)
    {
        *speed = XLNID_LINK_SPEED_1GB_FULL;
        *autoneg = true;
    }

    return XLNID_SUCCESS;
}

/**
 *  xlnid_check_mac_link_lodestar - Determine link and speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true when link is up
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 xlnid_check_mac_link_lodestar(struct xlnid_hw *hw, xlnid_link_speed *speed,
               bool *link_up, bool link_up_wait_to_complete)
{
    *speed = XLNID_LINK_SPEED_1GB_FULL;
    *link_up = true;
    return XLNID_SUCCESS;
}


