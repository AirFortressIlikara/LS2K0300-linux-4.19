// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2008 - 2023 Xel Technology. */

#include "xlnid_type.h"
#include "xlnid_lk10.h"
#include "xlnid_api.h"
#include "xlnid_common.h"

#define XLNID_LK10_MAX_TX_QUEUES    128
#define XLNID_LK10_MAX_RX_QUEUES    128
#define XLNID_LK10_RAR_ENTRIES      128
#define XLNID_LK10_MC_TBL_SIZE      128
#define XLNID_LK10_VFT_TBL_SIZE     128
#define XLNID_LK10_RX_PB_SIZE	    512

STATIC s32 xlnid_read_eeprom_lk10(struct xlnid_hw *hw,
				   u16 offset, u8 *data);
STATIC s32 xlnid_read_eeprom_buffer_lk10(struct xlnid_hw *hw, u16 offset,
					  u16 words, u8 *data);
STATIC s32 xlnid_write_eeprom_buffer_lk10(struct xlnid_hw *hw, u16 offset,
			   u16 bytes, u8 *data);

/**
 *  xlnid_get_device_caps_lk10 - Get additional device capabilities
 *  @hw: pointer to hardware structure
 *  @device_caps: the EEPROM word with the extra device capabilities
 *
 *  This function will read the EEPROM location for the device capabilities,
 *  and return the word through device_caps.
 **/
s32 xlnid_get_device_caps_lk10(struct xlnid_hw *hw, u16 *device_caps)
{
    *device_caps = XLNID_DEVICE_CAPS_NO_CROSSTALK_WR;

	return XLNID_SUCCESS;
}

/**
 *  xlnid_init_ops_lk10 - Inits func ptrs and MAC type
 *  @hw: pointer to hardware structure
 *
 *  Initialize the function pointers and assign the MAC type for LK10.
 *  Does not touch the hardware.
 **/
s32 xlnid_init_ops_lk10(struct xlnid_hw *hw)
{
	struct xlnid_mac_info *mac = &hw->mac;
	//struct xlnid_phy_info *phy = &hw->phy;
	struct xlnid_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val;

	DEBUGFUNC("xlnid_init_ops_lk10");

	ret_val = xlnid_init_ops_generic(hw);

	/* MAC */
    mac->ops.reset_hw = xlnid_reset_hw_lk10;
    mac->ops.get_media_type = xlnid_get_media_type_lk10;
	mac->ops.get_supported_physical_layer =NULL;
	mac->ops.disable_sec_rx_path = NULL;
	mac->ops.enable_sec_rx_path = NULL;
	mac->ops.read_analog_reg8 = NULL;
	mac->ops.write_analog_reg8 = NULL;
    mac->ops.get_device_caps = xlnid_get_device_caps_lk10;
	mac->ops.get_san_mac_addr = xlnid_get_san_mac_addr_generic;
	//mac->ops.set_san_mac_addr = xlnid_set_san_mac_addr_generic;
	mac->ops.get_wwn_prefix = xlnid_get_wwn_prefix_generic;
	mac->ops.get_fcoe_boot_status = xlnid_get_fcoe_boot_status_generic;

	/* RAR, Multicast, VLAN */
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

	/* Link */
	mac->ops.get_link_capabilities = xlnid_get_link_capabilities_lk10;
    mac->ops.check_link = xlnid_check_mac_link_lk10;
	mac->ops.setup_rxpba = xlnid_set_rxpba_generic;

	mac->mcft_size		= XLNID_LK10_MC_TBL_SIZE;
	mac->vft_size		= XLNID_LK10_VFT_TBL_SIZE;
	mac->num_rar_entries	= XLNID_LK10_RAR_ENTRIES;
	mac->rx_pb_size		= XLNID_LK10_RX_PB_SIZE;
	mac->max_rx_queues	= XLNID_LK10_MAX_RX_QUEUES;
	mac->max_tx_queues	= XLNID_LK10_MAX_TX_QUEUES;
	mac->max_msix_vectors	= xlnid_get_pcie_msix_count_generic(hw);

	mac->arc_subsystem_valid = false;

	//hw->mbx.ops.init_params = xlnid_init_mbx_params_pf;

	/* EEPROM */
    xlnid_init_eeprom_params_generic(hw);
	eeprom->ops.read = xlnid_read_eeprom_lk10;
	eeprom->ops.read_buffer = xlnid_read_eeprom_buffer_lk10;
	eeprom->ops.write_buffer = xlnid_write_eeprom_buffer_lk10;

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
 *  xlnid_reset_hw_lk10 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
s32 xlnid_reset_hw_lk10(struct xlnid_hw *hw)
{
    xlnid_write_xgmac_lk10(hw, LK10_XGMAC_RXMTU, 0x13FFF);
    xlnid_write_xgmac_lk10(hw, LK10_XGMAC_TXMTU, 0x13FFF);	
    xlnid_write_xgmac_lk10(hw, LK10_XGMAC_RXCONFIG, 0x12000000);

	/* Get MAC address from EEPROM */	
	hw->mac.ops.get_san_mac_addr(hw, hw->mac.perm_addr);

	if (xlnid_validate_mac_addr(hw->mac.perm_addr) != 0) 
	{	
		/* Get MAC address from register */	
		hw->mac.ops.get_mac_addr(hw, hw->mac.perm_addr);
	}
	
    return XLNID_SUCCESS;
}

/**
 *  xlnid_get_media_type_lk10 - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber)
 **/
enum xlnid_media_type xlnid_get_media_type_lk10(struct xlnid_hw *hw)
{
    return xlnid_media_type_fiber;
}

STATIC u32 xlnid_read_eeprom_repeat_lk10(struct xlnid_hw *hw, uint32_t addr)
{        
	uint32_t regval = 0xa0;

	regval |= addr << 8;	
	XLNID_WRITE_REG_MAC(hw, LK10_I2C_BASE + 0x1000, regval);
	udelay(100);
	
	XLNID_WRITE_REG_MAC(hw, LK10_I2C_BASE + 0x1004, 0x60402);	
	udelay(10000);
	
	XLNID_READ_REG_MAC(hw, LK10_I2C_BASE + 0x1044);

    return XLNID_READ_REG_MAC(hw, LK10_I2C_BASE + 0x1044);
}

/**
 *  xlnid_read_eeprom_buffer_lk10 - Read EEPROM word(s) using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Retrieves 16 bit word(s) read from EEPROM
 **/
STATIC s32 xlnid_read_eeprom_buffer_lk10(struct xlnid_hw *hw, u16 offset,
					  u16 words, u8 *data)
{
    uint32_t addr = 0;
    u32 i = 0;
    u32 value_i2c = 0;	
	
    if (words == 0)
    {
        return XLNID_ERR_INVALID_ARGUMENT;
    }
    
    if (offset >= (hw->eeprom.word_size * 2))
    {
        return XLNID_ERR_EEPROM;
    }

	for (i = 0; i < words; i = i + 4) 
	{
        addr = offset + i;  
    	value_i2c = xlnid_read_eeprom_repeat_lk10(hw, addr);		
    	data[i] = value_i2c;	
        data[i + 1] = value_i2c >> 8;
        data[i + 2] = value_i2c >> 16;   
        data[i + 3] = value_i2c >> 24;
	}

    return XLNID_SUCCESS;
}

/**
 *  xlnid_read_eeprom_lk10 - Read EEPROM word using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM
 **/
STATIC s32 xlnid_read_eeprom_lk10(struct xlnid_hw *hw,
				   u16 offset, u8 *data)
{
	 return xlnid_read_eeprom_buffer_lk10(hw, offset, 1, data);
}

STATIC void xlnid_write_eeprom_repeat_lk10(struct xlnid_hw *hw,
				 uint32_t addr, u8 *data)
{	   
    /* 0xa0: device address */
    uint32_t regval = 0xa0;
   
    regval |= ((long) &data << 16) | (addr << 8);    
    XLNID_WRITE_REG_MAC(hw, LK10_I2C_BASE + 0x1000, regval);
    udelay(100);
   
    XLNID_WRITE_REG_MAC(hw, LK10_I2C_BASE + 0x1004, 0x10003);
    udelay(10000);
	return;
}

/**
*  xlnid_write_eeprom_buffer_lk10
*  @hw: pointer to hardware structure
*  @offset: offset of byte in the EEPROM to write
*  @bytes: number of bytes
*  @data: byte(s) write to the EEPROM
*
*  Write a 16 bit byte(s) to the EEPROM .
**/
STATIC s32 xlnid_write_eeprom_buffer_lk10(struct xlnid_hw *hw, u16 offset,
			   u16 bytes, u8 *data)
{
   uint32_t addr = 0;
   u16 i = 0; 
	   
   if (bytes == 0)
   {
	   return XLNID_ERR_INVALID_ARGUMENT;
   }

   if ((offset + bytes) >= (hw->eeprom.word_size * 2))
   {
	   return XLNID_ERR_EEPROM;
   }

   for (i = 0; i < bytes; i++)
   {
	   addr = offset + i;
	   xlnid_write_eeprom_repeat_lk10(hw, addr, &data[i]);
   }

   return XLNID_SUCCESS;
}

/**
 * xlnid_get_link_capabilities_lk10- Determines link capabilities
 * @hw: pointer to hardware structure
 * @speed: pointer to link speed
 * @autoneg: boolean auto-negotiation value
 **/
s32 xlnid_get_link_capabilities_lk10(struct xlnid_hw *hw,
                xlnid_link_speed *speed,
                bool *autoneg)
{
    if (hw->mac.type == xlnid_mac_lk10)
    {
        *speed = XLNID_LINK_SPEED_10GB_FULL;
        *autoneg = true;
    }

    return XLNID_SUCCESS;
}

/**
 *  xlnid_check_mac_link_lk10 - Determine link and speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true when link is up
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 xlnid_check_mac_link_lk10(struct xlnid_hw *hw, xlnid_link_speed *speed,
               bool *link_up, bool link_up_wait_to_complete)
{
    u32 regval;

    regval = XLNID_READ_REG_MAC(hw, LK10_ETH_LINKUP);
    if (regval & 0x1)
    {
        *speed = XLNID_LINK_SPEED_10GB_FULL;
        *link_up = true;
    }
    else
    {
        *speed = XLNID_LINK_SPEED_UNKNOWN;
        *link_up = false;
    }

    return XLNID_SUCCESS;
}

u32 xlnid_read_xgmac_lk10(struct xlnid_hw *hw, u32 reg)
{
    XLNID_WRITE_REG_MAC(hw, LK10_XGMAC_ARADDR, (reg & XLNID_XGMAC_ADDRMASK));
    XLNID_WRITE_REG_MAC(hw, LK10_XGMAC_RSTARTUP, 0x1);
    udelay(10);
    
    return XLNID_READ_REG_MAC(hw, LK10_XGMAC_RDATA);
}

void xlnid_write_xgmac_lk10(struct xlnid_hw *hw, u32 reg, u32 data)
{
    XLNID_WRITE_REG_MAC(hw, LK10_XGMAC_WDATA, data);
    XLNID_WRITE_REG_MAC(hw, LK10_XGMAC_AWADDR, (reg & XLNID_XGMAC_ADDRMASK));
    XLNID_WRITE_REG_MAC(hw, LK10_XGMAC_WSTARTUP, 0x1);
    udelay(10);
}

