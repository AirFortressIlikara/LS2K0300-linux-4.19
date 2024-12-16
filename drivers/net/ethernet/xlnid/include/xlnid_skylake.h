/***************************************************************************************************
  Copyright (C), 2008-2023, XEL Tech. CO., Ltd.
  Filename: xlnid_skylake.h
  Description:
    Skylake/Westlake MAC and GEPHY funcs
  History:
    <author>    <time>    <version>    <desc>
    jacobshi    2022/12/29    1.0    create file
***************************************************************************************************/

#ifndef __XLNID_SKYLAKE_H__
#define __XLNID_SKYLAKE_H__

extern s32 xlnid_get_link_capabilities_skylake(struct xlnid_hw *hw,
                    xlnid_link_speed *speed, bool *autoneg);

extern s32 xlnid_check_mac_link_skylake(struct xlnid_hw *hw, xlnid_link_speed *speed,
               bool *link_up, bool link_up_wait_to_complete);

extern enum xlnid_media_type xlnid_get_media_type_skylake(struct xlnid_hw *hw);

extern void xlnid_disable_tx_laser_skylake(struct xlnid_hw *hw);

extern void xlnid_enable_tx_laser_skylake(struct xlnid_hw *hw);

extern void xlnid_flap_tx_laser_skylake(struct xlnid_hw *hw);

extern s32 xlnid_start_mac_link_skylake(struct xlnid_hw *hw,
                    bool autoneg_wait_to_complete);

extern s32 xlnid_setup_mac_link_skylake(struct xlnid_hw *hw, xlnid_link_speed speed,
                    bool autoneg_wait_to_complete);

extern s32 xlnid_setup_sfp_modules_skylake(struct xlnid_hw *hw);

extern void xlnid_init_mac_link_ops_skylake(struct xlnid_hw *hw);

extern s32 xlnid_reset_hw_skylake(struct xlnid_hw *hw);

extern s32 xlnid_read_analog_reg8_skylake(struct xlnid_hw *hw, u32 reg, u8 *val);

extern s32 xlnid_write_analog_reg8_skylake(struct xlnid_hw *hw, u32 reg, u8 val);

extern s32 xlnid_start_hw_skylake(struct xlnid_hw *hw);

extern s32 xlnid_identify_phy_skylake(struct xlnid_hw *hw);

extern s32 xlnid_init_phy_ops_skylake(struct xlnid_hw *hw);

extern u64 xlnid_get_supported_physical_layer_skylake(struct xlnid_hw *hw);

extern s32 xlnid_enable_rx_dma_skylake(struct xlnid_hw *hw, u32 regval);

extern s32 prot_autoc_read_skylake(struct xlnid_hw *hw, bool *locked, u32 *reg_val);

extern s32 prot_autoc_write_skylake(struct xlnid_hw *hw, u32 reg_val, bool locked);

#endif /* __XLNID_SKYLAKE_H__ */
