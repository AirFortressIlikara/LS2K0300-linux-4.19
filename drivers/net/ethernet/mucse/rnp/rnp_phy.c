#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/mdio.h>

#include "rnp_common.h"
#include "rnp_phy.h"

#define RNP_PHY_REVISION_MASK 0xFFFFFFF0
#define RNP_MAX_PHY_ADDR 32

static void rnp_i2c_start(struct rnp_hw *hw);
static void rnp_i2c_stop(struct rnp_hw *hw);
static s32 rnp_clock_in_i2c_byte(struct rnp_hw *hw, u8 *data);
static s32 rnp_clock_out_i2c_byte(struct rnp_hw *hw, u8 data);
static s32 rnp_get_i2c_ack(struct rnp_hw *hw);
static s32 rnp_clock_in_i2c_bit(struct rnp_hw *hw, bool *data);
static s32 rnp_clock_out_i2c_bit(struct rnp_hw *hw, bool data);
static void rnp_raise_i2c_clk(struct rnp_hw *hw, u32 *i2cctl);
static void rnp_lower_i2c_clk(struct rnp_hw *hw, u32 *i2cctl);
static s32 rnp_set_i2c_data(struct rnp_hw *hw, u32 *i2cctl, bool data);
static bool rnp_get_i2c_data(u32 *i2cctl);
static void rnp_i2c_bus_clear(struct rnp_hw *hw);
static enum rnp_phy_type rnp_get_phy_type_from_id(u32 phy_id);
static s32 rnp_get_phy_id(struct rnp_hw *hw);

/**
 *  rnp_identify_phy_generic - Get physical layer module
 *  @hw: pointer to hardware structure
 *
 *  Determines the physical layer module found on the current adapter.
 **/
s32 rnp_identify_phy_generic(struct rnp_hw *hw)
{
	s32 status = RNP_ERR_PHY_ADDR_INVALID;
	u32 phy_addr;
	u16 ext_ability = 0;

#if 0
	if (hw->phy.type == rnp_phy_unknown) {
		for (phy_addr = 0; phy_addr < RNP_MAX_PHY_ADDR; phy_addr++) {
			hw->phy.mdio.prtad = phy_addr;
			if (mdio45_probe(&hw->phy.mdio, phy_addr) == 0) {
				rnp_get_phy_id(hw);
				hw->phy.type =
				        rnp_get_phy_type_from_id(hw->phy.id);

				if (hw->phy.type == rnp_phy_unknown) {
					hw->phy.ops.read_reg(hw,
							     MDIO_PMA_EXTABLE,
							     MDIO_MMD_PMAPMD,
							     &ext_ability);
					if (ext_ability &
					    (MDIO_PMA_EXTABLE_10GBT |
					     MDIO_PMA_EXTABLE_1000BT))
						hw->phy.type = rnp_phy_unknown;
					else
						hw->phy.type =
							 rnp_phy_generic;
				}

				status = 0;
				break;
			}
		}
		/* clear value if nothing found */
		if (status != 0)
			hw->phy.mdio.prtad = 0;
	} else {
		status = 0;
	}
#endif
	return status;
}

/**
 *  rnp_get_phy_id - Get the phy type
 *  @hw: pointer to hardware structure
 *
 **/
static s32 rnp_get_phy_id(struct rnp_hw *hw)
{
	u32 status;
	u16 phy_id_high = 0;
	u16 phy_id_low = 0;

	status = hw->phy.ops.read_reg(hw, MDIO_DEVID1, MDIO_MMD_PMAPMD,
				      &phy_id_high);

	if (status == 0) {
		hw->phy.id = (u32)(phy_id_high << 16);
		status = hw->phy.ops.read_reg(hw, MDIO_DEVID2, MDIO_MMD_PMAPMD,
					      &phy_id_low);
		hw->phy.id |= (u32)(phy_id_low & RNP_PHY_REVISION_MASK);
		hw->phy.revision = (u32)(phy_id_low & ~RNP_PHY_REVISION_MASK);
	}
	return status;
}

/**
 *  rnp_get_phy_type_from_id - Get the phy type
 *  @hw: pointer to hardware structure
 *
 **/
static enum rnp_phy_type rnp_get_phy_type_from_id(u32 phy_id)
{
	enum rnp_phy_type phy_type = rnp_phy_unknown;

	return phy_type;
}

/**
 *  rnp_reset_phy_generic - Performs a PHY reset
 *  @hw: pointer to hardware structure
 **/
s32 rnp_reset_phy_generic(struct rnp_hw *hw)
{
	u32 i;
	u16 ctrl = 0;
	s32 status = 0;

#if 0
	if (hw->phy.type == rnp_phy_unknown)
		status = rnp_identify_phy_generic(hw);

	if (status != 0 || hw->phy.type == rnp_phy_none)
		goto out;

	/* Don't reset PHY if it's shut down due to overtemp. */
	if (!hw->phy.reset_if_overtemp &&
	    (RNP_ERR_OVERTEMP == hw->phy.ops.check_overtemp(hw)))
		goto out;

	/*
	 * Perform soft PHY reset to the PHY_XS.
	 * This will cause a soft reset to the PHY
	 */
	hw->phy.ops.write_reg(hw, MDIO_CTRL1,
			      MDIO_MMD_PHYXS,
			      MDIO_CTRL1_RESET);

	/*
	 * Poll for reset bit to self-clear indicating reset is complete.
	 * Some PHYs could take up to 3 seconds to complete and need about
	 * 1.7 usec delay after the reset is complete.
	 */
	for (i = 0; i < 30; i++) {
		msleep(100);
		hw->phy.ops.read_reg(hw, MDIO_CTRL1,
				     MDIO_MMD_PHYXS, &ctrl);
		if (!(ctrl & MDIO_CTRL1_RESET)) {
			udelay(2);
			break;
		}
	}

	if (ctrl & MDIO_CTRL1_RESET) {
		status = RNP_ERR_RESET_FAILED;
		hw_dbg(hw, "PHY reset polling failed to complete.\n");
	}

out:
#endif
	return status;
}

/**
 *  rnp_read_phy_reg_generic - Reads a value from a specified PHY register
 *  @hw: pointer to hardware structure
 *  @reg_addr: 32 bit address of PHY register to read
 *  @phy_data: Pointer to read data from PHY register
 **/
s32 rnp_read_phy_reg_generic(struct rnp_hw *hw, u32 reg_addr, u32 device_type,
			     u16 *phy_data)
{
	u32 command;
	u32 i;
	u32 data;
	s32 status = 0;
	u16 gssr;

#if 0
	if (RNP_READ_REG(hw, RNP_STATUS) & RNP_STATUS_LAN_ID_1)
		gssr = RNP_GSSR_PHY1_SM;
	else
		gssr = RNP_GSSR_PHY0_SM;

	if (hw->mac.ops.acquire_swfw_sync(hw, gssr) != 0)
		status = RNP_ERR_SWFW_SYNC;

	if (status == 0) {
		/* Setup and write the address cycle command */
		command = ((reg_addr << RNP_MSCA_NP_ADDR_SHIFT)  |
		           (device_type << RNP_MSCA_DEV_TYPE_SHIFT) |
		           (hw->phy.mdio.prtad << RNP_MSCA_PHY_ADDR_SHIFT) |
		           (RNP_MSCA_ADDR_CYCLE | RNP_MSCA_MDI_COMMAND));

		RNP_WRITE_REG(hw, RNP_MSCA, command);

		/*
		 * Check every 10 usec to see if the address cycle completed.
		 * The MDI Command bit will clear when the operation is
		 * complete
		 */
		for (i = 0; i < RNP_MDIO_COMMAND_TIMEOUT; i++) {
			udelay(10);

			command = RNP_READ_REG(hw, RNP_MSCA);

			if ((command & RNP_MSCA_MDI_COMMAND) == 0)
				break;
		}

		if ((command & RNP_MSCA_MDI_COMMAND) != 0) {
			hw_dbg(hw, "PHY address command did not complete.\n");
			status = RNP_ERR_PHY;
		}

		if (status == 0) {
			/*
			 * Address cycle complete, setup and write the read
			 * command
			 */
			command = ((reg_addr << RNP_MSCA_NP_ADDR_SHIFT)  |
			           (device_type << RNP_MSCA_DEV_TYPE_SHIFT) |
			           (hw->phy.mdio.prtad <<
				    RNP_MSCA_PHY_ADDR_SHIFT) |
			           (RNP_MSCA_READ | RNP_MSCA_MDI_COMMAND));

			RNP_WRITE_REG(hw, RNP_MSCA, command);

			/*
			 * Check every 10 usec to see if the address cycle
			 * completed. The MDI Command bit will clear when the
			 * operation is complete
			 */
			for (i = 0; i < RNP_MDIO_COMMAND_TIMEOUT; i++) {
				udelay(10);

				command = RNP_READ_REG(hw, RNP_MSCA);

				if ((command & RNP_MSCA_MDI_COMMAND) == 0)
					break;
			}

			if ((command & RNP_MSCA_MDI_COMMAND) != 0) {
				hw_dbg(hw, "PHY read command didn't complete\n");
				status = RNP_ERR_PHY;
			} else {
				/*
				 * Read operation is complete.  Get the data
				 * from MSRWD
				 */
				data = RNP_READ_REG(hw, RNP_MSRWD);
				data >>= RNP_MSRWD_READ_DATA_SHIFT;
				*phy_data = (u16)(data);
			}
		}

		hw->mac.ops.release_swfw_sync(hw, gssr);
	}

#endif
	return status;
}

/**
 *  rnp_write_phy_reg_generic - Writes a value to specified PHY register
 *  @hw: pointer to hardware structure
 *  @reg_addr: 32 bit PHY register to write
 *  @device_type: 5 bit device type
 *  @phy_data: Data to write to the PHY register
 **/
s32 rnp_write_phy_reg_generic(struct rnp_hw *hw, u32 reg_addr, u32 device_type,
			      u16 phy_data)
{
	u32 command;
	u32 i;
	s32 status = 0;
	u16 gssr;

#if 0
	if (RNP_READ_REG(hw, RNP_STATUS) & RNP_STATUS_LAN_ID_1)
		gssr = RNP_GSSR_PHY1_SM;
	else
		gssr = RNP_GSSR_PHY0_SM;

	if (hw->mac.ops.acquire_swfw_sync(hw, gssr) != 0)
		status = RNP_ERR_SWFW_SYNC;

	if (status == 0) {
		/* Put the data in the MDI single read and write data register*/
		RNP_WRITE_REG(hw, RNP_MSRWD, (u32)phy_data);

		/* Setup and write the address cycle command */
		command = ((reg_addr << RNP_MSCA_NP_ADDR_SHIFT)  |
		           (device_type << RNP_MSCA_DEV_TYPE_SHIFT) |
		           (hw->phy.mdio.prtad << RNP_MSCA_PHY_ADDR_SHIFT) |
		           (RNP_MSCA_ADDR_CYCLE | RNP_MSCA_MDI_COMMAND));

		RNP_WRITE_REG(hw, RNP_MSCA, command);

		/*
		 * Check every 10 usec to see if the address cycle completed.
		 * The MDI Command bit will clear when the operation is
		 * complete
		 */
		for (i = 0; i < RNP_MDIO_COMMAND_TIMEOUT; i++) {
			udelay(10);

			command = RNP_READ_REG(hw, RNP_MSCA);

			if ((command & RNP_MSCA_MDI_COMMAND) == 0)
				break;
		}

		if ((command & RNP_MSCA_MDI_COMMAND) != 0) {
			hw_dbg(hw, "PHY address cmd didn't complete\n");
			status = RNP_ERR_PHY;
		}

		if (status == 0) {
			/*
			 * Address cycle complete, setup and write the write
			 * command
			 */
			command = ((reg_addr << RNP_MSCA_NP_ADDR_SHIFT)  |
			           (device_type << RNP_MSCA_DEV_TYPE_SHIFT) |
			           (hw->phy.mdio.prtad <<
				    RNP_MSCA_PHY_ADDR_SHIFT) |
			           (RNP_MSCA_WRITE | RNP_MSCA_MDI_COMMAND));

			RNP_WRITE_REG(hw, RNP_MSCA, command);

			/*
			 * Check every 10 usec to see if the address cycle
			 * completed. The MDI Command bit will clear when the
			 * operation is complete
			 */
			for (i = 0; i < RNP_MDIO_COMMAND_TIMEOUT; i++) {
				udelay(10);

				command = RNP_READ_REG(hw, RNP_MSCA);

				if ((command & RNP_MSCA_MDI_COMMAND) == 0)
					break;
			}

			if ((command & RNP_MSCA_MDI_COMMAND) != 0) {
				hw_dbg(hw, "PHY address cmd didn't complete\n");
				status = RNP_ERR_PHY;
			}
		}

		hw->mac.ops.release_swfw_sync(hw, gssr);
	}

#endif
	return status;
}

/**
 *  rnp_setup_phy_link_generic - Set and restart autoneg
 *  @hw: pointer to hardware structure
 *
 *  Restart autonegotiation and PHY and waits for completion.
 **/
s32 rnp_setup_phy_link_generic(struct rnp_hw *hw)
{
	s32 status = 0;
	u32 time_out;
	u32 max_time_out = 10;
#if 0
	u16 autoneg_reg = RNP_MII_AUTONEG_REG;
	bool autoneg = false;
	rnp_link_speed speed;
	rnp_get_copper_link_capabilities_generic(hw, &speed, &autoneg);

	if (speed & RNP_LINK_SPEED_10GB_FULL) {
		/* Set or unset auto-negotiation 10G advertisement */
		hw->phy.ops.read_reg(hw, MDIO_AN_10GBT_CTRL,
				     MDIO_MMD_AN,
				     &autoneg_reg);

		autoneg_reg &= ~MDIO_AN_10GBT_CTRL_ADV10G;
		if (hw->phy.autoneg_advertised & RNP_LINK_SPEED_10GB_FULL)
			autoneg_reg |= MDIO_AN_10GBT_CTRL_ADV10G;

		hw->phy.ops.write_reg(hw, MDIO_AN_10GBT_CTRL,
				      MDIO_MMD_AN,
				      autoneg_reg);
	}

	if (speed & RNP_LINK_SPEED_1GB_FULL) {
		/* Set or unset auto-negotiation 1G advertisement */
		hw->phy.ops.read_reg(hw,
				     RNP_MII_AUTONEG_VENDOR_PROVISION_1_REG,
				     MDIO_MMD_AN,
				     &autoneg_reg);

		autoneg_reg &= ~RNP_MII_1GBASE_T_ADVERTISE;
		if (hw->phy.autoneg_advertised & RNP_LINK_SPEED_1GB_FULL)
			autoneg_reg |= RNP_MII_1GBASE_T_ADVERTISE;

		hw->phy.ops.write_reg(hw,
				      RNP_MII_AUTONEG_VENDOR_PROVISION_1_REG,
				      MDIO_MMD_AN,
				      autoneg_reg);
	}

	if (speed & RNP_LINK_SPEED_100_FULL) {
		/* Set or unset auto-negotiation 100M advertisement */
		hw->phy.ops.read_reg(hw, MDIO_AN_ADVERTISE,
				     MDIO_MMD_AN,
				     &autoneg_reg);

		autoneg_reg &= ~(ADVERTISE_100FULL |
				 ADVERTISE_100HALF);
		if (hw->phy.autoneg_advertised & RNP_LINK_SPEED_100_FULL)
			autoneg_reg |= ADVERTISE_100FULL;

		hw->phy.ops.write_reg(hw, MDIO_AN_ADVERTISE,
				      MDIO_MMD_AN,
				      autoneg_reg);
	}

	/* Restart PHY autonegotiation and wait for completion */
	hw->phy.ops.read_reg(hw, MDIO_CTRL1,
			     MDIO_MMD_AN, &autoneg_reg);

	autoneg_reg |= MDIO_AN_CTRL1_RESTART;

	hw->phy.ops.write_reg(hw, MDIO_CTRL1,
			      MDIO_MMD_AN, autoneg_reg);

	/* Wait for autonegotiation to finish */
	for (time_out = 0; time_out < max_time_out; time_out++) {
		udelay(10);
		/* Restart PHY autonegotiation and wait for completion */
		status = hw->phy.ops.read_reg(hw, MDIO_STAT1,
					      MDIO_MMD_AN,
					      &autoneg_reg);

		autoneg_reg &= MDIO_AN_STAT1_COMPLETE;
		if (autoneg_reg == MDIO_AN_STAT1_COMPLETE) {
			break;
		}
	}

	if (time_out == max_time_out) {
		status = RNP_ERR_LINK_SETUP;
		hw_dbg(hw, "rnp_setup_phy_link_generic: time out");
	}
#endif
	return status;
}

/**
 *  rnp_setup_phy_link_speed_generic - Sets the auto advertised capabilities
 *  @hw: pointer to hardware structure
 *  @speed: new link speed
 **/
s32 rnp_setup_phy_link_speed_generic(struct rnp_hw *hw, rnp_link_speed speed,
				     bool autoneg_wait_to_complete)
{
	/*
	 * Clear autoneg_advertised and set new values based on input link
	 * speed.
	 */
	hw->phy.autoneg_advertised = 0;

	if (speed & RNP_LINK_SPEED_10GB_FULL)
		hw->phy.autoneg_advertised |= RNP_LINK_SPEED_10GB_FULL;

	if (speed & RNP_LINK_SPEED_1GB_FULL)
		hw->phy.autoneg_advertised |= RNP_LINK_SPEED_1GB_FULL;

	if (speed & RNP_LINK_SPEED_100_FULL)
		hw->phy.autoneg_advertised |= RNP_LINK_SPEED_100_FULL;

	/* Setup link based on the new speed settings */
	hw->phy.ops.setup_link(hw);

	return 0;
}

/**
 * rnp_get_copper_link_capabilities_generic - Determines link capabilities
 * @hw: pointer to hardware structure
 * @speed: pointer to link speed
 * @autoneg: boolean auto-negotiation value
 *
 * Determines the link capabilities by reading the AUTOC register.
 */
s32 rnp_get_copper_link_capabilities_generic(struct rnp_hw *hw,
					     rnp_link_speed *speed,
					     bool *autoneg)
{
	s32 status = RNP_ERR_LINK_SETUP;
	u16 speed_ability;

	*speed = 0;
	*autoneg = true;

	status = hw->phy.ops.read_reg(hw, MDIO_SPEED, MDIO_MMD_PMAPMD,
				      &speed_ability);

	if (status == 0) {
		if (speed_ability & MDIO_SPEED_10G)
			*speed |= RNP_LINK_SPEED_10GB_FULL;
		if (speed_ability & MDIO_PMA_SPEED_1000)
			*speed |= RNP_LINK_SPEED_1GB_FULL;
		if (speed_ability & MDIO_PMA_SPEED_100)
			*speed |= RNP_LINK_SPEED_100_FULL;
	}

	return status;
}

/**
 *  rnp_check_phy_link_tnx - Determine link and speed status
 *  @hw: pointer to hardware structure
 *
 *  Reads the VS1 register to determine if link is up and the current speed for
 *  the PHY.
 **/
s32 rnp_check_phy_link_tnx(struct rnp_hw *hw, rnp_link_speed *speed,
			   bool *link_up)
{
	s32 status = 0;
	u32 time_out;
	u32 max_time_out = 10;
	u16 phy_link = 0;
	u16 phy_speed = 0;
	u16 phy_data = 0;

#if 0
	/* Initialize speed and link to default case */
	*link_up = false;
	*speed = RNP_LINK_SPEED_10GB_FULL;

	/*
	 * Check current speed and link status of the PHY register.
	 * This is a vendor specific register and may have to
	 * be changed for other copper PHYs.
	 */
	for (time_out = 0; time_out < max_time_out; time_out++) {
		udelay(10);
		status = hw->phy.ops.read_reg(hw,
					      MDIO_STAT1,
					      MDIO_MMD_VEND1,
					      &phy_data);
		phy_link = phy_data &
			    RNP_MDIO_VENDOR_SPECIFIC_1_LINK_STATUS;
		phy_speed = phy_data &
			    RNP_MDIO_VENDOR_SPECIFIC_1_SPEED_STATUS;
		if (phy_link == RNP_MDIO_VENDOR_SPECIFIC_1_LINK_STATUS) {
			*link_up = true;
			if (phy_speed ==
			    RNP_MDIO_VENDOR_SPECIFIC_1_SPEED_STATUS)
				*speed = RNP_LINK_SPEED_1GB_FULL;
			break;
		}
	}
#endif
	return status;
}

/**
 *	rnp_setup_phy_link_tnx - Set and restart autoneg
 *	@hw: pointer to hardware structure
 *
 *	Restart autonegotiation and PHY and waits for completion.
 **/
s32 rnp_setup_phy_link_tnx(struct rnp_hw *hw)
{
	s32 status = 0;
#if 0
	u32 time_out;
	u32 max_time_out = 10;
	u16 autoneg_reg = RNP_MII_AUTONEG_REG;
	bool autoneg = false;
	rnp_link_speed speed;
	rnp_get_copper_link_capabilities_generic(hw, &speed, &autoneg);

	if (speed & RNP_LINK_SPEED_10GB_FULL) {
		/* Set or unset auto-negotiation 10G advertisement */
		hw->phy.ops.read_reg(hw, MDIO_AN_10GBT_CTRL,
				     MDIO_MMD_AN,
				     &autoneg_reg);

		autoneg_reg &= ~MDIO_AN_10GBT_CTRL_ADV10G;
		if (hw->phy.autoneg_advertised & RNP_LINK_SPEED_10GB_FULL)
			autoneg_reg |= MDIO_AN_10GBT_CTRL_ADV10G;

		hw->phy.ops.write_reg(hw, MDIO_AN_10GBT_CTRL,
				      MDIO_MMD_AN,
				      autoneg_reg);
	}

	if (speed & RNP_LINK_SPEED_1GB_FULL) {
		/* Set or unset auto-negotiation 1G advertisement */
		hw->phy.ops.read_reg(hw, RNP_MII_AUTONEG_XNP_TX_REG,
				     MDIO_MMD_AN,
				     &autoneg_reg);

		autoneg_reg &= ~RNP_MII_1GBASE_T_ADVERTISE_XNP_TX;
		if (hw->phy.autoneg_advertised & RNP_LINK_SPEED_1GB_FULL)
			autoneg_reg |= RNP_MII_1GBASE_T_ADVERTISE_XNP_TX;

		hw->phy.ops.write_reg(hw, RNP_MII_AUTONEG_XNP_TX_REG,
				      MDIO_MMD_AN,
				      autoneg_reg);
	}

	if (speed & RNP_LINK_SPEED_100_FULL) {
		/* Set or unset auto-negotiation 100M advertisement */
		hw->phy.ops.read_reg(hw, MDIO_AN_ADVERTISE,
				     MDIO_MMD_AN,
				     &autoneg_reg);

		autoneg_reg &= ~(ADVERTISE_100FULL |
				 ADVERTISE_100HALF);
		if (hw->phy.autoneg_advertised & RNP_LINK_SPEED_100_FULL)
			autoneg_reg |= ADVERTISE_100FULL;

		hw->phy.ops.write_reg(hw, MDIO_AN_ADVERTISE,
				      MDIO_MMD_AN,
				      autoneg_reg);
	}

	/* Restart PHY autonegotiation and wait for completion */
	hw->phy.ops.read_reg(hw, MDIO_CTRL1,
			     MDIO_MMD_AN, &autoneg_reg);

	autoneg_reg |= MDIO_AN_CTRL1_RESTART;

	hw->phy.ops.write_reg(hw, MDIO_CTRL1,
			      MDIO_MMD_AN, autoneg_reg);

	/* Wait for autonegotiation to finish */
	for (time_out = 0; time_out < max_time_out; time_out++) {
		udelay(10);
		/* Restart PHY autonegotiation and wait for completion */
		status = hw->phy.ops.read_reg(hw, MDIO_STAT1,
					      MDIO_MMD_AN,
					      &autoneg_reg);

		autoneg_reg &= MDIO_AN_STAT1_COMPLETE;
		if (autoneg_reg == MDIO_AN_STAT1_COMPLETE)
			break;
	}

	if (time_out == max_time_out) {
		status = RNP_ERR_LINK_SETUP;
		hw_dbg(hw, "rnp_setup_phy_link_tnx: time out");
	}
#endif
	return status;
}

/**
 *  rnp_get_phy_firmware_version_tnx - Gets the PHY Firmware Version
 *  @hw: pointer to hardware structure
 *  @firmware_version: pointer to the PHY Firmware Version
 **/
s32 rnp_get_phy_firmware_version_tnx(struct rnp_hw *hw, u16 *firmware_version)
{
	s32 status = 0;
#if 0
	status = hw->phy.ops.read_reg(hw, TNX_FW_REV,
				      MDIO_MMD_VEND1,
				      firmware_version);
#endif
	return status;
}

/**
 *  rnp_get_phy_firmware_version_generic - Gets the PHY Firmware Version
 *  @hw: pointer to hardware structure
 *  @firmware_version: pointer to the PHY Firmware Version
 **/
s32 rnp_get_phy_firmware_version_generic(struct rnp_hw *hw,
					 u16 *firmware_version)
{
	s32 status = 0;
#if 0
	status = hw->phy.ops.read_reg(hw, AQ_FW_REV,
				      MDIO_MMD_VEND1,
				      firmware_version);
#endif
	return status;
}

/**
 *  rnp_reset_phy_nl - Performs a PHY reset
 *  @hw: pointer to hardware structure
 **/
s32 rnp_reset_phy_nl(struct rnp_hw *hw)
{
	s32 ret_val = 0;
#if 0
	u16 phy_offset, control, eword, edata, block_crc;
	bool end_data = false;
	u16 list_offset, data_offset;
	u16 phy_data = 0;
	u32 i;

	hw->phy.ops.read_reg(hw, MDIO_CTRL1, MDIO_MMD_PHYXS, &phy_data);

	/* reset the PHY and poll for completion */
	hw->phy.ops.write_reg(hw, MDIO_CTRL1, MDIO_MMD_PHYXS,
	                      (phy_data | MDIO_CTRL1_RESET));

	for (i = 0; i < 100; i++) {
		hw->phy.ops.read_reg(hw, MDIO_CTRL1, MDIO_MMD_PHYXS,
		                     &phy_data);
		if ((phy_data & MDIO_CTRL1_RESET) == 0)
			break;
		usleep_range(10000, 20000);
	}

	if ((phy_data & MDIO_CTRL1_RESET) != 0) {
		hw_dbg(hw, "PHY reset did not complete.\n");
		ret_val = RNP_ERR_PHY;
		goto out;
	}

	/* Get init offsets */
	ret_val = rnp_get_sfp_init_sequence_offsets(hw, &list_offset,
	                                              &data_offset);
	if (ret_val != 0)
		goto out;

	ret_val = hw->eeprom.ops.read(hw, data_offset, &block_crc);
	data_offset++;
	while (!end_data) {
		/*
		 * Read control word from PHY init contents offset
		 */
		ret_val = hw->eeprom.ops.read(hw, data_offset, &eword);
		control = (eword & RNP_CONTROL_MASK_NL) >>
		           RNP_CONTROL_SHIFT_NL;
		edata = eword & RNP_DATA_MASK_NL;
		switch (control) {
		case RNP_DELAY_NL:
			data_offset++;
			hw_dbg(hw, "DELAY: %d MS\n", edata);
			usleep_range(edata * 1000, edata * 2000);
			break;
		case RNP_DATA_NL:
			hw_dbg(hw, "DATA:\n");
			data_offset++;
			hw->eeprom.ops.read(hw, data_offset++,
			                    &phy_offset);
			for (i = 0; i < edata; i++) {
				hw->eeprom.ops.read(hw, data_offset, &eword);
				hw->phy.ops.write_reg(hw, phy_offset,
				                      MDIO_MMD_PMAPMD, eword);
				hw_dbg(hw, "Wrote %4.4x to %4.4x\n", eword,
				       phy_offset);
				data_offset++;
				phy_offset++;
			}
			break;
		case RNP_CONTROL_NL:
			data_offset++;
			hw_dbg(hw, "CONTROL:\n");
			if (edata == RNP_CONTROL_EOL_NL) {
				hw_dbg(hw, "EOL\n");
				end_data = true;
			} else if (edata == RNP_CONTROL_SOL_NL) {
				hw_dbg(hw, "SOL\n");
			} else {
				hw_dbg(hw, "Bad control value\n");
				ret_val = RNP_ERR_PHY;
				goto out;
			}
			break;
		default:
			hw_dbg(hw, "Bad control type\n");
			ret_val = RNP_ERR_PHY;
			goto out;
		}
	}

out:
#endif
	return ret_val;
}

/**
 *  rnp_identify_sfp_module_generic - Identifies SFP modules
 *  @hw: pointer to hardware structure
 *
 *  Searches for and identifies the SFP module and assigns appropriate PHY type.
 **/
s32 rnp_identify_sfp_module_generic(struct rnp_hw *hw)
{
#if 0
	struct rnp_adapter *adapter = hw->back;
	s32 status = RNP_ERR_PHY_ADDR_INVALID;
	u32 vendor_oui = 0;
	enum rnp_sfp_type stored_sfp_type = hw->phy.sfp_type;
	u8 identifier = 0;
	u8 comp_codes_1g = 0;
	u8 comp_codes_10g = 0;
	u8 oui_bytes[3] = {0, 0, 0};
	u8 cable_tech = 0;
	u8 cable_spec = 0;
	u16 enforce_sfp = 0;

	if (hw->mac.ops.get_media_type(hw) != rnp_media_type_fiber) {
		hw->phy.sfp_type = rnp_sfp_type_not_present;
		status = RNP_ERR_SFP_NOT_PRESENT;
		goto out;
	}

	status = hw->phy.ops.read_i2c_eeprom(hw,
					     RNP_SFF_IDENTIFIER,
					     &identifier);

	if (status != 0)
		goto err_read_i2c_eeprom;

	/* LAN ID is needed for sfp_type determination */
	hw->mac.ops.set_lan_id(hw);

	if (identifier != RNP_SFF_IDENTIFIER_SFP) {
		hw->phy.type = rnp_phy_sfp_unsupported;
		status = RNP_ERR_SFP_NOT_SUPPORTED;
	} else {
		status = hw->phy.ops.read_i2c_eeprom(hw,
						     RNP_SFF_1GBE_COMP_CODES,
						     &comp_codes_1g);

		if (status != 0)
			goto err_read_i2c_eeprom;

		status = hw->phy.ops.read_i2c_eeprom(hw,
						     RNP_SFF_10GBE_COMP_CODES,
						     &comp_codes_10g);

		if (status != 0)
			goto err_read_i2c_eeprom;
		status = hw->phy.ops.read_i2c_eeprom(hw,
						     RNP_SFF_CABLE_TECHNOLOGY,
						     &cable_tech);

		if (status != 0)
			goto err_read_i2c_eeprom;

		 /* ID Module
		  * =========
		  * 0   SFP_DA_CU
		  * 1   SFP_SR
		  * 2   SFP_LR
		  * 3   SFP_DA_CORE0 - n10-specific
		  * 4   SFP_DA_CORE1 - n10-specific
		  * 5   SFP_SR/LR_CORE0 - n10-specific
		  * 6   SFP_SR/LR_CORE1 - n10-specific
		  * 7   SFP_act_lmt_DA_CORE0 - n10-specific
		  * 8   SFP_act_lmt_DA_CORE1 - n10-specific
		  * 9   SFP_1g_cu_CORE0 - n10-specific
		  * 10  SFP_1g_cu_CORE1 - n10-specific
		  * 11  SFP_1g_sx_CORE0 - n10-specific
		  * 12  SFP_1g_sx_CORE1 - n10-specific
		  */
		if (hw->mac.type == rnp_mac_82598EB) {
			if (cable_tech & RNP_SFF_DA_PASSIVE_CABLE)
				hw->phy.sfp_type = rnp_sfp_type_da_cu;
			else if (comp_codes_10g & RNP_SFF_10GBASESR_CAPABLE)
				hw->phy.sfp_type = rnp_sfp_type_sr;
			else if (comp_codes_10g & RNP_SFF_10GBASELR_CAPABLE)
				hw->phy.sfp_type = rnp_sfp_type_lr;
			else
				hw->phy.sfp_type = rnp_sfp_type_unknown;
		} else if (hw->mac.type == rnp_mac_n10EB) {
			if (cable_tech & RNP_SFF_DA_PASSIVE_CABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
					             rnp_sfp_type_da_cu_core0;
				else
					hw->phy.sfp_type =
					             rnp_sfp_type_da_cu_core1;
			} else if (cable_tech & RNP_SFF_DA_ACTIVE_CABLE) {
				hw->phy.ops.read_i2c_eeprom(
						hw, RNP_SFF_CABLE_SPEC_COMP,
						&cable_spec);
				if (cable_spec &
				    RNP_SFF_DA_SPEC_ACTIVE_LIMITING) {
					if (hw->bus.lan_id == 0)
						hw->phy.sfp_type =
						rnp_sfp_type_da_act_lmt_core0;
					else
						hw->phy.sfp_type =
						rnp_sfp_type_da_act_lmt_core1;
				} else {
					hw->phy.sfp_type =
							rnp_sfp_type_unknown;
				}
			} else if (comp_codes_10g &
				   (RNP_SFF_10GBASESR_CAPABLE |
				    RNP_SFF_10GBASELR_CAPABLE)) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
					              rnp_sfp_type_srlr_core0;
				else
					hw->phy.sfp_type =
					              rnp_sfp_type_srlr_core1;
			} else if (comp_codes_1g & RNP_SFF_1GBASET_CAPABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						rnp_sfp_type_1g_cu_core0;
				else
					hw->phy.sfp_type =
						rnp_sfp_type_1g_cu_core1;
			} else if (comp_codes_1g & RNP_SFF_1GBASESX_CAPABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						rnp_sfp_type_1g_sx_core0;
				else
					hw->phy.sfp_type =
						rnp_sfp_type_1g_sx_core1;
			} else if (comp_codes_1g & RNP_SFF_1GBASELX_CAPABLE) {
				if (hw->bus.lan_id == 0)
					hw->phy.sfp_type =
						rnp_sfp_type_1g_lx_core0;
				else
					hw->phy.sfp_type =
						rnp_sfp_type_1g_lx_core1;
			} else {
				hw->phy.sfp_type = rnp_sfp_type_unknown;
			}
		}

		if (hw->phy.sfp_type != stored_sfp_type)
			hw->phy.sfp_setup_needed = true;

		/* Determine if the SFP+ PHY is dual speed or not. */
		hw->phy.multispeed_fiber = false;
		if (((comp_codes_1g & RNP_SFF_1GBASESX_CAPABLE) &&
		   (comp_codes_10g & RNP_SFF_10GBASESR_CAPABLE)) ||
		   ((comp_codes_1g & RNP_SFF_1GBASELX_CAPABLE) &&
		   (comp_codes_10g & RNP_SFF_10GBASELR_CAPABLE)))
			hw->phy.multispeed_fiber = true;

		/* Determine PHY vendor */
		if (hw->phy.type != rnp_phy_nl) {
			hw->phy.id = identifier;
			status = hw->phy.ops.read_i2c_eeprom(hw,
						    RNP_SFF_VENDOR_OUI_BYTE0,
						    &oui_bytes[0]);

			if (status != 0)
				goto err_read_i2c_eeprom;

			status = hw->phy.ops.read_i2c_eeprom(hw,
			                            RNP_SFF_VENDOR_OUI_BYTE1,
			                            &oui_bytes[1]);

			if (status != 0)
				goto err_read_i2c_eeprom;

			status = hw->phy.ops.read_i2c_eeprom(hw,
			                            RNP_SFF_VENDOR_OUI_BYTE2,
			                            &oui_bytes[2]);

			if (status != 0)
				goto err_read_i2c_eeprom;

			vendor_oui =
			  ((oui_bytes[0] << RNP_SFF_VENDOR_OUI_BYTE0_SHIFT) |
			   (oui_bytes[1] << RNP_SFF_VENDOR_OUI_BYTE1_SHIFT) |
			   (oui_bytes[2] << RNP_SFF_VENDOR_OUI_BYTE2_SHIFT));

			switch (vendor_oui) {
			case RNP_SFF_VENDOR_OUI_TYCO:
				if (cable_tech & RNP_SFF_DA_PASSIVE_CABLE)
					hw->phy.type =
						    rnp_phy_sfp_passive_tyco;
				break;
			case RNP_SFF_VENDOR_OUI_FTL:
				if (cable_tech & RNP_SFF_DA_ACTIVE_CABLE)
					hw->phy.type = rnp_phy_sfp_ftl_active;
				else
					hw->phy.type = rnp_phy_sfp_ftl;
				break;
			case RNP_SFF_VENDOR_OUI_AVAGO:
				hw->phy.type = rnp_phy_sfp_avago;
				break;
			case RNP_SFF_VENDOR_OUI_INTEL:
				hw->phy.type = rnp_phy_sfp_intel;
				break;
			default:
				if (cable_tech & RNP_SFF_DA_PASSIVE_CABLE)
					hw->phy.type =
						 rnp_phy_sfp_passive_unknown;
				else if (cable_tech & RNP_SFF_DA_ACTIVE_CABLE)
					hw->phy.type =
						rnp_phy_sfp_active_unknown;
				else
					hw->phy.type = rnp_phy_sfp_unknown;
				break;
			}
		}

		/* Allow any DA cable vendor */
		if (cable_tech & (RNP_SFF_DA_PASSIVE_CABLE |
		    RNP_SFF_DA_ACTIVE_CABLE)) {
			status = 0;
			goto out;
		}

		/* Verify supported 1G SFP modules */
		if (comp_codes_10g == 0 &&
		    !(hw->phy.sfp_type == rnp_sfp_type_1g_cu_core1 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_cu_core0 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_lx_core0 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_lx_core1 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_sx_core0 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_sx_core1)) {
			hw->phy.type = rnp_phy_sfp_unsupported;
			status = RNP_ERR_SFP_NOT_SUPPORTED;
			goto out;
		}

		/* Anything else 82598-based is supported */
		if (hw->mac.type == rnp_mac_82598EB) {
			status = 0;
			goto out;
		}

		hw->mac.ops.get_device_caps(hw, &enforce_sfp);
		if (!(enforce_sfp & RNP_DEVICE_CAPS_ALLOW_ANY_SFP) &&
		    !(hw->phy.sfp_type == rnp_sfp_type_1g_cu_core0 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_cu_core1 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_lx_core0 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_lx_core1 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_sx_core0 ||
		      hw->phy.sfp_type == rnp_sfp_type_1g_sx_core1)) {
			/* Make sure we're a supported PHY type */
			if (hw->phy.type == rnp_phy_sfp_intel) {
				status = 0;
			} else {
				if (hw->allow_unsupported_sfp) {
					e_warn(drv, "WARNING: Intel (R) Network Connections are quality tested using Intel (R) Ethernet Optics.  Using untested modules is not supported and may cause unstable operation or damage to the module or the adapter.  Intel Corporation is not responsible for any harm caused by using untested modules.");
					status = 0;
				} else {
					hw_dbg(hw,
					       "SFP+ module not supported\n");
					hw->phy.type =
						rnp_phy_sfp_unsupported;
					status = RNP_ERR_SFP_NOT_SUPPORTED;
				}
			}
		} else {
			status = 0;
		}
	}

out:
	return status;

err_read_i2c_eeprom:
	hw->phy.sfp_type = rnp_sfp_type_not_present;
	if (hw->phy.type != rnp_phy_nl) {
		hw->phy.id = 0;
		hw->phy.type = rnp_phy_unknown;
	}
#endif
	return RNP_ERR_SFP_NOT_PRESENT;
}

/**
 *  rnp_get_sfp_init_sequence_offsets - Provides offset of PHY init sequence
 *  @hw: pointer to hardware structure
 *  @list_offset: offset to the SFP ID list
 *  @data_offset: offset to the SFP data block
 *
 *  Checks the MAC's EEPROM to see if it supports a given SFP+ module type, if
 *  so it returns the offsets to the phy init sequence block.
 **/
s32 rnp_get_sfp_init_sequence_offsets(struct rnp_hw *hw, u16 *list_offset,
				      u16 *data_offset)
{
#if 0
	u16 sfp_id;
	u16 sfp_type = hw->phy.sfp_type;

	if (hw->phy.sfp_type == rnp_sfp_type_unknown)
		return RNP_ERR_SFP_NOT_SUPPORTED;

	if (hw->phy.sfp_type == rnp_sfp_type_not_present)
		return RNP_ERR_SFP_NOT_PRESENT;

	if ((hw->device_id == RNP_DEV_ID_82598_SR_DUAL_PORT_EM) &&
	    (hw->phy.sfp_type == rnp_sfp_type_da_cu))
		return RNP_ERR_SFP_NOT_SUPPORTED;

	/*
	 * Limiting active cables and 1G Phys must be initialized as
	 * SR modules
	 */
	if (sfp_type == rnp_sfp_type_da_act_lmt_core0 ||
	    sfp_type == rnp_sfp_type_1g_lx_core0 ||
	    sfp_type == rnp_sfp_type_1g_cu_core0 ||
	    sfp_type == rnp_sfp_type_1g_sx_core0)
		sfp_type = rnp_sfp_type_srlr_core0;
	else if (sfp_type == rnp_sfp_type_da_act_lmt_core1 ||
		 sfp_type == rnp_sfp_type_1g_lx_core1 ||
		 sfp_type == rnp_sfp_type_1g_cu_core1 ||
		 sfp_type == rnp_sfp_type_1g_sx_core1)
		sfp_type = rnp_sfp_type_srlr_core1;

	/* Read offset to PHY init contents */
	hw->eeprom.ops.read(hw, RNP_PHY_INIT_OFFSET_NL, list_offset);

	if ((!*list_offset) || (*list_offset == 0xFFFF))
		return RNP_ERR_SFP_NO_INIT_SEQ_PRESENT;

	/* Shift offset to first ID word */
	(*list_offset)++;

	/*
	 * Find the matching SFP ID in the EEPROM
	 * and program the init sequence
	 */
	hw->eeprom.ops.read(hw, *list_offset, &sfp_id);

	while (sfp_id != RNP_PHY_INIT_END_NL) {
		if (sfp_id == sfp_type) {
			(*list_offset)++;
			hw->eeprom.ops.read(hw, *list_offset, data_offset);
			if ((!*data_offset) || (*data_offset == 0xFFFF)) {
				hw_dbg(hw, "SFP+ module not supported\n");
				return RNP_ERR_SFP_NOT_SUPPORTED;
			} else {
				break;
			}
		} else {
			(*list_offset) += 2;
			if (hw->eeprom.ops.read(hw, *list_offset, &sfp_id))
				return RNP_ERR_PHY;
		}
	}

	if (sfp_id == RNP_PHY_INIT_END_NL) {
		hw_dbg(hw, "No matching SFP+ module found\n");
		return RNP_ERR_SFP_NOT_SUPPORTED;
	}
#endif
	return 0;
}

/**
 *  rnp_read_i2c_eeprom_generic - Reads 8 bit EEPROM word over I2C interface
 *  @hw: pointer to hardware structure
 *  @byte_offset: EEPROM byte offset to read
 *  @eeprom_data: value read
 *
 *  Performs byte read operation to SFP module's EEPROM over I2C interface.
 **/
s32 rnp_read_i2c_eeprom_generic(struct rnp_hw *hw, u8 byte_offset,
				u8 *eeprom_data)
{
	//return hw->phy.ops.read_i2c_byte(hw, byte_offset, RNP_I2C_EEPROM_DEV_ADDR, eeprom_data);
	return -EIO;
}

/**
 *  rnp_read_i2c_sff8472_generic - Reads 8 bit word over I2C interface
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset at address 0xA2
 *  @eeprom_data: value read
 *
 *  Performs byte read operation to SFP module's SFF-8472 data over I2C
 **/
s32 rnp_read_i2c_sff8472_generic(struct rnp_hw *hw, u8 byte_offset,
				 u8 *sff8472_data)
{
	//return hw->phy.ops.read_i2c_byte(hw, byte_offset, RNP_I2C_EEPROM_DEV_ADDR2, sff8472_data);
	return -EIO;
}

/**
 *  rnp_write_i2c_eeprom_generic - Writes 8 bit EEPROM word over I2C interface
 *  @hw: pointer to hardware structure
 *  @byte_offset: EEPROM byte offset to write
 *  @eeprom_data: value to write
 *
 *  Performs byte write operation to SFP module's EEPROM over I2C interface.
 **/
s32 rnp_write_i2c_eeprom_generic(struct rnp_hw *hw, u8 byte_offset,
				 u8 eeprom_data)
{
	//return hw->phy.ops.write_i2c_byte(hw, byte_offset, RNP_I2C_EEPROM_DEV_ADDR, eeprom_data);
	return -EIO;
}

/**
 *  rnp_read_i2c_byte_generic - Reads 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to read
 *  @data: value read
 *
 *  Performs byte read operation to SFP module's EEPROM over I2C interface at
 *  a specified device address.
 **/
s32 rnp_read_i2c_byte_generic(struct rnp_hw *hw, u8 byte_offset, u8 dev_addr,
			      u8 *data)
{
	s32 status = 0;
	u32 max_retry = 10;
	u32 retry = 0;
	u16 swfw_mask = 0;
	bool nack = true;
	*data = 0;
#if 0
	if (RNP_READ_REG(hw, RNP_STATUS) & RNP_STATUS_LAN_ID_1)
		swfw_mask = RNP_GSSR_PHY1_SM;
	else
		swfw_mask = RNP_GSSR_PHY0_SM;

	do {
		if (hw->mac.ops.acquire_swfw_sync(hw, swfw_mask) != 0) {
			status = RNP_ERR_SWFW_SYNC;
			goto read_byte_out;
		}

		rnp_i2c_start(hw);

		/* Device Address and write indication */
		status = rnp_clock_out_i2c_byte(hw, dev_addr);
		if (status != 0)
			goto fail;

		status = rnp_get_i2c_ack(hw);
		if (status != 0)
			goto fail;

		status = rnp_clock_out_i2c_byte(hw, byte_offset);
		if (status != 0)
			goto fail;

		status = rnp_get_i2c_ack(hw);
		if (status != 0)
			goto fail;

		rnp_i2c_start(hw);

		/* Device Address and read indication */
		status = rnp_clock_out_i2c_byte(hw, (dev_addr | 0x1));
		if (status != 0)
			goto fail;

		status = rnp_get_i2c_ack(hw);
		if (status != 0)
			goto fail;

		status = rnp_clock_in_i2c_byte(hw, data);
		if (status != 0)
			goto fail;

		status = rnp_clock_out_i2c_bit(hw, nack);
		if (status != 0)
			goto fail;

		rnp_i2c_stop(hw);
		break;

fail:
		rnp_i2c_bus_clear(hw);
		hw->mac.ops.release_swfw_sync(hw, swfw_mask);
		msleep(100);
		retry++;
		if (retry < max_retry)
			hw_dbg(hw, "I2C byte read error - Retrying.\n");
		else
			hw_dbg(hw, "I2C byte read error.\n");

	} while (retry < max_retry);

	hw->mac.ops.release_swfw_sync(hw, swfw_mask);

read_byte_out:
#endif
	return status;
}

/**
 *  rnp_write_i2c_byte_generic - Writes 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to write
 *  @data: value to write
 *
 *  Performs byte write operation to SFP module's EEPROM over I2C interface at
 *  a specified device address.
 **/
s32 rnp_write_i2c_byte_generic(struct rnp_hw *hw, u8 byte_offset, u8 dev_addr,
			       u8 data)
{
	s32 status = 0;
	u32 max_retry = 1;
	u32 retry = 0;
	u16 swfw_mask = 0;
#if 0
	if (RNP_READ_REG(hw, RNP_STATUS) & RNP_STATUS_LAN_ID_1)
		swfw_mask = RNP_GSSR_PHY1_SM;
	else
		swfw_mask = RNP_GSSR_PHY0_SM;

	if (hw->mac.ops.acquire_swfw_sync(hw, swfw_mask) != 0) {
		status = RNP_ERR_SWFW_SYNC;
		goto write_byte_out;
	}

	do {
		rnp_i2c_start(hw);

		status = rnp_clock_out_i2c_byte(hw, dev_addr);
		if (status != 0)
			goto fail;

		status = rnp_get_i2c_ack(hw);
		if (status != 0)
			goto fail;

		status = rnp_clock_out_i2c_byte(hw, byte_offset);
		if (status != 0)
			goto fail;

		status = rnp_get_i2c_ack(hw);
		if (status != 0)
			goto fail;

		status = rnp_clock_out_i2c_byte(hw, data);
		if (status != 0)
			goto fail;

		status = rnp_get_i2c_ack(hw);
		if (status != 0)
			goto fail;

		rnp_i2c_stop(hw);
		break;

fail:
		rnp_i2c_bus_clear(hw);
		retry++;
		if (retry < max_retry)
			hw_dbg(hw, "I2C byte write error - Retrying.\n");
		else
			hw_dbg(hw, "I2C byte write error.\n");
	} while (retry < max_retry);

	hw->mac.ops.release_swfw_sync(hw, swfw_mask);

write_byte_out:
#endif
	return status;
}

/**
 *  rnp_i2c_start - Sets I2C start condition
 *  @hw: pointer to hardware structure
 *
 *  Sets I2C start condition (High -> Low on SDA while SCL is High)
 **/
static void rnp_i2c_start(struct rnp_hw *hw)
{
#if 0
	u32 i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);

	/* Start condition must begin with data and clock high */
	rnp_set_i2c_data(hw, &i2cctl, 1);
	rnp_raise_i2c_clk(hw, &i2cctl);

	/* Setup time for start condition (4.7us) */
	udelay(RNP_I2C_T_SU_STA);

	rnp_set_i2c_data(hw, &i2cctl, 0);

	/* Hold time for start condition (4us) */
	udelay(RNP_I2C_T_HD_STA);

	rnp_lower_i2c_clk(hw, &i2cctl);

	/* Minimum low period of clock is 4.7 us */
	udelay(RNP_I2C_T_LOW);
#endif
}

/**
 *  rnp_i2c_stop - Sets I2C stop condition
 *  @hw: pointer to hardware structure
 *
 *  Sets I2C stop condition (Low -> High on SDA while SCL is High)
 **/
static void rnp_i2c_stop(struct rnp_hw *hw)
{
#if 0
	u32 i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);

	/* Stop condition must begin with data low and clock high */
	rnp_set_i2c_data(hw, &i2cctl, 0);
	rnp_raise_i2c_clk(hw, &i2cctl);

	/* Setup time for stop condition (4us) */
	udelay(RNP_I2C_T_SU_STO);

	rnp_set_i2c_data(hw, &i2cctl, 1);

	/* bus free time between stop and start (4.7us)*/
	udelay(RNP_I2C_T_BUF);
#endif
}

/**
 *  rnp_clock_in_i2c_byte - Clocks in one byte via I2C
 *  @hw: pointer to hardware structure
 *  @data: data byte to clock in
 *
 *  Clocks in one byte data via I2C data/clock
 **/
static s32 rnp_clock_in_i2c_byte(struct rnp_hw *hw, u8 *data)
{
	s32 i;
	bool bit = false;

	for (i = 7; i >= 0; i--) {
		rnp_clock_in_i2c_bit(hw, &bit);
		*data |= bit << i;
	}

	return 0;
}

/**
 *  rnp_clock_out_i2c_byte - Clocks out one byte via I2C
 *  @hw: pointer to hardware structure
 *  @data: data byte clocked out
 *
 *  Clocks out one byte data via I2C data/clock
 **/
static s32 rnp_clock_out_i2c_byte(struct rnp_hw *hw, u8 data)
{
	s32 status = 0;
	s32 i;
	u32 i2cctl;
	bool bit = false;
#if 0
	for (i = 7; i >= 0; i--) {
		bit = (data >> i) & 0x1;
		status = rnp_clock_out_i2c_bit(hw, bit);

		if (status != 0)
			break;
	}

	/* Release SDA line (set high) */
#endif
	return status;
}

/**
 *  rnp_get_i2c_ack - Polls for I2C ACK
 *  @hw: pointer to hardware structure
 *
 *  Clocks in/out one bit via I2C data/clock
 **/
static s32 rnp_get_i2c_ack(struct rnp_hw *hw)
{
	s32 status = 0;
#if 0
	u32 i = 0;
	u32 i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);
	u32 timeout = 10;
	bool ack = true;

	rnp_raise_i2c_clk(hw, &i2cctl);


	/* Minimum high period of clock is 4us */
	udelay(RNP_I2C_T_HIGH);

	/* Poll for ACK.  Note that ACK in I2C spec is
	 * transition from 1 to 0 */
	for (i = 0; i < timeout; i++) {
		i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);
		ack = rnp_get_i2c_data(&i2cctl);

		udelay(1);
		if (ack == 0)
			break;
	}

	if (ack == 1) {
		hw_dbg(hw, "I2C ack was not received.\n");
		status = RNP_ERR_I2C;
	}

	rnp_lower_i2c_clk(hw, &i2cctl);

	/* Minimum low period of clock is 4.7 us */
	udelay(RNP_I2C_T_LOW);
#endif
	return status;
}

/**
 *  rnp_clock_in_i2c_bit - Clocks in one bit via I2C data/clock
 *  @hw: pointer to hardware structure
 *  @data: read data value
 *
 *  Clocks in one bit via I2C data/clock
 **/
static s32 rnp_clock_in_i2c_bit(struct rnp_hw *hw, bool *data)
{
#if 0
	u32 i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);

	rnp_raise_i2c_clk(hw, &i2cctl);

	/* Minimum high period of clock is 4us */
	udelay(RNP_I2C_T_HIGH);

	i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);
	*data = rnp_get_i2c_data(&i2cctl);

	rnp_lower_i2c_clk(hw, &i2cctl);

	/* Minimum low period of clock is 4.7 us */
	udelay(RNP_I2C_T_LOW);
#endif
	return 0;
}

/**
 *  rnp_clock_out_i2c_bit - Clocks in/out one bit via I2C data/clock
 *  @hw: pointer to hardware structure
 *  @data: data value to write
 *
 *  Clocks out one bit via I2C data/clock
 **/
static s32 rnp_clock_out_i2c_bit(struct rnp_hw *hw, bool data)
{
	s32 status = 0;
#if 0
	u32 i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);

	status = rnp_set_i2c_data(hw, &i2cctl, data);
	if (status == 0) {
		rnp_raise_i2c_clk(hw, &i2cctl);

		/* Minimum high period of clock is 4us */
		udelay(RNP_I2C_T_HIGH);

		rnp_lower_i2c_clk(hw, &i2cctl);

		/* Minimum low period of clock is 4.7 us.
		 * This also takes care of the data hold time.
		 */
		udelay(RNP_I2C_T_LOW);
	} else {
		status = RNP_ERR_I2C;
		hw_dbg(hw, "I2C data was not set to %X\n", data);
	}
#endif
	return status;
}
/**
 *  rnp_raise_i2c_clk - Raises the I2C SCL clock
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *
 *  Raises the I2C clock line '0'->'1'
 **/
static void rnp_raise_i2c_clk(struct rnp_hw *hw, u32 *i2cctl)
{
	u32 i = 0;
#if 0
	u32 timeout = RNP_I2C_CLOCK_STRETCHING_TIMEOUT;
	u32 i2cctl_r = 0;

	for (i = 0; i < timeout; i++) {
		*i2cctl |= RNP_I2C_CLK_OUT;
		RNP_WRITE_REG(hw, RNP_I2CCTL, *i2cctl);
		RNP_WRITE_FLUSH(hw);
		/* SCL rise time (1000ns) */
		udelay(RNP_I2C_T_RISE);

		i2cctl_r = RNP_READ_REG(hw, RNP_I2CCTL);
		if (i2cctl_r & RNP_I2C_CLK_IN)
			break;
	}
#endif
}

/**
 *  rnp_lower_i2c_clk - Lowers the I2C SCL clock
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *
 *  Lowers the I2C clock line '1'->'0'
 **/
static void rnp_lower_i2c_clk(struct rnp_hw *hw, u32 *i2cctl)
{
#if 0
	*i2cctl &= ~RNP_I2C_CLK_OUT;

	RNP_WRITE_REG(hw, RNP_I2CCTL, *i2cctl);
	RNP_WRITE_FLUSH(hw);

	/* SCL fall time (300ns) */
	udelay(RNP_I2C_T_FALL);
#endif
}

/**
 *  rnp_set_i2c_data - Sets the I2C data bit
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *  @data: I2C data value (0 or 1) to set
 *
 *  Sets the I2C data bit
 **/
static s32 rnp_set_i2c_data(struct rnp_hw *hw, u32 *i2cctl, bool data)
{
	s32 status = 0;
#if 0
	if (data)
		*i2cctl |= RNP_I2C_DATA_OUT;
	else
		*i2cctl &= ~RNP_I2C_DATA_OUT;

	RNP_WRITE_REG(hw, RNP_I2CCTL, *i2cctl);
	RNP_WRITE_FLUSH(hw);

	/* Data rise/fall (1000ns/300ns) and set-up time (250ns) */
	udelay(RNP_I2C_T_RISE + RNP_I2C_T_FALL + RNP_I2C_T_SU_DATA);

	/* Verify data was set correctly */
	*i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);
	if (data != rnp_get_i2c_data(i2cctl)) {
		status = RNP_ERR_I2C;
		hw_dbg(hw, "Error - I2C data was not set to %X.\n", data);
	}
#endif
	return status;
}

/**
 *  rnp_get_i2c_data - Reads the I2C SDA data bit
 *  @hw: pointer to hardware structure
 *  @i2cctl: Current value of I2CCTL register
 *
 *  Returns the I2C data bit value
 **/
static bool rnp_get_i2c_data(u32 *i2cctl)
{
	bool data = false;
#if 0
	if (*i2cctl & RNP_I2C_DATA_IN)
		data = true;
	else
		data = false;
#endif
	return data;
}

/**
 *  rnp_i2c_bus_clear - Clears the I2C bus
 *  @hw: pointer to hardware structure
 *
 *  Clears the I2C bus by sending nine clock pulses.
 *  Used when data line is stuck low.
 **/
static void rnp_i2c_bus_clear(struct rnp_hw *hw)
{
#if 0
	u32 i2cctl = RNP_READ_REG(hw, RNP_I2CCTL);
	u32 i;

	rnp_i2c_start(hw);

	rnp_set_i2c_data(hw, &i2cctl, 1);

	for (i = 0; i < 9; i++) {
		rnp_raise_i2c_clk(hw, &i2cctl);

		/* Min high period of clock is 4us */
		udelay(RNP_I2C_T_HIGH);

		rnp_lower_i2c_clk(hw, &i2cctl);

		/* Min low period of clock is 4.7us*/
		udelay(RNP_I2C_T_LOW);
	}

	rnp_i2c_start(hw);

	/* Put the i2c bus back to default state */
	rnp_i2c_stop(hw);
#endif
}

/**
 *  rnp_tn_check_overtemp - Checks if an overtemp occurred.
 *  @hw: pointer to hardware structure
 *
 *  Checks if the LASI temp alarm status was triggered due to overtemp
 **/
s32 rnp_tn_check_overtemp(struct rnp_hw *hw)
{
	s32 status = 0;
	u16 phy_data = 0;
#if 0

	if (hw->device_id != RNP_DEV_ID_n10_T3_LOM)
		goto out;

	/* Check that the LASI temp alarm status was triggered */
	hw->phy.ops.read_reg(hw, RNP_TN_LASI_STATUS_REG,
	                     MDIO_MMD_PMAPMD, &phy_data);

	if (!(phy_data & RNP_TN_LASI_STATUS_TEMP_ALARM))
		goto out;

	status = RNP_ERR_OVERTEMP;
out:
#endif
	return status;
}
