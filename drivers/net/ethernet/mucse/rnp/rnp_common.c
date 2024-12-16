#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/netdevice.h>

#include "rnp.h"
#include "rnp_common.h"
#include "rnp_phy.h"
#include "rnp_mbx.h"

static s32 rnp_acquire_eeprom(struct rnp_hw *hw);
static s32 rnp_get_eeprom_semaphore(struct rnp_hw *hw);
static void rnp_release_eeprom_semaphore(struct rnp_hw *hw);
static s32 rnp_ready_eeprom(struct rnp_hw *hw);
static void rnp_standby_eeprom(struct rnp_hw *hw);
static void rnp_shift_out_eeprom_bits(struct rnp_hw *hw, u16 data, u16 count);
static u16 rnp_shift_in_eeprom_bits(struct rnp_hw *hw, u16 count);
static void rnp_raise_eeprom_clk(struct rnp_hw *hw, u32 *eec);
static void rnp_lower_eeprom_clk(struct rnp_hw *hw, u32 *eec);
static void rnp_release_eeprom(struct rnp_hw *hw);

static s32 rnp_mta_vector(struct rnp_hw *hw, u8 *mc_addr);
static s32 rnp_poll_eerd_eewr_done(struct rnp_hw *hw, u32 ee_reg);
static s32 rnp_read_eeprom_buffer_bit_bang(struct rnp_hw *hw,
										   u16 offset,
										   u16 words,
										   u16 *data);
static s32 rnp_write_eeprom_buffer_bit_bang(struct rnp_hw *hw,
											u16 offset,
											u16 words,
											u16 *data);
static s32 rnp_detect_eeprom_page_size_generic(struct rnp_hw *hw, u16 offset);
static s32 rnp_disable_pcie_master(struct rnp_hw *hw);

unsigned int rnp_loglevel = 0;
module_param(rnp_loglevel, uint, S_IRUSR | S_IWUSR);

/**
 *  rnp_device_supports_autoneg_fc - Check if phy supports autoneg flow
 *  control
 *  @hw: pointer to hardware structure
 *
 *  There are several phys that do not support autoneg flow control. This
 *  function check the device id to see if the associated phy supports
 *  autoneg flow control.
 **/
s32 rnp_device_supports_autoneg_fc(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_setup_fc - Set up flow control
 *  @hw: pointer to hardware structure
 *
 *  Called at init time to set up flow control.
 **/
static s32 rnp_setup_fc(struct rnp_hw *hw)
{
	s32 ret_val = 0;
	u32 reg = 0, reg_bp = 0;
	u16 reg_cu = 0;
	bool got_lock = false;
#if 0

	/*
	 * Validate the requested mode.  Strict IEEE mode does not allow
	 * rnp_fc_rx_pause because it will cause us to fail at UNH.
	 */
	if (hw->fc.strict_ieee && hw->fc.requested_mode == rnp_fc_rx_pause) {
		hw_dbg(hw, "rnp_fc_rx_pause not valid in strict IEEE mode\n");
		ret_val = RNP_ERR_INVALID_LINK_SETTINGS;
		goto out;
	}

	/*
	 * 10gig parts do not have a word in the EEPROM to determine the
	 * default flow control setting, so we explicitly set it to full.
	 */
	if (hw->fc.requested_mode == rnp_fc_default)
		hw->fc.requested_mode = rnp_fc_full;

	/*
	 * Set up the 1G and 10G flow control advertisement registers so the
	 * HW will be able to do fc autoneg once the cable is plugged in.  If
	 * we link at 10G, the 1G advertisement is harmless and vice versa.
	 */
	switch (hw->phy.media_type) {
	case rnp_media_type_fiber:
	case rnp_media_type_backplane:
		reg = RNP_READ_REG(hw, RNP_PCS1GANA);
		reg_bp = RNP_READ_REG(hw, RNP_AUTOC);
		break;
	case rnp_media_type_copper:
		hw->phy.ops.read_reg(hw, MDIO_AN_ADVERTISE,
					MDIO_MMD_AN, &reg_cu);
		break;
	default:
		break;
	}

	/*
	 * The possible values of fc.requested_mode are:
	 * 0: Flow control is completely disabled
	 * 1: Rx flow control is enabled (we can receive pause frames,
	 *    but not send pause frames).
	 * 2: Tx flow control is enabled (we can send pause frames but
	 *    we do not support receiving pause frames).
	 * 3: Both Rx and Tx flow control (symmetric) are enabled.
	 * other: Invalid.
	 */
	switch (hw->fc.requested_mode) {
	case rnp_fc_none:
		/* Flow control completely disabled by software override. */
		reg &= ~(RNP_PCS1GANA_SYM_PAUSE | RNP_PCS1GANA_ASM_PAUSE);
		if (hw->phy.media_type == rnp_media_type_backplane)
			reg_bp &= ~(RNP_AUTOC_SYM_PAUSE |
				    RNP_AUTOC_ASM_PAUSE);
		else if (hw->phy.media_type == rnp_media_type_copper)
			reg_cu &= ~(RNP_TAF_SYM_PAUSE | RNP_TAF_ASM_PAUSE);
		break;
	case rnp_fc_tx_pause:
		/*
		 * Tx Flow control is enabled, and Rx Flow control is
		 * disabled by software override.
		 */
		reg |= RNP_PCS1GANA_ASM_PAUSE;
		reg &= ~RNP_PCS1GANA_SYM_PAUSE;
		if (hw->phy.media_type == rnp_media_type_backplane) {
			reg_bp |= RNP_AUTOC_ASM_PAUSE;
			reg_bp &= ~RNP_AUTOC_SYM_PAUSE;
		} else if (hw->phy.media_type == rnp_media_type_copper) {
			reg_cu |= RNP_TAF_ASM_PAUSE;
			reg_cu &= ~RNP_TAF_SYM_PAUSE;
		}
		break;
	case rnp_fc_rx_pause:
		/*
		 * Rx Flow control is enabled and Tx Flow control is
		 * disabled by software override. Since there really
		 * isn't a way to advertise that we are capable of RX
		 * Pause ONLY, we will advertise that we support both
		 * symmetric and asymmetric Rx PAUSE, as such we fall
		 * through to the fc_full statement.  Later, we will
		 * disable the adapter's ability to send PAUSE frames.
		 */
	case rnp_fc_full:
		/* Flow control (both Rx and Tx) is enabled by SW override. */
		reg |= RNP_PCS1GANA_SYM_PAUSE | RNP_PCS1GANA_ASM_PAUSE;
		if (hw->phy.media_type == rnp_media_type_backplane)
			reg_bp |= RNP_AUTOC_SYM_PAUSE |
				  RNP_AUTOC_ASM_PAUSE;
		else if (hw->phy.media_type == rnp_media_type_copper)
			reg_cu |= RNP_TAF_SYM_PAUSE | RNP_TAF_ASM_PAUSE;
		break;
	default:
		hw_dbg(hw, "Flow control param set incorrectly\n");
		ret_val = RNP_ERR_CONFIG;
		goto out;
	}

	if (hw->mac.type != rnp_mac_X540) {
		/*
		 * Enable auto-negotiation between the MAC & PHY;
		 * the MAC will advertise clause 37 flow control.
		 */
		RNP_WRITE_REG(hw, RNP_PCS1GANA, reg);
		reg = RNP_READ_REG(hw, RNP_PCS1GLCTL);

		/* Disable AN timeout */
		if (hw->fc.strict_ieee)
			reg &= ~RNP_PCS1GLCTL_AN_1G_TIMEOUT_EN;

		RNP_WRITE_REG(hw, RNP_PCS1GLCTL, reg);
		hw_dbg(hw, "Set up FC; PCS1GLCTL = 0x%08X\n", reg);
	}

	/*
	 * AUTOC restart handles negotiation of 1G and 10G on backplane
	 * and copper. There is no need to set the PCS1GCTL register.
	 *
	 */
	if (hw->phy.media_type == rnp_media_type_backplane) {
		/* Need the SW/FW semaphore around AUTOC writes if n10 and
		 * LESM is on, likewise reset_pipeline requries the lock as
		 * it also writes AUTOC.
		 */
		if ((hw->mac.type == rnp_mac_n10EB) &&
		    rnp_verify_lesm_fw_enabled_n10(hw)) {
			ret_val = hw->mac.ops.acquire_swfw_sync(hw,
							RNP_GSSR_MAC_CSR_SM);
			if (ret_val)
				goto out;

			got_lock = true;
		}

		RNP_WRITE_REG(hw, RNP_AUTOC, reg_bp);

		if (hw->mac.type == rnp_mac_n10EB)
			rnp_reset_pipeline_n10(hw);

		if (got_lock)
			hw->mac.ops.release_swfw_sync(hw,
						      RNP_GSSR_MAC_CSR_SM);

	} else if ((hw->phy.media_type == rnp_media_type_copper) &&
		    (rnp_device_supports_autoneg_fc(hw) == 0)) {
		hw->phy.ops.write_reg(hw, MDIO_AN_ADVERTISE,
				      MDIO_MMD_AN, reg_cu);
	}

	hw_dbg(hw, "Set up FC; RNP_AUTOC = 0x%08X\n", reg);
out:
#endif
	return ret_val;
}

/**
 *  rnp_start_hw_generic - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware by filling the bus info structure and media type, clears
 *  all on chip counters, initializes receive address registers, multicast
 *  table, VLAN filter table, calls routine to set up link and flow control
 *  settings, and leaves transmit and receive units disabled and uninitialized
 **/
s32 rnp_start_hw_generic(struct rnp_hw *hw)
{
	//u32 ctrl_ext;

#ifdef U//V3P_1PF
	//return 0;
#endif

	///* Set the media type */
	////hw->phy.media_type = hw->mac.ops.get_media_type(hw);
	//hw->phy.media_type = rnp_media_type_fiber;

	///* Identify the PHY */
	////hw->phy.ops.identify(hw);
	//hw->phy.type = rnp_phy_sfp;

	///* Clear the VLAN filter table */
	////hw->mac.ops.clear_vfta(hw);
	//hw->ops.clr_vfta(hw);

	///* Clear statistics registers */
	////hw->mac.ops.clear_hw_cntrs(hw);

	///* Setup flow control */
	////rnp_setup_fc(hw);

	///* Clear adapter stopped flag */
	//hw->adapter_stopped = false;

	return 0;
}

/**
 *  rnp_start_hw_gen2 - Init sequence for common device family
 *  @hw: pointer to hw structure
 *
 * Performs the init sequence common to the second generation
 * of 10 GbE devices.
 * Devices in the second generation:
 *     n10
 *     X540
 **/
s32 rnp_start_hw_gen2(struct rnp_hw *hw)
{
	u32 i;
	u32 regval;

	/* Clear the rate limiters */
#if 0
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		;
		;
	}

	/* Disable relaxed ordering */
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		;
		;
	}

	for (i = 0; i < hw->mac.max_rx_queues; i++) {
		;
		;
	}
#endif
	return 0;
}

/**
 *  rnp_init_hw_generic - Generic hardware initialization
 *  @hw: pointer to hardware structure
 *
 *  Initialize the hardware by resetting the hardware, filling the bus info
 *  structure and media type, clears all on chip counters, initializes receive
 *  address registers, multicast table, VLAN filter table, calls routine to set
 *  up link and flow control settings, and leaves transmit and receive units
 *  disabled and uninitialized
 **/
s32 rnp_init_hw_generic(struct rnp_hw *hw)
{
//	s32 status;
//
//	/* Reset the hardware */
//	status = hw->mac.ops.reset_hw(hw);
//
//	if (status == 0) {
//		/* Start the HW */
//		status = hw->mac.ops.start_hw(hw);
//	}

	//return status;
	return 0;
}

void rnp_reset_msix_table_generic(struct rnp_hw *hw)
{
	int i;

	// reset NIC_RING_VECTOR table to 0
	for (i = 0; i < 128; i++)
		rnp_wr_reg(hw->ring_msix_base + RING_VECTOR(i), 0);
}

/**
 *  rnp_clear_hw_cntrs_generic - Generic clear hardware counters
 *  @hw: pointer to hardware structure
 *
 *  Clears all hardware statistics counters by reading them from the hardware
 *  Statistics counters are clear on read.
 **/
s32 rnp_clear_hw_cntrs_generic(struct rnp_hw *hw)
{
	u16 i = 0;

#if 0
	if (hw->mac.type == rnp_mac_X540) {
		if (hw->phy.id == 0)
			hw->phy.ops.identify(hw);
		hw->phy.ops.read_reg(hw, RNP_PCRC8ECL, MDIO_MMD_PCS, &i);
		hw->phy.ops.read_reg(hw, RNP_PCRC8ECH, MDIO_MMD_PCS, &i);
		hw->phy.ops.read_reg(hw, RNP_LDPCECL, MDIO_MMD_PCS, &i);
		hw->phy.ops.read_reg(hw, RNP_LDPCECH, MDIO_MMD_PCS, &i);
	}
#endif
	return 0;
}

/**
 *  rnp_read_pba_string_generic - Reads part number string from EEPROM
 *  @hw: pointer to hardware structure
 *  @pba_num: stores the part number string from the EEPROM
 *  @pba_num_size: part number string buffer length
 *
 *  Reads the part number string from the EEPROM.
 **/
s32 rnp_read_pba_string_generic(struct rnp_hw *hw,
								u8 *pba_num,
								u32 pba_num_size)
{
	return 0;
}

s32 rnp_get_permtion_mac_addr(struct rnp_hw *hw, u8 *mac_addr)
{
	u32 v;

#ifdef NO_MBX_VERSION
	TRACE();
#ifdef FIX_MAC_TEST
	v = 0x00004E46;
#else
	v = rd32(hw, RNP_TOP_MAC_OUI);
#endif
	mac_addr[0] = (u8)(v >> 16);
	mac_addr[1] = (u8)(v >> 8);
	mac_addr[2] = (u8)(v >> 0);

#ifdef FIX_MAC_TEST
	v = 0x00032F00 + rnp_is_pf1(hw->pdev);
#else
	v = rd32(hw, RNP_TOP_MAC_SN);
#endif
	mac_addr[3] = (u8)(v >> 16);
	mac_addr[4] = (u8)(v >> 8);
	mac_addr[5] = (u8)(v >> 0);
#else
	if (rnp_fw_get_macaddr(hw, hw->pfvfnum, mac_addr, hw->nr_lane) ||
		!is_valid_ether_addr(mac_addr)) {
		dbg("generate ramdom macaddress...\n");
		eth_random_addr(mac_addr);
	}
#endif

	hw->mac.mac_flags |= RNP_FLAGS_INIT_MAC_ADDRESS;
	dbg("%s mac:%pM\n", __func__, mac_addr);

	return 0;
}

/**
 *  rnp_get_mac_addr_generic - Generic get MAC address
 *  @hw: pointer to hardware structure
 *  @mac_addr: Adapter MAC address
 *
 *  Reads the adapter's MAC address from first Receive Address Register (RAR0)
 *  A reset of the adapter must be performed prior to calling this function
 *  in order for the MAC address to have been loaded from the EEPROM into RAR0
 **/
s32 rnp_get_mac_addr_generic(struct rnp_hw *hw, u8 *mac_addr)
{
	u32 rar_high, rar_low, i;

	rar_high = rd32(hw, RNP_ETH_RAR_RH(0));
	rar_low = rd32(hw, RNP_ETH_RAR_RL(0));

	for (i = 0; i < 4; i++)
		mac_addr[i] = (u8)(rar_low >> (i * 8));

	for (i = 0; i < 2; i++)
		mac_addr[i + 4] = (u8)(rar_high >> (i * 8));

	return 0;
}

/**
 *  rnp_stop_adapter_generic - Generic stop Tx/Rx units
 *  @hw: pointer to hardware structure
 *
 *  Sets the adapter_stopped flag within rnp_hw struct. Clears interrupts,
 *  disables transmit and receive units. The adapter_stopped flag is used by
 *  the shared code and drivers to determine if the adapter is in a stopped
 *  state and should not touch the hardware.
 **/
s32 rnp_stop_adapter_generic(struct rnp_hw *hw)
{
	u32 reg_val;
	u16 i;
	struct rnp_dma_info *dma = &hw->dma;

	/*
	 * Set the adapter_stopped flag so other driver functions stop touching
	 * the hardware
	 */
	hw->adapter_stopped = true;

	/* Disable the receive unit */

	/* Clear any pending interrupts, flush previous writes */

	/* Disable the transmit unit.  Each queue must be disabled. */
	for (i = 0; i < hw->mac.max_tx_queues; i++) {
		/* Clear interrupt mask to stop interrupts
		 * from being generated
		 */
		dma_ring_wr32(dma, RING_OFFSET(i) + RNP_DMA_INT_CLR, 0x3);

		dma_ring_wr32(dma, RING_OFFSET(i) + RNP_DMA_TX_START, 0);
	}

	/* Disable the receive unit by stopping each queue */
	for (i = 0; i < hw->mac.max_rx_queues; i++)
		dma_ring_wr32(dma, RING_OFFSET(i) + RNP_DMA_RX_START, 0);

	/* flush all queues disables */
	usleep_range(1000, 2000);

	/*
	 * Prevent the PCI-E bus from from hanging by disabling PCI-E master
	 * access and verify no pending requests
	 */
	return rnp_disable_pcie_master(hw);
}

/**
 *  rnp_led_on_generic - Turns on the software controllable LEDs.
 *  @hw: pointer to hardware structure
 *  @index: led number to turn on
 **/
s32 rnp_led_on_generic(struct rnp_hw *hw, u32 index)
{
	/* To turn on the LED, set mode to ON. */

	return 0;
}

/**
 *  rnp_led_off_generic - Turns off the software controllable LEDs.
 *  @hw: pointer to hardware structure
 *  @index: led number to turn off
 **/
s32 rnp_led_off_generic(struct rnp_hw *hw, u32 index)
{
	/* To turn off the LED, set mode to OFF. */

	return 0;
}

/**
 *  rnp_init_eeprom_params_generic - Initialize EEPROM params
 *  @hw: pointer to hardware structure
 *
 *  Initializes the EEPROM parameters rnp_eeprom_info within the
 *  rnp_hw struct in order to set up EEPROM access.
 **/
s32 rnp_init_eeprom_params_generic(struct rnp_hw *hw)
{
	struct rnp_eeprom_info *eeprom = &hw->eeprom;
	u32 eec;
	u16 eeprom_size;

	return 0;
}

/**
 *  rnp_write_eeprom_buffer_bit_bang_generic - Write EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to write
 *  @words: number of words
 *  @data: 16 bit word(s) to write to EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
s32 rnp_write_eeprom_buffer_bit_bang_generic(struct rnp_hw *hw,
											 u16 offset,
											 u16 words,
											 u16 *data)
{
	return -EINVAL;
}

/**
 *  rnp_write_eeprom_buffer_bit_bang - Writes 16 bit word(s) to EEPROM
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be written to
 *  @words: number of word(s)
 *  @data: 16 bit word(s) to be written to the EEPROM
 *
 *  If rnp_eeprom_update_checksum is not called after this function, the
 *  EEPROM will most likely contain an invalid checksum.
 **/
static s32 rnp_write_eeprom_buffer_bit_bang(struct rnp_hw *hw,
											u16 offset,
											u16 words,
											u16 *data)
{
	return -EINVAL;
}

/**
 *  rnp_write_eeprom_generic - Writes 16 bit value to EEPROM
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be written to
 *  @data: 16 bit word to be written to the EEPROM
 *
 *  If rnp_eeprom_update_checksum is not called after this function, the
 *  EEPROM will most likely contain an invalid checksum.
 **/
s32 rnp_write_eeprom_generic(struct rnp_hw *hw, u16 offset, u16 data)
{
	s32 status;

	hw->eeprom.ops.init_params(hw);

	if (offset >= hw->eeprom.word_size) {
		status = RNP_ERR_EEPROM;
		goto out;
	}

	status = rnp_write_eeprom_buffer_bit_bang(hw, offset, 1, &data);

out:
	return status;
}

/**
 *  rnp_read_eeprom_buffer_bit_bang_generic - Read EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @words: number of word(s)
 *  @data: read 16 bit words(s) from EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
s32 rnp_read_eeprom_buffer_bit_bang_generic(struct rnp_hw *hw,
											u16 offset,
											u16 words,
											u16 *data)
{
	return -EINVAL;
}

/**
 *  rnp_read_eeprom_buffer_bit_bang - Read EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @words: number of word(s)
 *  @data: read 16 bit word(s) from EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
static s32 rnp_read_eeprom_buffer_bit_bang(struct rnp_hw *hw,
										   u16 offset,
										   u16 words,
										   u16 *data)
{
	return -EINVAL;
}

/**
 *  rnp_read_eeprom_bit_bang_generic - Read EEPROM word using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @data: read 16 bit value from EEPROM
 *
 *  Reads 16 bit value from EEPROM through bit-bang method
 **/
s32 rnp_read_eeprom_bit_bang_generic(struct rnp_hw *hw, u16 offset, u16 *data)
{
	s32 status;

	hw->eeprom.ops.init_params(hw);

	if (offset >= hw->eeprom.word_size) {
		status = RNP_ERR_EEPROM;
		goto out;
	}

	status = rnp_read_eeprom_buffer_bit_bang(hw, offset, 1, data);

out:
	return status;
}

/**
 *  rnp_read_eerd_buffer_generic - Read EEPROM word(s) using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of word in the EEPROM to read
 *  @words: number of word(s)
 *  @data: 16 bit word(s) from the EEPROM
 *
 *  Reads a 16 bit word(s) from the EEPROM using the EERD register.
 **/
s32 rnp_read_eerd_buffer_generic(struct rnp_hw *hw,
								 u16 offset,
								 u16 words,
								 u16 *data)
{
	return -EINVAL;
}

/**
 *  rnp_detect_eeprom_page_size_generic - Detect EEPROM page size
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be used as a scratch pad
 *
 *  Discover EEPROM page size by writing marching data at given offset.
 *  This function is called only when we are writing a new large buffer
 *  at given offset so the data would be overwritten anyway.
 **/
static s32 rnp_detect_eeprom_page_size_generic(struct rnp_hw *hw, u16 offset)
{
	return -EINVAL;
}

/**
 *  rnp_read_eerd_generic - Read EEPROM word using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the EERD register.
 **/
s32 rnp_read_eerd_generic(struct rnp_hw *hw, u16 offset, u16 *data)
{
	return rnp_read_eerd_buffer_generic(hw, offset, 1, data);
}

/**
 *  rnp_write_eewr_buffer_generic - Write EEPROM word(s) using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @words: number of words
 *  @data: word(s) write to the EEPROM
 *
 *  Write a 16 bit word(s) to the EEPROM using the EEWR register.
 **/
s32 rnp_write_eewr_buffer_generic(struct rnp_hw *hw,
								  u16 offset,
								  u16 words,
								  u16 *data)
{
	return -EINVAL;
}

/**
 *  rnp_write_eewr_generic - Write EEPROM word using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @data: word write to the EEPROM
 *
 *  Write a 16 bit word to the EEPROM using the EEWR register.
 **/
s32 rnp_write_eewr_generic(struct rnp_hw *hw, u16 offset, u16 data)
{
	return rnp_write_eewr_buffer_generic(hw, offset, 1, &data);
}

/**
 *  rnp_poll_eerd_eewr_done - Poll EERD read or EEWR write status
 *  @hw: pointer to hardware structure
 *  @ee_reg: EEPROM flag for polling
 *
 *  Polls the status bit (bit 1) of the EERD or EEWR to determine when the
 *  read or write is done respectively.
 **/
static s32 rnp_poll_eerd_eewr_done(struct rnp_hw *hw, u32 ee_reg)
{
	return -EINVAL;
}

/**
 *  rnp_acquire_eeprom - Acquire EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *
 *  Prepares EEPROM for access using bit-bang method. This function should
 *  be called before issuing a command to the EEPROM.
 **/
static s32 rnp_acquire_eeprom(struct rnp_hw *hw)
{
	s32 status = 0;

	return status;
}

/**
 *  rnp_get_eeprom_semaphore - Get hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  Sets the hardware semaphores so EEPROM access can occur for bit-bang method
 **/
static s32 rnp_get_eeprom_semaphore(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_release_eeprom_semaphore - Release hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  This function clears hardware semaphore bits.
 **/
static void rnp_release_eeprom_semaphore(struct rnp_hw *hw)
{
}

/**
 *  rnp_ready_eeprom - Polls for EEPROM ready
 *  @hw: pointer to hardware structure
 **/
static s32 rnp_ready_eeprom(struct rnp_hw *hw)
{
	return -EINVAL;
}

/**
 *  rnp_standby_eeprom - Returns EEPROM to a "standby" state
 *  @hw: pointer to hardware structure
 **/
static void rnp_standby_eeprom(struct rnp_hw *hw)
{
}

/**
 *  rnp_shift_out_eeprom_bits - Shift data bits out to the EEPROM.
 *  @hw: pointer to hardware structure
 *  @data: data to send to the EEPROM
 *  @count: number of bits to shift out
 **/
static void rnp_shift_out_eeprom_bits(struct rnp_hw *hw, u16 data, u16 count)
{
}

/**
 *  rnp_shift_in_eeprom_bits - Shift data bits in from the EEPROM
 *  @hw: pointer to hardware structure
 **/
static u16 rnp_shift_in_eeprom_bits(struct rnp_hw *hw, u16 count)
{
	u32 eec;
	u32 i;
	u16 data = 0;

	return 0;
}

/**
 *  rnp_raise_eeprom_clk - Raises the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eec: EEC register's current value
 **/
static void rnp_raise_eeprom_clk(struct rnp_hw *hw, u32 *eec)
{
}

/**
 *  rnp_lower_eeprom_clk - Lowers the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eecd: EECD's current value
 **/
static void rnp_lower_eeprom_clk(struct rnp_hw *hw, u32 *eec)
{
}

/**
 *  rnp_release_eeprom - Release EEPROM, release semaphores
 *  @hw: pointer to hardware structure
 **/
static void rnp_release_eeprom(struct rnp_hw *hw)
{
}

/**
 *  rnp_calc_eeprom_checksum_generic - Calculates and returns the checksum
 *  @hw: pointer to hardware structure
 **/
u16 rnp_calc_eeprom_checksum_generic(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_validate_eeprom_checksum_generic - Validate EEPROM checksum
 *  @hw: pointer to hardware structure
 *  @checksum_val: calculated checksum
 *
 *  Performs checksum calculation and validates the EEPROM checksum.  If the
 *  caller does not need checksum_val, the value can be NULL.
 **/
s32 rnp_validate_eeprom_checksum_generic(struct rnp_hw *hw, u16 *checksum_val)
{
	return 0;
}

/**
 *  rnp_update_eeprom_checksum_generic - Updates the EEPROM checksum
 *  @hw: pointer to hardware structure
 **/
s32 rnp_update_eeprom_checksum_generic(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_set_rar_generic - Set Rx address register
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *  @addr: Address to put into receive address register
 *  @vmdq: VMDq "set" or "pool" index
 *  @enable_addr: set flag that address is active
 *
 *  Puts an ethernet address into a receive address register.
 **/
s32 rnp_set_rar_generic(
	struct rnp_hw *hw, u32 index, u8 *addr, u32 vmdq, u32 enable_addr)
{
//	u32 mcstctrl;
//	u32 rar_low, rar_high = 0;
//	u32 rar_entries = hw->mac.num_rar_entries;
//	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
//
//	// dump_stack();
//
//	/* Make sure we are using a valid rar index range */
//	if (index >= rar_entries) {
//		rnp_err("RAR index %d is out of range.\n", index);
//		return RNP_ERR_INVALID_ARGUMENT;
//	}
//
//	hw_dbg(hw,
//		   "    RAR[%d] <= %pM.  vmdq:%d enable:0x%x\n",
//		   index,
//		   addr,
//		   vmdq,
//		   enable_addr);
//
//	/* setup VMDq pool selection before this RAR gets enabled */
//	/* only sriov mode use this */
//	// FIXME
//	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
//		hw->mac.ops.set_vmdq(hw, index, vmdq);
//
//	/*
//	 * HW expects these in big endian so we reverse the byte
//	 * order from network order (big endian) to little endian
//	 */
//	rar_low = ((u32)addr[5] | ((u32)addr[4] << 8) | ((u32)addr[3] << 16) |
//			   ((u32)addr[2] << 24));
//	/*
//	 * Some parts put the VMDq setting in the extra RAH bits,
//	 * so save everything except the lower 16 bits that hold part
//	 * of the address and the address valid bit.
//	 */
//	rar_high = rd32(hw, RNP_ETH_RAR_RH(index));
//	rar_high &= ~(0x0000FFFF | RNP_RAH_AV);
//	rar_high |= ((u32)addr[1] | ((u32)addr[0] << 8));
//
//	if (enable_addr != 0)
//		rar_high |= RNP_RAH_AV;
//
//	wr32(hw, RNP_ETH_RAR_RL(index), rar_low);
//	wr32(hw, RNP_ETH_RAR_RH(index), rar_high);
//
//	/* open unicast filter */
//	/* we now not use unicast */
//	/* but we must open this since dest-mac filter | unicast table */
//	/* all packets up if close unicast table */
//	mcstctrl = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
//	mcstctrl |= RNP_MCSTCTRL_UNICASE_TBL_EN;
//	wr32(hw, RNP_ETH_DMAC_MCSTCTRL, mcstctrl);
//
//	// printk("set rar address %x\n", rd32(hw, RNP_ETH_DMAC_MCSTCTRL));
//
	return 0;
}

/**
 *  rnp_clear_rar_generic - Remove Rx address register
 *  @hw: pointer to hardware structure
 *  @index: Receive address register to write
 *
 *  Clears an ethernet address from a receive address register.
 **/
s32 rnp_clear_rar_generic(struct rnp_hw *hw, u32 index)
{
	//u32 rar_high;
	//u32 rar_entries = hw->mac.num_rar_entries;

	///* Make sure we are using a valid rar index range */
	//if (index >= rar_entries) {
	//	hw_dbg(hw, "RAR index %d is out of range.\n", index);
	//	return RNP_ERR_INVALID_ARGUMENT;
	//}

	///*
	// * Some parts put the VMDq setting in the extra RAH bits,
	// * so save everything except the lower 16 bits that hold part
	// * of the address and the address valid bit.
	// */
	//rar_high = rd32(hw, RNP_ETH_RAR_RH(index));
	//rar_high &= ~(0x0000FFFF | RNP_RAH_AV);

	//// hw_dbg(hw, "Clearing RAR[%d]\n", index);
	//wr32(hw, RNP_ETH_RAR_RL(index), 0);
	//wr32(hw, RNP_ETH_RAR_RH(index), rar_high);

	///* clear VMDq pool/queue selection for this RAR */
	//hw->mac.ops.clear_vmdq(hw, index, RNP_CLEAR_VMDQ_ALL);

	return 0;
}

/**
 *  rnp_init_rx_addrs_generic - Initializes receive address filters.
 *  @hw: pointer to hardware structure
 *
 *  Places the MAC address in receive address register 0 and clears the rest
 *  of the receive address registers. Clears the multicast table. Assumes
 *  the receiver is in reset when the routine is called.
 **/
s32 rnp_init_rx_addrs_generic(struct rnp_hw *hw)
{
//	u32 i;
//	u32 rar_entries = hw->mac.num_rar_entries;
//	u32 v;
//
//	hw_dbg(hw,
//		   "init_rx_addrs:rar_entries:%d, mac.addr:%pM\n",
//		   rar_entries,
//		   hw->mac.addr);
//	/*
//	 * If the current mac address is valid, assume it is a software override
//	 * to the permanent address.
//	 * Otherwise, use the permanent address from the eeprom.
//	 */
//	if (!is_valid_ether_addr(hw->mac.addr)) {
//		/* Get the MAC address from the RAR0 for later reference */
//		hw->mac.ops.get_mac_addr(hw, hw->mac.addr);
//		hw_dbg(hw, " Keeping Current RAR0 Addr =%pM\n", hw->mac.addr);
//	} else {
//		/* Setup the receive address. */
//		hw_dbg(hw, "Overriding MAC Address in RAR[0]\n");
//		hw_dbg(hw, " New MAC Addr =%pM\n", hw->mac.addr);
//
//		hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, RNP_RAH_AV);
//
//		/*  clear VMDq pool/queue selection for RAR 0 */
//		hw->mac.ops.clear_vmdq(hw, 0, RNP_CLEAR_VMDQ_ALL);
//	}
//	hw->addr_ctrl.overflow_promisc = 0;
//
//	hw->addr_ctrl.rar_used_count = 1;
//
//	/* Zero out the other receive addresses. */
//	hw_dbg(hw, "Clearing RAR[1-%d]\n", rar_entries - 1);
//	for (i = 1; i < rar_entries; i++) {
//		wr32(hw, RNP_ETH_RAR_RL(i), 0);
//		wr32(hw, RNP_ETH_RAR_RH(i), 0);
//	}
//
//	/* Clear the MTA */
//	hw->addr_ctrl.mta_in_use = 0;
//	v = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
//	v &= (~0x3);
//	v |= hw->mac.mc_filter_type;
//	wr32(hw, RNP_ETH_DMAC_MCSTCTRL, v);
//
//	hw_dbg(hw, " Clearing MTA\n");
//	for (i = 0; i < hw->mac.mcft_size; i++)
//		wr32(hw, RNP_MTA(i), 0);
//
//	if (hw->mac.ops.init_uta_tables)
//		hw->mac.ops.init_uta_tables(hw);

	return 0;
}

/**
 *  rnp_mta_vector - Determines bit-vector in multicast table to set
 *  @hw: pointer to hardware structure
 *  @mc_addr: the multicast address
 *
 *  Extracts the 12 bits, from a multicast address, to determine which
 *  bit-vector to set in the multicast table. The hardware uses 12 bits, from
 *  incoming rx multicast addresses, to determine the bit-vector to check in
 *  the MTA. Which of the 4 combination, of 12-bits, the hardware uses is set
 *  by the MO field of the MCSTCTRL. The MO field is set during initialization
 *  to mc_filter_type.
 **/
static s32 rnp_mta_vector(struct rnp_hw *hw, u8 *mc_addr)
{
	u32 vector = 0;

	switch (hw->mac.mc_filter_type) {
		case 0: /* use bits [36:47] of the address */
			vector = ((mc_addr[4] << 8) | (((u16)mc_addr[5])));
			break;
		case 1: /* use bits [35:46] of the address */
			vector = ((mc_addr[4] << 7) | (((u16)mc_addr[5]) >> 1));
			break;
		case 2: /* use bits [34:45] of the address */
			vector = ((mc_addr[4] << 6) | (((u16)mc_addr[5]) >> 2));
			break;
		case 3: /* use bits [32:43] of the address */
			vector = ((mc_addr[4] << 5) | (((u16)mc_addr[5]) >> 3));
			break;
		default: /* Invalid mc_filter_type */
			hw_dbg(hw, "MC filter type param set incorrectly\n");
			break;
	}

	/* vector can only be 12-bits or boundary will be exceeded */
	vector &= 0xFFF;
	return vector;
}

static void rnp_set_vf_mta(struct rnp_hw *hw, u16 vector)
{
	u32 vector_bit;
	u32 vector_reg;

	hw->addr_ctrl.mta_in_use++;

	vector_reg = (vector >> 5) & 0x7F;
	vector_bit = vector & 0x1F;
	hw_dbg(hw,
		   "\t\t vf M: MTA-BIT:%4d, MTA_REG[%d][%d] <= 1\n",
		   vector,
		   vector_reg,
		   vector_bit);
	hw->mac.mta_shadow[vector_reg] |= (1 << vector_bit);
}
/**
 *  rnp_set_mta - Set bit-vector in multicast table
 *  @hw: pointer to hardware structure
 *  @hash_value: Multicast address hash value
 *
 *  Sets the bit-vector in the multicast table.
 **/
static void rnp_set_mta(struct rnp_hw *hw, u8 *mc_addr)
{
	u32 vector;
	u32 vector_bit;
	u32 vector_reg;

	hw->addr_ctrl.mta_in_use++;

	vector = rnp_mta_vector(hw, mc_addr);

	/*
	 * The MTA is a register array of 128 32-bit registers. It is treated
	 * like an array of 4096 bits.  We want to set bit
	 * BitArray[vector_value]. So we figure out what register the bit is
	 * in, read it, OR in the new bit, then write back the new value.  The
	 * register is determined by the upper 7 bits of the vector value and
	 * the bit within that register are determined by the lower 5 bits of
	 * the value.
	 */
	vector_reg = (vector >> 5) & 0x7F;
	vector_bit = vector & 0x1F;
	hw_dbg(hw,
		   "\t\t%pM: MTA-BIT:%4d, MTA_REG[%d][%d] <= 1\n",
		   mc_addr,
		   vector,
		   vector_reg,
		   vector_bit);
	hw->mac.mta_shadow[vector_reg] |= (1 << vector_bit);
}

u8 *rnp_addr_list_itr(struct rnp_hw __maybe_unused *hw, u8 **mc_addr_ptr)
{
#ifdef NETDEV_HW_ADDR_T_MULTICAST
        struct netdev_hw_addr *mc_ptr;
#else   
        struct dev_mc_list *mc_ptr;
#endif  
        u8 *addr = *mc_addr_ptr;
        
        
#ifdef NETDEV_HW_ADDR_T_MULTICAST
        mc_ptr = container_of(addr, struct netdev_hw_addr, addr[0]);
        if (mc_ptr->list.next) {
                struct netdev_hw_addr *ha;
                
                ha = list_entry(mc_ptr->list.next, struct netdev_hw_addr, list);
                *mc_addr_ptr = ha->addr;
        }       
#else   
        mc_ptr = container_of(addr, struct dev_mc_list, dmi_addr[0]);
        if (mc_ptr->next)
                *mc_addr_ptr = mc_ptr->next->dmi_addr;
#endif          
        else
                *mc_addr_ptr = NULL;
                
        return addr;
} 

/**
 *  rnp_update_mc_addr_list_generic - Updates MAC list of multicast addresses
 *  @hw: pointer to hardware structure
 *  @netdev: pointer to net device structure
 *
 *  The given list replaces any existing list. Clears the MC addrs from receive
 *  address registers and the multicast table. Uses unused receive address
 *  registers for the first multicast addresses, and hashes the rest into the
 *  multicast table.
 **/
s32 rnp_update_mc_addr_list_generic(struct rnp_hw *hw,
		struct net_device *netdev)
{
#ifdef NETDEV_HW_ADDR_T_MULTICAST
	struct netdev_hw_addr *ha;
#endif
	u32 i;
	u32 v;
	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
	int addr_count = 0;
	u8 *addr_list = NULL;

	/*
	 * Set the new number of MC addresses that we are being requested to
	 * use.
	 */
	hw->addr_ctrl.num_mc_addrs = netdev_mc_count(netdev);
	hw->addr_ctrl.mta_in_use = 0;

	/* Clear mta_shadow */
	hw_dbg(hw, " Clearing MTA(multicast table)\n");
	memset(&hw->mac.mta_shadow, 0, sizeof(hw->mac.mta_shadow));

	/* Update mta shadow */
	hw_dbg(hw, " Updating MTA..\n");

	addr_count = netdev_mc_count(netdev);

#ifdef NETDEV_HW_ADDR_T_MULTICAST     
	ha = list_first_entry(&netdev->mc.list,
			struct netdev_hw_addr, list);
	addr_list = ha->addr;
#else   
	addr_list = netdev->mc_list->dmi_addr;
#endif
	for (i = 0; i < addr_count; i++) {
		hw_dbg(hw, " Adding the multicast addresses:\n");
		rnp_set_mta(hw, rnp_addr_list_itr(hw, &addr_list));
	}

	//netdev_for_each_mc_addr(ha, netdev) {
	//	rnp_set_mta(hw, ha->addr);
	//}
	

	// sriov mode should set for vf multicast
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		for (i = 0; i < adapter->num_vfs; i++) {
			struct vf_data_storage *vfinfo = &adapter->vfinfo[i];
			int j;

			for (j = 0; j < vfinfo->num_vf_mc_hashes; j++)
				rnp_set_vf_mta(hw, vfinfo->vf_mc_hashes[j]);
		}
	}

	/* Enable mta */
	for (i = 0; i < hw->mac.mcft_size; i++) {
		if (hw->addr_ctrl.mta_in_use)
			wr32(hw, RNP_ETH_MUTICAST_HASH_TABLE(i), hw->mac.mta_shadow[i]);
	}

	if (hw->addr_ctrl.mta_in_use > 0) {
		v = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
		wr32(hw,
			 RNP_ETH_DMAC_MCSTCTRL,
			 v | RNP_MCSTCTRL_MULTICASE_TBL_EN | hw->mac.mc_filter_type);
	}

	hw_dbg(hw, " update MTA Done. mta_in_use:%d\n", hw->addr_ctrl.mta_in_use);
	return hw->addr_ctrl.mta_in_use;
}

/**
 *  rnp_enable_mc_generic - Enable multicast address in RAR
 *  @hw: pointer to hardware structure
 *
 *  Enables multicast address in RAR and the use of the multicast hash table.
 **/
s32 rnp_enable_mc_generic(struct rnp_hw *hw)
{
	struct rnp_addr_filter_info *a = &hw->addr_ctrl;
	u32 v;

	if (a->mta_in_use > 0) {
		v = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
		v |= RNP_MCSTCTRL_MULTICASE_TBL_EN;
		wr32(hw, RNP_ETH_DMAC_MCSTCTRL, v);
	}

	return 0;
}

/**
 *  rnp_disable_mc_generic - Disable multicast address in RAR
 *  @hw: pointer to hardware structure
 *
 *  Disables multicast address in RAR and the use of the multicast hash table.
 **/
s32 rnp_disable_mc_generic(struct rnp_hw *hw)
{
	struct rnp_addr_filter_info *a = &hw->addr_ctrl;
	u32 v;

	if (a->mta_in_use > 0) {
		v = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
		v &= ~RNP_MCSTCTRL_MULTICASE_TBL_EN;
		wr32(hw, RNP_ETH_DMAC_MCSTCTRL, v);
	}

	return 0;
}

/**
 *  rnp_fc_enable_generic - Enable flow control
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to the current settings.
 **/
s32 rnp_fc_enable_generic(struct rnp_hw *hw)
{
	s32 ret_val = 0;
	u32 reg;
	u32 rxctl_reg, txctl_reg[RNP_MAX_TRAFFIC_CLASS];
	int i;

	hw->fc.current_mode = hw->fc.requested_mode;
	/*
	 * Validate the water mark configuration for packet buffer 0.  Zero
	 * water marks indicate that the packet buffer was not configured
	 * and the watermarks for packet buffer 0 should always be configured.
	 */
	if (!hw->fc.pause_time) {
		ret_val = RNP_ERR_INVALID_LINK_SETTINGS;
		goto out;
	}

	for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++) {
		if ((hw->fc.current_mode & rnp_fc_tx_pause) && hw->fc.high_water[i]) {
			if (!hw->fc.low_water[i] ||
				hw->fc.low_water[i] >= hw->fc.high_water[i]) {
				hw_dbg(hw, "Invalid water mark configuration\n");
				ret_val = RNP_ERR_INVALID_LINK_SETTINGS;
				goto out;
			}
		}
	}

	/* Negotiate the fc mode to use */
	// rnp_fc_autoneg(hw);

	/* Disable any previous flow control settings */
	rxctl_reg = rd32(hw, RNP_MAC_RX_FLOW_CTRL);
	rxctl_reg &= (~RNP_RX_FLOW_ENABLE_MASK);

	for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++) {
		txctl_reg[i] = rd32(hw, RNP_MAC_Q0_TX_FLOW_CTRL(i));
		txctl_reg[i] &= (~RNP_TX_FLOW_ENABLE_MASK);
	}
	/*
	 * The possible values of fc.current_mode are:
	 * 0: Flow control is completely disabled
	 * 1: Rx flow control is enabled (we can receive pause frames,
	 *    but not send pause frames).
	 * 2: Tx flow control is enabled (we can send pause frames but
	 *    we do not support receiving pause frames).
	 * 3: Both Rx and Tx flow control (symmetric) are enabled.
	 * other: Invalid.
	 */
	switch (hw->fc.current_mode) {
		case rnp_fc_none:
			/*
			 * Flow control is disabled by software override or autoneg.
			 * The code below will actually disable it in the HW.
			 */
			break;
		case rnp_fc_rx_pause:
			/*
			 * Rx Flow control is enabled and Tx Flow control is
			 * disabled by software override. Since there really
			 * isn't a way to advertise that we are capable of RX
			 * Pause ONLY, we will advertise that we support both
			 * symmetric and asymmetric Rx PAUSE.  Later, we will
			 * disable the adapter's ability to send PAUSE frames.
			 */
			rxctl_reg |= (RNP_RX_FLOW_ENABLE_MASK);
			break;
		case rnp_fc_tx_pause:
			/*
			 * Tx Flow control is enabled, and Rx Flow control is
			 * disabled by software override.
			 */
			for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++)
				txctl_reg[i] |= (RNP_TX_FLOW_ENABLE_MASK);
			break;
		case rnp_fc_full:
			/* Flow control (both Rx and Tx) is enabled by SW override. */
			rxctl_reg |= (RNP_RX_FLOW_ENABLE_MASK);
			for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++)
				txctl_reg[i] |= (RNP_TX_FLOW_ENABLE_MASK);
			break;
		default:
			hw_dbg(hw, "Flow control param set incorrectly\n");
			ret_val = RNP_ERR_CONFIG;
			goto out;
	}
	/* Set up and enable Rx high/low water mark thresholds, enable XON. */
	for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++) {
		if ((hw->fc.current_mode & rnp_fc_tx_pause)) {
			if (hw->fc.high_water[i]) {
				wr32(hw, RNP_ETH_HIGH_WATER(i), hw->fc.high_water[i]);
			}
			if (hw->fc.low_water[i]) {
				wr32(hw, RNP_ETH_LOW_WATER(i), hw->fc.low_water[i]);
			}
		}
	}

	/* Configure pause time (2 TCs per register) */
	reg = hw->fc.pause_time;
	for (i = 0; i < (RNP_MAX_TRAFFIC_CLASS); i++)
		txctl_reg[i] |= (reg << 16);

	/* Set 802.3x based flow control settings. */
	wr32(hw, RNP_MAC_RX_FLOW_CTRL, rxctl_reg);
	for (i = 0; i < (RNP_MAX_TRAFFIC_CLASS); i++)
		wr32(hw, RNP_MAC_Q0_TX_FLOW_CTRL(i), txctl_reg[i]);
out:
	return ret_val;
}

/**
 *  rnp_negotiate_fc - Negotiate flow control
 *  @hw: pointer to hardware structure
 *  @adv_reg: flow control advertised settings
 *  @lp_reg: link partner's flow control settings
 *  @adv_sym: symmetric pause bit in advertisement
 *  @adv_asm: asymmetric pause bit in advertisement
 *  @lp_sym: symmetric pause bit in link partner advertisement
 *  @lp_asm: asymmetric pause bit in link partner advertisement
 *
 *  Find the intersection between advertised settings and link partner's
 *  advertised settings
 **/
static s32 rnp_negotiate_fc(struct rnp_hw *hw,
							u32 adv_reg,
							u32 lp_reg,
							u32 adv_sym,
							u32 adv_asm,
							u32 lp_sym,
							u32 lp_asm)
{
#if 0
	if ((!(adv_reg)) ||  (!(lp_reg)))
		return RNP_ERR_FC_NOT_NEGOTIATED;

	if ((adv_reg & adv_sym) && (lp_reg & lp_sym)) {
		/*
		 * Now we need to check if the user selected Rx ONLY
		 * of pause frames.  In this case, we had to advertise
		 * FULL flow control because we could not advertise RX
		 * ONLY. Hence, we must now check to see if we need to
		 * turn OFF the TRANSMISSION of PAUSE frames.
		 */
		if (hw->fc.requested_mode == rnp_fc_full) {
			hw->fc.current_mode = rnp_fc_full;
			hw_dbg(hw, "Flow Control = FULL.\n");
		} else {
			hw->fc.current_mode = rnp_fc_rx_pause;
			hw_dbg(hw, "Flow Control=RX PAUSE frames only\n");
		}
	} else if (!(adv_reg & adv_sym) && (adv_reg & adv_asm) &&
			(lp_reg & lp_sym) && (lp_reg & lp_asm)) {
		hw->fc.current_mode = rnp_fc_tx_pause;
		hw_dbg(hw, "Flow Control = TX PAUSE frames only.\n");
	} else if ((adv_reg & adv_sym) && (adv_reg & adv_asm) &&
			!(lp_reg & lp_sym) && (lp_reg & lp_asm)) {
		hw->fc.current_mode = rnp_fc_rx_pause;
		hw_dbg(hw, "Flow Control = RX PAUSE frames only.\n");
	} else {
		hw->fc.current_mode = rnp_fc_none;
		hw_dbg(hw, "Flow Control = NONE.\n");
	}
#endif
	return 0;
}

/**
 *  rnp_fc_autoneg_fiber - Enable flow control on 1 gig fiber
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according on 1 gig fiber.
 **/
static s32 rnp_fc_autoneg_fiber(struct rnp_hw *hw)
{
	s32 ret_val = RNP_ERR_FC_NOT_NEGOTIATED;
#if 0
	u32 pcs_anadv_reg, pcs_lpab_reg, linkstat;

	/*
	 * On multispeed fiber at 1g, bail out if
	 * - link is up but AN did not complete, or if
	 * - link is up and AN completed but timed out
	 */

	linkstat = RNP_READ_REG(hw, RNP_PCS1GLSTA);
	if ((!!(linkstat & RNP_PCS1GLSTA_AN_COMPLETE) == 0) ||
			(!!(linkstat & RNP_PCS1GLSTA_AN_TIMED_OUT) == 1))
		goto out;

	pcs_anadv_reg = RNP_READ_REG(hw, RNP_PCS1GANA);
	pcs_lpab_reg = RNP_READ_REG(hw, RNP_PCS1GANLP);

	ret_val =  rnp_negotiate_fc(hw, pcs_anadv_reg,
			pcs_lpab_reg, RNP_PCS1GANA_SYM_PAUSE,
			RNP_PCS1GANA_ASM_PAUSE,
			RNP_PCS1GANA_SYM_PAUSE,
			RNP_PCS1GANA_ASM_PAUSE);

out:
#endif
	return ret_val;
}

/**
 *  rnp_fc_autoneg_backplane - Enable flow control IEEE clause 37
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to IEEE clause 37.
 **/
static s32 rnp_fc_autoneg_backplane(struct rnp_hw *hw)
{
	s32 ret_val = RNP_ERR_FC_NOT_NEGOTIATED;
#if 0
	u32 links2, anlp1_reg, autoc_reg, links;

	/*
	 * On backplane, bail out if
	 * - backplane autoneg was not completed, or if
	 * - we are n10 and link partner is not AN enabled
	 */
	links = RNP_READ_REG(hw, RNP_LINKS);
	if ((links & RNP_LINKS_KX_AN_COMP) == 0)
		goto out;

	if (hw->mac.type == rnp_mac_n10EB) {
		links2 = RNP_READ_REG(hw, RNP_LINKS2);
		if ((links2 & RNP_LINKS2_AN_SUPPORTED) == 0)
			goto out;
	}
	/*
	 * Read the 10g AN autoc and LP ability registers and resolve
	 * local flow control settings accordingly
	 */
	autoc_reg = RNP_READ_REG(hw, RNP_AUTOC);
	anlp1_reg = RNP_READ_REG(hw, RNP_ANLP1);

	ret_val = rnp_negotiate_fc(hw, autoc_reg,
			anlp1_reg, RNP_AUTOC_SYM_PAUSE, RNP_AUTOC_ASM_PAUSE,
			RNP_ANLP1_SYM_PAUSE, RNP_ANLP1_ASM_PAUSE);

out:
#endif
	return ret_val;
}

/**
 *  rnp_fc_autoneg_copper - Enable flow control IEEE clause 37
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to IEEE clause 37.
 **/
static s32 rnp_fc_autoneg_copper(struct rnp_hw *hw)
{
#if 0
	u16 technology_ability_reg = 0;
	u16 lp_technology_ability_reg = 0;

	hw->phy.ops.read_reg(hw, MDIO_AN_ADVERTISE,
			MDIO_MMD_AN,
			&technology_ability_reg);
	hw->phy.ops.read_reg(hw, MDIO_AN_LPA,
			MDIO_MMD_AN,
			&lp_technology_ability_reg);

	return rnp_negotiate_fc(hw, (u32)technology_ability_reg,
			(u32)lp_technology_ability_reg,
			RNP_TAF_SYM_PAUSE, RNP_TAF_ASM_PAUSE,
			RNP_TAF_SYM_PAUSE, RNP_TAF_ASM_PAUSE);
#else
	return 0;
#endif
}

/**
 *  rnp_fc_autoneg - Configure flow control
 *  @hw: pointer to hardware structure
 *
 *  Compares our advertised flow control capabilities to those advertised by
 *  our link partner, and determines the proper flow control mode to use.
 **/
void rnp_fc_autoneg(struct rnp_hw *hw)
{
	s32 ret_val = RNP_ERR_FC_NOT_NEGOTIATED;
#if 0
	rnp_link_speed speed;
	bool link_up;

	/*
	 * AN should have completed when the cable was plugged in.
	 * Look for reasons to bail out.  Bail out if:
	 * - FC autoneg is disabled, or if
	 * - link is not up.
	 *
	 * Since we're being called from an LSC, link is already known to be up.
	 * So use link_up_wait_to_complete=false.
	 */
	if (hw->fc.disable_fc_autoneg)
		goto out;

	hw->mac.ops.check_link(hw, &speed, &link_up, false);
	if (!link_up)
		goto out;

	switch (hw->phy.media_type) {
	/* Autoneg flow control on fiber adapters */
	case rnp_media_type_fiber:
		if (speed == RNP_LINK_SPEED_1GB_FULL)
			ret_val = rnp_fc_autoneg_fiber(hw);
		break;

		/* Autoneg flow control on backplane adapters */
	case rnp_media_type_backplane:
		ret_val = rnp_fc_autoneg_backplane(hw);
		break;

		/* Autoneg flow control on copper adapters */
	case rnp_media_type_copper:
		if (rnp_device_supports_autoneg_fc(hw) == 0)
			ret_val = rnp_fc_autoneg_copper(hw);
		break;

	default:
		break;
	}

out:
	if (ret_val == 0) {
		hw->fc.fc_was_autonegged = true;
	} else {
		hw->fc.fc_was_autonegged = false;
		hw->fc.current_mode = hw->fc.requested_mode;
	}
#endif
}

/**
 *  rnp_disable_pcie_master - Disable PCI-express master access
 *  @hw: pointer to hardware structure
 *
 *  Disables PCI-Express master access and verifies there are no pending
 *  requests. RNP_ERR_MASTER_REQUESTS_PENDING is returned if master disable
 *  bit hasn't caused the master requests to be disabled, else 0
 *  is returned signifying master requests disabled.
 **/
static s32 rnp_disable_pcie_master(struct rnp_hw *hw)
{
	struct rnp_adapter *adapter = hw->back;

	// disable dma rx/tx
	wr32(hw, RNP_DMA_AXI_EN, 0);

	return 0;
}

/**
 *  rnp_acquire_swfw_sync - Acquire SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to acquire
 *
 *  Acquires the SWFW semaphore through the GSSR register for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
s32 rnp_acquire_swfw_sync(struct rnp_hw *hw, u16 mask)
{
	u32 gssr;
	u32 swmask = mask;
	u32 fwmask = mask << 5;
	s32 timeout = 200;
#if 0
	while (timeout) {
		/*
		 * SW EEPROM semaphore bit is used for access to all
		 * SW_FW_SYNC/GSSR bits (not just EEPROM)
		 */
		if (rnp_get_eeprom_semaphore(hw))
			return RNP_ERR_SWFW_SYNC;

		gssr = RNP_READ_REG(hw, RNP_GSSR);
		if (!(gssr & (fwmask | swmask)))
			break;

		/*
		 * Firmware currently using resource (fwmask) or other software
		 * thread currently using resource (swmask)
		 */
		rnp_release_eeprom_semaphore(hw);
		usleep_range(5000, 10000);
		timeout--;
	}

	if (!timeout) {
		hw_dbg(hw, "Driver can't access resource, SW_FW_SYNC timeout.\n");
		return RNP_ERR_SWFW_SYNC;
	}

	gssr |= swmask;
	RNP_WRITE_REG(hw, RNP_GSSR, gssr);

	rnp_release_eeprom_semaphore(hw);
#endif
	return 0;
}

/**
 *  rnp_release_swfw_sync - Release SWFW semaphore
 *  @hw: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to release
 *
 *  Releases the SWFW semaphore through the GSSR register for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
void rnp_release_swfw_sync(struct rnp_hw *hw, u16 mask)
{
	u32 gssr;
	u32 swmask = mask;
#if 0

#endif
}

/**
 *  rnp_disable_rx_buff_generic - Stops the receive data path
 *  @hw: pointer to hardware structure
 *
 *  Stops the receive data path and waits for the HW to internally
 *  empty the Rx security block.
 **/
s32 rnp_disable_rx_buff_generic(struct rnp_hw *hw)
{
	u32 v;
#if 0

#endif
	return 0;
}

/**
 *  rnp_enable_rx_buff - Enables the receive data path
 *  @hw: pointer to hardware structure
 *
 *  Enables the receive data path
 **/
s32 rnp_enable_rx_buff_generic(struct rnp_hw *hw)
{
	int secrxreg;

#if 0

#endif

	return 0;
}

/**
 *  rnp_enable_rx_dma_generic - MAC Enable the Rx DMA unit
 *  @hw: pointer to hardware structure
 *  @regval: register value to write to RXCTRL
 *
 *  Enables the Rx DMA unit
 **/
s32 rnp_enable_rx_dma_generic(struct rnp_hw *hw, u32 regval)
{
	// RNP_WRITE_REG(hw, RNP_RXCTRL, regval);

	return 0;
}

/**
 *  rnp_blink_led_start_generic - Blink LED based on index.
 *  @hw: pointer to hardware structure
 *  @index: led number to blink
 **/
s32 rnp_blink_led_start_generic(struct rnp_hw *hw, u32 index)
{
	rnp_link_speed speed = 0;
	bool link_up = false;
	s32 ret_val = 0;

#if 0
	u32 autoc_reg = RNP_READ_REG(hw, RNP_AUTOC);
	u32 led_reg = RNP_READ_REG(hw, RNP_LEDCTL);

	/*
	 * Link must be up to auto-blink the LEDs;
	 * Force it if link is down.
	 */
	hw->mac.ops.check_link(hw, &speed, &link_up, false);

	if (!link_up) {
		/* Need the SW/FW semaphore around AUTOC writes if n10 and
		 * LESM is on.
		 */
		bool got_lock = false;

		if ((hw->mac.type == rnp_mac_n10EB) &&
		    rnp_verify_lesm_fw_enabled_n10(hw)) {
			ret_val = hw->mac.ops.acquire_swfw_sync(hw,
							RNP_GSSR_MAC_CSR_SM);
			if (ret_val)
				goto out;

			got_lock = true;
		}
		autoc_reg |= RNP_AUTOC_AN_RESTART;
		autoc_reg |= RNP_AUTOC_FLU;
		RNP_WRITE_REG(hw, RNP_AUTOC, autoc_reg);
		RNP_WRITE_FLUSH(hw);

		if (got_lock)
			hw->mac.ops.release_swfw_sync(hw,
						      RNP_GSSR_MAC_CSR_SM);
		usleep_range(10000, 20000);
	}

	led_reg &= ~RNP_LED_MODE_MASK(index);
	led_reg |= RNP_LED_BLINK(index);
	RNP_WRITE_REG(hw, RNP_LEDCTL, led_reg);
	RNP_WRITE_FLUSH(hw);

out:
#endif
	return ret_val;
}

/**
 *  rnp_blink_led_stop_generic - Stop blinking LED based on index.
 *  @hw: pointer to hardware structure
 *  @index: led number to stop blinking
 **/
s32 rnp_blink_led_stop_generic(struct rnp_hw *hw, u32 index)
{
	s32 ret_val = 0;
#if 0
	u32 autoc_reg = RNP_READ_REG(hw, RNP_AUTOC);
	u32 led_reg = RNP_READ_REG(hw, RNP_LEDCTL);
	bool got_lock = false;

	/* Need the SW/FW semaphore around AUTOC writes if n10 and
	 * LESM is on.
	 */
	if ((hw->mac.type == rnp_mac_n10EB) &&
	    rnp_verify_lesm_fw_enabled_n10(hw)) {
		ret_val = hw->mac.ops.acquire_swfw_sync(hw,
						RNP_GSSR_MAC_CSR_SM);
		if (ret_val)
			goto out;

		got_lock = true;
	}

	autoc_reg &= ~RNP_AUTOC_FLU;
	autoc_reg |= RNP_AUTOC_AN_RESTART;
	RNP_WRITE_REG(hw, RNP_AUTOC, autoc_reg);

	if (hw->mac.type == rnp_mac_n10EB)
		rnp_reset_pipeline_n10(hw);

	if (got_lock)
		hw->mac.ops.release_swfw_sync(hw, RNP_GSSR_MAC_CSR_SM);

	led_reg &= ~RNP_LED_MODE_MASK(index);
	led_reg &= ~RNP_LED_BLINK(index);
	led_reg |= RNP_LED_LINK_ACTIVE << RNP_LED_MODE_SHIFT(index);
	RNP_WRITE_REG(hw, RNP_LEDCTL, led_reg);
	RNP_WRITE_FLUSH(hw);

out:
#endif
	return ret_val;
}

/**
 *  rnp_get_pcie_msix_count_generic - Gets MSI-X vector count
 *  @hw: pointer to hardware structure
 *
 *  Read PCIe configuration space, and get the MSI-X vector count from
 *  the capabilities table.
 **/
u16 rnp_get_pcie_msix_count_generic(struct rnp_hw *hw)
{
	//u16 msix_count = pci_msix_vec_count(hw->pdev);

	//return msix_count;
	return 0;
}

/**
 *  rnp_clear_vmdq_generic - Disassociate a VMDq pool index from a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to disassociate
 *  @vmdq: VMDq pool index to remove from the rar
 **/
s32 rnp_clear_vmdq_generic(struct rnp_hw *hw, u32 rar, u32 vmdq)
{
	u32 rar_entries = hw->mac.num_rar_entries;

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		hw_dbg(hw, "RAR index %d is out of range.\n", rar);
		return RNP_ERR_INVALID_ARGUMENT;
	}

	wr32(hw, RNP_VM_DMAC_MPSAR_RING(rar), 0);

	return 0;
}

/**
 *  rnp_set_vmdq_generic - Associate a VMDq pool index with a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to associate with a VMDq index
 *  @vmdq: VMDq pool index
 **/
s32 rnp_set_vmdq_generic(struct rnp_hw *hw, u32 rar, u32 vmdq)
{
	u32 mpsar;
	u32 rar_entries = hw->mac.num_rar_entries;

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		hw_dbg(hw, "RAR index %d is out of range.\n", rar);
		return RNP_ERR_INVALID_ARGUMENT;
	}

	wr32(hw, RNP_VM_DMAC_MPSAR_RING(rar), vmdq);

	return 0;
}

/**
 *  rnp_init_uta_tables_generic - Initialize the Unicast Table Array
 *  @hw: pointer to hardware structure
 **/
s32 rnp_init_uta_tables_generic(struct rnp_hw *hw)
{
	int i;

	for (i = 0; i < 128; i++)
		wr32(hw, RNP_ETH_UTA(i), 0);
	return 0;
}

/**
 *  rnp_find_vlvf_slot - find the vlanid or the first empty slot
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *
 *  return the VLVF index where this VLAN id should be placed
 *
 **/
static s32 rnp_find_vlvf_slot(struct rnp_hw *hw, u32 vlan)
{
	u32 bits = 0;
	u32 first_empty_slot = 0;
	s32 regindex = -1;

	/* short cut the special case */
	if (vlan == 0)
		return 0;

	/*
	 * Search for the vlan id in the VLVF entries. Save off the first empty
	 * slot found along the way
	 */
	for (regindex = 1; regindex < RNP_VLVF_ENTRIES; regindex++) {
		bits = rd32(hw, RNP_VLVF(regindex));
		if (!bits && !(first_empty_slot))
			first_empty_slot = regindex;
		else if ((bits & 0x0FFF) == vlan)
			break;
	}

	/*
	 * If regindex is less than RNP_VLVF_ENTRIES, then we found the vlan
	 * in the VLVF. Else use the first empty VLVF register for this
	 * vlan id.
	 */
	if (regindex >= RNP_VLVF_ENTRIES) {
		if (first_empty_slot)
			regindex = first_empty_slot;
		else {
			hw_dbg(hw, "No space in VLVF.\n");
			regindex = RNP_ERR_NO_SPACE;
		}
	}
	return regindex;
}

/**
 *  rnp_set_vfta_generic - Set VLAN filter table
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vind: VMDq output index that maps queue to VLAN id in VFVFB
 *  @vlan_on: boolean flag to turn on/off VLAN in VFVF
 *
 *  Turn on/off specified VLAN in the VLAN filter table.
 **/
s32 rnp_set_vfta_generic(struct rnp_hw *hw, u32 vlan, u32 vind, bool vlan_on)
{
	s32 regindex;
	u32 bitindex;
	u32 vfta;
	u32 bits;
	u32 vt;
	u32 targetbit;
	bool vfta_changed = false;

	/* todo in vf mode vlvf regester can be set according to vind*/
	if (vlan > 4095)
		return RNP_ERR_PARAM;

	/*
	 * this is a 2 part operation - first the VFTA, then the
	 * VLVF and VLVFB if VT Mode is set
	 * We don't write the VFTA until we know the VLVF part succeeded.
	 */

	/* Part 1
	 * The VFTA is a bitstring made up of 128 32-bit registers
	 * that enable the particular VLAN id, much like the MTA:
	 *    bits[11-5]: which register
	 *    bits[4-0]:  which bit in the register
	 */
	regindex = (vlan >> 5) & 0x7F;
	bitindex = vlan & 0x1F;
	targetbit = (1 << bitindex);
	vfta = rd32(hw, RNP_VFTA(regindex));

	if (vlan_on) {
		if (!(vfta & targetbit)) {
			vfta |= targetbit;
			vfta_changed = true;
		}
	} else {
		if ((vfta & targetbit)) {
			vfta &= ~targetbit;
			vfta_changed = true;
		}
	}

/*
 * to enable two vf have same vlan feature, disable vlvf function.
 * as vlan has high-priority than mac-address filter, which means
 * two vf can't have same vlan.
 */
#if 0
	/* Part 2
	 * If VT Mode is set
	 *   Either vlan_on
	 *     make sure the vlan is in VLVF
	 *     set the vind bit in the matching VLVFB
	 *   Or !vlan_on
	 *     clear the pool bit and possibly the vind
	 */
	vt = rd32(hw, RNP_VT_CTL);
	if (vt & RNP_VT_CTL_VT_ENABLE) {
		s32 vlvf_index;

		vlvf_index = rnp_find_vlvf_slot(hw, vlan);
		if (vlvf_index < 0)
			return vlvf_index;

		if (vlan_on) {
			/* set the pool bit */
			if (vind < 32) {
				bits = rd32(hw,
						RNP_VLVFB(vlvf_index*2));
				bits |= (1 << vind);
				wr32(hw,
						RNP_VLVFB(vlvf_index*2),
						bits);
			} else {
				bits = rd32(hw,
						RNP_VLVFB((vlvf_index*2)+1));
				bits |= (1 << (vind-32));
				wr32(hw,
						RNP_VLVFB((vlvf_index*2)+1),
						bits);
			}
		} else {
			/* clear the pool bit */
			if (vind < 32) {
				bits = rd32(hw,
						RNP_VLVFB(vlvf_index*2));
				bits &= ~(1 << vind);
				wr32(hw,
						RNP_VLVFB(vlvf_index*2),
						bits);
				bits |= rd32(hw,
						RNP_VLVFB((vlvf_index*2)+1));
			} else {
				bits = rd32(hw,
						RNP_VLVFB((vlvf_index*2)+1));
				bits &= ~(1 << (vind-32));
				wr32(hw,
						RNP_VLVFB((vlvf_index*2)+1),
						bits);
				bits |= rd32(hw,
						RNP_VLVFB(vlvf_index*2));
			}
		}

		/*
		 * If there are still bits set in the VLVFB registers
		 * for the VLAN ID indicated we need to see if the
		 * caller is requesting that we clear the VFTA entry bit.
		 * If the caller has requested that we clear the VFTA
		 * entry bit but there are still pools/VFs using this VLAN
		 * ID entry then ignore the request.  We're not worried
		 * about the case where we're turning the VFTA VLAN ID
		 * entry bit on, only when requested to turn it off as
		 * there may be multiple pools and/or VFs using the
		 * VLAN ID entry.  In that case we cannot clear the
		 * VFTA bit until all pools/VFs using that VLAN ID have also
		 * been cleared.  This will be indicated by "bits" being
		 * zero.
		 */
		if (bits) {
			wr32(hw, RNP_VLVF(vlvf_index),
					(RNP_VLVF_VIEN | vlan));
			if (!vlan_on) {
				/* someone wants to clear the vfta entry
				 * but some pools/VFs are still using it.
				 * Ignore it.
				 */
				vfta_changed = false;
			}
		} else
			wr32(hw, RNP_VLVF(vlvf_index), 0);
	}
#endif

	if (vfta_changed)
		wr32(hw, RNP_VFTA(regindex), vfta);
	return 0;
}

/**
 *  rnp_clear_vfta_generic - Clear VLAN filter table
 *  @hw: pointer to hardware structure
 *
 *  Clears the VLAN filer table, and the VMDq index associated with the filter
 **/
s32 rnp_clear_vfta_generic(struct rnp_hw *hw)
{
	u32 offset;

	for (offset = 0; offset < hw->mac.vft_size; offset++)
		wr32(hw, RNP_VFTA(offset), 0);

	for (offset = 0; offset < RNP_VLVF_ENTRIES; offset++)
		wr32(hw, RNP_VLVF(offset), 0);

	return 0;
}

/**
 *  rnp_check_mac_link_generic - Determine link and speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true when link is up
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 rnp_check_mac_link_generic(struct rnp_hw *hw,
		rnp_link_speed *speed,
		bool *link_up,
		bool link_up_wait_to_complete)
{
	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
	struct rnp_pcs_info *pcs = &hw->pcs;

	u32 status;

#ifdef NO_MBX_VERSION

	// if vu440 , we assume link always
	if (hw->rss_type == rnp_rss_uv440) {

		*link_up = true;
		*speed = RNP_LINK_SPEED_40GB_FULL;
		goto skip_get_link;

	}

	status = pcs->ops.read(hw, 0, RNP_PCS_LINK_STATUS);

	if (status & RNP_PCS_LINKUP)
		*link_up = true;
	else
		*link_up = false;
	status = pcs->ops.read(hw, 0, RNP_PCS_LINK_SPEED);

	if (status & RNP_PCS_1G_OR_10G) {
		// 10G mode
		switch (status & RNP_PCS_SPPEED_MASK) {
			case RNP_PCS_SPPEED_10G:
				// printk("10G mode\n");
				*speed = RNP_LINK_SPEED_10GB_FULL;

				break;
			case RNP_PCS_SPPEED_40G:
				*speed = RNP_LINK_SPEED_40GB_FULL;
				// printk("40G mode\n");

				break;
		}
	} else {
		// printk("1G mode\n");
	}
#else

	if (hw->speed == 10) {
		*speed = RNP_LINK_SPEED_10_FULL;
	}else if (hw->speed == 100) {
		*speed = RNP_LINK_SPEED_100_FULL;
	} else if (hw->speed == 1000) {
		*speed = RNP_LINK_SPEED_1GB_FULL;
	} else if (hw->speed == 10000) {
		*speed = RNP_LINK_SPEED_10GB_FULL;
	} else if (hw->speed == 25000) {
		*speed = RNP_LINK_SPEED_25GB_FULL;
	} else if (hw->speed == 40000) {
		*speed = RNP_LINK_SPEED_40GB_FULL;
	} else {
		*speed = RNP_LINK_SPEED_UNKNOWN;
	}

	*link_up = hw->link;
#endif

skip_get_link:

	/* always assume link is up, if no check link function */
	//#if CONFIG_RNP_FPGA
	//	/* used to simulate link down */
	//	if (adapter->priv_flags & RNP_PRIV_FLAG_SIMUATE_DOWN) {
	//		dbg("simulate link is down\n");
	//		*link_up = false;
	//		*speed = RNP_LINK_SPEED_UNKNOWN;
	//	} else {
	//		*link_up = true;
	//		*speed = RNP_LINK_SPEED_10GB_FULL;
	//	}
	//#else
	//	link_up = false;
	//#endif
	return 0;
}

/**
 *  rnp_get_wwn_prefix_generic - Get alternative WWNN/WWPN prefix from
 *  the EEPROM
 *  @hw: pointer to hardware structure
 *  @wwnn_prefix: the alternative WWNN prefix
 *  @wwpn_prefix: the alternative WWPN prefix
 *
 *  This function will read the EEPROM from the alternative SAN MAC address
 *  block to check the support for the alternative WWNN/WWPN prefix support.
 **/
s32 rnp_get_wwn_prefix_generic(struct rnp_hw *hw,
							   u16 *wwnn_prefix,
							   u16 *wwpn_prefix)
{
#if 0
	u16 offset, caps;
	u16 alt_san_mac_blk_offset;

	/* clear output first */
	*wwnn_prefix = 0xFFFF;
	*wwpn_prefix = 0xFFFF;

	/* check if alternative SAN MAC is supported */
	hw->eeprom.ops.read(hw, RNP_ALT_SAN_MAC_ADDR_BLK_PTR,
			&alt_san_mac_blk_offset);

	if ((alt_san_mac_blk_offset == 0) ||
	    (alt_san_mac_blk_offset == 0xFFFF))
		goto wwn_prefix_out;

	/* check capability in alternative san mac address block */
	offset = alt_san_mac_blk_offset + RNP_ALT_SAN_MAC_ADDR_CAPS_OFFSET;
	hw->eeprom.ops.read(hw, offset, &caps);
	if (!(caps & RNP_ALT_SAN_MAC_ADDR_CAPS_ALTWWN))
		goto wwn_prefix_out;

	/* get the corresponding prefix for WWNN/WWPN */
	offset = alt_san_mac_blk_offset + RNP_ALT_SAN_MAC_ADDR_WWNN_OFFSET;
	hw->eeprom.ops.read(hw, offset, wwnn_prefix);

	offset = alt_san_mac_blk_offset + RNP_ALT_SAN_MAC_ADDR_WWPN_OFFSET;
	hw->eeprom.ops.read(hw, offset, wwpn_prefix);

wwn_prefix_out:
#endif
	return 0;
}

/**
 *  rnp_set_mac_anti_spoofing - Enable/Disable MAC anti-spoofing
 *  @hw: pointer to hardware structure
 *  @enable: enable or disable switch for anti-spoofing
 *  @pf: Physical Function pool - do not enable anti-spoofing for the PF
 *
 **/
void rnp_set_mac_anti_spoofing(struct rnp_hw *hw, bool enable, int pf)
{
	int j;
	int pf_target_reg = pf >> 3;
	int pf_target_shift = pf % 8;
	u32 pfvfspoof = 0;
#if 0

	if (hw->mac.type == rnp_mac_82598EB)
		return;

	if (enable)
		pfvfspoof = RNP_SPOOF_MACAS_MASK;

	/*
	 * PFVFSPOOF register array is size 8 with 8 bits assigned to
	 * MAC anti-spoof enables in each register array element.
	 */
	for (j = 0; j < pf_target_reg; j++)
		RNP_WRITE_REG(hw, RNP_PFVFSPOOF(j), pfvfspoof);

	/*
	 * The PF should be allowed to spoof so that it can support
	 * emulation mode NICs.  Do not set the bits assigned to the PF
	 */
	pfvfspoof &= (1 << pf_target_shift) - 1;
	RNP_WRITE_REG(hw, RNP_PFVFSPOOF(j), pfvfspoof);

	/*
	 * Remaining pools belong to the PF so they do not need to have
	 * anti-spoofing enabled.
	 */
	for (j++; j < RNP_PFVFSPOOF_REG_COUNT; j++)
		RNP_WRITE_REG(hw, RNP_PFVFSPOOF(j), 0);
#endif
}

/**
 *  rnp_set_vlan_anti_spoofing - Enable/Disable VLAN anti-spoofing
 *  @hw: pointer to hardware structure
 *  @enable: enable or disable switch for VLAN anti-spoofing
 *  @pf: Virtual Function pool - VF Pool to set for VLAN anti-spoofing
 *
 **/
void rnp_set_vlan_anti_spoofing(struct rnp_hw *hw, bool enable, int vf)
{
#if 0
	int vf_target_reg = vf >> 3;
	int vf_target_shift = vf % 8 + RNP_SPOOF_VLANAS_SHIFT;
	u32 pfvfspoof;

	if (hw->mac.type == rnp_mac_82598EB)
		return;

	pfvfspoof = RNP_READ_REG(hw, RNP_PFVFSPOOF(vf_target_reg));
	if (enable)
		pfvfspoof |= (1 << vf_target_shift);
	else
		pfvfspoof &= ~(1 << vf_target_shift);
	RNP_WRITE_REG(hw, RNP_PFVFSPOOF(vf_target_reg), pfvfspoof);
#endif
}

/**
 *  rnp_get_device_caps_generic - Get additional device capabilities
 *  @hw: pointer to hardware structure
 *  @device_caps: the EEPROM word with the extra device capabilities
 *
 *  This function will read the EEPROM location for the device capabilities,
 *  and return the word through device_caps.
 **/
s32 rnp_get_device_caps_generic(struct rnp_hw *hw, u16 *device_caps)
{
	// hw->eeprom.ops.read(hw, RNP_DEVICE_CAPS, device_caps);

	return 0;
}

/**
 * rnp_set_rxpba_generic - Initialize RX packet buffer
 * @hw: pointer to hardware structure
 * @num_pb: number of packet buffers to allocate
 * @headroom: reserve n KB of headroom
 * @strategy: packet buffer allocation strategy
 **/
void rnp_set_rxpba_generic(struct rnp_hw *hw,
						   int num_pb,
						   u32 headroom,
						   int strategy)
{
#if 0
	u32 pbsize = hw->mac.rx_pb_size;
	int i = 0;
	u32 rxpktsize, txpktsize, txpbthresh;

	/* Reserve headroom */
	pbsize -= headroom;

	if (!num_pb)
		num_pb = 1;

	/* Divide remaining packet buffer space amongst the number
	 * of packet buffers requested using supplied strategy.
	 */
	switch (strategy) {
	case (PBA_STRATEGY_WEIGHTED):
		/* pba_80_48 strategy weight first half of packet buffer with
		 * 5/8 of the packet buffer space.
		 */
		rxpktsize = ((pbsize * 5 * 2) / (num_pb * 8));
		pbsize -= rxpktsize * (num_pb / 2);
		rxpktsize <<= RNP_RXPBSIZE_SHIFT;
		for (; i < (num_pb / 2); i++)
			RNP_WRITE_REG(hw, RNP_RXPBSIZE(i), rxpktsize);
		/* Fall through to configure remaining packet buffers */
	case (PBA_STRATEGY_EQUAL):
		/* Divide the remaining Rx packet buffer evenly among the TCs */
		rxpktsize = (pbsize / (num_pb - i)) << RNP_RXPBSIZE_SHIFT;
		for (; i < num_pb; i++)
			RNP_WRITE_REG(hw, RNP_RXPBSIZE(i), rxpktsize);
		break;
	default:
		break;
	}

	/*
	 * Setup Tx packet buffer and threshold equally for all TCs
	 * TXPBTHRESH register is set in K so divide by 1024 and subtract
	 * 10 since the largest packet we support is just over 9K.
	 */
	txpktsize = RNP_TXPBSIZE_MAX / num_pb;
	txpbthresh = (txpktsize / 1024) - RNP_TXPKT_SIZE_MAX;
	for (i = 0; i < num_pb; i++) {
		RNP_WRITE_REG(hw, RNP_TXPBSIZE(i), txpktsize);
		RNP_WRITE_REG(hw, RNP_TXPBTHRESH(i), txpbthresh);
	}

	/* Clear unused TCs, if any, to zero buffer size*/
	for (; i < RNP_MAX_PB; i++) {
		RNP_WRITE_REG(hw, RNP_RXPBSIZE(i), 0);
		RNP_WRITE_REG(hw, RNP_TXPBSIZE(i), 0);
		RNP_WRITE_REG(hw, RNP_TXPBTHRESH(i), 0);
	}
#endif
}

/**
 *  rnp_calculate_checksum - Calculate checksum for buffer
 *  @buffer: pointer to EEPROM
 *  @length: size of EEPROM to calculate a checksum for
 *
 *  Calculates the checksum for some buffer on a specified length.  The
 *  checksum calculated is returned.
 **/
static u8 rnp_calculate_checksum(u8 *buffer, u32 length)
{
	u32 i;
	u8 sum = 0;

	if (!buffer)
		return 0;

	for (i = 0; i < length; i++)
		sum += buffer[i];

	return (u8)(0 - sum);
}

/**
 *  rnp_host_interface_command - Issue command to manageability block
 *  @hw: pointer to the HW structure
 *  @buffer: contains the command to write and where the return status will
 *           be placed
 *  @length: length of buffer, must be multiple of 4 bytes
 *
 *  Communicates with the manageability block.  On success return 0
 *  else return RNP_ERR_HOST_INTERFACE_COMMAND.
 **/
static s32
rnp_host_interface_command(struct rnp_hw *hw, u32 *buffer, u32 length)
{
	return -1;
}

/**
 *  rnp_set_fw_drv_ver_generic - Sends driver version to firmware
 *  @hw: pointer to the HW structure
 *  @maj: driver version major number
 *  @min: driver version minor number
 *  @build: driver version build number
 *  @sub: driver version sub build number
 *
 *  Sends driver version number to firmware through the manageability
 *  block.  On success return 0
 *  else returns RNP_ERR_SWFW_SYNC when encountering an error acquiring
 *  semaphore or RNP_ERR_HOST_INTERFACE_COMMAND when command fails.
 **/
s32 rnp_set_fw_drv_ver_generic(
	struct rnp_hw *hw, u8 maj, u8 min, u8 build, u8 sub)
{
	return -1;
}

/**
 * rnp_clear_tx_pending - Clear pending TX work from the PCIe fifo
 * @hw: pointer to the hardware structure
 *
 * The n10 and x540 MACs can experience issues if TX work is still pending
 * when a reset occurs.  This function prevents this by flushing the PCIe
 * buffers on the system.
 **/
void rnp_clear_tx_pending(struct rnp_hw *hw)
{
	u32 gcr_ext, hlreg0;

	/*
	 * If double reset is not requested then all transactions should
	 * already be clear and as such there is no work to do
	 */
	if (!(hw->mac.mac_flags & RNP_FLAGS_DOUBLE_RESET_REQUIRED))
		return;
}

// common ethtool add here




