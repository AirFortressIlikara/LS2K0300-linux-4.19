// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2022 - 2023 Mucse Corporation. */

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/netdevice.h>

#include "rnpgbe.h"
#include "rnpgbe_common.h"
#include "rnpgbe_mbx.h"

static s32 rnpgbe_acquire_eeprom(struct rnpgbe_hw *hw);
static s32 rnpgbe_get_eeprom_semaphore(struct rnpgbe_hw *hw);
static void rnpgbe_release_eeprom_semaphore(struct rnpgbe_hw *hw);
static s32 rnpgbe_ready_eeprom(struct rnpgbe_hw *hw);
static void rnpgbe_standby_eeprom(struct rnpgbe_hw *hw);
static void rnpgbe_shift_out_eeprom_bits(struct rnpgbe_hw *hw, u16 data,
					 u16 count);
static u16 rnpgbe_shift_in_eeprom_bits(struct rnpgbe_hw *hw, u16 count);
static void rnpgbe_raise_eeprom_clk(struct rnpgbe_hw *hw, u32 *eec);
static void rnpgbe_lower_eeprom_clk(struct rnpgbe_hw *hw, u32 *eec);
static void rnpgbe_release_eeprom(struct rnpgbe_hw *hw);

static s32 rnpgbe_mta_vector(struct rnpgbe_hw *hw, u8 *mc_addr);
static s32 rnpgbe_poll_eerd_eewr_done(struct rnpgbe_hw *hw, u32 ee_reg);
static s32 rnpgbe_read_eeprom_buffer_bit_bang(struct rnpgbe_hw *hw, u16 offset,
					      u16 words, u16 *data);
static s32 rnpgbe_write_eeprom_buffer_bit_bang(struct rnpgbe_hw *hw, u16 offset,
					       u16 words, u16 *data);
static s32 rnpgbe_detect_eeprom_page_size_generic(struct rnpgbe_hw *hw,
						  u16 offset);
static s32 rnpgbe_disable_pcie_master(struct rnpgbe_hw *hw);

unsigned int rnpgbe_loglevel = 0x00;
module_param(rnpgbe_loglevel, uint, 0600);


void rnpgbe_reset_msix_table_generic(struct rnpgbe_hw *hw)
{
	int i;

	// reset NIC_RING_VECTOR table to 0
	for (i = 0; i < 26; i++)
		rnpgbe_wr_reg(hw->ring_msix_base + RING_VECTOR(i), 0);
}


/**
 *  rnpgbe_read_pba_string_generic - Reads part number string from EEPROM
 *  @hw: pointer to hardware structure
 *  @pba_num: stores the part number string from the EEPROM
 *  @pba_num_size: part number string buffer length
 *
 *  Reads the part number string from the EEPROM.
 **/
s32 rnpgbe_read_pba_string_generic(struct rnpgbe_hw *hw, u8 *pba_num,
				   u32 pba_num_size)
{
	return 0;
}

s32 rnpgbe_get_permtion_mac_addr(struct rnpgbe_hw *hw, u8 *mac_addr)
{
#ifdef NO_CM3_MBX
	u32 v;

#ifdef FIX_MAC_TEST
	v = 0x00004E46;
#else
	v = rd32(hw, RNP_TOP_MAC_OUI);
#endif
	mac_addr[0] = (u8)(v >> 16);
	mac_addr[1] = (u8)(v >> 8);
	mac_addr[2] = (u8)(v >> 0);

#ifdef FIX_MAC_TEST
	v = 0x00032F00 + rnpgbe_is_pf1(hw->pdev);
#else
	v = rd32(hw, RNP_TOP_MAC_SN);
#endif
	mac_addr[3] = (u8)(v >> 16);
	mac_addr[4] = (u8)(v >> 8);
	mac_addr[5] = (u8)(v >> 0);
#else
	if (rnpgbe_fw_get_macaddr(hw, hw->pfvfnum, mac_addr, hw->nr_lane) ||
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
 *  rnpgbe_get_mac_addr_generic - Generic get MAC address
 *  @hw: pointer to hardware structure
 *  @mac_addr: Adapter MAC address
 *
 *  Reads the adapter's MAC address from first Receive Address Register (RAR0)
 *  A reset of the adapter must be performed prior to calling this function
 *  in order for the MAC address to have been loaded from the EEPROM into RAR0
 **/
s32 rnpgbe_get_mac_addr_generic(struct rnpgbe_hw *hw, u8 *mac_addr)
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
 *  rnpgbe_stop_adapter_generic - Generic stop Tx/Rx units
 *  @hw: pointer to hardware structure
 *
 *  Sets the adapter_stopped flag within rnpgbe_hw struct. Clears interrupts,
 *  disables transmit and receive units. The adapter_stopped flag is used by
 *  the shared code and drivers to determine if the adapter is in a stopped
 *  state and should not touch the hardware.
 **/
s32 rnpgbe_stop_adapter_generic(struct rnpgbe_hw *hw)
{
	u16 i;
	struct rnpgbe_dma_info *dma = &hw->dma;

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
	 * Prevent the PCI-E bus from hanging by disabling PCI-E master
	 * access and verify no pending requests
	 */
	return rnpgbe_disable_pcie_master(hw);
}

/**
 *  rnpgbe_led_on_generic - Turns on the software controllable LEDs.
 *  @hw: pointer to hardware structure
 *  @index: led number to turn on
 **/
s32 rnpgbe_led_on_generic(struct rnpgbe_hw *hw, u32 index)
{
	/* To turn on the LED, set mode to ON. */

	return 0;
}

/**
 *  rnpgbe_led_off_generic - Turns off the software controllable LEDs.
 *  @hw: pointer to hardware structure
 *  @index: led number to turn off
 **/
s32 rnpgbe_led_off_generic(struct rnpgbe_hw *hw, u32 index)
{
	/* To turn off the LED, set mode to OFF. */

	return 0;
}

/**
 *  rnpgbe_init_eeprom_params_generic - Initialize EEPROM params
 *  @hw: pointer to hardware structure
 *
 *  Initializes the EEPROM parameters rnpgbe_eeprom_info within the
 *  rnpgbe_hw struct in order to set up EEPROM access.
 **/
s32 rnpgbe_init_eeprom_params_generic(struct rnpgbe_hw *hw)
{
	//struct rnpgbe_eeprom_info *eeprom = &hw->eeprom;
	//u32 eec;
	//u16 eeprom_size;

	return 0;
}

/**
 *  rnpgbe_write_eeprom_buffer_bit_bang_generic - Write EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to write
 *  @words: number of words
 *  @data: 16 bit word(s) to write to EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
s32 rnpgbe_write_eeprom_buffer_bit_bang_generic(struct rnpgbe_hw *hw,
						u16 offset, u16 words,
						u16 *data)
{
	return -EINVAL;
}

/**
 *  rnpgbe_write_eeprom_buffer_bit_bang - Writes 16 bit word(s) to EEPROM
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be written to
 *  @words: number of word(s)
 *  @data: 16 bit word(s) to be written to the EEPROM
 *
 *  If rnpgbe_eeprom_update_checksum is not called after this function, the
 *  EEPROM will most likely contain an invalid checksum.
 **/
static s32 rnpgbe_write_eeprom_buffer_bit_bang(struct rnpgbe_hw *hw, u16 offset,
					       u16 words, u16 *data)
{
	return -EINVAL;
}

/**
 *  rnpgbe_write_eeprom_generic - Writes 16 bit value to EEPROM
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be written to
 *  @data: 16 bit word to be written to the EEPROM
 *
 *  If rnpgbe_eeprom_update_checksum is not called after this function, the
 *  EEPROM will most likely contain an invalid checksum.
 **/
s32 rnpgbe_write_eeprom_generic(struct rnpgbe_hw *hw, u16 offset, u16 data)
{
	s32 status;

	hw->eeprom.ops.init_params(hw);

	if (offset >= hw->eeprom.word_size) {
		status = RNP_ERR_EEPROM;
		goto out;
	}

	status = rnpgbe_write_eeprom_buffer_bit_bang(hw, offset, 1, &data);

out:
	return status;
}

/**
 *  rnpgbe_read_eeprom_buffer_bit_bang_generic - Read EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @words: number of word(s)
 *  @data: read 16 bit words(s) from EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
s32 rnpgbe_read_eeprom_buffer_bit_bang_generic(struct rnpgbe_hw *hw, u16 offset,
					       u16 words, u16 *data)
{
	return -EINVAL;
}

/**
 *  rnpgbe_read_eeprom_buffer_bit_bang - Read EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @words: number of word(s)
 *  @data: read 16 bit word(s) from EEPROM
 *
 *  Reads 16 bit word(s) from EEPROM through bit-bang method
 **/
static s32 rnpgbe_read_eeprom_buffer_bit_bang(struct rnpgbe_hw *hw, u16 offset,
					      u16 words, u16 *data)
{
	return -EINVAL;
}

/**
 *  rnpgbe_read_eeprom_bit_bang_generic - Read EEPROM word using bit-bang
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be read
 *  @data: read 16 bit value from EEPROM
 *
 *  Reads 16 bit value from EEPROM through bit-bang method
 **/
s32 rnpgbe_read_eeprom_bit_bang_generic(struct rnpgbe_hw *hw, u16 offset,
					u16 *data)
{
	s32 status;

	hw->eeprom.ops.init_params(hw);

	if (offset >= hw->eeprom.word_size) {
		status = RNP_ERR_EEPROM;
		goto out;
	}

	status = rnpgbe_read_eeprom_buffer_bit_bang(hw, offset, 1, data);

out:
	return status;
}

/**
 *  rnpgbe_read_eerd_buffer_generic - Read EEPROM word(s) using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of word in the EEPROM to read
 *  @words: number of word(s)
 *  @data: 16 bit word(s) from the EEPROM
 *
 *  Reads a 16 bit word(s) from the EEPROM using the EERD register.
 **/
s32 rnpgbe_read_eerd_buffer_generic(struct rnpgbe_hw *hw, u16 offset, u16 words,
				    u16 *data)
{
	return -EINVAL;
}

/**
 *  rnpgbe_detect_eeprom_page_size_generic - Detect EEPROM page size
 *  @hw: pointer to hardware structure
 *  @offset: offset within the EEPROM to be used as a scratch pad
 *
 *  Discover EEPROM page size by writing marching data at given offset.
 *  This function is called only when we are writing a new large buffer
 *  at given offset so the data would be overwritten anyway.
 **/
__maybe_unused static s32
rnpgbe_detect_eeprom_page_size_generic(struct rnpgbe_hw *hw, u16 offset)
{
	return -EINVAL;
}

/**
 *  rnpgbe_read_eerd_generic - Read EEPROM word using EERD
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the EERD register.
 **/
s32 rnpgbe_read_eerd_generic(struct rnpgbe_hw *hw, u16 offset, u16 *data)
{
	return rnpgbe_read_eerd_buffer_generic(hw, offset, 1, data);
}

/**
 *  rnpgbe_write_eewr_buffer_generic - Write EEPROM word(s) using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @words: number of words
 *  @data: word(s) write to the EEPROM
 *
 *  Write a 16 bit word(s) to the EEPROM using the EEWR register.
 **/
s32 rnpgbe_write_eewr_buffer_generic(struct rnpgbe_hw *hw, u16 offset,
				     u16 words, u16 *data)
{
	return -EINVAL;
}

/**
 *  rnpgbe_write_eewr_generic - Write EEPROM word using EEWR
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to write
 *  @data: word write to the EEPROM
 *
 *  Write a 16 bit word to the EEPROM using the EEWR register.
 **/
s32 rnpgbe_write_eewr_generic(struct rnpgbe_hw *hw, u16 offset, u16 data)
{
	return rnpgbe_write_eewr_buffer_generic(hw, offset, 1, &data);
}

/**
 *  rnpgbe_poll_eerd_eewr_done - Poll EERD read or EEWR write status
 *  @hw: pointer to hardware structure
 *  @ee_reg: EEPROM flag for polling
 *
 *  Polls the status bit (bit 1) of the EERD or EEWR to determine when the
 *  read or write is done respectively.
 **/
__maybe_unused static s32 rnpgbe_poll_eerd_eewr_done(struct rnpgbe_hw *hw,
						     u32 ee_reg)
{
	return -EINVAL;
}

/**
 *  rnpgbe_acquire_eeprom - Acquire EEPROM using bit-bang
 *  @hw: pointer to hardware structure
 *
 *  Prepares EEPROM for access using bit-bang method. This function should
 *  be called before issuing a command to the EEPROM.
 **/
__maybe_unused static s32 rnpgbe_acquire_eeprom(struct rnpgbe_hw *hw)
{
	s32 status = 0;

	return status;
}

/**
 *  rnpgbe_get_eeprom_semaphore - Get hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  Sets the hardware semaphores so EEPROM access can occur for bit-bang method
 **/
__maybe_unused static s32 rnpgbe_get_eeprom_semaphore(struct rnpgbe_hw *hw)
{
	return 0;
}

/**
 *  rnpgbe_release_eeprom_semaphore - Release hardware semaphore
 *  @hw: pointer to hardware structure
 *
 *  This function clears hardware semaphore bits.
 **/
__maybe_unused static void rnpgbe_release_eeprom_semaphore(struct rnpgbe_hw *hw)
{
}

/**
 *  rnpgbe_ready_eeprom - Polls for EEPROM ready
 *  @hw: pointer to hardware structure
 **/
__maybe_unused static s32 rnpgbe_ready_eeprom(struct rnpgbe_hw *hw)
{
	return -EINVAL;
}

/**
 *  rnpgbe_standby_eeprom - Returns EEPROM to a "standby" state
 *  @hw: pointer to hardware structure
 **/
__maybe_unused static void rnpgbe_standby_eeprom(struct rnpgbe_hw *hw)
{
}

/**
 *  rnpgbe_shift_out_eeprom_bits - Shift data bits out to the EEPROM.
 *  @hw: pointer to hardware structure
 *  @data: data to send to the EEPROM
 *  @count: number of bits to shift out
 **/
__maybe_unused static void rnpgbe_shift_out_eeprom_bits(struct rnpgbe_hw *hw,
							u16 data, u16 count)
{
}

/**
 *  rnpgbe_shift_in_eeprom_bits - Shift data bits in from the EEPROM
 *  @hw: pointer to hardware structure
 **/
__maybe_unused static u16 rnpgbe_shift_in_eeprom_bits(struct rnpgbe_hw *hw,
						      u16 count)
{
	return 0;
}

/**
 *  rnpgbe_raise_eeprom_clk - Raises the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eec: EEC register's current value
 **/
__maybe_unused static void rnpgbe_raise_eeprom_clk(struct rnpgbe_hw *hw,
						   u32 *eec)
{
}

/**
 *  rnpgbe_lower_eeprom_clk - Lowers the EEPROM's clock input.
 *  @hw: pointer to hardware structure
 *  @eecd: EECD's current value
 **/
__maybe_unused static void rnpgbe_lower_eeprom_clk(struct rnpgbe_hw *hw,
						   u32 *eec)
{
}

/**
 *  rnpgbe_release_eeprom - Release EEPROM, release semaphores
 *  @hw: pointer to hardware structure
 **/
__maybe_unused static void rnpgbe_release_eeprom(struct rnpgbe_hw *hw)
{
}

/**
 *  rnpgbe_calc_eeprom_checksum_generic - Calculates and returns the checksum
 *  @hw: pointer to hardware structure
 **/
u16 rnpgbe_calc_eeprom_checksum_generic(struct rnpgbe_hw *hw)
{
	return 0;
}

/**
 *  rnpgbe_validate_eeprom_checksum_generic - Validate EEPROM checksum
 *  @hw: pointer to hardware structure
 *  @checksum_val: calculated checksum
 *
 *  Performs checksum calculation and validates the EEPROM checksum.  If the
 *  caller does not need checksum_val, the value can be NULL.
 **/
s32 rnpgbe_validate_eeprom_checksum_generic(struct rnpgbe_hw *hw,
					    u16 *checksum_val)
{
	return 0;
}

/**
 *  rnpgbe_update_eeprom_checksum_generic - Updates the EEPROM checksum
 *  @hw: pointer to hardware structure
 **/
s32 rnpgbe_update_eeprom_checksum_generic(struct rnpgbe_hw *hw)
{
	return 0;
}



/**
 *  rnpgbe_mta_vector - Determines bit-vector in multicast table to set
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
static s32 rnpgbe_mta_vector(struct rnpgbe_hw *hw, u8 *mc_addr)
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

static void rnpgbe_set_vf_mta(struct rnpgbe_hw *hw, u16 vector)
{
	u32 vector_bit;
	u32 vector_reg;

	hw->addr_ctrl.mta_in_use++;

	vector_reg = (vector >> 5) & 0x7F;
	vector_bit = vector & 0x1F;
	hw_dbg(hw, "\t\t vf M: MTA-BIT:%4d, MTA_REG[%d][%d] <= 1\n", vector,
	       vector_reg, vector_bit);
	hw->mac.mta_shadow[vector_reg] |= (1 << vector_bit);
}
/**
 *  rnpgbe_set_mta - Set bit-vector in multicast table
 *  @hw: pointer to hardware structure
 *  @hash_value: Multicast address hash value
 *
 *  Sets the bit-vector in the multicast table.
 **/
static void rnpgbe_set_mta(struct rnpgbe_hw *hw, u8 *mc_addr)
{
	u32 vector;
	u32 vector_bit;
	u32 vector_reg;

	hw->addr_ctrl.mta_in_use++;

	vector = rnpgbe_mta_vector(hw, mc_addr);

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
	hw_dbg(hw, "\t\t%pM: MTA-BIT:%4d, MTA_REG[%d][%d] <= 1\n", mc_addr,
	       vector, vector_reg, vector_bit);
	hw->mac.mta_shadow[vector_reg] |= (1 << vector_bit);
}

u8 *rnpgbe_addr_list_itr(struct rnpgbe_hw __maybe_unused *hw, u8 **mc_addr_ptr)
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
 *  rnpgbe_update_mc_addr_list_generic - Updates MAC list of multicast addresses
 *  @hw: pointer to hardware structure
 *  @netdev: pointer to net device structure
 *
 *  The given list replaces any existing list. Clears the MC addrs from receive
 *  address registers and the multicast table. Uses unused receive address
 *  registers for the first multicast addresses, and hashes the rest into the
 *  multicast table.
 **/
s32 rnpgbe_update_mc_addr_list_generic(struct rnpgbe_hw *hw,
				       struct net_device *netdev)
{
#ifdef NETDEV_HW_ADDR_T_MULTICAST
	struct netdev_hw_addr *ha;
#endif
	u32 i;
	u32 v;
	struct rnpgbe_adapter *adapter = (struct rnpgbe_adapter *)hw->back;
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
	ha = list_first_entry(&netdev->mc.list, struct netdev_hw_addr, list);
	addr_list = ha->addr;
#else
	addr_list = netdev->mc_list->dmi_addr;
#endif
	for (i = 0; i < addr_count; i++) {
		hw_dbg(hw, " Adding the multicast addresses:\n");
		rnpgbe_set_mta(hw, rnpgbe_addr_list_itr(hw, &addr_list));
	}

	//netdev_for_each_mc_addr(ha, netdev) {
	//	rnpgbe_set_mta(hw, ha->addr);
	//}

	// sriov mode should set for vf multicast
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		for (i = 0; i < adapter->num_vfs; i++) {
			struct vf_data_storage *vfinfo = &adapter->vfinfo[i];
			int j;

			for (j = 0; j < vfinfo->num_vf_mc_hashes; j++)
				rnpgbe_set_vf_mta(hw, vfinfo->vf_mc_hashes[j]);
		}
	}

	/* Enable mta */
	for (i = 0; i < hw->mac.mcft_size; i++) {
		if (hw->addr_ctrl.mta_in_use)
			wr32(hw, RNP_ETH_MUTICAST_HASH_TABLE(i),
			     hw->mac.mta_shadow[i]);
	}

	if (hw->addr_ctrl.mta_in_use > 0) {
		v = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
		wr32(hw, RNP_ETH_DMAC_MCSTCTRL,
		     v | RNP_MCSTCTRL_MULTICASE_TBL_EN |
			     hw->mac.mc_filter_type);
	}

	hw_dbg(hw, " update MTA Done. mta_in_use:%d\n",
	       hw->addr_ctrl.mta_in_use);
	return hw->addr_ctrl.mta_in_use;
}

/**
 *  rnpgbe_enable_mc_generic - Enable multicast address in RAR
 *  @hw: pointer to hardware structure
 *
 *  Enables multicast address in RAR and the use of the multicast hash table.
 **/
s32 rnpgbe_enable_mc_generic(struct rnpgbe_hw *hw)
{
	struct rnpgbe_addr_filter_info *a = &hw->addr_ctrl;
	u32 v;

	if (a->mta_in_use > 0) {
		v = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
		v |= RNP_MCSTCTRL_MULTICASE_TBL_EN;
		wr32(hw, RNP_ETH_DMAC_MCSTCTRL, v);
	}

	return 0;
}

/**
 *  rnpgbe_disable_mc_generic - Disable multicast address in RAR
 *  @hw: pointer to hardware structure
 *
 *  Disables multicast address in RAR and the use of the multicast hash table.
 **/
s32 rnpgbe_disable_mc_generic(struct rnpgbe_hw *hw)
{
	struct rnpgbe_addr_filter_info *a = &hw->addr_ctrl;
	u32 v;

	if (a->mta_in_use > 0) {
		v = rd32(hw, RNP_ETH_DMAC_MCSTCTRL);
		v &= ~RNP_MCSTCTRL_MULTICASE_TBL_EN;
		wr32(hw, RNP_ETH_DMAC_MCSTCTRL, v);
	}

	return 0;
}

/**
 *  rnpgbe_fc_enable_generic - Enable flow control
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to the current settings.
 **/
s32 rnpgbe_fc_enable_generic(struct rnpgbe_hw *hw)
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
		if ((hw->fc.current_mode & rnpgbe_fc_tx_pause) &&
		    hw->fc.high_water[i]) {
			if (!hw->fc.low_water[i] ||
			    hw->fc.low_water[i] >= hw->fc.high_water[i]) {
				hw_dbg(hw,
				       "Invalid water mark configuration\n");
				ret_val = RNP_ERR_INVALID_LINK_SETTINGS;
				goto out;
			}
		}
	}


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
	case rnpgbe_fc_none:
		/*
		 * Flow control is disabled by software override or autoneg.
		 * The code below will actually disable it in the HW.
		 */
		break;
	case rnpgbe_fc_rx_pause:
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
	case rnpgbe_fc_tx_pause:
		/*
		 * Tx Flow control is enabled, and Rx Flow control is
		 * disabled by software override.
		 */
		for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++)
			txctl_reg[i] |= (RNP_TX_FLOW_ENABLE_MASK);
		break;
	case rnpgbe_fc_full:
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
		if ((hw->fc.current_mode & rnpgbe_fc_tx_pause)) {
			if (hw->fc.high_water[i]) {
				wr32(hw, RNP_ETH_HIGH_WATER(i),
				     hw->fc.high_water[i]);
			}
			if (hw->fc.low_water[i]) {
				wr32(hw, RNP_ETH_LOW_WATER(i),
				     hw->fc.low_water[i]);
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
 *  rnpgbe_disable_pcie_master - Disable PCI-express master access
 *  @hw: pointer to hardware structure
 *
 *  Disables PCI-Express master access and verifies there are no pending
 *  requests. RNP_ERR_MASTER_REQUESTS_PENDING is returned if master disable
 *  bit hasn't caused the master requests to be disabled, else 0
 *  is returned signifying master requests disabled.
 **/
static s32 rnpgbe_disable_pcie_master(struct rnpgbe_hw *hw)
{
	//struct rnpgbe_adapter *adapter = hw->back;

	// disable dma rx/tx
	wr32(hw, RNP_DMA_AXI_EN, 0);

	return 0;
}


/**
 *  rnpgbe_clear_vmdq_generic - Disassociate a VMDq pool index from a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to disassociate
 *  @vmdq: VMDq pool index to remove from the rar
 **/
s32 rnpgbe_clear_vmdq_generic(struct rnpgbe_hw *hw, u32 rar, u32 vmdq)
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
 *  rnpgbe_set_vmdq_generic - Associate a VMDq pool index with a rx address
 *  @hw: pointer to hardware struct
 *  @rar: receive address register index to associate with a VMDq index
 *  @vmdq: VMDq pool index
 **/
s32 rnpgbe_set_vmdq_generic(struct rnpgbe_hw *hw, u32 rar, u32 vmdq)
{
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
 *  rnpgbe_init_uta_tables_generic - Initialize the Unicast Table Array
 *  @hw: pointer to hardware structure
 **/
s32 rnpgbe_init_uta_tables_generic(struct rnpgbe_hw *hw)
{
	int i;

	for (i = 0; i < 128; i++)
		wr32(hw, RNP_ETH_UTA(i), 0);
	return 0;
}

/**
 *  rnpgbe_find_vlvf_slot - find the vlanid or the first empty slot
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *
 *  return the VLVF index where this VLAN id should be placed
 *
 **/
__maybe_unused static s32 rnpgbe_find_vlvf_slot(struct rnpgbe_hw *hw, u32 vlan)
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
 *  rnpgbe_set_vfta_generic - Set VLAN filter table
 *  @hw: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vind: VMDq output index that maps queue to VLAN id in VFVFB
 *  @vlan_on: boolean flag to turn on/off VLAN in VFVF
 *
 *  Turn on/off specified VLAN in the VLAN filter table.
 **/
s32 rnpgbe_set_vfta_generic(struct rnpgbe_hw *hw, u32 vlan, u32 vind,
			    bool vlan_on)
{
	s32 regindex;
	u32 bitindex;
	u32 vfta;
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

	if (vfta_changed)
		wr32(hw, RNP_VFTA(regindex), vfta);
	return 0;
}

/**
 *  rnpgbe_clear_vfta_generic - Clear VLAN filter table
 *  @hw: pointer to hardware structure
 *
 *  Clears the VLAN filer table, and the VMDq index associated with the filter
 **/
s32 rnpgbe_clear_vfta_generic(struct rnpgbe_hw *hw)
{
	u32 offset;

	for (offset = 0; offset < hw->mac.vft_size; offset++)
		wr32(hw, RNP_VFTA(offset), 0);

	for (offset = 0; offset < RNP_VLVF_ENTRIES; offset++)
		wr32(hw, RNP_VLVF(offset), 0);

	return 0;
}

/**
 *  rnpgbe_check_mac_link_generic - Determine link and speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true when link is up
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 rnpgbe_check_mac_link_generic(struct rnpgbe_hw *hw,
				  rnpgbe_link_speed *speed, bool *link_up,
				  bool link_up_wait_to_complete)
{
#ifdef NO_MBX_VERSION
	struct rnpgbe_pcs_info *pcs = &hw->pcs;
	u32 status;

	// if vu440 , we assume link always
	if (hw->rss_type == rnpgbe_rss_uv440) {
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
skip_get_link:
#else

	if (hw->speed == 10)
		*speed = RNP_LINK_SPEED_10_FULL;
	else if (hw->speed == 100)
		*speed = RNP_LINK_SPEED_100_FULL;
	else if (hw->speed == 1000)
		*speed = RNP_LINK_SPEED_1GB_FULL;
	else if (hw->speed == 10000)
		*speed = RNP_LINK_SPEED_10GB_FULL;
	else if (hw->speed == 25000)
		*speed = RNP_LINK_SPEED_25GB_FULL;
	else if (hw->speed == 40000)
		*speed = RNP_LINK_SPEED_40GB_FULL;
	else
		*speed = RNP_LINK_SPEED_UNKNOWN;

	*link_up = hw->link;
#endif

	return 0;
}



/**
 *  rnpgbe_get_device_caps_generic - Get additional device capabilities
 *  @hw: pointer to hardware structure
 *  @device_caps: the EEPROM word with the extra device capabilities
 *
 *  This function will read the EEPROM location for the device capabilities,
 *  and return the word through device_caps.
 **/
s32 rnpgbe_get_device_caps_generic(struct rnpgbe_hw *hw, u16 *device_caps)
{
	// hw->eeprom.ops.read(hw, RNP_DEVICE_CAPS, device_caps);

	return 0;
}


/**
 *  rnpgbe_calculate_checksum - Calculate checksum for buffer
 *  @buffer: pointer to EEPROM
 *  @length: size of EEPROM to calculate a checksum for
 *
 *  Calculates the checksum for some buffer on a specified length.  The
 *  checksum calculated is returned.
 **/
__maybe_unused static u8 rnpgbe_calculate_checksum(u8 *buffer, u32 length)
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
 *  rnpgbe_host_interface_command - Issue command to manageability block
 *  @hw: pointer to the HW structure
 *  @buffer: contains the command to write and where the return status will
 *           be placed
 *  @length: length of buffer, must be multiple of 4 bytes
 *
 *  Communicates with the manageability block.  On success return 0
 *  else return RNP_ERR_HOST_INTERFACE_COMMAND.
 **/
__maybe_unused static s32 rnpgbe_host_interface_command(struct rnpgbe_hw *hw,
							u32 *buffer, u32 length)
{
	return -1;
}

/**
 *  rnpgbe_set_fw_drv_ver_generic - Sends driver version to firmware
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
s32 rnpgbe_set_fw_drv_ver_generic(struct rnpgbe_hw *hw, u8 maj, u8 min,
				  u8 build, u8 sub)
{
	return -1;
}

/**
 * rnpgbe_clear_tx_pending - Clear pending TX work from the PCIe fifo
 * @hw: pointer to the hardware structure
 *
 * The n10 and x540 MACs can experience issues if TX work is still pending
 * when a reset occurs.  This function prevents this by flushing the PCIe
 * buffers on the system.
 **/
void rnpgbe_clear_tx_pending(struct rnpgbe_hw *hw)
{
	//u32 gcr_ext, hlreg0;

	/*
	 * If double reset is not requested then all transactions should
	 * already be clear and as such there is no work to do
	 */
	if (!(hw->mac.mac_flags & RNP_FLAGS_DOUBLE_RESET_REQUIRED))
		return;
}

