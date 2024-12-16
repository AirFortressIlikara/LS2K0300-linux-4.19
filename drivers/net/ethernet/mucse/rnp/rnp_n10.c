#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include "rnp.h"
#include "rnp_phy.h"
#include "rnp_mbx.h"
#include "rnp_pcs.h"
#include "rnp_ethtool.h"
#include "rnp_sriov.h"

#define RNP_N400_MAX_VF 8
#define RNP_N400_RSS_TBL_NUM 128
#define RNP_N400_RSS_TC_TBL_NUM 8
#define RNP_N400_MAX_TX_QUEUES 8
#define RNP_N400_MAX_RX_QUEUES 8
#define RNP_N400_RAR_NCSI_RAR_ENTRIES 0

#define RNP_N10_MAX_VF 64
#define RNP_N10_RSS_TBL_NUM 128
#define RNP_N10_RSS_TC_TBL_NUM 8
#define RNP_N10_MAX_TX_QUEUES 128
#define RNP_N10_MAX_RX_QUEUES 128
#define RNP_N10_RAR_NCSI_RAR_ENTRIES 0
#ifdef NIC_VF_FXIED
#define RNP_N10_RAR_ENTRIES (127 - RNP_N10_RAR_NCSI_RAR_ENTRIES)
#else
#define RNP_N10_RAR_ENTRIES (128 - RNP_N10_RAR_NCSI_RAR_ENTRIES)
#endif
#define RNP_N10_MC_TBL_SIZE 128
#define RNP_N10_VFT_TBL_SIZE 128
#define RNP_N10_RX_PB_SIZE 512
#ifndef RNP_N10_MSIX_VECTORS
#define RNP_N10_MSIX_VECTORS 64
#endif
#define RNP_N400_MSIX_VECTORS 17

#define RNP10_MAX_LAYER2_FILTERS 16
#define RNP10_MAX_TCAM_FILTERS 4096
#define RNP10_MAX_TUPLE5_FILTERS 128

static bool rnp_mng_enabled(struct rnp_hw *hw)
{
	return false;
}

static void rnp_init_mac_link_ops_n10(struct rnp_hw *hw)
{
}


/**
 *  rnp_init_phy_ops_n10 - PHY/SFP specific init
 *  @hw: pointer to hardware structure
 *
 *  Initialize any function pointers that were not able to be
 *  set during get_invariants because the PHY/SFP type was
 *  not known.  Perform the SFP init if necessary.
 *
 **/
static s32 rnp_init_phy_ops_n10(struct rnp_hw *hw)
{
	struct rnp_mac_info *mac = &hw->mac;
	struct rnp_phy_info *phy = &hw->phy;
	s32 ret_val = 0;

	hw->phy.sfp_setup_needed = true;

	return ret_val;
}

static s32 rnp_setup_sfp_modules_n10(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_reinit_fdir_tables_n10 - Reinitialize Flow Director tables.
 *  @hw: pointer to hardware structure
 **/
s32 rnp_reinit_fdir_tables_n10(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_fdir_enable_n10 - Initialize Flow Director control registers
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register
 **/
static void rnp_fdir_enable_n10(struct rnp_hw *hw, u32 fdirctrl)
{
}

/**
 *  rnp_init_fdir_signature_n10 - Initialize Flow Director signature filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *             contains just the value of the Rx packet buffer allocation
 **/
s32 rnp_init_fdir_signature_n10(struct rnp_hw *hw, u32 fdirctrl)
{
#if 0
	/*
	 * Continue setup of fdirctrl register bits:
	 *  Move the flexible bytes to use the ethertype - shift 6 words
	 *  Set the maximum length per hash bucket to 0xA filters
	 *  Send interrupt when 64 filters are left
	 */
	fdirctrl |= (0x6 << RNP_FDIRCTRL_FLEX_SHIFT) |
		(0xA << RNP_FDIRCTRL_MAX_LENGTH_SHIFT) |
		(4 << RNP_FDIRCTRL_FULL_THRESH_SHIFT);

	/* write hashes and fdirctrl register, poll for completion */
	rnp_fdir_enable_n10(hw, fdirctrl);
#endif

	return 0;
}

/**
 *  rnp_init_fdir_perfect_n10 - Initialize Flow Director perfect filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *             contains just the value of the Rx packet buffer allocation
 **/
s32 rnp_init_fdir_perfect_n10(struct rnp_hw *hw, u32 fdirctrl)
{
#if 0
	/*
	 * Continue setup of fdirctrl register bits:
	 *  Turn perfect match filtering on
	 *  Report hash in RSS field of Rx wb descriptor
	 *  Initialize the drop queue
	 *  Move the flexible bytes to use the ethertype - shift 6 words
	 *  Set the maximum length per hash bucket to 0xA filters
	 *  Send interrupt when 64 (0x4 * 16) filters are left
	 */
	fdirctrl |= RNP_FDIRCTRL_PERFECT_MATCH |
		RNP_FDIRCTRL_REPORT_STATUS |
		(RNP_FDIR_DROP_QUEUE << RNP_FDIRCTRL_DROP_Q_SHIFT) |
		(0x6 << RNP_FDIRCTRL_FLEX_SHIFT) |
		(0xA << RNP_FDIRCTRL_MAX_LENGTH_SHIFT) |
		(4 << RNP_FDIRCTRL_FULL_THRESH_SHIFT);

	/* write hashes and fdirctrl register, poll for completion */
	rnp_fdir_enable_n10(hw, fdirctrl);

#endif
	return 0;
}

/*
 * These defines allow us to quickly generate all of the necessary instructions
 * in the function below by simply calling out RNP_COMPUTE_SIG_HASH_ITERATION
 * for values 0 through 15
 */
#define RNP_ATR_COMMON_HASH_KEY                                                \
	(RNP_ATR_BUCKET_HASH_KEY & RNP_ATR_SIGNATURE_HASH_KEY)
#define RNP_COMPUTE_SIG_HASH_ITERATION(_n)                                     \
	do {                                                                   \
	} while (0)

/**
 *  rnp_atr_compute_sig_hash_n10 - Compute the signature hash
 *  @stream: input bitstream to compute the hash on
 *
 *  This function is almost identical to the function above but contains
 *  several optomizations such as unwinding all of the loops, letting the
 *  compiler work out all of the conditional ifs since the keys are static
 *  defines, and computing two keys at once since the hashed dword stream
 *  will be the same for both keys.
 **/
static u32 rnp_atr_compute_sig_hash_n10(union rnp_atr_hash_dword input,
					  union rnp_atr_hash_dword common)
{
#if 0
	u32 hi_hash_dword, lo_hash_dword, flow_vm_vlan;
	u32 sig_hash = 0, bucket_hash = 0, common_hash = 0;

	/* record the flow_vm_vlan bits as they are a key part to the hash */
	flow_vm_vlan = ntohl(input.dword);

	/* generate common hash dword */
	hi_hash_dword = ntohl(common.dword);

	/* low dword is word swapped version of common */
	lo_hash_dword = (hi_hash_dword >> 16) | (hi_hash_dword << 16);

	/* apply flow ID/VM pool/VLAN ID bits to hash words */
	hi_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan >> 16);

	/* Process bits 0 and 16 */
	RNP_COMPUTE_SIG_HASH_ITERATION(0);

	/*
	 * apply flow ID/VM pool/VLAN ID bits to lo hash dword, we had to
	 * delay this because bit 0 of the stream should not be processed
	 * so we do not add the vlan until after bit 0 was processed
	 */
	lo_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan << 16);

	/* Process remaining 30 bit of the key */
	RNP_COMPUTE_SIG_HASH_ITERATION(1);
	RNP_COMPUTE_SIG_HASH_ITERATION(2);
	RNP_COMPUTE_SIG_HASH_ITERATION(3);
	RNP_COMPUTE_SIG_HASH_ITERATION(4);
	RNP_COMPUTE_SIG_HASH_ITERATION(5);
	RNP_COMPUTE_SIG_HASH_ITERATION(6);
	RNP_COMPUTE_SIG_HASH_ITERATION(7);
	RNP_COMPUTE_SIG_HASH_ITERATION(8);
	RNP_COMPUTE_SIG_HASH_ITERATION(9);
	RNP_COMPUTE_SIG_HASH_ITERATION(10);
	RNP_COMPUTE_SIG_HASH_ITERATION(11);
	RNP_COMPUTE_SIG_HASH_ITERATION(12);
	RNP_COMPUTE_SIG_HASH_ITERATION(13);
	RNP_COMPUTE_SIG_HASH_ITERATION(14);
	RNP_COMPUTE_SIG_HASH_ITERATION(15);

	/* combine common_hash result with signature and bucket hashes */
	bucket_hash ^= common_hash;
	bucket_hash &= RNP_ATR_HASH_MASK;

	sig_hash ^= common_hash << 16;
	sig_hash &= RNP_ATR_HASH_MASK << 16;

	/* return completed signature hash */
	return sig_hash ^ bucket_hash;
#else
	return 0;
#endif
}

/**
 *  rnp_atr_add_signature_filter_n10 - Adds a signature hash filter
 *  @hw: pointer to hardware structure
 *  @input: unique input dword
 *  @common: compressed common input dword
 *  @queue: queue index to direct traffic to
 **/
s32 rnp_fdir_add_signature_filter_n10(struct rnp_hw *hw,
					union rnp_atr_hash_dword input,
					union rnp_atr_hash_dword common,
					u8 queue)
{
	u64 fdirhashcmd;
	u32 fdircmd;
#if 0
	/*
	 * Get the flow_type in order to program FDIRCMD properly
	 * lowest 2 bits are FDIRCMD.L4TYPE, third lowest bit is FDIRCMD.IPV6
	 */
	switch (input.formatted.flow_type) {
	case RNP_ATR_FLOW_TYPE_TCPV4:
	case RNP_ATR_FLOW_TYPE_UDPV4:
	case RNP_ATR_FLOW_TYPE_SCTPV4:
	case RNP_ATR_FLOW_TYPE_TCPV6:
	case RNP_ATR_FLOW_TYPE_UDPV6:
	case RNP_ATR_FLOW_TYPE_SCTPV6:
		break;
	default:
		hw_dbg(hw, " Error on flow type input\n");
		return RNP_ERR_CONFIG;
	}

	/* configure FDIRCMD register */
	fdircmd = RNP_FDIRCMD_CMD_ADD_FLOW | RNP_FDIRCMD_FILTER_UPDATE |
		RNP_FDIRCMD_LAST | RNP_FDIRCMD_QUEUE_EN;
	fdircmd |= input.formatted.flow_type << RNP_FDIRCMD_FLOW_TYPE_SHIFT;
	fdircmd |= (u32)queue << RNP_FDIRCMD_RX_QUEUE_SHIFT;

	/*
	 * The lower 32-bits of fdirhashcmd is for FDIRHASH, the upper 32-bits
	 * is for FDIRCMD.  Then do a 64-bit register write from FDIRHASH.
	 */
	fdirhashcmd = (u64)fdircmd << 32;
	fdirhashcmd |= rnp_atr_compute_sig_hash_n10(input, common);
	//RNP_WRITE_REG64(hw, RNP_FDIRHASH, fdirhashcmd);

	hw_dbg(hw, "Tx Queue=%x hash=%x\n", queue, (u32)fdirhashcmd);
#endif
	return 0;
}

#define RNP_COMPUTE_BKT_HASH_ITERATION(_n)                                     \
	do {                                                                   \
		u32 n = (_n);                                                  \
		if (RNP_ATR_BUCKET_HASH_KEY & (0x01 << n))                     \
			bucket_hash ^= lo_hash_dword >> n;                     \
		if (RNP_ATR_BUCKET_HASH_KEY & (0x01 << (n + 16)))              \
			bucket_hash ^= hi_hash_dword >> n;                     \
	} while (0)

/**
 *  rnp_atr_compute_perfect_hash_n10 - Compute the perfect filter hash
 *  @atr_input: input bitstream to compute the hash on
 *  @input_mask: mask for the input bitstream
 *
 *  This function serves two main purposes.  First it applys the input_mask
 *  to the atr_input resulting in a cleaned up atr_input data stream.
 *  Secondly it computes the hash and stores it in the bkt_hash field at
 *  the end of the input byte stream.  This way it will be available for
 *  future use without needing to recompute the hash.
 **/
void rnp_atr_compute_perfect_hash_n10(union rnp_atr_input *input,
					union rnp_atr_input *input_mask)
{
	u32 hi_hash_dword, lo_hash_dword, flow_vm_vlan;
	u32 bucket_hash = 0;
#if 0

	/* Apply masks to input data */
	input->dword_stream[0]  &= input_mask->dword_stream[0];
	input->dword_stream[1]  &= input_mask->dword_stream[1];
	input->dword_stream[2]  &= input_mask->dword_stream[2];
	input->dword_stream[3]  &= input_mask->dword_stream[3];
	input->dword_stream[4]  &= input_mask->dword_stream[4];
	input->dword_stream[5]  &= input_mask->dword_stream[5];
	input->dword_stream[6]  &= input_mask->dword_stream[6];
	input->dword_stream[7]  &= input_mask->dword_stream[7];
	input->dword_stream[8]  &= input_mask->dword_stream[8];
	input->dword_stream[9]  &= input_mask->dword_stream[9];
	input->dword_stream[10] &= input_mask->dword_stream[10];

	/* record the flow_vm_vlan bits as they are a key part to the hash */
	flow_vm_vlan = ntohl(input->dword_stream[0]);

	/* generate common hash dword */
	hi_hash_dword = ntohl(input->dword_stream[1] ^
			input->dword_stream[2] ^
			input->dword_stream[3] ^
			input->dword_stream[4] ^
			input->dword_stream[5] ^
			input->dword_stream[6] ^
			input->dword_stream[7] ^
			input->dword_stream[8] ^
			input->dword_stream[9] ^
			input->dword_stream[10]);

	/* low dword is word swapped version of common */
	lo_hash_dword = (hi_hash_dword >> 16) | (hi_hash_dword << 16);

	/* apply flow ID/VM pool/VLAN ID bits to hash words */
	hi_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan >> 16);

	/* Process bits 0 and 16 */
	RNP_COMPUTE_BKT_HASH_ITERATION(0);

	/*
	 * apply flow ID/VM pool/VLAN ID bits to lo hash dword, we had to
	 * delay this because bit 0 of the stream should not be processed
	 * so we do not add the vlan until after bit 0 was processed
	 */
	lo_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan << 16);

	/* Process remaining 30 bit of the key */
	RNP_COMPUTE_BKT_HASH_ITERATION(1);
	RNP_COMPUTE_BKT_HASH_ITERATION(2);
	RNP_COMPUTE_BKT_HASH_ITERATION(3);
	RNP_COMPUTE_BKT_HASH_ITERATION(4);
	RNP_COMPUTE_BKT_HASH_ITERATION(5);
	RNP_COMPUTE_BKT_HASH_ITERATION(6);
	RNP_COMPUTE_BKT_HASH_ITERATION(7);
	RNP_COMPUTE_BKT_HASH_ITERATION(8);
	RNP_COMPUTE_BKT_HASH_ITERATION(9);
	RNP_COMPUTE_BKT_HASH_ITERATION(10);
	RNP_COMPUTE_BKT_HASH_ITERATION(11);
	RNP_COMPUTE_BKT_HASH_ITERATION(12);
	RNP_COMPUTE_BKT_HASH_ITERATION(13);
	RNP_COMPUTE_BKT_HASH_ITERATION(14);
	RNP_COMPUTE_BKT_HASH_ITERATION(15);

	/*
	 * Limit hash to 13 bits since max bucket count is 8K.
	 * Store result at the end of the input stream.
	 */
	input->formatted.bkt_hash = bucket_hash & 0x1FFF;
#endif
}

/**
 *  rnp_get_fdirtcpm_n10 - generate a tcp port from atr_input_masks
 *  @input_mask: mask to be bit swapped
 *
 *  The source and destination port masks for flow director are bit swapped
 *  in that bit 15 effects bit 0, 14 effects 1, 13, 2 etc.  In order to
 *  generate a correctly swapped value we need to bit swap the mask and that
 *  is what is accomplished by this function.
 **/
static u32 rnp_get_fdirtcpm_n10(union rnp_atr_input *input_mask)
{
#if 0
	u32 mask = ntohs(input_mask->formatted.dst_port);

	mask <<= RNP_FDIRTCPM_DPORTM_SHIFT;
	mask |= ntohs(input_mask->formatted.src_port);
	mask = ((mask & 0x55555555) << 1) | ((mask & 0xAAAAAAAA) >> 1);
	mask = ((mask & 0x33333333) << 2) | ((mask & 0xCCCCCCCC) >> 2);
	mask = ((mask & 0x0F0F0F0F) << 4) | ((mask & 0xF0F0F0F0) >> 4);
	return ((mask & 0x00FF00FF) << 8) | ((mask & 0xFF00FF00) >> 8);
#else
	return 0;
#endif
}

/*
 * These two macros are meant to address the fact that we have registers
 * that are either all or in part big-endian.  As a result on big-endian
 * systems we will end up byte swapping the value to little-endian before
 * it is byte swapped again and written to the hardware in the original
 * big-endian format.
 */
#define RNP_STORE_AS_BE32(_value)                                              \
	(((u32)(_value) >> 24) | (((u32)(_value)&0x00FF0000) >> 8) |           \
	 (((u32)(_value)&0x0000FF00) << 8) | ((u32)(_value) << 24))

#define RNP_WRITE_REG_BE32(a, reg, value)                                      \
	RNP_WRITE_REG((a), (reg), RNP_STORE_AS_BE32(ntohl(value)))

#define RNP_STORE_AS_BE16(_value)                                              \
	ntohs(((u16)(_value) >> 8) | ((u16)(_value) << 8))

s32 rnp_fdir_set_input_mask_n10(struct rnp_hw *hw,
				  union rnp_atr_input *input_mask)
{
#if 0
	/* mask IPv6 since it is currently not supported */
	u32 fdirm = RNP_FDIRM_DIPv6;
	u32 fdirtcpm;

	/*
	 * Program the relevant mask registers.  If src/dst_port or src/dst_addr
	 * are zero, then assume a full mask for that field.  Also assume that
	 * a VLAN of 0 is unspecified, so mask that out as well.  L4type
	 * cannot be masked out in this implementation.
	 *
	 * This also assumes IPv4 only.  IPv6 masking isn't supported at this
	 * point in time.
	 */

	/* verify bucket hash is cleared on hash generation */
	if (input_mask->formatted.bkt_hash)
		hw_dbg(hw, " bucket hash should always be 0 in mask\n");

	/* Program FDIRM and verify partial masks */
	switch (input_mask->formatted.vm_pool & 0x7F) {
	case 0x0:
		fdirm |= RNP_FDIRM_POOL;
	case 0x7F:
		break;
	default:
		hw_dbg(hw, " Error on vm pool mask\n");
		return RNP_ERR_CONFIG;
	}

	switch (input_mask->formatted.flow_type & RNP_ATR_L4TYPE_MASK) {
	case 0x0:
		fdirm |= RNP_FDIRM_L4P;
		if (input_mask->formatted.dst_port ||
				input_mask->formatted.src_port) {
			hw_dbg(hw, " Error on src/dst port mask\n");
			return RNP_ERR_CONFIG;
		}
	case RNP_ATR_L4TYPE_MASK:
		break;
	default:
		hw_dbg(hw, " Error on flow type mask\n");
		return RNP_ERR_CONFIG;
	}

	switch (ntohs(input_mask->formatted.vlan_id) & 0xEFFF) {
	case 0x0000:
		/* mask VLAN ID, fall through to mask VLAN priority */
		fdirm |= RNP_FDIRM_VLANID;
	case 0x0FFF:
		/* mask VLAN priority */
		fdirm |= RNP_FDIRM_VLANP;
		break;
	case 0xE000:
		/* mask VLAN ID only, fall through */
		fdirm |= RNP_FDIRM_VLANID;
	case 0xEFFF:
		/* no VLAN fields masked */
		break;
	default:
		hw_dbg(hw, " Error on VLAN mask\n");
		return RNP_ERR_CONFIG;
	}

	switch (input_mask->formatted.flex_bytes & 0xFFFF) {
	case 0x0000:
		/* Mask Flex Bytes, fall through */
		fdirm |= RNP_FDIRM_FLEX;
	case 0xFFFF:
		break;
	default:
		hw_dbg(hw, " Error on flexible byte mask\n");
		return RNP_ERR_CONFIG;
	}

	/* Now mask VM pool and destination IPv6 - bits 5 and 2 */
	RNP_WRITE_REG(hw, RNP_FDIRM, fdirm);

	/* store the TCP/UDP port masks, bit reversed from port layout */
	fdirtcpm = rnp_get_fdirtcpm_n10(input_mask);

	/* write both the same so that UDP and TCP use the same mask */
	RNP_WRITE_REG(hw, RNP_FDIRTCPM, ~fdirtcpm);
	RNP_WRITE_REG(hw, RNP_FDIRUDPM, ~fdirtcpm);

	/* store source and destination IP masks (big-enian) */
	RNP_WRITE_REG_BE32(hw, RNP_FDIRSIP4M,
			~input_mask->formatted.src_ip[0]);
	RNP_WRITE_REG_BE32(hw, RNP_FDIRDIP4M,
			~input_mask->formatted.dst_ip[0]);

#endif
	return 0;
}

s32 rnp_fdir_write_perfect_filter_n10(struct rnp_hw *hw,
					union rnp_atr_input *input, u16 soft_id,
					u8 queue)
{
#if 0
	u32 fdirport, fdirvlan, fdirhash, fdircmd;

	/* currently IPv6 is not supported, must be programmed with 0 */
	RNP_WRITE_REG_BE32(hw, RNP_FDIRSIPv6(0),
			input->formatted.src_ip[0]);
	RNP_WRITE_REG_BE32(hw, RNP_FDIRSIPv6(1),
			input->formatted.src_ip[1]);
	RNP_WRITE_REG_BE32(hw, RNP_FDIRSIPv6(2),
			input->formatted.src_ip[2]);

	/* record the source address (big-endian) */
	RNP_WRITE_REG_BE32(hw, RNP_FDIRIPSA, input->formatted.src_ip[0]);

	/* record the first 32 bits of the destination address (big-endian) */
	RNP_WRITE_REG_BE32(hw, RNP_FDIRIPDA, input->formatted.dst_ip[0]);

	/* record source and destination port (little-endian)*/
	fdirport = ntohs(input->formatted.dst_port);
	fdirport <<= RNP_FDIRPORT_DESTINATION_SHIFT;
	fdirport |= ntohs(input->formatted.src_port);
	RNP_WRITE_REG(hw, RNP_FDIRPORT, fdirport);

	/* record vlan (little-endian) and flex_bytes(big-endian) */
	fdirvlan = RNP_STORE_AS_BE16(input->formatted.flex_bytes);
	fdirvlan <<= RNP_FDIRVLAN_FLEX_SHIFT;
	fdirvlan |= ntohs(input->formatted.vlan_id);
	RNP_WRITE_REG(hw, RNP_FDIRVLAN, fdirvlan);

	/* configure FDIRHASH register */
	fdirhash = input->formatted.bkt_hash;
	fdirhash |= soft_id << RNP_FDIRHASH_SIG_SW_INDEX_SHIFT;
	RNP_WRITE_REG(hw, RNP_FDIRHASH, fdirhash);

	/*
	 * flush all previous writes to make certain registers are
	 * programmed prior to issuing the command
	 */
	RNP_WRITE_FLUSH(hw);

	/* configure FDIRCMD register */
	fdircmd = RNP_FDIRCMD_CMD_ADD_FLOW | RNP_FDIRCMD_FILTER_UPDATE |
		RNP_FDIRCMD_LAST | RNP_FDIRCMD_QUEUE_EN;
	if (queue == RNP_FDIR_DROP_QUEUE)
		fdircmd |= RNP_FDIRCMD_DROP;
	fdircmd |= input->formatted.flow_type << RNP_FDIRCMD_FLOW_TYPE_SHIFT;
	fdircmd |= (u32)queue << RNP_FDIRCMD_RX_QUEUE_SHIFT;
	fdircmd |= (u32)input->formatted.vm_pool << RNP_FDIRCMD_VT_POOL_SHIFT;

	RNP_WRITE_REG(hw, RNP_FDIRCMD, fdircmd);
#endif
	return 0;
}

s32 rnp_fdir_erase_perfect_filter_n10(struct rnp_hw *hw,
					union rnp_atr_input *input, u16 soft_id)
{
	u32 fdirhash;
	u32 fdircmd = 0;
	u32 retry_count;
	s32 err = 0;
#if 0

	/* configure FDIRHASH register */
	fdirhash = input->formatted.bkt_hash;
	fdirhash |= soft_id << RNP_FDIRHASH_SIG_SW_INDEX_SHIFT;
	RNP_WRITE_REG(hw, RNP_FDIRHASH, fdirhash);

	/* flush hash to HW */
	RNP_WRITE_FLUSH(hw);

	/* Query if filter is present */
	RNP_WRITE_REG(hw, RNP_FDIRCMD, RNP_FDIRCMD_CMD_QUERY_REM_FILT);

	for (retry_count = 10; retry_count; retry_count--) {
		/* allow 10us for query to process */
		udelay(10);
		/* verify query completed successfully */
		fdircmd = RNP_READ_REG(hw, RNP_FDIRCMD);
		if (!(fdircmd & RNP_FDIRCMD_CMD_MASK))
			break;
	}

	if (!retry_count)
		err = RNP_ERR_FDIR_REINIT_FAILED;

	/* if filter exists in hardware then remove it */
	if (fdircmd & RNP_FDIRCMD_FILTER_VALID) {
		RNP_WRITE_REG(hw, RNP_FDIRHASH, fdirhash);
		RNP_WRITE_FLUSH(hw);
		RNP_WRITE_REG(hw, RNP_FDIRCMD,
				RNP_FDIRCMD_CMD_REMOVE_FLOW);
	}
#endif
	return err;
}

/**
 *  rnp_identify_phy_n10 - Get physical layer module
 *  @hw: pointer to hardware structure
 *
 *  Determines the physical layer module found on the current adapter.
 *  If PHY already detected, maintains current PHY type in hw struct,
 *  otherwise executes the PHY detection routine.
 **/
static s32 rnp_identify_phy_n10(struct rnp_hw *hw)
{
	s32 status = RNP_ERR_PHY_ADDR_INVALID;

	hw->phy.type = rnp_phy_sfp;

	return 0;
}

static s32 rnp_identify_sfp_module_n10(struct rnp_hw *hw)
{
	hw->phy.sfp_type = rnp_sfp_type_da_cu;

	return 0;
}


/**
 *  rnp_verify_fw_version_n10 - verify fw version for n10
 *  @hw: pointer to hardware structure
 *
 *  Verifies that installed the firmware version is 0.6 or higher
 *  for SFI devices. All n10 SFI devices should have version 0.6 or higher.
 *
 *  Returns RNP_ERR_EEPROM_VERSION if the FW is not present or
 *  if the FW version is not supported.
 **/
static s32 rnp_verify_fw_version_n10(struct rnp_hw *hw)
{
	s32 status = RNP_ERR_EEPROM_VERSION;
	u16 fw_offset, fw_ptp_cfg_offset;
	u16 fw_version = 0;

	return 0;
}

/**
 *  rnp_verify_lesm_fw_enabled_n10 - Checks LESM FW module state.
 *  @hw: pointer to hardware structure
 *
 *  Returns true if the LESM FW module is present and enabled. Otherwise
 *  returns false. Smart Speed must be disabled if LESM FW module is enabled.
 **/
bool rnp_verify_lesm_fw_enabled_n10(struct rnp_hw *hw)
{
	bool lesm_enabled = false;

	return lesm_enabled;
}

/**
 *  rnp_read_eeprom_buffer_n10 - Read EEPROM word(s) using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Retrieves 16 bit word(s) read from EEPROM
 **/
static s32 rnp_read_eeprom_buffer_n10(struct rnp_hw *hw, u16 offset,
					u16 words, u16 *data)
{
	struct rnp_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val = RNP_ERR_CONFIG;
#if 0

	/*
	 * If EEPROM is detected and can be addressed using 14 bits,
	 * use EERD otherwise use bit bang
	 */
	if ((eeprom->type == rnp_eeprom_spi) &&
			(offset + (words - 1) <= RNP_EERD_MAX_ADDR))
		ret_val = rnp_read_eerd_buffer_generic(hw, offset, words,
				data);
	else
		ret_val = rnp_read_eeprom_buffer_bit_bang_generic(hw, offset,
				words,
				data);

#endif
	return ret_val;
}

/**
 *  rnp_read_eeprom_n10 - Read EEPROM word using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM
 **/
static s32 rnp_read_eeprom_n10(struct rnp_hw *hw, u16 offset, u16 *data)
{
	struct rnp_eeprom_info *eeprom = &hw->eeprom;
	s32 ret_val = RNP_ERR_CONFIG;

#if 0
	/*
	 * If EEPROM is detected and can be addressed using 14 bits,
	 * use EERD otherwise use bit bang
	 */
	if ((eeprom->type == rnp_eeprom_spi) &&
			(offset <= RNP_EERD_MAX_ADDR))
		ret_val = rnp_read_eerd_generic(hw, offset, data);
	else
		ret_val = rnp_read_eeprom_bit_bang_generic(hw, offset, data);
#endif
	return ret_val;
}

/**
 * rnp_reset_pipeline_n10 - perform pipeline reset
 *
 * @hw: pointer to hardware structure
 *
 * Reset pipeline by asserting Restart_AN together with LMS change to ensure
 * full pipeline reset.  Note - We must hold the SW/FW semaphore before writing
 * to AUTOC, so this function assumes the semaphore is held.
 **/
s32 rnp_reset_pipeline_n10(struct rnp_hw *hw)
{
	s32 ret_val;
	u32 anlp1_reg = 0;
	u32 i, autoc_reg, autoc2_reg;

	/* Enable link if disabled in NVM */

	/* Write AUTOC register with toggled LMS[2] bit and Restart_AN */

	/* Wait for AN to leave state 0 */
	for (i = 0; i < 10; i++) {
		usleep_range(4000, 8000);
		break;
	}

	ret_val = 0;

reset_pipeline_out:
	/* Write AUTOC register with original LMS field and Restart_AN */

	return ret_val;
}

/**
 *  rnp_reset_hw_n10 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
//static s32 rnp_reset_hw_n10(struct rnp_hw *hw)
//{
//	int i;
//	s32 status;
//	u32 reg = 0;
//	int timeout = 0;
//	struct rnp_dma_info *dma = &hw->dma;
//
//	/* Call adapter stop to disable tx/rx and clear interrupts */
//	wr32(hw, RNP_DMA_AXI_EN, 0);
//
//	//nic_reset
//
//	wr32(hw, RNP_TOP_NIC_REST_N, NIC_RESET);
//	/*
//	 * we need this
//	 */
//	wmb();
//	wr32(hw, RNP_TOP_NIC_REST_N, ~NIC_RESET);
//
//#define TSRN10_REG_DEBUG_VALUE          (0x1a2b3c4d)
//
//#ifdef NO_MBX_VERSION
//	wr32(hw, RNP_DMA_DUMY, TSRN10_REG_DEBUG_VALUE);
//
//	while(!((reg = rd32(hw, RNP_DMA_DUMY)) == TSRN10_REG_DEBUG_VALUE + 1)) {
//		usleep_range(100, 200);
//		timeout++;
//
//		if (timeout > 10000) {
//			printk("wait reset timeout\n");
//			break;
//		}
//		//printk("wait reset");
//	}
//
//#else
//	rnp_mbx_fw_reset_phy(hw);
//#endif
//	printk("mac-rx-cfg: %x\n", rd32(hw, RNP_MAC_RX_CFG));
//	/* should set all tx-start to 1 */
//	for (i = 0; i < RNP_N10_MAX_TX_QUEUES; i++)
//		dma_ring_wr32(dma, RING_OFFSET(i) + RNP_DMA_TX_START, 1);
//
//#if 0
//	for (i = 0; i < 30; i++)
//		msleep(100);
//#endif
//	// default open this patch
//	wr32(hw, RNP_TOP_ETH_BUG_40G_PATCH, 1);
//	//wr32(hw, RNP_ETH_EXCEPT_DROP_PROC, 0xf);
//
//
//	/* Identify PHY and related function pointers */
//	status = hw->phy.ops.init(hw);
//
//	/* Setup SFP module if there is one present. */
//	if (hw->phy.sfp_setup_needed) {
//		status = hw->mac.ops.setup_sfp(hw);
//		hw->phy.sfp_setup_needed = false;
//	}
//
//	/* Reset PHY */
//	if (hw->phy.reset_disable == false && hw->phy.ops.reset != NULL)
//		hw->phy.ops.reset(hw);
//
//	/*todo earase tcm */
//	wr32(hw, RNP_ETH_TCAM_EN, 1);
//	wr32(hw, RNP_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
//	wr32(hw, RNP_TCAM_MODE, 2);
//	/* dont't open tcam cache */
//	wr32(hw, RNP_TCAM_CACHE_ENABLE, 0);
//
//	for (i = 0; i < 4096; i++) {
//		wr32(hw, RNP_TCAM_SDPQF(i), 0);
//		wr32(hw, RNP_TCAM_DAQF(i), 0);
//		wr32(hw, RNP_TCAM_SAQF(i), 0);
//		wr32(hw, RNP_TCAM_APQF(i), 0);
//
//		wr32(hw, RNP_TCAM_SDPQF_MASK(i), 0);
//		wr32(hw, RNP_TCAM_DAQF_MASK(i), 0);
//		wr32(hw, RNP_TCAM_SAQF_MASK(i), 0);
//		wr32(hw, RNP_TCAM_APQF_MASK(i), 0);
//	}
//	wr32(hw, RNP_TCAM_MODE, 1);
//
//	/* Store the permanent mac address */
//	if (!(hw->mac.mac_flags & RNP_FLAGS_INIT_MAC_ADDRESS)) {
//		rnp_get_permtion_mac_addr(hw, hw->mac.perm_addr);
//		memcpy(hw->mac.addr, hw->mac.perm_addr, ETH_ALEN);
//	}
//
//	hw->mac.num_rar_entries = RNP_N10_RAR_ENTRIES;
//	hw->mac.ops.init_rx_addrs(hw);
//
//	/* open vxlan default */
//#define VXLAN_HW_ENABLE (1)
//	wr32(hw, RNP_ETH_TUNNEL_MOD, VXLAN_HW_ENABLE);
//
//	/* set pkt_len_err and hdr_len_err default to 1 */
//#define PKT_LEN_ERR (2)
//#define HDR_LEN_ERR (1)
//	wr32(hw, RNP_ETH_ERR_MASK_VECTOR, PKT_LEN_ERR | HDR_LEN_ERR);
//
//	/*=====  mac steup ===*/
//	/* open mac at last */
//	// dwc_xlgmac_databook.pdf
////	for (i = 0; i < 1; i++) {
////		//wr32(hw, RNP_MAC_TX_CFG, 0x40010001);
////		wr32(hw, RNP_MAC_RX_CFG, rd32(hw, RNP_MAC_RX_CFG) | 0x01); 
////		//wr32(hw, RNP_MAC_RX_CFG, 0x07d001c7);
////		wr32(hw, RNP_MAC_PKT_FLT, 0x80000001);
////		wr32(hw, RNP_MAC_LPI_CTRL, 0x00060000);
////	}
//reset_hw_out:
//	rnp_reset_msix_table_generic(hw);
//
//	return 0;
//}

/**
 *  rnp_start_hw_n10 - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function
 *  and the generation start_hw function.
 *  Then performs revision-specific operations, if any.
 **/
//static s32 rnp_start_hw_n10(struct rnp_hw *hw)
//{
//	s32 ret_val = 0;
//	int i;
//
//	ret_val = rnp_start_hw_generic(hw);
//	if (ret_val != 0)
//		goto out;
//
//	ret_val = rnp_start_hw_gen2(hw);
//	if (ret_val != 0)
//		goto out;
//
//	// ETH Registers
//	//wr32(hw, RNP_ETH_ERR_MASK_VECTOR,~ETH_IGNORE_ALL_ERR);
//	wr32(hw, RNP_ETH_ERR_MASK_VECTOR, 0);
//	wr32(hw, RNP_ETH_BYPASS, 0);
//	wr32(hw, RNP_ETH_DEFAULT_RX_RING, 0);
//
//	wr32(hw, RNP_TOP_NIC_CONFIG,
//	     hw->mode
//#ifdef CONFIG_RNP_FPGA
//		     | hw->default_rx_queue << 24
//#endif
//	);
//
//	// DMA common Registers
//	wr32(hw, RNP_DMA_CONFIG, DMA_VEB_BYPASS);
//
//	// enable-dma-axi
//	wr32(hw, RNP_DMA_AXI_EN, (RX_AXI_RW_EN | TX_AXI_RW_EN));
//
//	if (ret_val == 0)
//		ret_val = rnp_verify_fw_version_n10(hw);
//out:
//	return ret_val;
//}

/**
 *  rnp_get_media_type_n10 - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber, copper, backplane)
 **/
static enum rnp_media_type rnp_get_media_type_n10(struct rnp_hw *hw)
{
	enum rnp_media_type media_type = rnp_media_type_fiber;
	return media_type;
}

/**
 *  rnp_get_supported_physical_layer_n10 - Returns physical layer type
 *  @hw: pointer to hardware structure
 *
 *  Determines physical layer capabilities of the current configuration.
 **/
static u32 rnp_get_supported_physical_layer_n10(struct rnp_hw *hw)
{
	u32 physical_layer = 0;
	return physical_layer;
}

static s32 rnp_get_link_capabilities_n10(struct rnp_hw *hw,
					   rnp_link_speed *speed, bool *autoneg)
{
	/* fix setup */
	/* reletive with firmware */
	*speed = RNP_LINK_SPEED_10GB_FULL;
	*autoneg = false;

	return 0;
}

/**
 *  rnp_get_mac_addr_n10 - Generic get MAC address
 *  @hw: pointer to hardware structure
 *  @mac_addr: Adapter MAC address
 *
 *  Reads the adapter's MAC address from first Receive Address Register (RAR0)
 *  A reset of the adapter must be performed prior to calling this function
 *  in order for the MAC address to have been loaded from the EEPROM into RAR0
 **/
s32 rnp_get_mac_addr_n10(struct rnp_eth_info *eth, u8 *mac_addr)
{
	u32 rar_high, rar_low, i;

	rar_high = eth_rd32(eth, RNP10_ETH_RAR_RH(0));
	rar_low = eth_rd32(eth, RNP10_ETH_RAR_RL(0));

	for (i = 0; i < 4; i++)
		mac_addr[i] = (u8)(rar_low >> (i * 8));

	for (i = 0; i < 2; i++)
		mac_addr[i + 4] = (u8)(rar_high >> (i * 8));

	return 0;
}



static struct rnp_phy_operations phy_ops_n10 = {
	.identify = &rnp_identify_phy_n10,
	.identify_sfp = &rnp_identify_sfp_module_n10,
	.init = &rnp_init_phy_ops_n10,
	.reset = &rnp_reset_phy_generic,
	.read_reg = &rnp_read_phy_reg_generic,
	.write_reg = &rnp_write_phy_reg_generic,
	.setup_link = &rnp_setup_phy_link_generic,
	.setup_link_speed = &rnp_setup_phy_link_speed_generic,
	.read_i2c_byte = &rnp_read_i2c_byte_generic,
	.write_i2c_byte = &rnp_write_i2c_byte_generic,
	.read_i2c_sff8472 = &rnp_read_i2c_sff8472_generic,
	.read_i2c_eeprom = &rnp_read_i2c_eeprom_generic,
	.write_i2c_eeprom = &rnp_write_i2c_eeprom_generic,
	.check_overtemp = &rnp_tn_check_overtemp,
};

// dma ops
/* setup queue speed limit to max_rate */
static void rnp_dma_set_tx_maxrate_n10(struct rnp_dma_info *dma, u16 queue, u32 max_rate)
{
	struct rnp_hw *hw = (struct rnp_hw *)dma->back;
	// no use here ,use ring_wr32 instead


}

/* setup mac with vf_num to veb table */
static void rnp_dma_set_veb_mac_n10(struct rnp_dma_info *dma, u8 *mac, u32 vfnum, u32 ring)
{

	u32 maclow, machi, ring_vfnum;
	int port;

	maclow = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) |
		mac[5];
	machi = (mac[0] << 8) | mac[1];
	ring_vfnum = ring | ((0x80 | vfnum) << 8);
	for (port = 0; port < 4; port++) {
		dma_wr32(dma, RNP10_DMA_PORT_VBE_MAC_LO_TBL(port, vfnum),
				maclow);
		dma_wr32(dma, RNP10_DMA_PORT_VBE_MAC_HI_TBL(port, vfnum),
				machi);
		dma_wr32(dma, RNP10_DMA_PORT_VEB_VF_RING_TBL(port, vfnum),
				ring_vfnum);
	}
}

/* setup vlan with vf_num to veb table */
static void rnp_dma_set_veb_vlan_n10(struct rnp_dma_info *dma, u16 vlan, u32 vfnum)
{
	int port;

	/* each vf can support only one vlan */
	for (port = 0; port < 4; port++)
		dma_wr32(dma, RNP10_DMA_PORT_VEB_VID_TBL(port, vfnum), vlan);

}

static void rnp_dma_clr_veb_all_n10(struct rnp_dma_info *dma)
{
	int port, i;

	for (port = 0; port < 4; port++) {
		for (i = 0; i < VEB_TBL_CNTS; i++) {
			dma_wr32(dma, RNP_DMA_PORT_VBE_MAC_LO_TBL(port, i), 0);
			dma_wr32(dma, RNP_DMA_PORT_VBE_MAC_HI_TBL(port, i), 0);
			dma_wr32(dma, RNP_DMA_PORT_VEB_VID_TBL(port, i), 0);
			dma_wr32(dma, RNP_DMA_PORT_VEB_VF_RING_TBL(port, i), 0);
		}
	}

}

static struct rnp_dma_operations dma_ops_n10 = {
	.set_tx_maxrate = &rnp_dma_set_tx_maxrate_n10,
	.set_veb_mac = &rnp_dma_set_veb_mac_n10,
	.set_veb_vlan = &rnp_dma_set_veb_vlan_n10,
	.clr_veb_all = &rnp_dma_clr_veb_all_n10,


};

// eth ops
/**
 *  rnp_eth_set_rar_n10 - Set Rx address register
 *  @eth: pointer to eth structure
 *  @index: Receive address register to write
 *  @addr: Address to put into receive address register
 *  @vmdq: VMDq "set" or "pool" index
 *  @enable_addr: set flag that address is active
 *  @sriov_flag 
 *
 *  Puts an ethernet address into a receive address register.
 **/
s32 rnp_eth_set_rar_n10(
	struct rnp_eth_info *eth, u32 index, u8 *addr,
	bool enable_addr)
{
	u32 mcstctrl;
	u32 rar_low, rar_high = 0;
	u32 rar_entries = eth->num_rar_entries;

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
		rnp_err("RAR index %d is out of range.\n", index);
		return RNP_ERR_INVALID_ARGUMENT;
	}

	eth_dbg(eth,
		   "    RAR[%d] <= %pM.  vmdq:%d enable:0x%x\n",
		   index,
		   addr);

	/* setup VMDq pool selection before this RAR gets enabled */
	/* only sriov mode use this */
	/*
	if (sriov_on)
		eth->ops.set_vmdq(eth, index, vmdq);
		*/

	/*
	 * HW expects these in big endian so we reverse the byte
	 * order from network order (big endian) to little endian
	 */
	rar_low = ((u32)addr[5] | ((u32)addr[4] << 8) | ((u32)addr[3] << 16) |
			   ((u32)addr[2] << 24));
	/*
	 * Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	rar_high = eth_rd32(eth, RNP10_ETH_RAR_RH(index));
	rar_high &= ~(0x0000FFFF | RNP10_RAH_AV);
	rar_high |= ((u32)addr[1] | ((u32)addr[0] << 8));

	if (enable_addr)
		rar_high |= RNP10_RAH_AV;

	eth_wr32(eth, RNP10_ETH_RAR_RL(index), rar_low);
	eth_wr32(eth, RNP10_ETH_RAR_RH(index), rar_high);

	/* open unicast filter */
	/* we now not use unicast */
	/* but we must open this since dest-mac filter | unicast table */
	/* all packets up if close unicast table */
	mcstctrl = eth_rd32(eth, RNP10_ETH_DMAC_MCSTCTRL);
	mcstctrl |= RNP10_MCSTCTRL_UNICASE_TBL_EN;
	eth_wr32(eth, RNP10_ETH_DMAC_MCSTCTRL, mcstctrl);

	return 0;
}

/**
 *  rnp_eth_clear_rar_n10 - Remove Rx address register
 *  @eth: pointer to eth structure
 *  @index: Receive address register to write
 *
 *  Clears an ethernet address from a receive address register.
 **/
s32 rnp_eth_clear_rar_n10(struct rnp_eth_info *eth, u32 index)
{
	u32 rar_high;
	u32 rar_entries = eth->num_rar_entries;

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
		eth_dbg(eth, "RAR index %d is out of range.\n", index);
		return RNP_ERR_INVALID_ARGUMENT;
	}

	/*
	 * Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	rar_high = eth_rd32(eth, RNP10_ETH_RAR_RH(index));
	rar_high &= ~(0x0000FFFF | RNP10_RAH_AV);

	// hw_dbg(hw, "Clearing RAR[%d]\n", index);
	eth_wr32(eth, RNP10_ETH_RAR_RL(index), 0);
	eth_wr32(eth, RNP10_ETH_RAR_RH(index), rar_high);

	/* clear VMDq pool/queue selection for this RAR */
	eth->ops.clear_vmdq(eth, index, RNP_CLEAR_VMDQ_ALL);

	return 0;
}

/**
 *  rnp_eth_set_vmdq_n10 - Associate a VMDq pool index with a rx address
 *  @eth: pointer to eth struct
 *  @rar: receive address register index to associate with a VMDq index
 *  @vmdq: VMDq pool index
 *  only mac->vf
 **/
s32 rnp_eth_set_vmdq_n10(struct rnp_eth_info *eth, u32 rar, u32 vmdq)
{
	u32 mpsar;
	u32 rar_entries = eth->num_rar_entries;

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		eth_dbg(eth, "RAR index %d is out of range.\n", rar);
		return RNP_ERR_INVALID_ARGUMENT;
	}

	eth_wr32(eth, RNP10_VM_DMAC_MPSAR_RING(rar), vmdq);

	return 0;
}

/**
 *  rnp_eth_clear_vmdq_n10 - Disassociate a VMDq pool index from a rx address
 *  @eth: pointer to eth struct
 *  @rar: receive address register index to disassociate
 *  @vmdq: VMDq pool index to remove from the rar
 **/
s32 rnp_eth_clear_vmdq_n10(struct rnp_eth_info *eth, u32 rar, u32 vmdq)
{
	u32 rar_entries = eth->num_rar_entries;

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		eth_dbg(eth, "RAR index %d is out of range.\n", rar);
		return RNP_ERR_INVALID_ARGUMENT;
	}

	eth_wr32(eth, RNP10_VM_DMAC_MPSAR_RING(rar), 0);

	return 0;
}



static s32 rnp10_mta_vector(struct rnp_eth_info *eth, u8 *mc_addr)
{
	u32 vector = 0;

	switch (eth->mc_filter_type) {
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

static void rnp10_set_mta(struct rnp_hw *hw, u8 *mc_addr)
{
	u32 vector;
	u32 vector_bit;
	u32 vector_reg;
	struct rnp_eth_info *eth = &hw->eth;

	hw->addr_ctrl.mta_in_use++;

	// n10 use eth mta 
	vector = rnp10_mta_vector(eth, mc_addr);

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
	eth->mta_shadow[vector_reg] |= (1 << vector_bit);
}

static void rnp10_set_vf_mta(struct rnp_hw *hw, u16 vector)
{
	u32 vector_bit;
	u32 vector_reg;
	struct rnp_eth_info *eth = &hw->eth;

	hw->addr_ctrl.mta_in_use++;

	vector_reg = (vector >> 5) & 0x7F;
	vector_bit = vector & 0x1F;
	hw_dbg(hw,
		   "\t\t vf M: MTA-BIT:%4d, MTA_REG[%d][%d] <= 1\n",
		   vector,
		   vector_reg,
		   vector_bit);
	eth->mta_shadow[vector_reg] |= (1 << vector_bit);
}

/**
 *  rnp_update_mc_addr_list_n10 - Updates MAC list of multicast addresses
 *  @hw: pointer to hardware structure
 *  @netdev: pointer to net device structure
 *
 *  The given list replaces any existing list. Clears the MC addrs from receive
 *  address registers and the multicast table. Uses unused receive address
 *  registers for the first multicast addresses, and hashes the rest into the
 *  multicast table.
 **/
s32 rnp_eth_update_mc_addr_list_n10(struct rnp_eth_info *eth,
		struct net_device *netdev, bool sriov_on)
{
	struct rnp_hw *hw = (struct rnp_hw *)eth->back;
#ifdef NETDEV_HW_ADDR_T_MULTICAST
	struct netdev_hw_addr *ha;
#endif
	u32 i;
	u32 v;
	int addr_count = 0;
	u8 *addr_list = NULL;

	/*
	 * Set the new number of MC addresses that we are being requested to
	 * use.
	 */
	hw->addr_ctrl.num_mc_addrs = netdev_mc_count(netdev);
	hw->addr_ctrl.mta_in_use = 0;

	/* Clear mta_shadow */
	eth_dbg(eth, " Clearing MTA(multicast table)\n");

	memset(&eth->mta_shadow, 0, sizeof(eth->mta_shadow));

	/* Update mta shadow */
	eth_dbg(eth, " Updating MTA..\n");

	addr_count = netdev_mc_count(netdev);

#ifdef NETDEV_HW_ADDR_T_MULTICAST     
	ha = list_first_entry(&netdev->mc.list,
			struct netdev_hw_addr, list);
	addr_list = ha->addr;
#else   
	addr_list = netdev->mc_list->dmi_addr;
#endif
	for (i = 0; i < addr_count; i++) {
		eth_dbg(eth, " Adding the multicast addresses:\n");
		rnp10_set_mta(hw, rnp_addr_list_itr(hw, &addr_list));
	}

	

	// sriov mode should set for vf multicast
	// not so good
	if (sriov_on) {
		struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
		if (!test_and_set_bit(__RNP_USE_VFINFI, &adapter->state)) {
			for (i = 0; i < adapter->num_vfs; i++) {
				if (adapter->vfinfo) {
					struct vf_data_storage *vfinfo = &adapter->vfinfo[i];
					int j;

					for (j = 0; j < vfinfo->num_vf_mc_hashes; j++)
						rnp10_set_vf_mta(hw, vfinfo->vf_mc_hashes[j]);
				} else {
					printk("error sriov on but vfinfo is null\n");

				}
			}
			clear_bit(__RNP_USE_VFINFI, &adapter->state);
		}
	}

	/* Enable mta */
	for (i = 0; i < hw->mac.mcft_size; i++) {
		if (hw->addr_ctrl.mta_in_use)
			eth_wr32(eth, RNP10_ETH_MUTICAST_HASH_TABLE(i), eth->mta_shadow[i]);
	}

	if (hw->addr_ctrl.mta_in_use > 0) {
		struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
		if (!(adapter->flags & RNP_FLAG_SWITCH_LOOPBACK_EN)) {
			v = eth_rd32(eth, RNP10_ETH_DMAC_MCSTCTRL);
			eth_wr32(eth,
					 RNP10_ETH_DMAC_MCSTCTRL,
					 v | RNP10_MCSTCTRL_MULTICASE_TBL_EN | eth->mc_filter_type);
		}
	}

	eth_dbg(eth, " update MTA Done. mta_in_use:%d\n", hw->addr_ctrl.mta_in_use);
	return hw->addr_ctrl.mta_in_use;
}

/* clean all mc addr */
void rnp_eth_clr_mc_addr_n10(struct rnp_eth_info *eth)
{
	int i;

	for (i = 0; i < eth->mcft_size; i++)
		eth_wr32(eth, RNP10_ETH_MUTICAST_HASH_TABLE(i), 0);

}

/**
 *  rnp_eth_update_rss_key_n10 - Remove Rx address register
 *  @eth: pointer to eth structure
 *  @sriov_flag sriov status
 *
 *  update rss key to eth regs
 **/

void rnp_eth_update_rss_key_n10(struct rnp_eth_info *eth, bool sriov_flag)
{
	struct rnp_hw *hw = (struct rnp_hw *)eth->back;
	int i;
	u8 *key_temp;
	int key_len = RNP_RSS_KEY_SIZE;
	u8 *key =hw->rss_key;

	u32 iov_en = (sriov_flag) ? RNP10_IOV_ENABLED : 0;

	key_temp = kmalloc(key_len, GFP_KERNEL);
	//reoder the key
	for (i = 0; i < key_len; i++)
		*(key_temp + key_len - i - 1) = *(key + i);

	memcpy((u8 *)(eth->eth_base_addr + RNP10_ETH_RSS_KEY), key_temp, key_len);
	kfree(key_temp);

	// open rss now
	eth_wr32(eth, RNP10_ETH_RSS_CONTROL, RNP10_ETH_ENABLE_RSS_ONLY | iov_en);
}

/**
 *  rnp_eth_update_rss_table_n10 - Remove Rx address register
 *  @eth: pointer to eth structure
 *
 *  update rss table to eth regs
 **/
void rnp_eth_update_rss_table_n10(struct rnp_eth_info *eth)
{
	struct rnp_hw *hw = (struct rnp_hw *)eth->back;
	// setup rss info to hw regs
	u32 reta_entries = hw->rss_indir_tbl_num;
	u32 tc_entries = hw->rss_tc_tbl_num;
	int i;

	for (i = 0; i < tc_entries; i++)
		eth_wr32(eth, RNP10_ETH_TC_IPH_OFFSET_TABLE(i), hw->rss_tc_tbl[i]);

	for (i = 0; i < reta_entries; i++) 
		eth_wr32(eth, RNP10_ETH_RSS_INDIR_TBL(i), hw->rss_indir_tbl[i]);
		
}
/**
 *  rnp_eth_set_vfta_n10 - Set VLAN filter table
 *  @eth: pointer to eth structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vlan_on: boolean flag to turn on/off VLAN in VFVF
 *
 *  Turn on/off specified VLAN in the VLAN filter table.
 **/
s32 rnp_eth_set_vfta_n10(struct rnp_eth_info *eth, u32 vlan, bool vlan_on)
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
	vfta = eth_rd32(eth, RNP10_VFTA(regindex));

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

	// todo 
	/*
	 * to enable two vf have same vlan feature, disable vlvf function.
	 * as vlan has high-priority than mac-address filter, which means
	 * two vf can't have same vlan.
	 */

	if (vfta_changed)
		eth_wr32(eth, RNP10_VFTA(regindex), vfta);
	return 0;
}

void rnp_eth_clr_vfta_n10(struct rnp_eth_info *eth)
{
	u32 offset;

	for (offset = 0; offset < eth->vft_size; offset++)
		eth_wr32(eth, RNP10_VFTA(offset), 0);
#define RNP10_VLVF_ENTRIES 64
	for (offset = 0; offset < RNP10_VLVF_ENTRIES; offset++)
		eth_wr32(eth, RNP10_VLVF(offset), 0);

}
/**
 *  rnp_eth_set_vlan_filter_n10 - Set VLAN filter table
 *  @eth: pointer to eth structure
 *  @status: on |off
 *  Turn on/off VLAN filter table.
 **/
static void rnp_eth_set_vlan_filter_n10(struct rnp_eth_info *eth, bool status)
{
#define ETH_VLAN_FILTER_BIT (30)
	u32 value = eth_rd32(eth, RNP10_ETH_VLAN_FILTER_ENABLE);

	// clear bit first 
	value &= (~(0x01 << ETH_VLAN_FILTER_BIT));
	if (status)
		value |= (0x01 << ETH_VLAN_FILTER_BIT);
	eth_wr32(eth, RNP10_ETH_VLAN_FILTER_ENABLE, value);
}

u16 rnp_layer2_pritologic_n10(u16 hw_id)
{
	return hw_id;
}

void rnp_eth_set_layer2_n10(struct rnp_eth_info *eth, union rnp_atr_input *input,
				u16 pri_id, u8 queue)
{
	u16 hw_id;

	hw_id = rnp_layer2_pritologic_n10(pri_id);
	/* enable layer2 */
	eth_wr32(eth, RNP10_ETH_LAYER2_ETQF(hw_id),
	     (0x1 << 31) | (ntohs(input->layer2_formate.proto)));

	/* setup action */
	if (queue == RNP_FDIR_DROP_QUEUE) {
		eth_wr32(eth, RNP10_ETH_LAYER2_ETQS(hw_id), (0x1 << 31));
	} else {
		/* setup ring_number */
		eth_wr32(eth, RNP10_ETH_LAYER2_ETQS(hw_id),
		     (0x1 << 30) | (queue << 20));
	}
}

void rnp_eth_clr_layer2_n10(struct rnp_eth_info *eth, u16 pri_id)
{
	u16 hw_id;

	hw_id = rnp_layer2_pritologic_n10(pri_id);
	eth_wr32(eth, RNP10_ETH_LAYER2_ETQF(hw_id), 0);

}

void rnp_eth_clr_all_layer2_n10(struct rnp_eth_info *eth)
{
	int i;
#define RNP10_MAX_LAYER2_FILTERS 16
	for (i = 0; i < RNP10_MAX_LAYER2_FILTERS; i++) 
		eth_wr32(eth, RNP10_ETH_LAYER2_ETQF(i), 0);

}

u16 rnp_tuple5_pritologic_n10(u16 hw_id)
{
	return hw_id;
}

u16 rnp_tuple5_pritologic_tcam_n10(u16 pri_id)
{
	int i;
	int hw_id = 0;
	int step = 32;
	for (i = 0; i < pri_id; i++) {
		hw_id += step;
		if (hw_id > RNP10_MAX_TCAM_FILTERS) {
			hw_id = hw_id - RNP10_MAX_TCAM_FILTERS + 1;
		}
	}

	return hw_id;
}

void rnp_eth_set_tuple5_n10(struct rnp_eth_info *eth, union rnp_atr_input *input,
				u16 pri_id, u8 queue)
{
	struct rnp_hw *hw = (struct rnp_hw *)eth->back;

#define RNP10_SRC_IP_MASK BIT(0)
#define RNP10_DST_IP_MASK BIT(1)
#define RNP10_SRC_PORT_MASK BIT(2)
#define RNP10_DST_PORT_MASK BIT(3)
#define RNP10_L4_PROTO_MASK BIT(4)

	if (hw->fdir_mode != fdir_mode_tcam) {
		u32 port = 0;
		u8 mask_temp = 0;
		u8 l4_proto_type = 0;
		u16 hw_id;

		hw_id = rnp_tuple5_pritologic_n10(pri_id);	
		dbg("try to eable tuple 5 %x\n", hw_id);
		if (input->formatted.src_ip[0] != 0) {
			eth_wr32(eth, RNP10_ETH_TUPLE5_SAQF(hw_id),
					htonl(input->formatted.src_ip[0]));
		} else {
			mask_temp |= RNP10_SRC_IP_MASK;
		}
		if (input->formatted.dst_ip[0] != 0) {
			eth_wr32(eth, RNP10_ETH_TUPLE5_DAQF(hw_id),
					htonl(input->formatted.dst_ip[0]));
		} else
			mask_temp |= RNP10_DST_IP_MASK;
		if (input->formatted.src_port != 0)
			port |= (htons(input->formatted.src_port));
		else
			mask_temp |= RNP10_SRC_PORT_MASK;
		if (input->formatted.dst_port != 0)
			port |= (htons(input->formatted.dst_port) << 16);
		else
			mask_temp |= RNP10_DST_PORT_MASK;

		if (port != 0)
			eth_wr32(eth, RNP10_ETH_TUPLE5_SDPQF(hw_id), port);

		switch (input->formatted.flow_type) {
		case RNP_ATR_FLOW_TYPE_TCPV4:
			l4_proto_type = IPPROTO_TCP;
			break;
		case RNP_ATR_FLOW_TYPE_UDPV4:
			l4_proto_type = IPPROTO_UDP;
			break;
		case RNP_ATR_FLOW_TYPE_SCTPV4:
			l4_proto_type = IPPROTO_SCTP;
			break;
		case RNP_ATR_FLOW_TYPE_IPV4:
			l4_proto_type = input->formatted.inner_mac[0];
			break;
		default:
			l4_proto_type = 0;
		}

		if (l4_proto_type == 0)
			mask_temp |= RNP10_L4_PROTO_MASK;

		/* setup ftqf*/
		/* always set 0x3 */
		eth_wr32(eth, RNP10_ETH_TUPLE5_FTQF(hw_id),
				(1 << 31) | (mask_temp << 25) | (l4_proto_type << 16) | 0x3);

		/* setup action */
		if (queue == RNP_FDIR_DROP_QUEUE) {
			eth_wr32(eth, RNP10_ETH_TUPLE5_POLICY(hw_id), (0x1 << 31));
		} else {
			/* setup ring_number */
			eth_wr32(eth, RNP10_ETH_TUPLE5_POLICY(hw_id),
					((0x1 << 30) | (queue << 20)));
		}

	} else {
		u32 port = 0;
		u32 port_mask = 0;
		u8 l4_proto_type = 0;
		u8 l4_proto_mask = 0xff;
		u32 action = 0;
		u32 mark = 0;
		u16 hw_id;

		hw_id = rnp_tuple5_pritologic_tcam_n10(pri_id);
		eth_wr32(eth, RNP10_TCAM_MODE, 2);
		//wr32(hw, RNP_TCAM_CACHE_ENABLE, 0);
		dbg("try to eable tcam %d\n", hw_id);
		if (input->formatted.src_ip[0] != 0) {
			eth_wr32(eth, RNP10_TCAM_SAQF(hw_id),
					htonl(input->formatted.src_ip[0]));
			eth_wr32(eth, RNP10_TCAM_SAQF_MASK(hw_id),
					htonl(input->formatted.src_ip_mask[0]));

			dbg("tcam src ip 0%x ---> 0x%x\n", htonl(input->formatted.src_ip[0]),
					RNP10_TCAM_SAQF(hw_id));
			dbg("tcam src ip mask 0%x ---> 0x%x\n", htonl(input->formatted.src_ip_mask[0]),
					RNP10_TCAM_SAQF_MASK(hw_id));
		} else {
			eth_wr32(eth, RNP10_TCAM_SAQF(hw_id), 0);
			eth_wr32(eth, RNP10_TCAM_SAQF_MASK(hw_id), 0);
			dbg("tcam src ip 0%x ---> 0x%x\n", 0,
					RNP10_TCAM_SAQF(hw_id));
			dbg("tcam src ip mask 0%x ---> 0x%x\n", 0,
					RNP10_TCAM_SAQF_MASK(hw_id));
		}
		if (input->formatted.dst_ip[0] != 0) {
			eth_wr32(eth, RNP10_TCAM_DAQF(hw_id),
					htonl(input->formatted.dst_ip[0]));
			eth_wr32(eth, RNP10_TCAM_DAQF_MASK(hw_id),
					htonl(input->formatted.dst_ip_mask[0]));
			dbg("tcam dst ip 0%x ---> 0x%x\n", htonl(input->formatted.dst_ip[0]),
					RNP10_TCAM_DAQF(hw_id));
			dbg("tcam dst ip mask 0%x ---> 0x%x\n", htonl(input->formatted.dst_ip_mask[0]),
					RNP10_TCAM_DAQF_MASK(hw_id));
		} else {
			eth_wr32(eth, RNP10_TCAM_DAQF(hw_id), 0);
			eth_wr32(eth, RNP10_TCAM_DAQF_MASK(hw_id), 0);
			dbg("tcam dst ip 0%x ---> 0x%x\n", 0,
					RNP10_TCAM_DAQF(hw_id));
			dbg("tcam dst ip mask 0%x ---> 0x%x\n", 0,
					RNP10_TCAM_DAQF_MASK(hw_id));
		}
		if (input->formatted.src_port != 0) {
			port |= (htons(input->formatted.src_port) << 16);
			port_mask |= (htons(input->formatted.src_port_mask) << 16);

		} else {
		}
		if (input->formatted.dst_port != 0) {
			port |= (htons(input->formatted.dst_port));
			port_mask |= (htons(input->formatted.dst_port_mask));
		} else {
		}

		/* setup src & dst port */
		if (port != 0) {
			eth_wr32(eth, RNP10_TCAM_SDPQF(hw_id), port);
			eth_wr32(eth, RNP10_TCAM_SDPQF_MASK(hw_id), port_mask);

			dbg("tcam port 0%x ---> 0x%x\n", port,
					RNP10_TCAM_SDPQF(hw_id));
			dbg("tcam port mask 0%x ---> 0x%x\n", port_mask,
					RNP10_TCAM_SDPQF_MASK(hw_id));
		} else {
			eth_wr32(eth, RNP10_TCAM_SDPQF(hw_id), 0);
			eth_wr32(eth, RNP10_TCAM_SDPQF_MASK(hw_id), 0);
			dbg("tcam port 0%x ---> 0x%x\n", port,
					RNP10_TCAM_SDPQF(hw_id));
			dbg("tcam port mask 0%x ---> 0x%x\n", port_mask,
					RNP10_TCAM_SDPQF_MASK(hw_id));
		}

		switch (input->formatted.flow_type) {
			case RNP_ATR_FLOW_TYPE_TCPV4:
				l4_proto_type = IPPROTO_TCP;
				break;
			case RNP_ATR_FLOW_TYPE_UDPV4:
				l4_proto_type = IPPROTO_UDP;
				break;
			case RNP_ATR_FLOW_TYPE_SCTPV4:
				l4_proto_type = IPPROTO_SCTP;
				break;
			case RNP_ATR_FLOW_TYPE_IPV4:
				l4_proto_type = input->formatted.inner_mac[0];
				l4_proto_mask = input->formatted.inner_mac_mask[0];
				break;
			default:
				l4_proto_type = 0;
				l4_proto_mask = 0;
		}

		if (l4_proto_type != 0) {
			action |= l4_proto_type;
			mark |= l4_proto_mask;
		} else {
		}

		/* setup action */
		if (queue == RNP_FDIR_DROP_QUEUE) {
			eth_wr32(eth, RNP10_TCAM_APQF(hw_id), (0x1 << 31) | action);
			eth_wr32(eth, RNP10_TCAM_APQF_MASK(hw_id), mark);
			dbg("tcam action 0%x ---> 0x%x\n", (0x1 << 31) | action,
					RNP10_TCAM_APQF(hw_id));
			dbg("tcam action mask 0%x ---> 0x%x\n", mark,
					RNP10_TCAM_APQF_MASK(hw_id));
		} else {
			/* setup ring_number */
			eth_wr32(eth, RNP10_TCAM_APQF(hw_id),
		     ((0x1 << 30) | (queue << 16) | action));
			eth_wr32(eth, RNP10_TCAM_APQF_MASK(hw_id), mark);

			dbg("tcam action 0%x ---> 0x%x\n", (0x1 << 30) | (queue << 16) | action,
					RNP10_TCAM_APQF(hw_id));
			dbg("tcam action mask 0%x ---> 0x%x\n", mark,
					RNP10_TCAM_APQF_MASK(hw_id));
		}
		eth_wr32(eth, RNP10_TCAM_MODE, 1);

	}

}

void rnp_eth_clr_tuple5_n10(struct rnp_eth_info *eth, u16 pri_id)
{
	u16 hw_id;
	struct rnp_hw *hw = (struct rnp_hw *)eth->back;

	if (hw->fdir_mode != fdir_mode_tcam) {
		hw_id = rnp_tuple5_pritologic_n10(pri_id);	
		eth_wr32(eth, RNP10_ETH_TUPLE5_FTQF(hw_id), 0);
	} else {
		hw_id = rnp_tuple5_pritologic_tcam_n10(pri_id);
		dbg("disble tcam tuple5 %d\n", hw_id);
		/* earase tcam */
		eth_wr32(eth, RNP10_TCAM_MODE, 2);
		//wr32(hw, RNP_TCAM_CACHE_ENABLE, 0);

		eth_wr32(eth, RNP10_TCAM_SAQF(hw_id), 0);
		eth_wr32(eth, RNP10_TCAM_SAQF_MASK(hw_id), 0);

		eth_wr32(eth, RNP10_TCAM_DAQF(hw_id), 0);
		eth_wr32(eth, RNP10_TCAM_DAQF_MASK(hw_id), 0);

		eth_wr32(eth, RNP10_TCAM_SDPQF(hw_id), 0);
		eth_wr32(eth, RNP10_TCAM_SDPQF_MASK(hw_id), 0);

		eth_wr32(eth, RNP10_TCAM_APQF(hw_id), 0);
		eth_wr32(eth, RNP10_TCAM_APQF_MASK(hw_id), 0);

		eth_wr32(eth, RNP10_TCAM_MODE, 1);

	}
}

void rnp_eth_clr_all_tuple5_n10(struct rnp_eth_info *eth)
{
	int i;

	struct rnp_hw *hw = (struct rnp_hw *)eth->back;

	if (hw->fdir_mode != fdir_mode_tcam) {
		for (i = 0; i < RNP10_MAX_TUPLE5_FILTERS; i++)
			eth_wr32(eth, RNP10_ETH_TUPLE5_FTQF(i), 0);
		eth_wr32(eth, RNP10_ETH_TCAM_EN, 0);
	} else {

		/*todo earase tcm */
		eth_wr32(eth, RNP10_ETH_TCAM_EN, 1);
		eth_wr32(eth, RNP10_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
		eth_wr32(eth, RNP10_TCAM_MODE, 2);
		/* dont't open tcam cache */
		eth_wr32(eth, RNP10_TCAM_CACHE_ENABLE, 0);

		for (i = 0; i < RNP10_MAX_TUPLE5_FILTERS; i++) {
			eth_wr32(eth, RNP10_TCAM_SDPQF(i), 0);
			eth_wr32(eth, RNP10_TCAM_DAQF(i), 0);
			eth_wr32(eth, RNP10_TCAM_SAQF(i), 0);
			eth_wr32(eth, RNP10_TCAM_APQF(i), 0);

			eth_wr32(eth, RNP10_TCAM_SDPQF_MASK(i), 0);
			eth_wr32(eth, RNP10_TCAM_DAQF_MASK(i), 0);
			eth_wr32(eth, RNP10_TCAM_SAQF_MASK(i), 0);
			eth_wr32(eth, RNP10_TCAM_APQF_MASK(i), 0);
		}
		eth_wr32(eth, RNP10_TCAM_MODE, 1);
	}
}

static void rnp_eth_set_min_max_packets_n10(struct rnp_eth_info *eth, int min, int max)
{
	eth_wr32(eth, RNP10_ETH_DEFAULT_RX_MIN_LEN, min); 	
	eth_wr32(eth, RNP10_ETH_DEFAULT_RX_MAX_LEN, max); 	
}

static void rnp_eth_set_vlan_strip_n10(struct rnp_eth_info *eth, u16 queue, bool enable)
{
	u32 reg = RNP10_ETH_VLAN_VME_REG(queue / 32);
	u32 offset = queue % 32;
	u32 data = eth_rd32(eth, reg);

	if (enable == true)
		data |= (1 << offset);
	else
		data &= ~(1 << offset);

	eth_wr32(eth, reg, data);
}

static void rnp_eth_set_vxlan_port_n10(struct rnp_eth_info *eth, u32 port)
{
	eth_wr32(eth, RNP10_ETH_VXLAN_PORT, port);
}

static void rnp_eth_set_vxlan_mode_n10(struct rnp_eth_info *eth, bool inner)
{
	if (inner)
		eth_wr32(eth, RNP10_ETH_WRAP_FIELD_TYPE, 1);
	else
		eth_wr32(eth, RNP10_ETH_WRAP_FIELD_TYPE, 0);

}

static void rnp_eth_set_rx_hash_n10(struct rnp_eth_info *eth, bool status, bool sriov_flag)
{
	u32 iov_en = (sriov_flag) ? RNP10_IOV_ENABLED : 0;

	if (status) 
		eth_wr32(eth, RNP10_ETH_RSS_CONTROL,
				RNP10_ETH_ENABLE_RSS_ONLY | iov_en);
	else 
		eth_wr32(eth, RNP10_ETH_RSS_CONTROL,
				RNP10_ETH_DISABLE_RSS | iov_en);


}

static s32 rnp_eth_set_fc_mode_n10(struct rnp_eth_info *eth)
{
	struct rnp_hw *hw = (struct rnp_hw *)eth->back;
	s32 ret_val = 0;
	int i;

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

	for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++) {
		if ((hw->fc.current_mode & rnp_fc_tx_pause)) {
			if (hw->fc.high_water[i]) {
				eth_wr32(eth, RNP10_ETH_HIGH_WATER(i), hw->fc.high_water[i]);
			}
			if (hw->fc.low_water[i]) {
				eth_wr32(eth, RNP10_ETH_LOW_WATER(i), hw->fc.low_water[i]);
			}
		}
	} 
out:	
	return ret_val;

}

static struct rnp_eth_operations eth_ops_n10 = {
	.set_rar = &rnp_eth_set_rar_n10,
	.clear_rar = &rnp_eth_clear_rar_n10,
	.set_vmdq = &rnp_eth_set_vmdq_n10,
	.clear_vmdq = &rnp_eth_clear_vmdq_n10,
	//.get_mac_addr = &rnp_eth_get_mac_addr_n10,

	.update_mc_addr_list = &rnp_eth_update_mc_addr_list_n10,
	.clr_mc_addr = &rnp_eth_clr_mc_addr_n10,

	/* store rss info to eth */
	.set_rss_key = &rnp_eth_update_rss_key_n10,
	.set_rss_table = &rnp_eth_update_rss_table_n10,
	.set_vfta = &rnp_eth_set_vfta_n10,
	.clr_vfta = &rnp_eth_clr_vfta_n10,
	.set_vlan_filter = &rnp_eth_set_vlan_filter_n10,

	.set_layer2_remapping = &rnp_eth_set_layer2_n10,
	.clr_layer2_remapping = &rnp_eth_clr_layer2_n10,
	.clr_all_layer2_remapping = &rnp_eth_clr_all_layer2_n10,
	.set_tuple5_remapping = &rnp_eth_set_tuple5_n10,
	.clr_tuple5_remapping = &rnp_eth_clr_tuple5_n10,
	.clr_all_tuple5_remapping = &rnp_eth_clr_all_tuple5_n10,

	.set_min_max_packet = &rnp_eth_set_min_max_packets_n10,
	.set_vlan_strip = &rnp_eth_set_vlan_strip_n10,
	.set_vxlan_port = &rnp_eth_set_vxlan_port_n10,
	.set_vxlan_mode = &rnp_eth_set_vxlan_mode_n10,
	.set_rx_hash = &rnp_eth_set_rx_hash_n10,
	.set_fc_mode = &rnp_eth_set_fc_mode_n10,
	// setup init sriov

	// mac 
	// setup fcs mode 
	// setup rx status

	// init_filter_default

};

/**
 *  rnp_init_hw_n10 - Generic hardware initialization
 *  @hw: pointer to hardware structure
 *
 *  Initialize the hardware by resetting the hardware, filling the bus info
 *  structure and media type, clears all on chip counters, initializes receive
 *  address registers, multicast table, VLAN filter table, calls routine to set
 *  up link and flow control settings, and leaves transmit and receive units
 *  disabled and uninitialized
 **/
s32 rnp_init_hw_ops_n10(struct rnp_hw *hw)
{
	s32 status = 0;

	/* Reset the hardware */
	status = hw->ops.reset_hw(hw);

	if (status == 0) {
		/* Start the HW */
		status = hw->ops.start_hw(hw);
	}

	return status;
}

s32 rnp_get_permtion_mac_addr_n10(struct rnp_hw *hw, u8 *mac_addr)
{
	u32 v;

#ifdef NO_MBX_VERSION
#ifdef FIX_MAC_TEST
	v = 0x00004E46;
#else
	v = rd32(hw, RNP10_TOP_MAC_OUI);
#endif
	mac_addr[0] = (u8)(v >> 16);
	mac_addr[1] = (u8)(v >> 8);
	mac_addr[2] = (u8)(v >> 0);

#ifdef FIX_MAC_TEST
	v = 0x00032F00 + rnp_is_pf1(hw->pdev);
#else
	v = rd32(hw, RNP10_TOP_MAC_SN);
#endif
	mac_addr[3] = (u8)(v >> 16);
	mac_addr[4] = (u8)(v >> 8);
	mac_addr[5] = (u8)(v >> 0);
#else
	if (rnp_fw_get_macaddr(hw, hw->pfvfnum, mac_addr, hw->nr_lane)) {
		dbg("generate ramdom macaddress...\n");
		eth_random_addr(mac_addr);
	}
#endif

	// try to fixme
	hw->mac.mac_flags |= RNP_FLAGS_INIT_MAC_ADDRESS;
	dbg("%s mac:%pM\n", __func__, mac_addr);

	return 0;
}

s32 rnp_reset_hw_ops_n10(struct rnp_hw *hw)
{
	int i;
	s32 status;
	u32 reg = 0;
	int timeout = 0;
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_eth_info *eth = &hw->eth;

	/* Call adapter stop to disable tx/rx and clear interrupts */
	dma_wr32(dma, RNP_DMA_AXI_EN, 0);

	//nic_reset
#define N10_NIC_RESET 0
	wr32(hw, RNP10_TOP_NIC_REST_N, N10_NIC_RESET);
	/*
	 * we need this
	 */
	wmb();
	wr32(hw, RNP10_TOP_NIC_REST_N, ~N10_NIC_RESET);

	rnp_mbx_fw_reset_phy(hw);
	/* should set all tx-start to 1 */
	for (i = 0; i < RNP_N10_MAX_TX_QUEUES; i++)
		dma_ring_wr32(dma, RING_OFFSET(i) + RNP_DMA_TX_START, 1);

	// default open this patch
	wr32(hw, RNP10_TOP_ETH_BUG_40G_PATCH, 1);
	// set 2046 --> 0x18070
	eth_wr32(eth, RNP10_ETH_RX_PROGFULL_THRESH_PORT, DROP_ALL_THRESH);

	/* Identify PHY and related function pointers */
	//status = hw->phy.ops.init(hw);

	/* Setup SFP module if there is one present. */
	/*
	if (hw->phy.sfp_setup_needed) {
		status = hw->mac.ops.setup_sfp(hw);
		hw->phy.sfp_setup_needed = false;
	} */

	/* Reset PHY */
	/*if (hw->phy.reset_disable == false && hw->phy.ops.reset != NULL)
		hw->phy.ops.reset(hw);
		*/
	/* tcam not reset */
	eth->ops.clr_all_tuple5_remapping(eth);

	// fixme 
	/* Store the permanent mac address */
	if (!(hw->mac.mac_flags & RNP_FLAGS_INIT_MAC_ADDRESS)) {
		rnp_get_permtion_mac_addr_n10(hw, hw->mac.perm_addr);
		memcpy(hw->mac.addr, hw->mac.perm_addr, ETH_ALEN);
	}

	// fix me 
	//hw->mac.num_rar_entries = RNP_N10_RAR_ENTRIES;
	//hw->mac.ops.init_rx_addrs(hw);
	hw->ops.init_rx_addrs(hw);

	/* open vxlan default */
#define VXLAN_HW_ENABLE (1)
	eth_wr32(eth, RNP10_ETH_TUNNEL_MOD, VXLAN_HW_ENABLE);


reset_hw_out:
	// reset all ring msix table to 0
	// each ring has one vector	
	for (i = 0; i < dma->max_tx_queues; i++)
		rnp_wr_reg(hw->ring_msix_base + RING_VECTOR(i), 0);

	return 0;
}

s32 rnp_start_hw_ops_n10(struct rnp_hw *hw)
{
	s32 ret_val = 0;
	int i;
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_dma_info *dma = &hw->dma;

	ret_val = rnp_start_hw_generic(hw);
	if (ret_val != 0)
		goto out;

	//ret_val = rnp_start_hw_gen2(hw);
	//if (ret_val != 0)
		//goto out;

	// ETH Registers
	//wr32(hw, RNP_ETH_ERR_MASK_VECTOR,~ETH_IGNORE_ALL_ERR);
	eth_wr32(eth, RNP10_ETH_ERR_MASK_VECTOR, 0x3);
	eth_wr32(eth, RNP10_ETH_BYPASS, 0);
	eth_wr32(eth, RNP10_ETH_DEFAULT_RX_RING, 0);

/*
	wr32(hw, RNP10_TOP_NIC_CONFIG,
	     hw->mode
#ifdef CONFIG_RNP_FPGA
		     | hw->default_rx_queue << 24
#endif
	);
*/

	// DMA common Registers
	dma_wr32(dma, RNP_DMA_CONFIG, DMA_VEB_BYPASS);

	// enable-dma-axi
	dma_wr32(dma, RNP_DMA_AXI_EN, (RX_AXI_RW_EN | TX_AXI_RW_EN));

	/*if (ret_val == 0)
	  ret_val = rnp_verify_fw_version_n10(hw);
	  */
out:
	return ret_val;
}

/* set n10 min/max packet according to new_mtu 
 * we support mtu + 14 + 4 * 3 as max packet len*/
static void rnp_set_mtu_hw_ops_n10(struct rnp_hw *hw, int new_mtu)
{
	struct rnp_eth_info *eth = &hw->eth;

	int min = 60;
	int max = new_mtu + ETH_HLEN + ETH_FCS_LEN * 3;

	eth->ops.set_min_max_packet(eth, min, max);
}

/* setup n10 vlan filter status */
static void rnp_set_vlan_filter_en_hw_ops_n10(struct rnp_hw *hw, bool status)
{
	struct rnp_eth_info *eth = &hw->eth;
	eth->ops.set_vlan_filter(eth, status);
}

/* set vlan to n10 vlan filter table & veb */
/* pf setup call */
static void rnp_set_vlan_filter_hw_ops_n10(struct rnp_hw *hw, u16 vid, bool enable, bool sriov_flag)
{
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_dma_info *dma = &hw->dma;

	//todo set up own veb , use the last vfnum
	u32 vfnum = hw->max_vfs - 1;

	// vid 0 do nothing
	//if (!vid)
	//	return;
	/* setup n10 eth vlan table */
	eth->ops.set_vfta(eth, vid, enable);

	/* setup veb */
	if (sriov_flag) {
		if (enable) {
			dma->ops.set_veb_vlan(dma, vid, vfnum);
		} else {
			dma->ops.set_veb_vlan(dma, 0, vfnum);
		}

	}
}

static void rnp_set_vf_vlan_filter_hw_ops_n10(struct rnp_hw *hw, u16 vid, int vf, bool enable, bool veb_only)
{
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_dma_info *dma = &hw->dma;

	if (!veb_only) {
		// call set vfta without veb setup
		hw->ops.set_vlan_filter(hw, vid, enable, false);

	} else {
		if (enable) {
			dma->ops.set_veb_vlan(dma, vid, vf);
		} else {
			dma->ops.set_veb_vlan(dma, 0, vf);
		}
	}

}

static void rnp_clr_vlan_veb_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_dma_info *dma = &hw->dma;
	u32 vfnum = hw->vfnum;

	dma->ops.set_veb_vlan(dma, 0, vfnum);

}

/* setup n10 vlan strip status */
static void rnp_set_vlan_strip_hw_ops_n10(struct rnp_hw *hw, u16 queue, bool strip)
{
	struct rnp_eth_info *eth = &hw->eth;
	
	eth->ops.set_vlan_strip(eth, queue, strip);
}

/* update new n10 mac */
static void rnp_set_mac_hw_ops_n10(struct rnp_hw *hw, u8 *mac, bool sriov_flag)
{
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_mac_info *mac_info = &hw->mac;
	/* use this queue index to setup veb */
	/* now pf use queu 0 /1 
	 * vfnum is the last vfnum */
	int queue = hw->veb_ring;
	int vfnum = hw->vfnum;
	// update new mac in index 0
	eth->ops.set_rar(eth, 0, mac, true);
	// if in sriov mode ,should update veb
	if (sriov_flag) {
		eth->ops.set_vmdq(eth, 0, queue / 2); 
		dma->ops.set_veb_mac(dma, mac, vfnum, queue);
	}
	// should also setup mac
	
	mac_info->ops.set_mac(mac_info, mac, 0);
}

/**
 * rnp_write_uc_addr_list - write unicast addresses to RAR table
 * @netdev: network interface device structure
 *
 * Writes unicast address list to the RAR table.
 * Returns: -ENOMEM on failure/insufficient address space
 *                0 on no addresses written
 *                X on writing X addresses to the RAR table
 **/
static int rnp_write_uc_addr_list_n10(struct rnp_hw *hw, struct net_device *netdev, bool sriov_flag)
{
	unsigned int rar_entries = hw->num_rar_entries - 1;
	u32 vfnum = hw->vfnum;
	struct rnp_eth_info *eth = &hw->eth;
	int count = 0;

	/* In SR-IOV mode significantly less RAR entries are available */
	if (sriov_flag)
		rar_entries = RNP_MAX_PF_MACVLANS - 1;

	/* return ENOMEM indicating insufficient memory for addresses */
	if (netdev_uc_count(netdev) > rar_entries)
		return -ENOMEM;

	if (!netdev_uc_empty(netdev)) {
		struct netdev_hw_addr *ha;

		hw_dbg(hw, "%s: rar_entries:%d, uc_count:%d\n", __func__,
		       hw->num_rar_entries, netdev_uc_count(netdev));

		/* return error if we do not support writing to RAR table */
		if (!eth->ops.set_rar)
			return -ENOMEM;

		netdev_for_each_uc_addr(ha, netdev) {
			if (!rar_entries)
				break;
			/* VMDQ_P(0) is num_vfs pf use the last
			 * vf in sriov mode
			 */
			/* that's ok */
			eth->ops.set_rar(eth, rar_entries, ha->addr, RNP10_RAH_AV);	
			//hw->mac.ops.set_rar(hw, rar_entries--, ha->addr,
			//		    VMDQ_P(0), RNP_RAH_AV);
			if (sriov_flag)
				eth->ops.set_vmdq(eth, rar_entries, vfnum); 	

			rar_entries--;

			count++;
		}
	}
	/* write the addresses in reverse order to avoid write combining */

	hw_dbg(hw, "%s: Clearing RAR[1 - %d]\n", __func__, rar_entries);
	for (; rar_entries > 0; rar_entries--)
		eth->ops.clear_rar(eth, rar_entries);

	return count;
}
static void rnp_set_rx_mode_hw_ops_n10(struct rnp_hw *hw, struct net_device *netdev, bool sriov_flag)
{

	//struct rnp_adapter *adapter = netdev_priv(netdev);
	u32 fctrl, mcstctrl;
	netdev_features_t features = netdev->features;
	int count;
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;

	hw_dbg(hw, "%s\n", __func__);

	/* broadcast always bypass */
	fctrl = eth_rd32(eth, RNP10_ETH_DMAC_FCTRL) | RNP10_FCTRL_BPE;
	/* clear the bits we are changing the status of */
	fctrl &= ~(RNP10_FCTRL_UPE | RNP10_FCTRL_MPE);
	/* promisc mode */
	if (netdev->flags & IFF_PROMISC) {
		hw->addr_ctrl.user_set_promisc = true;
		fctrl |= (RNP10_FCTRL_UPE | RNP10_FCTRL_MPE);
		/* disable hardware filter vlans in promisc mode */
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
		features &= ~NETIF_F_HW_VLAN_CTAG_FILTER;
#endif
#ifdef NETIF_F_HW_VLAN_CTAG_RX
		features &= ~NETIF_F_HW_VLAN_CTAG_RX;
#endif
	} else {
		if (netdev->flags & IFF_ALLMULTI) {
			fctrl |= RNP10_FCTRL_MPE;
		} else {
			/* Write addresses to the MTA, if the attempt fails
			 * then we should just turn on promiscuous mode so
			 * that we can at least receive multicast traffic
			 */
			// we always update vf multicast info
			count = eth->ops.update_mc_addr_list(eth, netdev, true); 
			//count = hw->mac.ops.update_mc_addr_list(hw, netdev);
			if (count < 0) {
				fctrl |= RNP10_FCTRL_MPE;
				// mcstctrl &= ~RNP_MCSTCTRL_MULTICASE_TBL_EN;
			} else if (count) {
				// mcstctrl |= RNP_MCSTCTRL_MULTICASE_TBL_EN;
			}
		}
		hw->addr_ctrl.user_set_promisc = false;
	}

	/*
	 * Write addresses to available RAR registers, if there is not
	 * sufficient space to store all the addresses then enable
	 * unicast promiscuous mode
	 */
	if (rnp_write_uc_addr_list_n10(hw, netdev, sriov_flag) < 0) {
		fctrl |= RNP10_FCTRL_UPE;
	}

	/* not so good */
	// update this outside hw_ops
	// todo 
	/*if (adapter->num_vfs)
		rnp_restore_vf_multicasts(adapter);
	*/
	eth_wr32(eth, RNP10_ETH_DMAC_FCTRL, fctrl);
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
	if (features & NETIF_F_HW_VLAN_CTAG_FILTER)
		eth->ops.set_vlan_filter(eth, true);
	else
		eth->ops.set_vlan_filter(eth, false);
#endif

	/* not so good */
	// maybe set this outside ?
	/*
#ifdef NETIF_F_HW_VLAN_CTAG_RX
	{
		int i;
		for (i = 0; i < adapter->num_rx_queues; i++) {
			struct rnp_ring *tx_ring;

			tx_ring = adapter->rx_ring[i];
			if (features & NETIF_F_HW_VLAN_CTAG_RX)
				eth->ops.set_vlan_strip(eth, tx_ring->rnp_queue_idx, true);
			else
				eth->ops.set_vlan_strip(eth, tx_ring->rnp_queue_idx, false);
		}
	}
#endif
*/
	// if in promisc mode 
	if ((hw->addr_ctrl.user_set_promisc == true) ||
	(adapter->priv_flags & RNP_PRIV_FLAG_REC_HDR_LEN_ERR)) {

	/* set pkt_len_err and hdr_len_err default to 1 */
		eth_wr32(eth, RNP10_ETH_ERR_MASK_VECTOR, PKT_LEN_ERR | HDR_LEN_ERR);
	}else {

		eth_wr32(eth, RNP10_ETH_ERR_MASK_VECTOR, 0);
	}
	// update mtu 
	hw->ops.set_mtu(hw, netdev->mtu);

}

/* setup an rar with vfnum */
static void rnp_set_rar_with_vf_hw_ops_n10(struct rnp_hw *hw, u8 *mac,
		int idx, u32 vfnum, bool enable)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.set_rar(eth, idx, mac, enable);
	/* should check error or not ?*/
	eth->ops.set_vmdq(eth, idx, vfnum); 

}

static void rnp_clr_rar_hw_ops_n10(struct rnp_hw *hw, int idx)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.clear_rar(eth, idx);

}
static void rnp_set_fcs_mode_hw_ops_n10(struct rnp_hw *hw, bool status)
{
	struct rnp_mac_info *mac = &hw->mac;
	// call mac to up fcs
	mac->ops.set_mac_fcs(mac, status);

}

static void rnp_set_vxlan_port_hw_ops_n10(struct rnp_hw *hw, u32 port)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.set_vxlan_port(eth, port);
	//set vxlan port to eth mode

}

static void rnp_set_vxlan_mode_hw_ops_n10(struct rnp_hw *hw, bool inner)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.set_vxlan_mode(eth, inner);
	

}



static void rnp_set_mac_rx_hw_ops_n10(struct rnp_hw *hw, bool status)
{
	struct rnp_mac_info *mac = &hw->mac;
	struct rnp_eth_info *eth = &hw->eth;

	if (status)
		eth_wr32(eth, RNP10_ETH_RX_PROGFULL_THRESH_PORT, RECEIVE_ALL_THRESH);
	else 
		eth_wr32(eth, RNP10_ETH_RX_PROGFULL_THRESH_PORT, DROP_ALL_THRESH);

	//set mac rx status
	mac->ops.set_mac_rx(mac, status);
 
}

static void rnp_set_sriov_status_hw_ops_n10(struct rnp_hw *hw, bool status)
{
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_eth_info *eth = &hw->eth;
	u32 v;
	//open sriov or not
	//
	if (status) {
		dma_wr32(dma, RNP_DMA_CONFIG, dma_rd32(dma, RNP_DMA_CONFIG) & (~DMA_VEB_BYPASS));
		v = eth_rd32(eth, RNP10_MRQC_IOV_EN);
		v |= RNP10_IOV_ENABLED;
		eth_wr32(eth, RNP10_MRQC_IOV_EN, v);
	} else {

		v = eth_rd32(eth, RNP10_MRQC_IOV_EN);
		v &= ~(RNP10_IOV_ENABLED);
		eth_wr32(eth, RNP10_MRQC_IOV_EN, v);

		dma->ops.clr_veb_all(dma);
		// clean veb ?

	}
	
#ifdef NIC_VF_FXIED
	eth_wr32(eth, RNP10_VM_DMAC_MPSAR_RING(127), RNP_N10_MAX_VF - 1);
#endif


	/*
	wr32(hw, RNP_HOST_FILTER_EN, 1);
	wr32(hw, RNP_REDIR_EN, 1);
	wr32(hw, RNP_MRQC_IOV_EN, RNP_IOV_ENABLED);
	wr32(hw, RNP_ETH_DMAC_FCTRL,
	     rd32(hw, RNP_ETH_DMAC_FCTRL) | RNP_FCTRL_BROADCASE_BYPASS);
	// wr32(hw, RNP_ETH_DMAC_MCSTCTRL, RNP_MCSTCTRL_DMAC_47);
	*/
	/* Map PF MAC address in RAR Entry 0 to first pool following VFs */
	//hw->mac.ops.set_vmdq(hw, 0, ring / 2);

}


/* setup a vf mac a new */
//static void rnp_set_sriov_vf_mac_hw_ops_n10(struct rnp_hw *hw, u8 *mac, int vf, bool enable)
//{
//
//	struct rnp_eth_info *eth = &hw->eth;
//	int rar_entry = hw->num_rar_entries - (vf + 1);
//#ifdef FIX_VF_BUG
//	/* we use the next vf ring in condition*/
//	int vf_ring = vf * 2 + 2; 
//#else
//	int vf_ring = vf * 2; 
//#endif
//	// setup eth rar
//	eth->ops.set_rar(eth, rar_entry, mac, enable);	
//	// setup vmdq 
//	eth->ops.set_vmdq(eth, rar_entry, vf_ring / 2);
//	// now only setup own veb
//
//}
static void rnp_set_sriov_vf_mc_hw_ops_n10(struct rnp_hw *hw, u16 mc_addr)
{
	struct rnp_eth_info *eth = &hw->eth;
	u32 vector_bit;
	u32 vector_reg;
	u32 mta_reg;

	vector_reg = (mc_addr >> 5) & 0x7F;
	vector_bit = mc_addr & 0x1F;
	mta_reg = eth_rd32(eth, RNP10_ETH_MUTICAST_HASH_TABLE(vector_reg));
	mta_reg |= (1 << vector_bit);
	eth_wr32(eth, RNP10_ETH_MUTICAST_HASH_TABLE(vector_reg), mta_reg);
}

/*static void rnp_set_sriov_vf_vlan_hw_ops_n10(struct rnp_hw *hw, int vf, u16 vlan ,u8 qos)
  {

	// setup veb only ?
}*/

static void rnp_update_sriov_info_hw_ops_n10(struct rnp_hw *hw)
{

	// update sriov info to hw 
}

static void rnp_set_pause_mode_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_mac_info *mac = &hw->mac;
	struct rnp_eth_info *eth = &hw->eth;

	mac->ops.set_fc_mode(mac);
	eth->ops.set_fc_mode(eth);
	//set up pasus info to he
	// call 

}

static void rnp_update_hw_info_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
	u32 data;
	//1 enable eth filter
	eth_wr32(eth, RNP10_HOST_FILTER_EN, 1);
	//2 open redir en
	eth_wr32(eth, RNP10_REDIR_EN, 1);
	
	//3 open sctp checksum and other checksum?
	if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM)
		eth_wr32(eth, RNP10_ETH_SCTP_CHECKSUM_EN, 1);
	
	//4 mark muticaset as broadcast
	dma_wr32(dma, RNP_VEB_MAC_MASK_LO, 0xffffffff);
	dma_wr32(dma, RNP_VEB_MAC_MASK_HI, 0xfeff);
	//5 setup dma split 

	data = dma_rd32(dma, RNP_DMA_CONFIG);
	data &= (0x00000ffff);
#ifdef FT_PADDING
#define PADDING_BIT 8
	if (adapter->priv_flags & RNP_PRIV_FLAG_FT_PADDING)
		SET_BIT(PADDING_BIT, data);
#endif
	// n500 not use this 
#define RX_MAX_DWORD (96)
	data |= (RX_MAX_DWORD << 16);
	dma_wr32(dma, RNP_DMA_CONFIG, data);
	//rnp_setup_dma_rx(adapter, 96);
	//6 open vxlan inner match?

	//7
}

static void rnp_set_rx_hash_hw_ops_n10(struct rnp_hw *hw, bool status, bool sriov_flag)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.set_rx_hash(eth, status, sriov_flag);

}

/* setup mac to rar 0 
 * clean vmdq 
 * clean mc addr */
static s32 rnp_init_rx_addrs_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_eth_info *eth = &hw->eth;
	
	u32 i;
	u32 rar_entries = eth->num_rar_entries;
	u32 v;

	hw_dbg(hw,
		   "init_rx_addrs:rar_entries:%d, mac.addr:%pM\n",
		   rar_entries,
		   hw->mac.addr);
	/*
	 * If the current mac address is valid, assume it is a software override
	 * to the permanent address.
	 * Otherwise, use the permanent address from the eeprom.
	 */
	if (!is_valid_ether_addr(hw->mac.addr)) {
		/* Get the MAC address from the RAR0 for later reference */
		// fixme 
		// use perm mac 
		//hw->mac.ops.get_mac_addr(hw, hw->mac.addr);
		memcpy(hw->mac.addr, hw->mac.perm_addr, ETH_ALEN);
		hw_dbg(hw, " Keeping Current RAR0 Addr =%pM\n", hw->mac.addr);
	} else {
		/* Setup the receive address. */
		hw_dbg(hw, "Overriding MAC Address in RAR[0]\n");
		hw_dbg(hw, " New MAC Addr =%pM\n", hw->mac.addr);

		// set rar 0 
		eth->ops.set_rar(eth, 0, hw->mac.addr, true);
		//hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, RNP_RAH_AV);

		/*  clear VMDq pool/queue selection for RAR 0 */
		eth->ops.clear_vmdq(eth, 0, RNP_CLEAR_VMDQ_ALL); 
		//hw->mac.ops.clear_vmdq(hw, 0, RNP_CLEAR_VMDQ_ALL);
	}
	hw->addr_ctrl.overflow_promisc = 0;
	hw->addr_ctrl.rar_used_count = 1;

	/* Zero out the other receive addresses. */
	hw_dbg(hw, "Clearing RAR[1-%d]\n", rar_entries - 1);
	for (i = 1; i < rar_entries; i++) {
		eth->ops.clear_rar(eth, i);
	}

	/* Clear the MTA */
	hw->addr_ctrl.mta_in_use = 0;
	v = eth_rd32(eth, RNP10_ETH_DMAC_MCSTCTRL);
	v &= (~0x3);
	v |= eth->mc_filter_type;
	eth_wr32(eth, RNP10_ETH_DMAC_MCSTCTRL, v);

	hw_dbg(hw, " Clearing MTA\n");
	eth->ops.clr_mc_addr(eth);

	/* we not use uta filter now */
	//if (hw->mac.ops.init_uta_tables)
	//	hw->mac.ops.init_uta_tables(hw);

	return 0;
}

// clean vlan filter tables
static void rnp_clr_vfta_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.clr_vfta(eth);
}

static void rnp_set_txvlan_mode_hw_ops_n10(struct rnp_hw *hw, bool cvlan)
{
	struct rnp_mac_info *mac = &hw->mac;
	if (cvlan) {
		mac_wr32(mac, RNP10_MAC_TX_VLAN_TAG, 0x4000000);
		mac_wr32(mac, RNP10_MAC_TX_VLAN_MODE, 0x100000);
		mac_wr32(mac, RNP10_MAC_INNER_VLAN_INCL, 0x100000);
	} else {
		mac_wr32(mac, RNP10_MAC_TX_VLAN_TAG, 0xc600000);
		mac_wr32(mac, RNP10_MAC_TX_VLAN_MODE, 0x180000);
		mac_wr32(mac, RNP10_MAC_INNER_VLAN_INCL, 0x100000);
	}

}

static void rnp_set_rss_key_hw_ops_n10(struct rnp_hw *hw, bool sriov_flag)
{
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
	int key_len = RNP_RSS_KEY_SIZE;

	memcpy(hw->rss_key, adapter->rss_key, key_len);

	eth->ops.set_rss_key(eth, sriov_flag);


}

static void rnp_set_rss_table_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;
	u32 reta_entries = hw->rss_indir_tbl_num;
	u32 tc_entries = hw->rss_tc_tbl_num;
	int i;
	struct rnp_eth_info *eth = &hw->eth;

	//for (i = 0; i < tc_entries; i++)
	//	hw->rss_tc_tbl[i] = adapter->rss_tc_tbl[i];

	//for (i = 0; i < reta_entries; i++) 
	//	hw->rss_indir_tbl[i] = adapter->rss_indir_tbl[i];



	eth->ops.set_rss_table(eth);

}

static void rnp_set_mbx_link_event_hw_ops_n10(struct rnp_hw *hw, int enable)
{

	rnp_mbx_link_event_enable(hw, enable);

}

static void rnp_set_mbx_ifup_hw_ops_n10(struct rnp_hw *hw, int enable)
{

	rnp_mbx_ifup_down(hw, enable);

}

/**
 *  rnp_check_mac_link_n10 - Determine link and speed status
 *  @hw: pointer to hardware structure
 *  @speed: pointer to link speed
 *  @link_up: true when link is up
 *  @link_up_wait_to_complete: bool used to wait for link up or not
 *
 *  Reads the links register to determine if link is up and the current speed
 **/
s32 rnp_check_mac_link_hw_ops_n10(struct rnp_hw *hw,
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
	} else if (hw->speed == 100) {
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

s32 rnp_setup_mac_link_hw_ops_n10(struct rnp_hw *hw, u32 adv, bool enable)
{


	return 0;
}


void rnp_clean_link_hw_ops_n10(struct rnp_hw *hw)
{
	hw->link = 0;	
	//wr32(hw, RNP_ETH_EXCEPT_DROP_PROC, 0xf);
}

static void rnp_set_layer2_hw_ops_n10(struct rnp_hw *hw, union rnp_atr_input *input,
	u16 pri_id, u8 queue)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.set_layer2_remapping(eth, input, pri_id, queue);

}

static void rnp_clr_layer2_hw_ops_n10(struct rnp_hw *hw, u16 pri_id)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.clr_layer2_remapping(eth, pri_id);
}

static void rnp_clr_all_layer2_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.clr_all_layer2_remapping(eth);

}

static void rnp_clr_all_tuple5_hw_ops_n10(struct rnp_hw *hw)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.clr_all_tuple5_remapping(eth);

}

static void rnp_update_msix_count_hw_ops_n10(struct rnp_hw *hw, int msix_count)
{
	int msix_count_new;
	struct rnp_mac_info *mac = &hw->mac;


	msix_count_new = clamp_t(int, msix_count, 2, RNP_N10_MSIX_VECTORS);

	mac->max_msix_vectors = msix_count_new;
	hw->max_msix_vectors = msix_count_new;
}


static void rnp_set_tuple5_hw_ops_n10(struct rnp_hw *hw, union rnp_atr_input *input,
	u16 pri_id, u8 queue)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.set_tuple5_remapping(eth, input, pri_id, queue);

}

static void rnp_clr_tuple5_hw_ops_n10(struct rnp_hw *hw, u16 pri_id)
{
	struct rnp_eth_info *eth = &hw->eth;

	eth->ops.clr_tuple5_remapping(eth, pri_id);
}

static void rnp_update_hw_status_hw_ops_n10(struct rnp_hw *hw,
		struct rnp_hw_stats *hw_stats,
		struct net_device_stats *net_stats)
{
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_eth_info *eth = &hw->eth;
	struct rnp_mac_info *mac = &hw->mac;
	int port;

	hw_stats->dma_to_dma = dma_rd32(dma, RNP_DMA_STATS_DMA_TO_MAC_CHANNEL_0) +
			       dma_rd32(dma, RNP_DMA_STATS_DMA_TO_MAC_CHANNEL_1) +
			       dma_rd32(dma, RNP_DMA_STATS_DMA_TO_MAC_CHANNEL_2) +
			       dma_rd32(dma, RNP_DMA_STATS_DMA_TO_MAC_CHANNEL_3);

	hw_stats->dma_to_switch = dma_rd32(dma, RNP_DMA_STATS_DMA_TO_SWITCH);
	//hw_stats->mac_to_mac = rd32(hw, RNP_DMA_STATS_MAC_TO_MAC);
	//hw_stats->switch_to_switch = rd32(hw, RNP_DMA_STATS_SWITCH_TO_SWITCH);
	hw_stats->mac_to_dma = dma_rd32(dma, RNP_DMA_STATS_MAC_TO_DMA);

	//=== rx
	//hw_stats->dma_to_host = rd32(hw, RNP_PCI_WR_TO_HOST) / 2;

	for (port = 0; port < 4; port++) {
		/* we use Hardware stats? */
		//net_stats->rx_packets += rd32(hw, RNP_RXTRANS_RX_PKTS(port));
		//net_stats->rx_dropped += rd32(hw, RNP_RXTRANS_DROP_PKTS(port));
		net_stats->rx_crc_errors =
			eth_rd32(eth, RNP10_RXTRANS_CRC_ERR_PKTS(port));

		net_stats->rx_errors +=
			eth_rd32(eth, RNP10_RXTRANS_WDT_ERR_PKTS(port)) +
			eth_rd32(eth, RNP10_RXTRANS_CODE_ERR_PKTS(port)) +
			eth_rd32(eth, RNP10_RXTRANS_CRC_ERR_PKTS(port)) +
			eth_rd32(eth, RNP10_RXTRANS_SLEN_ERR_PKTS(port)) +
			eth_rd32(eth, RNP10_RXTRANS_GLEN_ERR_PKTS(port)) +
			eth_rd32(eth, RNP10_RXTRANS_IPH_ERR_PKTS(port)) +
			eth_rd32(eth, RNP10_RXTRANS_LEN_ERR_PKTS(port)) +
			eth_rd32(eth, RNP10_RXTRANS_CUT_ERR_PKTS(port));
		// tx
		/* use hardware reg status */
		//net_stats->tx_packets +=
		//	rd32(hw, RNP_ETH_EMAC_TX_TO_PHY_PKTS(port));
	}
	//=== drop ===
	hw_stats->invalid_droped_packets = eth_rd32(eth, RNP10_ETH_INVALID_DROP_PKTS);
	hw_stats->rx_capabity_lost = eth_rd32(eth, RNP10_RXTRANS_DROP(0)) +
		eth_rd32(eth, RNP10_RXTRANS_CUT_ERR_PKTS(0));
	hw_stats->filter_dropped_packets = eth_rd32(eth, RNP10_ETH_FILTER_DROP_PKTS);
	hw_stats->host_l2_match_drop = eth_rd32(eth, RNP10_ETH_HOST_L2_DROP_PKTS);
	hw_stats->redir_input_match_drop =
		eth_rd32(eth, RNP10_ETH_REDIR_INPUT_MATCH_DROP_PKTS);
	hw_stats->redir_etype_match_drop = eth_rd32(eth, RNP10_ETH_ETYPE_DROP_PKTS);
	hw_stats->redir_tcp_syn_match_drop =
		eth_rd32(eth, RNP10_ETH_TCP_SYN_DROP_PKTS);
	hw_stats->redir_tuple5_match_drop =
		eth_rd32(eth, RNP10_ETH_REDIR_TUPLE5_DROP_PKTS);
	hw_stats->redir_tcam_match_drop =
		eth_rd32(eth, RNP10_ETH_REDIR_TCAM_DROP_PKTS);
	hw_stats->bmc_dropped_packets = eth_rd32(eth, RNP10_ETH_DECAP_BMC_DROP_NUM);
	hw_stats->switch_dropped_packets =
		eth_rd32(eth, RNP10_ETH_DECAP_SWITCH_DROP_NUM);
	hw_stats->mac_rx_broadcast = mac_rd32(mac, RNP10_MAC_STATS_BROADCAST_LOW);
	hw_stats->mac_rx_broadcast +=
		((u64)mac_rd32(mac, RNP10_MAC_STATS_BROADCAST_HIGH) << 32);

	hw_stats->mac_rx_multicast = mac_rd32(mac, RNP10_MAC_STATS_MULTICAST_LOW);
	hw_stats->mac_rx_multicast +=
		((u64)mac_rd32(mac, RNP10_MAC_STATS_MULTICAST_HIGH) << 32);


}

#ifdef HAVE_ETHTOOL_GET_SSET_COUNT

enum n10_priv_bits
{
	n10_mac_loopback = 0,
	n10_switch_loopback = 1,
	n10_veb_enable = 4,
	n10_padding_enable = 8,
	n10_padding_debug_enable = 0x10,
};

static const char rnp10_priv_flags_strings[][ETH_GSTRING_LEN] = {
#define RNP10_MAC_LOOPBACK	  BIT(0)
#define RNP10_SWITCH_LOOPBACK	  BIT(1)
#define RNP10_VEB_ENABLE	  BIT(2)
#define RNP10_FT_PADDING	  BIT(3)
#define RNP10_PADDING_DEBUG	  BIT(4)
#define RNP10_PTP_FEATURE         BIT(5)
#define RNP10_SIMULATE_DOWN	  BIT(6)
#define RNP10_VXLAN_INNER_MATCH   BIT(7)
#define RNP10_REC_HDR_LEN_ERR     BIT(8)
	"mac_loopback",
	"switch_loopback",
	"veb_enable",
	"pcie_patch",
	"padding_debug",
	"ptp_performance_debug",
	"simulate_link_down",
	"vxlan_inner_match",
	"mask_len_err"};

#define RNP10_PRIV_FLAGS_STR_LEN ARRAY_SIZE(rnp10_priv_flags_strings)
#endif

const struct rnp_stats rnp10_gstrings_net_stats[] = {
	RNP_NETDEV_STAT(rx_packets),
	RNP_NETDEV_STAT(tx_packets),
	RNP_NETDEV_STAT(rx_bytes),
	RNP_NETDEV_STAT(tx_bytes),
	RNP_NETDEV_STAT(rx_errors),
	RNP_NETDEV_STAT(tx_errors),
	RNP_NETDEV_STAT(rx_dropped),
	RNP_NETDEV_STAT(tx_dropped),
	RNP_NETDEV_STAT(multicast),
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

#define RNP10_GLOBAL_STATS_LEN ARRAY_SIZE(rnp10_gstrings_net_stats)

static struct rnp_stats rnp10_hwstrings_stats[] = {
	RNP_HW_STAT("dma_to_mac", hw_stats.dma_to_dma),
	RNP_HW_STAT("dma_to_switch", hw_stats.dma_to_switch),
	// RNP_HW_STAT("mac_to_mac", hw_stats.mac_to_mac),
	// RNP_HW_STAT("switch_to_switch", hw_stats.switch_to_switch),
	RNP_HW_STAT("eth_to_dma", hw_stats.mac_to_dma),
	// RNP_HW_STAT("switch_to_dma", hw_stats.switch_to_dma),
	RNP_HW_STAT("vlan_add_cnt", hw_stats.vlan_add_cnt),
	RNP_HW_STAT("vlan_strip_cnt", hw_stats.vlan_strip_cnt),
	//=== drop==
	RNP_HW_STAT("invalid_droped_packets", hw_stats.invalid_droped_packets),
	RNP_HW_STAT("rx_capabity_drop", hw_stats.rx_capabity_lost),

	RNP_HW_STAT("filter_dropped_packets", hw_stats.filter_dropped_packets),
	RNP_HW_STAT("host_l2_match_drop", hw_stats.host_l2_match_drop),
	RNP_HW_STAT("redir_input_match_drop", hw_stats.redir_input_match_drop),
	RNP_HW_STAT("redir_etype_match_drop", hw_stats.redir_etype_match_drop),
	RNP_HW_STAT("redir_tcp_syn_match_drop", hw_stats.redir_tcp_syn_match_drop),
	RNP_HW_STAT("redir_tuple5_match_drop", hw_stats.redir_tuple5_match_drop),
	RNP_HW_STAT("redir_tcam_match_drop", hw_stats.redir_tcam_match_drop),

	// RNP_HW_STAT("driver_dropped_packets", hw_stats.driver_dropped_packets),
	RNP_HW_STAT("bmc_dropped_packets", hw_stats.bmc_dropped_packets),
	RNP_HW_STAT("switch_dropped_packets", hw_stats.switch_dropped_packets),
	RNP_HW_STAT("rx_csum_offload_errors", hw_csum_rx_error),
	RNP_HW_STAT("rx_csum_offload_good", hw_csum_rx_good),
	RNP_HW_STAT("rx_broadcast_count", hw_stats.mac_rx_broadcast),
	RNP_HW_STAT("rx_multicast_count", hw_stats.mac_rx_multicast),

};

#define RNP10_HWSTRINGS_STATS_LEN ARRAY_SIZE(rnp10_hwstrings_stats)

#define RNP10_STATS_LEN \
	(RNP10_GLOBAL_STATS_LEN + RNP10_HWSTRINGS_STATS_LEN + RNP_QUEUE_STATS_LEN)


#ifndef CLOST_SELF_TEST
#ifdef ETHTOOL_TEST
static const char rnp10_gstrings_test[][ETH_GSTRING_LEN] = {
	"Register test  (offline)",
	"Eeprom test    (offline)",
	"Interrupt test (offline)",
	"Loopback test  (offline)",
	"Link test   (on/offline)"};

#define RNP10_TEST_LEN (sizeof(rnp10_gstrings_test) / ETH_GSTRING_LEN)
#else
#define RNP10_TEST_LEN 0
#endif
#else
#define RNP10_TEST_LEN 0
#endif


static int rnp10_get_regs_len(struct net_device *netdev)
{
#define RNP10_REGS_LEN 1
	return RNP10_REGS_LEN * sizeof(u32);
}

static void rnp10_get_drvinfo(struct net_device *netdev,
		struct ethtool_drvinfo *drvinfo)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;

	strlcpy(drvinfo->driver, rnp_driver_name, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, rnp_driver_version, sizeof(drvinfo->version));

	snprintf(drvinfo->fw_version,
			 sizeof(drvinfo->fw_version),
			 "%d.%d.%d.%d 0x%08x",
			 ((char *)&(hw->fw_version))[3],
			 ((char *)&(hw->fw_version))[2],
			 ((char *)&(hw->fw_version))[1],
			 ((char *)&(hw->fw_version))[0],
			 hw->bd_uid);

	strlcpy(
		drvinfo->bus_info, pci_name(adapter->pdev), sizeof(drvinfo->bus_info));
	drvinfo->n_stats = RNP10_STATS_LEN;
	drvinfo->testinfo_len = RNP10_TEST_LEN;
	drvinfo->regdump_len = rnp10_get_regs_len(netdev);
#ifdef HAVE_ETHTOOL_GET_SSET_COUNT
	drvinfo->n_priv_flags = RNP10_PRIV_FLAGS_STR_LEN;
#endif
}


static void
rnp10_get_regs(struct net_device *netdev, struct ethtool_regs *regs, void *p)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	u32 *regs_buff = p;
	int i;

	memset(p, 0, RNP10_REGS_LEN * sizeof(u32));

	for (i = 0; i < RNP10_REGS_LEN; i++)
		regs_buff[i] = rd32(hw, i * 4);
}

static void rnp10_get_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	char *p = (char *)data;
	int i;
	struct rnp_ring *ring;
	u32 dma_ch;

	switch (stringset) {
		/* maybe we don't support test? */
#ifndef CLOST_SELF_TEST
		case ETH_SS_TEST:
			for (i = 0; i < RNP10_TEST_LEN; i++) {
				memcpy(data, rnp10_gstrings_test[i], ETH_GSTRING_LEN);
				data += ETH_GSTRING_LEN;
			}
			break;
#endif
		case ETH_SS_STATS:
			for (i = 0; i < RNP10_GLOBAL_STATS_LEN; i++) {
				memcpy(
					p, rnp10_gstrings_net_stats[i].stat_string, ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
			for (i = 0; i < RNP10_HWSTRINGS_STATS_LEN; i++) {
				memcpy(p, rnp10_hwstrings_stats[i].stat_string, ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
			for (i = 0; i < RNP_NUM_TX_QUEUES; i++) {
				//====  tx ========
				ring = adapter->tx_ring[i];
				dma_ch = ring->rnp_queue_idx;
#define SHORT_STATS
#ifdef SHORT_STATS
				sprintf(p, "---\n     queue%u_tx_packets", i);
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
				sprintf(p, "queue%u_tx_sw_next_to_use", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_send_bytes", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_send_bytes_to_hw", i);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_todo_update", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_send_done_bytes", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_added_vlan_packets", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_tx_next_to_clean", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_tx_irq_miss", i);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_tx_equal_count", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_tx_clean_times", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_tx_clean_count", i);
				p += ETH_GSTRING_LEN;

				//====  rx ========
				ring = adapter->rx_ring[i];
				dma_ch = ring->rnp_queue_idx;
				sprintf(p, "queue%u_rx_packets", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_bytes", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_driver_drop_packets", i);
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
				sprintf(p, "queue%u_rx_alloc_page", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_csum_offload_errs", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_csum_offload_good", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_poll_again_count", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_rm_vlan_packets", i);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_rx_hw_head", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_hw_tail", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_sw_next_to_use", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_sw_next_to_clean", i);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_rx_next_to_clean", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_irq_miss", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_equal_count", i);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_rx_clean_times", i);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_rx_clean_count", i);
				p += ETH_GSTRING_LEN;
#else
				sprintf(p, "\n     queue%u_dma%u_tx_packets", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_bytes", i, dma_ch);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_dma%u_tx_restart", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_busy", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_done_old", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_clean_desc", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_poll_count", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_irq_more", i, dma_ch);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_dma%u_tx_hw_head", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_hw_tail", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_sw_next_to_clean", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_sw_next_to_use", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_send_bytes", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_send_bytes_to_hw", i, dma_ch);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_dma%u_todo_update", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_send_done_bytes", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_added_vlan_packets", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_next_to_clean", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_irq_miss", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_equal_count", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_tx_clean_times", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dam%u_tx_clean_count", i, dma_ch);
				p += ETH_GSTRING_LEN;

				//====  rx ========
				ring = adapter->rx_ring[i];
				dma_ch = ring->rnp_queue_idx;
				sprintf(p, "----\n     queue%u_dma%u_rx_packets", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_bytes", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_driver_drop_packets", i, dma_ch);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_dma%u_rx_rsc", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_rsc_flush", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_non_eop_descs", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_alloc_page_failed", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_alloc_buff_failed", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_csum_offload_errs", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_csum_offload_good", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_poll_again_count", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_rm_vlan_packets", i, dma_ch);
				p += ETH_GSTRING_LEN;

				sprintf(p, "queue%u_dma%u_rx_hw_head", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_hw_tail", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_sw_next_to_use", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_sw_next_to_clean", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_next_to_clean", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_irq_miss", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_equal_count", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dma%u_rx_clean_times", i, dma_ch);
				p += ETH_GSTRING_LEN;
				sprintf(p, "queue%u_dam%u_rx_clean_count", i, dma_ch);
				p += ETH_GSTRING_LEN;
#endif
			}

			break;
#ifdef HAVE_ETHTOOL_GET_SSET_COUNT
		case ETH_SS_PRIV_FLAGS:
			memcpy(data,
				   rnp10_priv_flags_strings,
				   RNP10_PRIV_FLAGS_STR_LEN * ETH_GSTRING_LEN);
			break;
#endif /* HAVE_ETHTOOL_GET_SSET_COUNT */
	}
}

#ifndef HAVE_ETHTOOL_GET_SSET_COUNT
static int rnp10_get_stats_count(struct net_device *netdev)
{

	return RNP10_STATS_LEN;
}

#else

static int rnp10_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
		/* now we don't support test */
#ifndef CLOST_SELF_TEST
		case ETH_SS_TEST:
			return RNP10_TEST_LEN;
#endif
		case ETH_SS_STATS:
			return RNP10_STATS_LEN;
		case ETH_SS_PRIV_FLAGS:
			return RNP10_PRIV_FLAGS_STR_LEN;
		default:
			return -EOPNOTSUPP;
	}
}

static u32 rnp10_get_priv_flags(struct net_device *netdev)
{
	struct rnp_adapter *adapter = (struct rnp_adapter *)netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	u32 priv_flags = 0;
	// dbg("adapter priv is %x\n",iface->priv_flags);

	if (adapter->priv_flags & RNP_PRIV_FLAG_MAC_LOOPBACK)
		priv_flags |= RNP10_MAC_LOOPBACK;
	if (adapter->priv_flags & RNP_PRIV_FLAG_SWITCH_LOOPBACK)
		priv_flags |= RNP10_SWITCH_LOOPBACK;
	if (adapter->priv_flags & RNP_PRIV_FLAG_VEB_ENABLE)
		priv_flags |= RNP10_VEB_ENABLE;
	if (adapter->priv_flags & RNP_PRIV_FLAG_FT_PADDING)
		priv_flags |= RNP10_FT_PADDING;
	if (adapter->priv_flags & RNP_PRIV_FLAG_PADDING_DEBUG)
		priv_flags |= RNP10_PADDING_DEBUG;
	if (adapter->priv_flags & RNP_PRIV_FLAG_PTP_DEBUG)
		priv_flags |= RNP10_PTP_FEATURE;
	if (adapter->priv_flags & RNP_PRIV_FLAG_SIMUATE_DOWN)
		priv_flags |= RNP10_SIMULATE_DOWN;
	if (adapter->priv_flags & RNP_PRIV_FLAG_VXLAN_INNER_MATCH)
		priv_flags |= RNP10_VXLAN_INNER_MATCH;
	if (adapter->priv_flags & RNP_PRIV_FLAG_REC_HDR_LEN_ERR)
		priv_flags |= RNP10_REC_HDR_LEN_ERR;

	return priv_flags;
}

static int rnp10_set_priv_flags(struct net_device *netdev, u32 priv_flags)
{
	struct rnp_adapter *adapter = (struct rnp_adapter *)netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_eth_info *eth = &hw->eth;
	u32 data_old;
	u32 data_new;

	data_old = dma_rd32(dma, RNP_DMA_CONFIG);
	data_new = data_old;
	dbg("data old is %x\n", data_old);

	if (priv_flags & RNP10_MAC_LOOPBACK) {
		SET_BIT(n10_mac_loopback, data_new);
		adapter->priv_flags |= RNP_PRIV_FLAG_MAC_LOOPBACK;
	} else if (adapter->priv_flags & RNP_PRIV_FLAG_MAC_LOOPBACK) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_MAC_LOOPBACK);
		CLR_BIT(n10_mac_loopback, data_new);
	}

	if (priv_flags & RNP10_SWITCH_LOOPBACK) {
		SET_BIT(n10_switch_loopback, data_new);
		adapter->priv_flags |= RNP_PRIV_FLAG_SWITCH_LOOPBACK;
	} else if (adapter->priv_flags & RNP_PRIV_FLAG_SWITCH_LOOPBACK) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_SWITCH_LOOPBACK);
		CLR_BIT(n10_switch_loopback, data_new);
	}

	if (priv_flags & RNP10_VEB_ENABLE) {
		SET_BIT(n10_veb_enable, data_new);
		adapter->priv_flags |= RNP_PRIV_FLAG_VEB_ENABLE;
	} else if (adapter->priv_flags & RNP_PRIV_FLAG_VEB_ENABLE) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_VEB_ENABLE);
		CLR_BIT(n10_veb_enable, data_new);
	}

	if (priv_flags & RNP10_FT_PADDING) {
		SET_BIT(n10_padding_enable, data_new);
		adapter->priv_flags |= RNP_PRIV_FLAG_FT_PADDING;
	} else if (adapter->priv_flags & RNP_PRIV_FLAG_FT_PADDING) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_FT_PADDING);
		CLR_BIT(n10_padding_enable, data_new);
	}

	if (priv_flags & RNP10_PADDING_DEBUG)
		adapter->priv_flags |= RNP_PRIV_FLAG_PADDING_DEBUG;
	else if (adapter->priv_flags & RNP_PRIV_FLAG_PADDING_DEBUG)
		adapter->priv_flags &= (~RNP_PRIV_FLAG_PADDING_DEBUG);

	if (priv_flags & RNP10_PTP_FEATURE) {
		adapter->priv_flags |= RNP_PRIV_FLAG_PTP_DEBUG;
		adapter->flags2 |= ~RNP_FLAG2_PTP_ENABLED;
	} else if (adapter->priv_flags & RNP_PRIV_FLAG_PTP_DEBUG) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_PTP_DEBUG);
		adapter->flags2 &= (~RNP_FLAG2_PTP_ENABLED);
	}

	if (priv_flags & RNP10_SIMULATE_DOWN) {
		adapter->priv_flags |= RNP_PRIV_FLAG_SIMUATE_DOWN;
		/* set check link again */
		adapter->flags |= RNP_FLAG_NEED_LINK_UPDATE;
	} else if (adapter->priv_flags & RNP_PRIV_FLAG_SIMUATE_DOWN) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_SIMUATE_DOWN);
		/* set check link again */
		adapter->flags |= RNP_FLAG_NEED_LINK_UPDATE;
	}

	if (priv_flags & RNP10_VXLAN_INNER_MATCH) {
		adapter->priv_flags |= RNP_PRIV_FLAG_VXLAN_INNER_MATCH;
		hw->ops.set_vxlan_mode(hw, true); 
		//wr32(hw, RNP_ETH_WRAP_FIELD_TYPE, 1);
	} else if (adapter->priv_flags & RNP_PRIV_FLAG_VXLAN_INNER_MATCH) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_VXLAN_INNER_MATCH);
		hw->ops.set_vxlan_mode(hw, false); 
		//wr32(hw, RNP_ETH_WRAP_FIELD_TYPE, 0);
	}

	if (priv_flags & RNP10_REC_HDR_LEN_ERR) {
		adapter->priv_flags |= RNP_PRIV_FLAG_REC_HDR_LEN_ERR;
		eth_wr32(eth, RNP10_ETH_ERR_MASK_VECTOR, PKT_LEN_ERR | HDR_LEN_ERR);

	} else if (adapter->priv_flags & RNP_PRIV_FLAG_REC_HDR_LEN_ERR) {
		adapter->priv_flags &= (~RNP_PRIV_FLAG_REC_HDR_LEN_ERR);
		// clean mask
		eth_wr32(eth, RNP10_ETH_ERR_MASK_VECTOR, 0);
	}

	dbg("data new is %x\n", data_new);
	if (data_old != data_new)
		dma_wr32(dma, RNP_DMA_CONFIG, data_new);
	/* if ft_padding changed */
	if (CHK_BIT(n10_padding_enable, data_old) !=
		CHK_BIT(n10_padding_enable, data_new)) {
		rnp_msg_post_status(adapter, PF_FT_PADDING_STATUS);
	}

	return 0;
}

#endif

static void rnp10_get_ethtool_stats(struct net_device *netdev,
		struct ethtool_stats *stats,
		u64 *data)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rtnl_link_stats64 temp;
	struct rnp_hw *hw = &adapter->hw;
	struct net_device_stats *net_stats = &netdev->stats;
	unsigned int start;
	struct rnp_ring *ring;
	int i, j;
	char *p = NULL;

	// test prio map
	// for (i = 0; i < 16; i++)
	//	printk("prio_map %d is %d\n", i, netdev->prio_tc_map[i & TC_BITMASK]);

	rnp_update_stats(adapter);

	for (i = 0; i < RNP10_GLOBAL_STATS_LEN; i++) {
		p = (char *)net_stats + rnp10_gstrings_net_stats[i].stat_offset;
		data[i] = (rnp10_gstrings_net_stats[i].sizeof_stat == sizeof(u64))
					  ? *(u64 *)p
					  : *(u32 *)p;
	}
	for (j = 0; j < RNP10_HWSTRINGS_STATS_LEN; j++, i++) {
		p = (char *)adapter + rnp10_hwstrings_stats[j].stat_offset;
		data[i] = (rnp10_hwstrings_stats[j].sizeof_stat == sizeof(u64))
					  ? *(u64 *)p
					  : *(u32 *)p;
	}

	BUG_ON(RNP_NUM_TX_QUEUES != RNP_NUM_RX_QUEUES);

	for (j = 0; j < RNP_NUM_TX_QUEUES; j++) {
		int idx;
		/* tx-ring */
		ring = adapter->tx_ring[j];
		if (!ring) {
			// tx
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
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			// rx
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
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			continue;
		}
		idx = ring->rnp_queue_idx;

		data[i++] = ring->stats.packets;
		data[i++] = ring->stats.bytes;

		data[i++] = ring->tx_stats.restart_queue;
		data[i++] = ring->tx_stats.tx_busy;
		data[i++] = ring->tx_stats.tx_done_old;
		data[i++] = ring->tx_stats.clean_desc;
		data[i++] = ring->tx_stats.poll_count;
		data[i++] = ring->tx_stats.irq_more_count;

		/* rnp_tx_queue_ring_stat */
		data[i++] = ring_rd32(ring, RNP_DMA_REG_TX_DESC_BUF_HEAD);
		data[i++] = ring_rd32(ring, RNP_DMA_REG_TX_DESC_BUF_TAIL);
		data[i++] = ring->next_to_clean;
		data[i++] = ring->next_to_use;
		data[i++] = ring->tx_stats.send_bytes;
		data[i++] = ring->tx_stats.send_bytes_to_hw;
		data[i++] = ring->tx_stats.todo_update;
		data[i++] = ring->tx_stats.send_done_bytes;
		data[i++] = ring->tx_stats.vlan_add;
		if (ring->tx_stats.tx_next_to_clean == -1)
			data[i++] = ring->count;
		else
			data[i++] = ring->tx_stats.tx_next_to_clean;
		data[i++] = ring->tx_stats.tx_irq_miss;
		data[i++] = ring->tx_stats.tx_equal_count;
		data[i++] = ring->tx_stats.tx_clean_times;
		data[i++] = ring->tx_stats.tx_clean_count;

		/* rx-ring */
		ring = adapter->rx_ring[j];
		if (!ring) {
			// rx
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
			data[i++] = 0;
			data[i++] = 0;
			data[i++] = 0;
			continue;
		}
		idx = ring->rnp_queue_idx;
		data[i++] = ring->stats.packets;
		data[i++] = ring->stats.bytes;

		data[i++] = ring->rx_stats.driver_drop_packets;
		data[i++] = ring->rx_stats.rsc_count;
		data[i++] = ring->rx_stats.rsc_flush;
		data[i++] = ring->rx_stats.non_eop_descs;
		data[i++] = ring->rx_stats.alloc_rx_page_failed;
		data[i++] = ring->rx_stats.alloc_rx_buff_failed;
		data[i++] = ring->rx_stats.alloc_rx_page;
		data[i++] = ring->rx_stats.csum_err;
		data[i++] = ring->rx_stats.csum_good;
		data[i++] = ring->rx_stats.poll_again_count;
		data[i++] = ring->rx_stats.vlan_remove;

		/* rnp_rx_queue_ring_stat */
		data[i++] = ring_rd32(ring, RNP_DMA_REG_RX_DESC_BUF_HEAD);
		data[i++] = ring_rd32(ring, RNP_DMA_REG_RX_DESC_BUF_TAIL);
		data[i++] = ring->next_to_use;
		data[i++] = ring->next_to_clean;
		if (ring->rx_stats.rx_next_to_clean == -1)
			data[i++] = ring->count;
		else
			data[i++] = ring->rx_stats.rx_next_to_clean;
		data[i++] = ring->rx_stats.rx_irq_miss;
		data[i++] = ring->rx_stats.rx_equal_count;
		data[i++] = ring->rx_stats.rx_clean_times;
		data[i++] = ring->rx_stats.rx_clean_count;
	}
}

int rnp_nway_reset(struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	netdev_info(netdev, "NIC Link is Down\n");
	rnp_down(adapter);
	msleep(10);
	rnp_up(adapter);
	return 0;
}
/* n10 ethtool_ops ops here */

static const struct ethtool_ops rnp10_ethtool_ops = {

#ifdef HAVE_ETHTOOL_CONVERT_U32_AND_LINK_MODE
	.get_link_ksettings = rnp_get_link_ksettings,
	.set_link_ksettings = rnp_set_link_ksettings,
#else
	.get_settings = rnp_get_settings,
	.set_settings = rnp_set_settings,
#endif
	.get_drvinfo = rnp10_get_drvinfo,

	.get_regs_len = rnp10_get_regs_len,
	.get_regs = rnp10_get_regs,
	.get_wol = rnp_get_wol,
	.set_wol = rnp_set_wol,
	.nway_reset = rnp_nway_reset,
	.get_link = ethtool_op_get_link,
	//.get_eeprom_len         = rnp_get_eeprom_len,
	//.get_eeprom             = rnp_get_eeprom,
	//.set_eeprom             = rnp_set_eeprom,
	.get_ringparam = rnp_get_ringparam,
	.set_ringparam = rnp_set_ringparam,
	.get_pauseparam = rnp_get_pauseparam,
	.set_pauseparam = rnp_set_pauseparam,
	.get_msglevel = rnp_get_msglevel,
	.set_msglevel = rnp_set_msglevel,
#ifdef ETHTOOL_GFECPARAM
	.get_fecparam = rnp_get_fecparam,
	.set_fecparam = rnp_set_fecparam,
#endif
#ifndef CLOST_SELF_TEST
#ifndef HAVE_ETHTOOL_GET_SSET_COUNT
//.self_test_count        = rnp_diag_test_count,
#endif
	// todo
	.self_test = rnp_diag_test,
#endif
	.get_strings = rnp10_get_strings,
#ifndef HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT
#ifdef HAVE_ETHTOOL_SET_PHYS_ID
	.set_phys_id = rnp_set_phys_id,
#else
//.phys_id  = rnp_phys_id,
#endif /* HAVE_ETHTOOL_SET_PHYS_ID */
#endif /* HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT */
#ifndef HAVE_ETHTOOL_GET_SSET_COUNT
	.get_stats_count = rnp10_get_stats_count,
#else  /* HAVE_ETHTOOL_GET_SSET_COUNT */
	.get_sset_count = rnp10_get_sset_count,
	.get_priv_flags = rnp10_get_priv_flags, // priv flags
	// todo
	.set_priv_flags = rnp10_set_priv_flags,
#endif /* HAVE_ETHTOOL_GET_SSET_COUNT */
	.get_ethtool_stats = rnp10_get_ethtool_stats,
#ifdef HAVE_ETHTOOL_GET_PERM_ADDR
	.get_perm_addr = ethtool_op_get_perm_addr,
#endif /* HAVE_ETHTOOL_GET_PERM_ADDR */
	.get_coalesce = rnp_get_coalesce,
	.set_coalesce = rnp_set_coalesce,
#ifdef ETHTOOL_COALESCE_USECS
	.supported_coalesce_params = ETHTOOL_COALESCE_USECS,
#endif /* ETHTOOL_COALESCE_USECS */

#ifndef HAVE_NDO_SET_FEATURES
	.get_rx_csum = rnp_get_rx_csum,
	.set_rx_csum = rnp_set_rx_csum,
	.get_tx_csum = ethtool_op_get_tx_csum,
	.set_tx_csum = rnp_set_tx_csum,
	.get_sg = ethtool_op_get_sg,
	.set_sg = ethtool_op_set_sg,
#ifdef NETIF_F_TSO
	.get_tso = ethtool_op_get_tso,
	.set_tso = rnp_set_tso,
#endif /* NETIF_F_TSO */
#ifdef ETHTOOL_GFLAGS
	.get_flags = ethtool_op_get_flags,
//.set_flags              = rnp_set_flags,
#endif
#endif /* HAVE_NDO_SET_FEATURES */
#ifdef ETHTOOL_GRXRINGS
	.get_rxnfc = rnp_get_rxnfc,
	.set_rxnfc = rnp_set_rxnfc,
#endif

#ifdef ETHTOOL_SRXNTUPLE
	.set_rx_ntuple = rnp_set_rx_ntuple,
#endif
#ifndef HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT
#ifdef ETHTOOL_GEEE
//.get_eee                = ixgbe_get_eee,
#endif /* ETHTOOL_GEEE */
#ifdef ETHTOOL_SEEE
//.set_eee                = ixgbe_set_eee,
#endif /* ETHTOOL_SEEE */
#ifdef ETHTOOL_SCHANNELS
	.get_channels = rnp_get_channels,
	.set_channels = rnp_set_channels,
#endif
#ifdef ETHTOOL_GMODULEINFO
	.get_module_info = rnp_get_module_info,
	.get_module_eeprom = rnp_get_module_eeprom,
#endif
#ifdef HAVE_ETHTOOL_GET_TS_INFO
	.get_ts_info = rnp_get_ts_info,
#endif
#if defined(ETHTOOL_GRSSH) && defined(ETHTOOL_SRSSH)
	.get_rxfh_indir_size = rnp_rss_indir_size,
	.get_rxfh_key_size = rnp_get_rxfh_key_size,
	.get_rxfh = rnp_get_rxfh,
	.set_rxfh = rnp_set_rxfh,
#endif /* ETHTOOL_GRSSH && ETHTOOL_SRSSH */

	.get_dump_flag = rnp_get_dump_flag,
	.get_dump_data = rnp_get_dump_data,
	.set_dump = rnp_set_dump,
#endif /* HAVE_RHEL6_ETHTOOL_OPS_EXT_STRUCT */

#ifdef HAVE_DDP_PROFILE_UPLOAD_SUPPORT
	.flash_device = rnp_flash_device,
#endif /* HAVE_DDP_PROFILE_UPLOAD_SUPPORT */
};


void rnp_set_ethtool_hw_ops_n10(struct net_device *netdev)
{
#ifndef ETHTOOL_OPS_COMPAT
	netdev->ethtool_ops = &rnp10_ethtool_ops;
#else
	SET_ETHTOOL_OPS(netdev, &rnp10_ethtool_ops);
#endif


}

// only choose eth or mac func
static struct rnp_hw_operations hw_ops_n10 = {
	.init_hw = &rnp_init_hw_ops_n10,
	.reset_hw = &rnp_reset_hw_ops_n10,
	.start_hw = &rnp_start_hw_ops_n10,

	.set_mtu = &rnp_set_mtu_hw_ops_n10,
	.set_vlan_filter_en = &rnp_set_vlan_filter_en_hw_ops_n10,
	.set_vlan_filter = &rnp_set_vlan_filter_hw_ops_n10,
	.set_vf_vlan_filter = &rnp_set_vf_vlan_filter_hw_ops_n10,
	.set_vlan_strip = &rnp_set_vlan_strip_hw_ops_n10,
	.set_mac = &rnp_set_mac_hw_ops_n10,
	//.set_virtualization = &rnp_set_virtualization_n10, 
	.set_rx_mode = &rnp_set_rx_mode_hw_ops_n10,
	// only update rar ,no veb
	.set_rar_with_vf = &rnp_set_rar_with_vf_hw_ops_n10, 
	.clr_rar = &rnp_clr_rar_hw_ops_n10,
	.clr_vlan_veb = &rnp_clr_vlan_veb_hw_ops_n10,
	.set_txvlan_mode = &rnp_set_txvlan_mode_hw_ops_n10,
	.set_fcs_mode = &rnp_set_fcs_mode_hw_ops_n10,
	.set_vxlan_port = &rnp_set_vxlan_port_hw_ops_n10,
	.set_vxlan_mode = &rnp_set_vxlan_mode_hw_ops_n10,
	.set_mac_rx = &rnp_set_mac_rx_hw_ops_n10,
	.set_rx_hash = &rnp_set_rx_hash_hw_ops_n10,
	.set_pause_mode = &rnp_set_pause_mode_hw_ops_n10,
	.update_hw_info = &rnp_update_hw_info_hw_ops_n10,

	.update_sriov_info = &rnp_update_sriov_info_hw_ops_n10,
	// sriov relative
	.set_sriov_status = &rnp_set_sriov_status_hw_ops_n10,
	//.set_sriov_vf_mac = &rnp_set_sriov_vf_mac_hw_ops_n10,
	.set_sriov_vf_mc = &rnp_set_sriov_vf_mc_hw_ops_n10,
	//.set_sriov_vf_vlan = &rnp_set_sriov_vf_vlan_hw_ops_n10,

	.init_rx_addrs = &rnp_init_rx_addrs_hw_ops_n10,
	.clr_vfta = &rnp_clr_vfta_hw_ops_n10,

	.set_rss_key = &rnp_set_rss_key_hw_ops_n10,
	.set_rss_table = &rnp_set_rss_table_hw_ops_n10,
	
	.update_hw_status = &rnp_update_hw_status_hw_ops_n10,

	// mbx add here?
	.set_mbx_link_event = &rnp_set_mbx_link_event_hw_ops_n10,
	.set_mbx_ifup = &rnp_set_mbx_ifup_hw_ops_n10,
	// mbx write or read ?
	
	.check_link = &rnp_check_mac_link_hw_ops_n10,
	.setup_link = &rnp_setup_mac_link_hw_ops_n10,
	.clean_link = &rnp_clean_link_hw_ops_n10,

	.set_layer2_remapping = &rnp_set_layer2_hw_ops_n10,
	.clr_layer2_remapping = &rnp_clr_layer2_hw_ops_n10,
	.clr_all_layer2_remapping = &rnp_clr_all_layer2_hw_ops_n10,
	.set_tuple5_remapping = &rnp_set_tuple5_hw_ops_n10,
	.clr_tuple5_remapping = &rnp_clr_tuple5_hw_ops_n10,
	.clr_all_tuple5_remapping = &rnp_clr_all_tuple5_hw_ops_n10,
	.update_msix_count = &rnp_update_msix_count_hw_ops_n10,


	// ethtool callback 
	.setup_ethtool = &rnp_set_ethtool_hw_ops_n10,
	// todo 
	//.set_rx_status = &rnp_rx_status_hw_ops_n10;
	// setup mbx?



};

static void rnp_mac_set_rx_n10(struct rnp_mac_info *mac, bool status)
{
	struct rnp_hw *hw = (struct rnp_hw *)mac->back;
	struct rnp_adapter *adapter = (struct rnp_adapter *)hw->back;

	// control mac rx
	u32 value = 0;
	u32 count = 0;

	if (status) {
		// set rx open 
		do {
			mac_wr32(
				mac, RNP10_MAC_RX_CFG, mac_rd32(mac, RNP10_MAC_RX_CFG) | 0x01);
			usleep_range(100, 200);
			value = mac_rd32(mac, RNP10_MAC_RX_CFG);
			count++;
			if (count > 1000) {
				printk("setup rx on timeout\n");
				break;
			}
			// printk("wait set rx open %x\n", value);
		} while (!(value & 0x01));

		if (adapter->flags & RNP_FLAG_SWITCH_LOOPBACK_EN) {
			// mac-recv all
			mac_wr32(mac, RNP10_MAC_PKT_FLT, BIT(31) | BIT(0));
			// disable unicase-table filter
			eth_wr32(&hw->eth, RNP10_ETH_DMAC_MCSTCTRL, 0x0);
		} else {
			// clean loop back
			do {
				mac_wr32(mac,
						RNP10_MAC_RX_CFG,
						mac_rd32(mac, RNP10_MAC_RX_CFG) & (~0x400));
				usleep_range(100, 200);
				value = mac_rd32(mac, RNP_MAC_RX_CFG);
				count++;
				if (count > 1000) {
					printk("setup rx off timeout\n");
					break;
				}
			} while (value & 0x400);
			mac_wr32(mac, RNP10_MAC_PKT_FLT, 0x00000001);
		}
		// wr32(hw, RNP_MAC_LPI_CTRL, 0x00060000);
	} else {
	//	do {
	//		mac_wr32(mac,
	//				 RNP10_MAC_RX_CFG,
	//				 mac_rd32(mac, RNP10_MAC_RX_CFG) & (~0x01));
	//		usleep_range(100, 200);
	//		value = mac_rd32(mac, RNP10_MAC_RX_CFG);
	//		count++;
	//		if (count > 1000) {
	//			printk("setup rx off timeout\n");
	//			break;
	//		}
	//	} while (value & 0x01);
		// wr32(hw, RNP_MAC_RX_CFG, 0x07d001c7);
                // set loopback
                do {
                        mac_wr32(mac,
                                 RNP10_MAC_RX_CFG,
                                 mac_rd32(mac, RNP10_MAC_RX_CFG) | 0x400);
                        usleep_range(100, 200);
                        value = mac_rd32(mac, RNP10_MAC_RX_CFG);
                        count++;
                        if (count > 1000) {
                                printk("setup rx on timeout\n");
                                break;
                        }
                        // printk("wait set rx open %x\n", value);
                } while (!(value & 0x400));
		mac_wr32(mac, RNP10_MAC_PKT_FLT, 0x0);
	}
}
static void rnp_mac_fcs_n10(struct rnp_mac_info *mac, bool status)
{
	u32 value;
	// set acs cst
#define FCS_MASK (0x6)
	value = mac_rd32(mac, RNP10_MAC_RX_CFG);
	if (status) {
		value &= (~FCS_MASK);
		
	} else {
		value |= FCS_MASK;	
	}

	mac_wr32(mac, RNP10_MAC_RX_CFG, value);

}
/**
 *  rnp_fc_mode_n10 - Enable flow control
 *  @hw: pointer to hardware structure
 *
 *  Enable flow control according to the current settings.
 **/
s32 rnp_mac_fc_mode_n10(struct rnp_mac_info *mac)
{
	struct rnp_hw *hw = (struct rnp_hw *)mac->back;
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


	/* Negotiate the fc mode to use */
	// rnp_fc_autoneg(hw);

	/* Disable any previous flow control settings */
	rxctl_reg = mac_rd32(mac, RNP10_MAC_RX_FLOW_CTRL);
	rxctl_reg &= (~RNP10_RX_FLOW_ENABLE_MASK);

	for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++) {
		txctl_reg[i] = mac_rd32(mac, RNP10_MAC_Q0_TX_FLOW_CTRL(i));
		txctl_reg[i] &= (~RNP10_TX_FLOW_ENABLE_MASK);
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
			rxctl_reg |= (RNP10_RX_FLOW_ENABLE_MASK);
			break;
		case rnp_fc_tx_pause:
			/*
			 * Tx Flow control is enabled, and Rx Flow control is
			 * disabled by software override.
			 */
			for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++)
				txctl_reg[i] |= (RNP10_TX_FLOW_ENABLE_MASK);
			break;
		case rnp_fc_full:
			/* Flow control (both Rx and Tx) is enabled by SW override. */
			rxctl_reg |= (RNP10_RX_FLOW_ENABLE_MASK);
			for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++)
				txctl_reg[i] |= (RNP10_TX_FLOW_ENABLE_MASK);
			break;
		default:
			hw_dbg(hw, "Flow control param set incorrectly\n");
			ret_val = RNP_ERR_CONFIG;
			goto out;
	}
	/* Set up and enable Rx high/low water mark thresholds, enable XON. */
	/* setup eth water not so good  */
	/*
	for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++) {
		if ((hw->fc.current_mode & rnp_fc_tx_pause)) {
			if (hw->fc.high_water[i]) {
				wr32(hw, RNP_ETH_HIGH_WATER(i), hw->fc.high_water[i]);
			}
			if (hw->fc.low_water[i]) {
				wr32(hw, RNP_ETH_LOW_WATER(i), hw->fc.low_water[i]);
			}
		}
	} */

	/* Configure pause time (2 TCs per register) */
	reg = hw->fc.pause_time;
	for (i = 0; i < (RNP_MAX_TRAFFIC_CLASS); i++)
		txctl_reg[i] |= (reg << 16);

	/* Set 802.3x based flow control settings. */
	mac_wr32(mac, RNP10_MAC_RX_FLOW_CTRL, rxctl_reg);
	for (i = 0; i < (RNP_MAX_TRAFFIC_CLASS); i++)
		mac_wr32(mac, RNP10_MAC_Q0_TX_FLOW_CTRL(i), txctl_reg[i]);
out:
	return ret_val;
}

void rnp_mac_set_mac_n10(struct rnp_mac_info *mac, u8 *addr, int index)
{
	u32 rar_low, rar_high = 0;
	rar_low = ((u32)addr[0] |
			((u32)addr[1] << 8) |
			((u32)addr[2] << 16) |
			((u32)addr[3] << 24));

	rar_high = RNP_RAH_AV | ((u32)addr[4] | (u32)addr[5] << 8);

	mac_wr32(mac, RNP10_MAC_UNICAST_HIGH(index), rar_high);
	mac_wr32(mac, RNP10_MAC_UNICAST_LOW(index), rar_low);



}


static struct rnp_mac_operations mac_ops_n10 = {
	.set_mac_rx = &rnp_mac_set_rx_n10,
	.set_mac_fcs = &rnp_mac_fcs_n10,
	.set_fc_mode = &rnp_mac_fc_mode_n10,
	.set_mac = &rnp_mac_set_mac_n10,


//	.init_hw = &rnp_init_hw_generic,
//	.reset_hw = &rnp_reset_hw_n10,
//
//	.start_hw = &rnp_start_hw_n10,
//	.clear_hw_cntrs = &rnp_clear_hw_cntrs_generic,
//	.get_media_type = &rnp_get_media_type_n10,
//	.get_supported_physical_layer = &rnp_get_supported_physical_layer_n10,
//	.enable_rx_dma = &rnp_enable_rx_dma_n10,
//	.disable_rx_buff = &rnp_disable_rx_buff_generic,
//	.enable_rx_buff = &rnp_enable_rx_buff_generic,
//	.get_mac_addr = &rnp_get_mac_addr_generic,
//	.get_device_caps = &rnp_get_device_caps_generic,
//	//.setup_link             = &ixgbe_setup_mac_link_82599,
//	.get_wwn_prefix = &rnp_get_wwn_prefix_generic,
//	.stop_adapter = &rnp_stop_adapter_generic,
//	//.set_rxpba		        = &rnp_set_rxpba_generic,
//	.check_link = &rnp_check_mac_link_generic,
//	.get_link_capabilities = &rnp_get_link_capabilities_n10,
//	.led_on = &rnp_led_on_generic,
//	.led_off = &rnp_led_off_generic,
//	.blink_led_start = &rnp_blink_led_start_generic,
//	.blink_led_stop = &rnp_blink_led_stop_generic,
//	//.get_bus_info           = &rnp_get_bus_info_generic,
//	.set_rar = &rnp_set_rar_generic,
//	.clear_rar = &rnp_clear_rar_generic,
//	.set_vmdq = &rnp_set_vmdq_generic,
//	//.set_vmdq_san_mac	= &rnp_set_vmdq_san_mac_generic,
//	.clear_vmdq = &rnp_clear_vmdq_generic,
//	.init_rx_addrs = &rnp_init_rx_addrs_generic,
//	.update_mc_addr_list = &rnp_update_mc_addr_list_generic,
//	.enable_mc = &rnp_enable_mc_generic,
//	.disable_mc = &rnp_disable_mc_generic,
//	.clear_vfta = &rnp_clear_vfta_generic,
//	.set_vfta = &rnp_set_vfta_generic,
//	.fc_enable = &rnp_fc_enable_generic,
//	.set_fw_drv_ver = &rnp_set_fw_drv_ver_generic,
//	.init_uta_tables = &rnp_init_uta_tables_generic,
//	.setup_sfp = &rnp_setup_sfp_modules_n10,
//	.mng_fw_enabled = &rnp_mng_enabled,
};

static s32 rnp_get_invariants_n10(struct rnp_hw *hw)
{
	struct rnp_mac_info *mac = &hw->mac;
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_eth_info *eth = &hw->eth;

	/* setup dma info */
	dma->dma_base_addr = hw->hw_addr;
	dma->dma_ring_addr = hw->hw_addr + RNP10_RING_BASE;
	dma->max_tx_queues = RNP_N10_MAX_TX_QUEUES;
	dma->max_rx_queues = RNP_N10_MAX_RX_QUEUES;
	dma->back = hw;
	memcpy(&hw->dma.ops, &dma_ops_n10, sizeof(hw->dma.ops));

	/* setup eth info */
	memcpy(&hw->eth.ops, &eth_ops_n10, sizeof(hw->eth.ops));

	eth->eth_base_addr = hw->hw_addr + RNP10_ETH_BASE;
	printk(" eth_base is %p\n", eth->eth_base_addr);
	eth->back = hw;
	eth->mc_filter_type = 0;
	eth->mcft_size = RNP_N10_MC_TBL_SIZE;
	eth->vft_size = RNP_N10_VFT_TBL_SIZE;
	eth->num_rar_entries = RNP_N10_RAR_ENTRIES;
	eth->max_rx_queues = RNP_N10_MAX_RX_QUEUES;
	eth->max_tx_queues = RNP_N10_MAX_TX_QUEUES;

	/* setup mac info */
	memcpy(&hw->mac.ops, &mac_ops_n10, sizeof(hw->mac.ops));
	mac->mac_addr = hw->hw_addr + RNP10_MAC_BASE;
	mac->back = hw;
	mac->mac_type = mac_dwc_xlg;
	/* move this to eth todo */
	mac->mc_filter_type = 0;
	mac->mcft_size = RNP_N10_MC_TBL_SIZE;
	mac->vft_size = RNP_N10_VFT_TBL_SIZE;
	mac->num_rar_entries = RNP_N10_RAR_ENTRIES;
	mac->max_rx_queues = RNP_N10_MAX_RX_QUEUES;
	mac->max_tx_queues = RNP_N10_MAX_TX_QUEUES;
	//mac->max_msix_vectors = rnp_get_pcie_msix_count_generic(hw);
	mac->max_msix_vectors = RNP_N10_MSIX_VECTORS;
	if (!hw->axi_mhz)
		hw->usecstocount = 500;
	else 
		hw->usecstocount = hw->axi_mhz;

	//set up hw feature
	hw->feature_flags |= RNP_NET_FEATURE_SG
			  | RNP_NET_FEATURE_TX_CHECKSUM
			  | RNP_NET_FEATURE_RX_CHECKSUM
			  | RNP_NET_FEATURE_TSO
			  | RNP_NET_FEATURE_TX_UDP_TUNNEL
			  | RNP_NET_FEATURE_VLAN_FILTER
			  | RNP_NET_FEATURE_VLAN_OFFLOAD
			  | RNP_NET_FEATURE_RX_NTUPLE_FILTER
			  | RNP_NET_FEATURE_TCAM
			  | RNP_NET_FEATURE_RX_HASH
			  | RNP_NET_FEATURE_RX_FCS;
			  /* maybe supported future*/
			 // | RNP_NET_FEATURE_HW_TC
	/* setup some fdir resource */
	hw->min_length = RNP_MIN_MTU;
	hw->max_length = RNP_MAX_JUMBO_FRAME_SIZE;
	hw->max_msix_vectors = RNP_N10_MSIX_VECTORS;
	hw->num_rar_entries = RNP_N10_RAR_ENTRIES;
	//hw->fdir_mode = fdir_mode_tcam; 
	hw->fdir_mode = fdir_mode_tuple5; 
	hw->max_vfs = RNP_N10_MAX_VF;
	hw->sriov_ring_limit = 2;
	if (hw->fdir_mode == fdir_mode_tcam) {
		hw->layer2_count = RNP10_MAX_LAYER2_FILTERS - 1;
		hw->tuple5_count = RNP10_MAX_TCAM_FILTERS - 1;
	} else {
		hw->layer2_count = RNP10_MAX_LAYER2_FILTERS - 1;
		hw->tuple5_count = RNP10_MAX_TUPLE5_FILTERS - 1;
	}

	hw->default_rx_queue = 0;
	// todo 
	hw->rss_indir_tbl_num = RNP_N10_RSS_TBL_NUM;
	hw->rss_tc_tbl_num = RNP_N10_RSS_TC_TBL_NUM;
	/* vf use the last vfnum */
	hw->vfnum = RNP_N10_MAX_VF - 1;

	memcpy(&hw->ops, &hw_ops_n10, sizeof(hw->ops));
	//sriov num

	/* PHY */
	memcpy(&hw->phy.ops, &phy_ops_n10, sizeof(hw->phy.ops));
	/* setup pcs */
	memcpy(&hw->pcs.ops, &pcs_ops_generic, sizeof(hw->pcs.ops));

	return 0;
}

struct rnp_info rnp_n10_2ports_info = {
	.one_pf_with_two_dma = false,
	.total_queue_pair_cnts = RNP_N10_MAX_TX_QUEUES,
	.adapter_cnt = 1,
	.rss_type = rnp_rss_n10,
	.hw_type = rnp_hw_n10,
	.get_invariants = &rnp_get_invariants_n10,
	.mac_ops = &mac_ops_n10,
	.eeprom_ops = NULL,
	.phy_ops = &phy_ops_n10,
	.mbx_ops = &mbx_ops_generic,
	.pcs_ops = &pcs_ops_generic, 
};

static s32 rnp_get_invariants_n400(struct rnp_hw *hw)
{
	struct rnp_mac_info *mac = &hw->mac;
	struct rnp_dma_info *dma = &hw->dma;
	struct rnp_eth_info *eth = &hw->eth;

	/* setup dma info */
	dma->dma_base_addr = hw->hw_addr;
	dma->dma_ring_addr = hw->hw_addr + RNP10_RING_BASE;
	dma->max_tx_queues = RNP_N400_MAX_TX_QUEUES;
	dma->max_rx_queues = RNP_N400_MAX_RX_QUEUES;
	dma->back = hw;
	memcpy(&hw->dma.ops, &dma_ops_n10, sizeof(hw->dma.ops));

	/* setup eth info */
	memcpy(&hw->eth.ops, &eth_ops_n10, sizeof(hw->eth.ops));

	eth->eth_base_addr = hw->hw_addr + RNP10_ETH_BASE;
	printk(" eth_base is %p\n", eth->eth_base_addr);
	eth->back = hw;
	eth->mc_filter_type = 0;
	eth->mcft_size = RNP_N10_MC_TBL_SIZE;
	eth->vft_size = RNP_N10_VFT_TBL_SIZE;
	eth->num_rar_entries = RNP_N10_RAR_ENTRIES;
	eth->max_rx_queues = RNP_N400_MAX_RX_QUEUES;
	eth->max_tx_queues = RNP_N400_MAX_TX_QUEUES;

	/* setup mac info */
	memcpy(&hw->mac.ops, &mac_ops_n10, sizeof(hw->mac.ops));
	mac->mac_addr = hw->hw_addr + RNP10_MAC_BASE;
	mac->back = hw;
	mac->mac_type = mac_dwc_xlg;
	/* move this to eth todo */
	mac->mc_filter_type = 0;
	mac->mcft_size = RNP_N10_MC_TBL_SIZE;
	mac->vft_size = RNP_N10_VFT_TBL_SIZE;
	mac->num_rar_entries = RNP_N10_RAR_ENTRIES;
	mac->max_rx_queues = RNP_N400_MAX_RX_QUEUES;
	mac->max_tx_queues = RNP_N400_MAX_TX_QUEUES;
	mac->max_msix_vectors = RNP_N400_MSIX_VECTORS;
	if (!hw->axi_mhz)
		hw->usecstocount = 500;
	else 
		hw->usecstocount = hw->axi_mhz;

	//set up hw feature
	hw->feature_flags |= RNP_NET_FEATURE_SG
			  | RNP_NET_FEATURE_TX_CHECKSUM
			  | RNP_NET_FEATURE_RX_CHECKSUM
			  | RNP_NET_FEATURE_TSO
			  | RNP_NET_FEATURE_TX_UDP_TUNNEL
			  | RNP_NET_FEATURE_VLAN_FILTER
			  | RNP_NET_FEATURE_VLAN_OFFLOAD
			  | RNP_NET_FEATURE_RX_NTUPLE_FILTER
			  | RNP_NET_FEATURE_TCAM
			  | RNP_NET_FEATURE_RX_HASH
			  | RNP_NET_FEATURE_RX_FCS;
			  /* maybe supported future*/
			 // | RNP_NET_FEATURE_HW_TC
	/* setup some fdir resource */
	hw->min_length = RNP_MIN_MTU;
	hw->max_length = RNP_MAX_JUMBO_FRAME_SIZE;
	hw->max_msix_vectors = RNP_N400_MSIX_VECTORS;
	hw->num_rar_entries = RNP_N10_RAR_ENTRIES;
	//hw->fdir_mode = fdir_mode_tcam; 
	hw->fdir_mode = fdir_mode_tuple5; 
	hw->max_vfs = RNP_N400_MAX_VF;
	hw->sriov_ring_limit = 1;
	if (hw->fdir_mode == fdir_mode_tcam) {
		hw->layer2_count = RNP10_MAX_LAYER2_FILTERS - 1;
		hw->tuple5_count = RNP10_MAX_TCAM_FILTERS - 1;
	} else {
		hw->layer2_count = RNP10_MAX_LAYER2_FILTERS - 1;
		hw->tuple5_count = RNP10_MAX_TUPLE5_FILTERS - 1;
	}

	hw->default_rx_queue = 0;
	// todo 
	hw->rss_indir_tbl_num = RNP_N10_RSS_TBL_NUM;
	hw->rss_tc_tbl_num = RNP_N10_RSS_TC_TBL_NUM;
	/* vf use the last vfnum */
	hw->vfnum = RNP_N400_MAX_VF - 1;

	memcpy(&hw->ops, &hw_ops_n10, sizeof(hw->ops));
	//sriov num

	/* PHY */
	memcpy(&hw->phy.ops, &phy_ops_n10, sizeof(hw->phy.ops));
	/* setup pcs */
	memcpy(&hw->pcs.ops, &pcs_ops_generic, sizeof(hw->pcs.ops));

	return 0;
}

struct rnp_info rnp_n400_2ports_info = {
	.one_pf_with_two_dma = false,
	.total_queue_pair_cnts = RNP_N400_MAX_TX_QUEUES,
	.adapter_cnt = 1,
	.rss_type = rnp_rss_n10,
	.hw_type = rnp_hw_n10,
	.get_invariants = &rnp_get_invariants_n400,
	.mac_ops = &mac_ops_n10,
	.eeprom_ops = NULL,
	.phy_ops = &phy_ops_n10,
	.mbx_ops = &mbx_ops_generic,
	.pcs_ops = &pcs_ops_generic, 
};
