#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include "rnp.h"
#include "rnp_phy.h"
#include "rnp_mbx.h"
#include "rnp_pcs.h"

#define RNP_N20_MAX_TX_QUEUES 128
#define RNP_N20_MAX_RX_QUEUES 128
#define RNP_N20_RAR_ENTRIES 128
#define RNP_N20_MC_TBL_SIZE 128
#define RNP_N20_VFT_TBL_SIZE 128
#define RNP_N20_RX_PB_SIZE 512
#define RNP_N20_MSIX_VECTORS 64

static bool rnp_mng_enabled(struct rnp_hw *hw)
{
	return false;
}

static void rnp_init_mac_link_ops_n20(struct rnp_hw *hw)
{
}

static s32 rnp_get_invariants_n20(struct rnp_hw *hw)
{
	struct rnp_mac_info *mac = &hw->mac;
	struct rnp_dma_info *dma = &hw->dma;

	/* setup dma info */
	dma->dma_base_addr = hw->hw_addr;
	dma->dma_ring_addr = hw->hw_addr + RNP20_RING_BASE;
	dma->max_tx_queues = RNP_N20_MAX_TX_QUEUES;
	dma->max_rx_queues = RNP_N20_MAX_RX_QUEUES;

	hw->usecstocount = 500;
	//rnp_init_mac_link_ops_n20(hw);

	mac->mc_filter_type = 0;
	mac->mcft_size = RNP_N20_MC_TBL_SIZE;
	mac->vft_size = RNP_N20_VFT_TBL_SIZE;
	mac->num_rar_entries = RNP_N20_RAR_ENTRIES;
	mac->max_rx_queues = RNP_N20_MAX_RX_QUEUES;
	mac->max_tx_queues = RNP_N20_MAX_TX_QUEUES;
	//mac->max_msix_vectors = rnp_get_pcie_msix_count_generic(hw);
	mac->max_msix_vectors = RNP_N20_MSIX_VECTORS;

	//set up hw feature
	hw->feature_flags |= RNP_NET_FEATURE_SG
			  //| RNP_NET_FEATURE_TX_CHECKSUM
			  //| RNP_NET_FEATURE_RX_CHECKSUM
			  //| RNP_NET_FEATURE_TSO
			  //| RNP_NET_FEATURE_TX_UDP_TUNNEL
			  //| RNP_NET_FEATURE_VLAN_FILTER
			  //| RNP_NET_FEATURE_VLAN_OFFLOAD
			  //| RNP_NET_FEATURE_RX_NTUPLE_FILTER
			  | RNP_NET_FEATURE_TCAM
			  //| RNP_NET_FEATURE_RX_HASH
			  //| RNP_NET_FEATURE_RX_FCS;
			  ;

	return 0;
}

/**
 *  rnp_init_phy_ops_n20 - PHY/SFP specific init
 *  @hw: pointer to hardware structure
 *
 *  Initialize any function pointers that were not able to be
 *  set during get_invariants because the PHY/SFP type was
 *  not known.  Perform the SFP init if necessary.
 *
 **/
static s32 rnp_init_phy_ops_n20(struct rnp_hw *hw)
{
	struct rnp_mac_info *mac = &hw->mac;
	struct rnp_phy_info *phy = &hw->phy;
	s32 ret_val = 0;

	hw->phy.sfp_setup_needed = true;

	return ret_val;
}

static s32 rnp_setup_sfp_modules_n20(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_reinit_fdir_tables_n20 - Reinitialize Flow Director tables.
 *  @hw: pointer to hardware structure
 **/
s32 rnp_reinit_fdir_tables_n20(struct rnp_hw *hw)
{
	return 0;
}

/**
 *  rnp_fdir_enable_n20 - Initialize Flow Director control registers
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register
 **/
static void rnp_fdir_enable_n20(struct rnp_hw *hw, u32 fdirctrl)
{
}

/**
 *  rnp_init_fdir_signature_n20 - Initialize Flow Director signature filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *             contains just the value of the Rx packet buffer allocation
 **/
s32 rnp_init_fdir_signature_n20(struct rnp_hw *hw, u32 fdirctrl)
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
	rnp_fdir_enable_n20(hw, fdirctrl);
#endif

	return 0;
}

/**
 *  rnp_init_fdir_perfect_n20 - Initialize Flow Director perfect filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *             contains just the value of the Rx packet buffer allocation
 **/
s32 rnp_init_fdir_perfect_n20(struct rnp_hw *hw, u32 fdirctrl)
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
	rnp_fdir_enable_n20(hw, fdirctrl);

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
 *  rnp_atr_compute_sig_hash_n20 - Compute the signature hash
 *  @stream: input bitstream to compute the hash on
 *
 *  This function is almost identical to the function above but contains
 *  several optomizations such as unwinding all of the loops, letting the
 *  compiler work out all of the conditional ifs since the keys are static
 *  defines, and computing two keys at once since the hashed dword stream
 *  will be the same for both keys.
 **/
static u32 rnp_atr_compute_sig_hash_n20(union rnp_atr_hash_dword input,
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
 *  rnp_atr_add_signature_filter_n20 - Adds a signature hash filter
 *  @hw: pointer to hardware structure
 *  @input: unique input dword
 *  @common: compressed common input dword
 *  @queue: queue index to direct traffic to
 **/
s32 rnp_fdir_add_signature_filter_n20(struct rnp_hw *hw,
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
	fdirhashcmd |= rnp_atr_compute_sig_hash_n20(input, common);
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
 *  rnp_atr_compute_perfect_hash_n20 - Compute the perfect filter hash
 *  @atr_input: input bitstream to compute the hash on
 *  @input_mask: mask for the input bitstream
 *
 *  This function serves two main purposes.  First it applys the input_mask
 *  to the atr_input resulting in a cleaned up atr_input data stream.
 *  Secondly it computes the hash and stores it in the bkt_hash field at
 *  the end of the input byte stream.  This way it will be available for
 *  future use without needing to recompute the hash.
 **/
void rnp_atr_compute_perfect_hash_n20(union rnp_atr_input *input,
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
 *  rnp_get_fdirtcpm_n20 - generate a tcp port from atr_input_masks
 *  @input_mask: mask to be bit swapped
 *
 *  The source and destination port masks for flow director are bit swapped
 *  in that bit 15 effects bit 0, 14 effects 1, 13, 2 etc.  In order to
 *  generate a correctly swapped value we need to bit swap the mask and that
 *  is what is accomplished by this function.
 **/
static u32 rnp_get_fdirtcpm_n20(union rnp_atr_input *input_mask)
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

s32 rnp_fdir_set_input_mask_n20(struct rnp_hw *hw,
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
	fdirtcpm = rnp_get_fdirtcpm_n20(input_mask);

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

s32 rnp_fdir_write_perfect_filter_n20(struct rnp_hw *hw,
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

s32 rnp_fdir_erase_perfect_filter_n20(struct rnp_hw *hw,
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
 *  rnp_identify_phy_n20 - Get physical layer module
 *  @hw: pointer to hardware structure
 *
 *  Determines the physical layer module found on the current adapter.
 *  If PHY already detected, maintains current PHY type in hw struct,
 *  otherwise executes the PHY detection routine.
 **/
static s32 rnp_identify_phy_n20(struct rnp_hw *hw)
{
	s32 status = RNP_ERR_PHY_ADDR_INVALID;

	hw->phy.type = rnp_phy_sfp;

	return 0;
}

static s32 rnp_identify_sfp_module_n20(struct rnp_hw *hw)
{
	hw->phy.sfp_type = rnp_sfp_type_da_cu;

	return 0;
}

/**
 *  rnp_enable_rx_dma_n20 - Enable the Rx DMA unit on n20
 *  @hw: pointer to hardware structure
 *  @regval: register value to write to RXCTRL
 *
 *  Enables the Rx DMA unit for n20
 **/
static s32 rnp_enable_rx_dma_n20(struct rnp_hw *hw, u32 regval)
{
	/*
	 * Workaround for n20 silicon errata when enabling the Rx datapath.
	 * If traffic is incoming before we enable the Rx unit, it could hang
	 * the Rx DMA unit.  Therefore, make sure the security engine is
	 * completely disabled prior to enabling the Rx unit.
	 */
	hw->mac.ops.disable_rx_buff(hw);

	//RNP_WRITE_REG(hw, RNP_RXCTRL, regval);

	hw->mac.ops.enable_rx_buff(hw);

	return 0;
}

/**
 *  rnp_verify_fw_version_n20 - verify fw version for n20
 *  @hw: pointer to hardware structure
 *
 *  Verifies that installed the firmware version is 0.6 or higher
 *  for SFI devices. All n20 SFI devices should have version 0.6 or higher.
 *
 *  Returns RNP_ERR_EEPROM_VERSION if the FW is not present or
 *  if the FW version is not supported.
 **/
static s32 rnp_verify_fw_version_n20(struct rnp_hw *hw)
{
	s32 status = RNP_ERR_EEPROM_VERSION;
	u16 fw_offset, fw_ptp_cfg_offset;
	u16 fw_version = 0;

	return 0;
}

/**
 *  rnp_verify_lesm_fw_enabled_n20 - Checks LESM FW module state.
 *  @hw: pointer to hardware structure
 *
 *  Returns true if the LESM FW module is present and enabled. Otherwise
 *  returns false. Smart Speed must be disabled if LESM FW module is enabled.
 **/
bool rnp_verify_lesm_fw_enabled_n20(struct rnp_hw *hw)
{
	bool lesm_enabled = false;

	return lesm_enabled;
}

/**
 *  rnp_read_eeprom_buffer_n20 - Read EEPROM word(s) using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Retrieves 16 bit word(s) read from EEPROM
 **/
static s32 rnp_read_eeprom_buffer_n20(struct rnp_hw *hw, u16 offset,
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
 *  rnp_read_eeprom_n20 - Read EEPROM word using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM
 **/
static s32 rnp_read_eeprom_n20(struct rnp_hw *hw, u16 offset, u16 *data)
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
 * rnp_reset_pipeline_n20 - perform pipeline reset
 *
 * @hw: pointer to hardware structure
 *
 * Reset pipeline by asserting Restart_AN together with LMS change to ensure
 * full pipeline reset.  Note - We must hold the SW/FW semaphore before writing
 * to AUTOC, so this function assumes the semaphore is held.
 **/
s32 rnp_reset_pipeline_n20(struct rnp_hw *hw)
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
 *  rnp_reset_hw_n20 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
static s32 rnp_reset_hw_n20(struct rnp_hw *hw)
{
	int i;
	s32 status;
	u32 reg = 0;
	int timeout = 0;
	struct rnp_dma_info *dma = &hw->dma;

	/* Call adapter stop to disable tx/rx and clear interrupts */
	wr32(hw, RNP_DMA_AXI_EN, 0);

	//nic_reset

#ifdef NO_MBX_VERSION
	wr32(hw, RNP_TOP_NIC_REST_N, NIC_RESET);
	/*
	 * we need this
	 */
	wmb();
	wr32(hw, RNP_TOP_NIC_REST_N, ~NIC_RESET);
/*
 
#define TSRN20_REG_DEBUG_VALUE          (0x1a2b3c4d)

	wr32(hw, RNP_DMA_DUMY, TSRN20_REG_DEBUG_VALUE);

	while(!((reg = rd32(hw, RNP_DMA_DUMY)) == TSRN20_REG_DEBUG_VALUE + 1)) {
		usleep_range(100, 200);
		timeout++;

		if (timeout > 10000) {
			printk("wait reset timeout\n");
			break;
		}
		//printk("wait reset");
	}
*/
#else

	rnp_mbx_fw_reset_phy(hw);

#endif
	/* should set all tx-start to 1 */
	for (i = 0; i < 8; i++)
		dma_ring_wr32(dma, RING_OFFSET(i) + RNP_DMA_TX_START, 1);

	for (i = 0; i < 10; i++)
		msleep(100);



	/* Identify PHY and related function pointers */
	status = hw->phy.ops.init(hw);

	/* Setup SFP module if there is one present. */
	if (hw->phy.sfp_setup_needed) {
		status = hw->mac.ops.setup_sfp(hw);
		hw->phy.sfp_setup_needed = false;
	}

	/* Reset PHY */
	if (hw->phy.reset_disable == false && hw->phy.ops.reset != NULL)
		hw->phy.ops.reset(hw);

	/*todo earase tcm */
	wr32(hw, RNP_ETH_TCAM_EN, 1);
	wr32(hw, RNP_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
	wr32(hw, RNP_TCAM_MODE, 2);
	/* dont't open tcam cache */
	wr32(hw, RNP_TCAM_CACHE_ENABLE, 0);

	for (i = 0; i < 4096; i++) {
		wr32(hw, RNP_TCAM_SDPQF(i), 0);
		wr32(hw, RNP_TCAM_DAQF(i), 0);
		wr32(hw, RNP_TCAM_SAQF(i), 0);
		wr32(hw, RNP_TCAM_APQF(i), 0);

		wr32(hw, RNP_TCAM_SDPQF_MASK(i), 0);
		wr32(hw, RNP_TCAM_DAQF_MASK(i), 0);
		wr32(hw, RNP_TCAM_SAQF_MASK(i), 0);
		wr32(hw, RNP_TCAM_APQF_MASK(i), 0);
	}
	wr32(hw, RNP_TCAM_MODE, 1);

	/* Store the permanent mac address */
	if (!(hw->mac.mac_flags & RNP_FLAGS_INIT_MAC_ADDRESS)) {
		rnp_get_permtion_mac_addr(hw, hw->mac.perm_addr);
		memcpy(hw->mac.addr, hw->mac.perm_addr, ETH_ALEN);
	}

	hw->mac.num_rar_entries = RNP_N20_RAR_ENTRIES;
	hw->mac.ops.init_rx_addrs(hw);

	/* open vxlan default */
#define VXLAN_HW_ENABLE (1)
	wr32(hw, RNP_ETH_TUNNEL_MOD, VXLAN_HW_ENABLE);
	/*=====  mac steup ===*/
	/* open mac at last */
	// dwc_xlgmac_databook.pdf
	for (i = 0; i < 1; i++) {
		//wr32(hw, RNP_MAC_TX_CFG, 0x40010001);
		wr32(hw, RNP_MAC_RX_CFG, rd32(hw, RNP_MAC_RX_CFG) | 0x01); 
		//wr32(hw, RNP_MAC_RX_CFG, 0x07d001c7);
		wr32(hw, RNP_MAC_PKT_FLT, 0x80000000);
		wr32(hw, RNP_MAC_LPI_CTRL, 0x00060000);
	}
reset_hw_out:
	rnp_reset_msix_table_generic(hw);

	return 0;
}

/**
 *  rnp_start_hw_n20 - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function
 *  and the generation start_hw function.
 *  Then performs revision-specific operations, if any.
 **/
static s32 rnp_start_hw_n20(struct rnp_hw *hw)
{
	s32 ret_val = 0;
	int i;

	ret_val = rnp_start_hw_generic(hw);
	if (ret_val != 0)
		goto out;

	ret_val = rnp_start_hw_gen2(hw);
	if (ret_val != 0)
		goto out;

	// ETH Registers
	//wr32(hw, RNP_ETH_ERR_MASK_VECTOR,~ETH_IGNORE_ALL_ERR);
	wr32(hw, RNP_ETH_ERR_MASK_VECTOR, 0);
	wr32(hw, RNP_ETH_BYPASS, 0);
	wr32(hw, RNP_ETH_DEFAULT_RX_RING, 0);

	wr32(hw, RNP_TOP_NIC_CONFIG,
	     hw->mode
#ifdef CONFIG_RNP_FPGA
		     | hw->default_rx_queue << 24
#endif
	);

	// DMA common Registers
	wr32(hw, RNP_DMA_CONFIG, DMA_VEB_BYPASS);

	// enable-dma-axi
	wr32(hw, RNP_DMA_AXI_EN, (RX_AXI_RW_EN | TX_AXI_RW_EN));

	if (ret_val == 0)
		ret_val = rnp_verify_fw_version_n20(hw);
out:
	return ret_val;
}

/**
 *  rnp_get_media_type_n20 - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber, copper, backplane)
 **/
static enum rnp_media_type rnp_get_media_type_n20(struct rnp_hw *hw)
{
	enum rnp_media_type media_type = rnp_media_type_fiber;
	return media_type;
}

/**
 *  rnp_get_supported_physical_layer_n20 - Returns physical layer type
 *  @hw: pointer to hardware structure
 *
 *  Determines physical layer capabilities of the current configuration.
 **/
static u32 rnp_get_supported_physical_layer_n20(struct rnp_hw *hw)
{
	u32 physical_layer = 0;
	return physical_layer;
}

static s32 rnp_get_link_capabilities_n20(struct rnp_hw *hw,
					   rnp_link_speed *speed, bool *autoneg)
{
	/* fix setup */
	/* reletive with firmware */
	*speed = RNP_LINK_SPEED_10GB_FULL;
	*autoneg = false;

	return 0;
}

static struct rnp_phy_operations phy_ops_n20 = {
	.identify = &rnp_identify_phy_n20,
	.identify_sfp = &rnp_identify_sfp_module_n20,
	.init = &rnp_init_phy_ops_n20,
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

static struct rnp_mac_operations mac_ops_n20 = {
	.init_hw = &rnp_init_hw_generic,
	.reset_hw = &rnp_reset_hw_n20,
	.start_hw = &rnp_start_hw_n20,
	.clear_hw_cntrs = &rnp_clear_hw_cntrs_generic,
	.get_media_type = &rnp_get_media_type_n20,
	.get_supported_physical_layer = &rnp_get_supported_physical_layer_n20,
	.enable_rx_dma = &rnp_enable_rx_dma_n20,
	.disable_rx_buff = &rnp_disable_rx_buff_generic,
	.enable_rx_buff = &rnp_enable_rx_buff_generic,
	.get_mac_addr = &rnp_get_mac_addr_generic,
	.get_device_caps = &rnp_get_device_caps_generic,
	//.setup_link             = &ixgbe_setup_mac_link_82599,
	.get_wwn_prefix = &rnp_get_wwn_prefix_generic,
	.stop_adapter = &rnp_stop_adapter_generic,
	//.set_rxpba		        = &rnp_set_rxpba_generic,
	.check_link = &rnp_check_mac_link_generic,
	.get_link_capabilities = &rnp_get_link_capabilities_n20,
	.led_on = &rnp_led_on_generic,
	.led_off = &rnp_led_off_generic,
	.blink_led_start = &rnp_blink_led_start_generic,
	.blink_led_stop = &rnp_blink_led_stop_generic,
	//.get_bus_info           = &rnp_get_bus_info_generic,
	.set_rar = &rnp_set_rar_generic,
	.clear_rar = &rnp_clear_rar_generic,
	.set_vmdq = &rnp_set_vmdq_generic,
	//.set_vmdq_san_mac	= &rnp_set_vmdq_san_mac_generic,
	.clear_vmdq = &rnp_clear_vmdq_generic,
	.init_rx_addrs = &rnp_init_rx_addrs_generic,
	.update_mc_addr_list = &rnp_update_mc_addr_list_generic,
	.enable_mc = &rnp_enable_mc_generic,
	.disable_mc = &rnp_disable_mc_generic,
	.clear_vfta = &rnp_clear_vfta_generic,
	.set_vfta = &rnp_set_vfta_generic,
	.fc_enable = &rnp_fc_enable_generic,
	.set_fw_drv_ver = &rnp_set_fw_drv_ver_generic,
	.init_uta_tables = &rnp_init_uta_tables_generic,
	.setup_sfp = &rnp_setup_sfp_modules_n20,
	.mng_fw_enabled = &rnp_mng_enabled,
};

struct rnp_info rnp_n20_2ports_info = {
	.one_pf_with_two_dma = false,
	.total_queue_pair_cnts = RNP_N20_MAX_TX_QUEUES,
	.adapter_cnt = 1,
	.rss_type = rnp_rss_n20,
	.hw_type = rnp_hw_n20,
	.get_invariants = &rnp_get_invariants_n20,
	.mac_ops = &mac_ops_n20,
	.eeprom_ops = NULL,
	.phy_ops = &phy_ops_n20,
	.mbx_ops = &mbx_ops_generic,
	.pcs_ops = &pcs_ops_generic, 
};
