#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include "rnpm.h"
#include "rnpm_phy.h"
#include "rnpm_mbx.h"

#define RNPM_vu440_MAX_TX_QUEUES 128
#define RNPM_vu440_MAX_RX_QUEUES 128
#define RNPM_vu440_RAR_ENTRIES   128
#define RNPM_vu440_MC_TBL_SIZE   128
#define RNPM_vu440_MC_TBL_SIZE_MAC 8
#define RNPM_vu440_VFT_TBL_SIZE  128
#define RNPM_vu440_VFT_TBL_SIZE_MAC  1
#define RNPM_vu440_RX_PB_SIZE	512
#define RNPM_vu440_MAX_MSIX_COUNT 16
#define RNPM_vu440_DEFAULT_TXD  1024

#define  NET_FEATURE_TCAM 1
static bool rnpm_mng_enabled(struct rnpm_hw *hw)
{
	return false;
}

__maybe_unused static void rnpm_init_mac_link_ops_vu440(struct rnpm_hw *hw)
{
	return;
}

static s32 rnpm_get_invariants_vu440(struct rnpm_hw *hw)
{
	struct rnpm_mac_info *mac = &hw->mac;

	//rnpm_init_mac_link_ops_vu440(hw);
	// mode is setup here 
	
	hw->usecstocount = 62;

// fixme should use adapter->mode
#if defined(MODE_4_PORT) || defined(MODE_2_PORT)	
	/* now use mac mc filter table in muti ports */
	mac->mc_filter_type = rnpm_mc_filter_type4;
	mac->mc_location = rnpm_mc_location_mac;
	mac->mcft_size = RNPM_vu440_MC_TBL_SIZE_MAC;
	mac->vlan_location = rnpm_vlan_location_mac;
	mac->vft_size = RNPM_vu440_VFT_TBL_SIZE_MAC;
#else
	mac->mc_location = rnpm_mc_location_nic;
	mac->mcft_size = RNPM_vu440_MC_TBL_SIZE;
	mac->mc_filter_type = rnpm_mc_filter_type0;
	mac->vlan_location = rnpm_vlan_location_nic;
	mac->vft_size = RNPM_vu440_VFT_TBL_SIZE;
#endif

	mac->num_rar_entries = RNPM_vu440_RAR_ENTRIES;
	mac->max_rx_queues = RNPM_vu440_MAX_RX_QUEUES;
	mac->max_tx_queues = RNPM_vu440_MAX_TX_QUEUES;
	//mac->max_msix_vectors = rnpm_get_pcie_msix_count_generic(hw);
	mac->max_msix_vectors = RNPM_vu440_MAX_MSIX_COUNT;
	//mac->max_msix_vectors = 8;
        hw->feature_flags |= RNPM_NET_FEATURE_SG
                          | RNPM_NET_FEATURE_TX_CHECKSUM
                          | RNPM_NET_FEATURE_RX_CHECKSUM
                          | RNPM_NET_FEATURE_TSO
                          | RNPM_NET_FEATURE_TX_UDP_TUNNEL
                          | RNPM_NET_FEATURE_VLAN_FILTER
                          | RNPM_NET_FEATURE_VLAN_OFFLOAD
                          | RNPM_NET_FEATURE_TCAM
                          | RNPM_NET_FEATURE_RX_HASH
                          | RNPM_NET_FEATURE_RX_FCS;

	return 0;
}

/**
 *  rnpm_init_phy_ops_vu440 - PHY/SFP specific init
 *  @hw: pointer to hardware structure
 *
 *  Initialize any function pointers that were not able to be
 *  set during get_invariants because the PHY/SFP type was
 *  not known.  Perform the SFP init if necessary.
 *
 **/
static s32 rnpm_init_phy_ops_vu440(struct rnpm_hw *hw)
{
	// struct rnpm_mac_info *mac = &hw->mac;
	// struct rnpm_phy_info *phy = &hw->phy;
	s32 ret_val = 0;

	hw->phy.sfp_setup_needed = true;
	return ret_val;
}

static s32 rnpm_setup_sfp_modules_440(struct rnpm_hw *hw)
{
	return 0;
}


/**
 *  rnpm_reinit_fdir_tables_vu440 - Reinitialize Flow Director tables.
 *  @hw: pointer to hardware structure
 **/
s32 rnpm_reinit_fdir_tables_vu440(struct rnpm_hw *hw)
{

	return 0;
}

/**
 *  rnpm_fdir_enable_vu440 - Initialize Flow Director control registers
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register
 **/
__maybe_unused static void rnpm_fdir_enable_vu440(struct rnpm_hw *hw,
												  u32 fdirctrl)
{
}

/**
 *  rnpm_init_fdir_signature_vu440 - Initialize Flow Director signature filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *             contains just the value of the Rx packet buffer allocation
 **/
s32 rnpm_init_fdir_signature_vu440(struct rnpm_hw *hw, u32 fdirctrl)
{
#if 0
	/*
	 * Continue setup of fdirctrl register bits:
	 *  Move the flexible bytes to use the ethertype - shift 6 words
	 *  Set the maximum length per hash bucket to 0xA filters
	 *  Send interrupt when 64 filters are left
	 */
	fdirctrl |= (0x6 << RNPM_FDIRCTRL_FLEX_SHIFT) |
		    (0xA << RNPM_FDIRCTRL_MAX_LENGTH_SHIFT) |
		    (4 << RNPM_FDIRCTRL_FULL_THRESH_SHIFT);

	/* write hashes and fdirctrl register, poll for completion */
	rnpm_fdir_enable_vu440(hw, fdirctrl);
#endif

	return 0;
}

/**
 *  rnpm_init_fdir_perfect_vu440 - Initialize Flow Director perfect filters
 *  @hw: pointer to hardware structure
 *  @fdirctrl: value to write to flow director control register, initially
 *             contains just the value of the Rx packet buffer allocation
 **/
s32 rnpm_init_fdir_perfect_vu440(struct rnpm_hw *hw, u32 fdirctrl)
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
	fdirctrl |= RNPM_FDIRCTRL_PERFECT_MATCH |
		    RNPM_FDIRCTRL_REPORT_STATUS |
		    (RNPM_FDIR_DROP_QUEUE << RNPM_FDIRCTRL_DROP_Q_SHIFT) |
		    (0x6 << RNPM_FDIRCTRL_FLEX_SHIFT) |
		    (0xA << RNPM_FDIRCTRL_MAX_LENGTH_SHIFT) |
		    (4 << RNPM_FDIRCTRL_FULL_THRESH_SHIFT);

	/* write hashes and fdirctrl register, poll for completion */
	rnpm_fdir_enable_vu440(hw, fdirctrl);

#endif
	return 0;
}

/*
 * These defines allow us to quickly generate all of the necessary instructions
 * in the function below by simply calling out RNPM_COMPUTE_SIG_HASH_ITERATION
 * for values 0 through 15
 */
#define RNPM_ATR_COMMON_HASH_KEY \
		(RNPM_ATR_BUCKET_HASH_KEY & RNPM_ATR_SIGNATURE_HASH_KEY)
#define RNPM_COMPUTE_SIG_HASH_ITERATION(_n) \
do { \
} while (0)

/**
 *  rnpm_atr_compute_sig_hash_vu440 - Compute the signature hash
 *  @stream: input bitstream to compute the hash on
 *
 *  This function is almost identical to the function above but contains
 *  several optomizations such as unwinding all of the loops, letting the
 *  compiler work out all of the conditional ifs since the keys are static
 *  defines, and computing two keys at once since the hashed dword stream
 *  will be the same for both keys.
 **/
static u32 __maybe_unused rnpm_atr_compute_sig_hash_vu440(
	union rnpm_atr_hash_dword input, union rnpm_atr_hash_dword common)
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
	RNPM_COMPUTE_SIG_HASH_ITERATION(0);

	/*
	 * apply flow ID/VM pool/VLAN ID bits to lo hash dword, we had to
	 * delay this because bit 0 of the stream should not be processed
	 * so we do not add the vlan until after bit 0 was processed
	 */
	lo_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan << 16);

	/* Process remaining 30 bit of the key */
	RNPM_COMPUTE_SIG_HASH_ITERATION(1);
	RNPM_COMPUTE_SIG_HASH_ITERATION(2);
	RNPM_COMPUTE_SIG_HASH_ITERATION(3);
	RNPM_COMPUTE_SIG_HASH_ITERATION(4);
	RNPM_COMPUTE_SIG_HASH_ITERATION(5);
	RNPM_COMPUTE_SIG_HASH_ITERATION(6);
	RNPM_COMPUTE_SIG_HASH_ITERATION(7);
	RNPM_COMPUTE_SIG_HASH_ITERATION(8);
	RNPM_COMPUTE_SIG_HASH_ITERATION(9);
	RNPM_COMPUTE_SIG_HASH_ITERATION(10);
	RNPM_COMPUTE_SIG_HASH_ITERATION(11);
	RNPM_COMPUTE_SIG_HASH_ITERATION(12);
	RNPM_COMPUTE_SIG_HASH_ITERATION(13);
	RNPM_COMPUTE_SIG_HASH_ITERATION(14);
	RNPM_COMPUTE_SIG_HASH_ITERATION(15);

	/* combine common_hash result with signature and bucket hashes */
	bucket_hash ^= common_hash;
	bucket_hash &= RNPM_ATR_HASH_MASK;

	sig_hash ^= common_hash << 16;
	sig_hash &= RNPM_ATR_HASH_MASK << 16;

	/* return completed signature hash */
	return sig_hash ^ bucket_hash;
#else
	return 0;
#endif
}

/**
 *  rnpm_atr_add_signature_filter_vu440 - Adds a signature hash filter
 *  @hw: pointer to hardware structure
 *  @input: unique input dword
 *  @common: compressed common input dword
 *  @queue: queue index to direct traffic to
 **/
s32 rnpm_fdir_add_signature_filter_vu440(struct rnpm_hw *hw,
		union rnpm_atr_hash_dword input,
		union rnpm_atr_hash_dword common,
		u8 queue)
{
#if 0
	u64  fdirhashcmd;
	u32  fdircmd;
	/*
	 * Get the flow_type in order to program FDIRCMD properly
	 * lowest 2 bits are FDIRCMD.L4TYPE, third lowest bit is FDIRCMD.IPV6
	 */
	switch (input.formatted.flow_type) {
	case RNPM_ATR_FLOW_TYPE_TCPV4:
	case RNPM_ATR_FLOW_TYPE_UDPV4:
	case RNPM_ATR_FLOW_TYPE_SCTPV4:
	case RNPM_ATR_FLOW_TYPE_TCPV6:
	case RNPM_ATR_FLOW_TYPE_UDPV6:
	case RNPM_ATR_FLOW_TYPE_SCTPV6:
		break;
	default:
		hw_dbg(hw, " Error on flow type input\n");
		return RNPM_ERR_CONFIG;
	}

	/* configure FDIRCMD register */
	fdircmd = RNPM_FDIRCMD_CMD_ADD_FLOW | RNPM_FDIRCMD_FILTER_UPDATE |
		RNPM_FDIRCMD_LAST | RNPM_FDIRCMD_QUEUE_EN;
	fdircmd |= input.formatted.flow_type << RNPM_FDIRCMD_FLOW_TYPE_SHIFT;
	fdircmd |= (u32)queue << RNPM_FDIRCMD_RX_QUEUE_SHIFT;

	/*
	 * The lower 32-bits of fdirhashcmd is for FDIRHASH, the upper 32-bits
	 * is for FDIRCMD.  Then do a 64-bit register write from FDIRHASH.
	 */
	fdirhashcmd = (u64)fdircmd << 32;
	fdirhashcmd |= rnpm_atr_compute_sig_hash_vu440(input, common);
	//RNPM_WRITE_REG64(hw, RNPM_FDIRHASH, fdirhashcmd);

	hw_dbg(hw, "Tx Queue=%x hash=%x\n", queue, (u32)fdirhashcmd);
#endif
	return 0;
}

#define RNPM_COMPUTE_BKT_HASH_ITERATION(_n) \
do { \
	u32 n = (_n); \
	if (RNPM_ATR_BUCKET_HASH_KEY & (0x01 << n)) \
		bucket_hash ^= lo_hash_dword >> n; \
	if (RNPM_ATR_BUCKET_HASH_KEY & (0x01 << (n + 16))) \
		bucket_hash ^= hi_hash_dword >> n; \
} while (0)

/**
 *  rnpm_atr_compute_perfect_hash_vu440 - Compute the perfect filter hash
 *  @atr_input: input bitstream to compute the hash on
 *  @input_mask: mask for the input bitstream
 *
 *  This function serves two main purposes.  First it applys the input_mask
 *  to the atr_input resulting in a cleaned up atr_input data stream.
 *  Secondly it computes the hash and stores it in the bkt_hash field at
 *  the end of the input byte stream.  This way it will be available for
 *  future use without needing to recompute the hash.
 **/
void rnpm_atr_compute_perfect_hash_vu440(union rnpm_atr_input *input,
					  union rnpm_atr_input *input_mask)
{

#if 0
	u32 hi_hash_dword, lo_hash_dword, flow_vm_vlan;
	u32 bucket_hash = 0;

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
	RNPM_COMPUTE_BKT_HASH_ITERATION(0);

	/*
	 * apply flow ID/VM pool/VLAN ID bits to lo hash dword, we had to
	 * delay this because bit 0 of the stream should not be processed
	 * so we do not add the vlan until after bit 0 was processed
	 */
	lo_hash_dword ^= flow_vm_vlan ^ (flow_vm_vlan << 16);

	/* Process remaining 30 bit of the key */
	RNPM_COMPUTE_BKT_HASH_ITERATION(1);
	RNPM_COMPUTE_BKT_HASH_ITERATION(2);
	RNPM_COMPUTE_BKT_HASH_ITERATION(3);
	RNPM_COMPUTE_BKT_HASH_ITERATION(4);
	RNPM_COMPUTE_BKT_HASH_ITERATION(5);
	RNPM_COMPUTE_BKT_HASH_ITERATION(6);
	RNPM_COMPUTE_BKT_HASH_ITERATION(7);
	RNPM_COMPUTE_BKT_HASH_ITERATION(8);
	RNPM_COMPUTE_BKT_HASH_ITERATION(9);
	RNPM_COMPUTE_BKT_HASH_ITERATION(10);
	RNPM_COMPUTE_BKT_HASH_ITERATION(11);
	RNPM_COMPUTE_BKT_HASH_ITERATION(12);
	RNPM_COMPUTE_BKT_HASH_ITERATION(13);
	RNPM_COMPUTE_BKT_HASH_ITERATION(14);
	RNPM_COMPUTE_BKT_HASH_ITERATION(15);

	/*
	 * Limit hash to 13 bits since max bucket count is 8K.
	 * Store result at the end of the input stream.
	 */
	input->formatted.bkt_hash = bucket_hash & 0x1FFF;
#endif
}

/**
 *  rnpm_get_fdirtcpm_vu440 - generate a tcp port from atr_input_masks
 *  @input_mask: mask to be bit swapped
 *
 *  The source and destination port masks for flow director are bit swapped
 *  in that bit 15 effects bit 0, 14 effects 1, 13, 2 etc.  In order to
 *  generate a correctly swapped value we need to bit swap the mask and that
 *  is what is accomplished by this function.
 **/
static u32 __maybe_unused
rnpm_get_fdirtcpm_vu440(union rnpm_atr_input *input_mask)
{
#if 0
	u32 mask = ntohs(input_mask->formatted.dst_port);

	mask <<= RNPM_FDIRTCPM_DPORTM_SHIFT;
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
#define RNPM_STORE_AS_BE32(_value) \
	(((u32)(_value) >> 24) | (((u32)(_value) & 0x00FF0000) >> 8) | \
	 (((u32)(_value) & 0x0000FF00) << 8) | ((u32)(_value) << 24))

#define RNPM_WRITE_REG_BE32(a, reg, value) \
	RNPM_WRITE_REG((a), (reg), RNPM_STORE_AS_BE32(ntohl(value)))

#define RNPM_STORE_AS_BE16(_value) \
	ntohs(((u16)(_value) >> 8) | ((u16)(_value) << 8))

s32 rnpm_fdir_set_input_mask_vu440(struct rnpm_hw *hw,
				    union rnpm_atr_input *input_mask)
{
#if 0
	/* mask IPv6 since it is currently not supported */
	u32 fdirm = RNPM_FDIRM_DIPv6;
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
		fdirm |= RNPM_FDIRM_POOL;
	case 0x7F:
		break;
	default:
		hw_dbg(hw, " Error on vm pool mask\n");
		return RNPM_ERR_CONFIG;
	}

	switch (input_mask->formatted.flow_type & RNPM_ATR_L4TYPE_MASK) {
	case 0x0:
		fdirm |= RNPM_FDIRM_L4P;
		if (input_mask->formatted.dst_port ||
		    input_mask->formatted.src_port) {
			hw_dbg(hw, " Error on src/dst port mask\n");
			return RNPM_ERR_CONFIG;
		}
	case RNPM_ATR_L4TYPE_MASK:
		break;
	default:
		hw_dbg(hw, " Error on flow type mask\n");
		return RNPM_ERR_CONFIG;
	}

	switch (ntohs(input_mask->formatted.vlan_id) & 0xEFFF) {
	case 0x0000:
		/* mask VLAN ID, fall through to mask VLAN priority */
		fdirm |= RNPM_FDIRM_VLANID;
	case 0x0FFF:
		/* mask VLAN priority */
		fdirm |= RNPM_FDIRM_VLANP;
		break;
	case 0xE000:
		/* mask VLAN ID only, fall through */
		fdirm |= RNPM_FDIRM_VLANID;
	case 0xEFFF:
		/* no VLAN fields masked */
		break;
	default:
		hw_dbg(hw, " Error on VLAN mask\n");
		return RNPM_ERR_CONFIG;
	}

	switch (input_mask->formatted.flex_bytes & 0xFFFF) {
	case 0x0000:
		/* Mask Flex Bytes, fall through */
		fdirm |= RNPM_FDIRM_FLEX;
	case 0xFFFF:
		break;
	default:
		hw_dbg(hw, " Error on flexible byte mask\n");
		return RNPM_ERR_CONFIG;
	}

	/* Now mask VM pool and destination IPv6 - bits 5 and 2 */
	RNPM_WRITE_REG(hw, RNPM_FDIRM, fdirm);

	/* store the TCP/UDP port masks, bit reversed from port layout */
	fdirtcpm = rnpm_get_fdirtcpm_vu440(input_mask);

	/* write both the same so that UDP and TCP use the same mask */
	RNPM_WRITE_REG(hw, RNPM_FDIRTCPM, ~fdirtcpm);
	RNPM_WRITE_REG(hw, RNPM_FDIRUDPM, ~fdirtcpm);

	/* store source and destination IP masks (big-enian) */
	RNPM_WRITE_REG_BE32(hw, RNPM_FDIRSIP4M,
			     ~input_mask->formatted.src_ip[0]);
	RNPM_WRITE_REG_BE32(hw, RNPM_FDIRDIP4M,
			     ~input_mask->formatted.dst_ip[0]);

#endif
	return 0;
}

s32 rnpm_fdir_write_perfect_filter_vu440(struct rnpm_hw *hw,
					  union rnpm_atr_input *input,
					  u16 soft_id, u8 queue)
{
#if 0
	u32 fdirport, fdirvlan, fdirhash, fdircmd;

	/* currently IPv6 is not supported, must be programmed with 0 */
	RNPM_WRITE_REG_BE32(hw, RNPM_FDIRSIPv6(0),
			     input->formatted.src_ip[0]);
	RNPM_WRITE_REG_BE32(hw, RNPM_FDIRSIPv6(1),
			     input->formatted.src_ip[1]);
	RNPM_WRITE_REG_BE32(hw, RNPM_FDIRSIPv6(2),
			     input->formatted.src_ip[2]);

	/* record the source address (big-endian) */
	RNPM_WRITE_REG_BE32(hw, RNPM_FDIRIPSA, input->formatted.src_ip[0]);

	/* record the first 32 bits of the destination address (big-endian) */
	RNPM_WRITE_REG_BE32(hw, RNPM_FDIRIPDA, input->formatted.dst_ip[0]);

	/* record source and destination port (little-endian)*/
	fdirport = ntohs(input->formatted.dst_port);
	fdirport <<= RNPM_FDIRPORT_DESTINATION_SHIFT;
	fdirport |= ntohs(input->formatted.src_port);
	RNPM_WRITE_REG(hw, RNPM_FDIRPORT, fdirport);

	/* record vlan (little-endian) and flex_bytes(big-endian) */
	fdirvlan = RNPM_STORE_AS_BE16(input->formatted.flex_bytes);
	fdirvlan <<= RNPM_FDIRVLAN_FLEX_SHIFT;
	fdirvlan |= ntohs(input->formatted.vlan_id);
	RNPM_WRITE_REG(hw, RNPM_FDIRVLAN, fdirvlan);

	/* configure FDIRHASH register */
	fdirhash = input->formatted.bkt_hash;
	fdirhash |= soft_id << RNPM_FDIRHASH_SIG_SW_INDEX_SHIFT;
	RNPM_WRITE_REG(hw, RNPM_FDIRHASH, fdirhash);

	/*
	 * flush all previous writes to make certain registers are
	 * programmed prior to issuing the command
	 */
	RNPM_WRITE_FLUSH(hw);

	/* configure FDIRCMD register */
	fdircmd = RNPM_FDIRCMD_CMD_ADD_FLOW | RNPM_FDIRCMD_FILTER_UPDATE |
		  RNPM_FDIRCMD_LAST | RNPM_FDIRCMD_QUEUE_EN;
	if (queue == RNPM_FDIR_DROP_QUEUE)
		fdircmd |= RNPM_FDIRCMD_DROP;
	fdircmd |= input->formatted.flow_type << RNPM_FDIRCMD_FLOW_TYPE_SHIFT;
	fdircmd |= (u32)queue << RNPM_FDIRCMD_RX_QUEUE_SHIFT;
	fdircmd |= (u32)input->formatted.vm_pool << RNPM_FDIRCMD_VT_POOL_SHIFT;

	RNPM_WRITE_REG(hw, RNPM_FDIRCMD, fdircmd);
#endif
	return 0;
}

s32 rnpm_fdir_erase_perfect_filter_vu440(struct rnpm_hw *hw,
					  union rnpm_atr_input *input,
					  u16 soft_id)
{
	s32 err = 0;
#if 0
	u32 fdirhash;
	u32 fdircmd = 0;
	u32 retry_count;

	/* configure FDIRHASH register */
	fdirhash = input->formatted.bkt_hash;
	fdirhash |= soft_id << RNPM_FDIRHASH_SIG_SW_INDEX_SHIFT;
	RNPM_WRITE_REG(hw, RNPM_FDIRHASH, fdirhash);

	/* flush hash to HW */
	RNPM_WRITE_FLUSH(hw);

	/* Query if filter is present */
	RNPM_WRITE_REG(hw, RNPM_FDIRCMD, RNPM_FDIRCMD_CMD_QUERY_REM_FILT);

	for (retry_count = 10; retry_count; retry_count--) {
		/* allow 10us for query to process */
		udelay(10);
		/* verify query completed successfully */
		fdircmd = RNPM_READ_REG(hw, RNPM_FDIRCMD);
		if (!(fdircmd & RNPM_FDIRCMD_CMD_MASK))
			break;
	}

	if (!retry_count)
		err = RNPM_ERR_FDIR_REINIT_FAILED;

	/* if filter exists in hardware then remove it */
	if (fdircmd & RNPM_FDIRCMD_FILTER_VALID) {
		RNPM_WRITE_REG(hw, RNPM_FDIRHASH, fdirhash);
		RNPM_WRITE_FLUSH(hw);
		RNPM_WRITE_REG(hw, RNPM_FDIRCMD,
				RNPM_FDIRCMD_CMD_REMOVE_FLOW);
	}
#endif
	return err;
}


/**
 *  rnpm_identify_phy_vu440 - Get physical layer module
 *  @hw: pointer to hardware structure
 *
 *  Determines the physical layer module found on the current adapter.
 *  If PHY already detected, maintains current PHY type in hw struct,
 *  otherwise executes the PHY detection routine.
 **/
static s32 rnpm_identify_phy_vu440(struct rnpm_hw *hw)
{
	// s32 status = RNPM_ERR_PHY_ADDR_INVALID;

	hw->phy.type = rnpm_phy_sfp;

	return 0;
}

static s32 rnpm_identify_sfp_module_vu440(struct rnpm_hw *hw)
{
	hw->phy.sfp_type = rnpm_sfp_type_da_cu;

	return 0;
}


/**
 *  rnpm_enable_rx_dma_vu440 - Enable the Rx DMA unit on vu440
 *  @hw: pointer to hardware structure
 *  @regval: register value to write to RXCTRL
 *
 *  Enables the Rx DMA unit for vu440
 **/
static s32 rnpm_enable_rx_dma_vu440(struct rnpm_hw *hw, u32 regval)
{
	/*
	 * Workaround for vu440 silicon errata when enabling the Rx datapath.
	 * If traffic is incoming before we enable the Rx unit, it could hang
	 * the Rx DMA unit.  Therefore, make sure the security engine is
	 * completely disabled prior to enabling the Rx unit.
	 */
	hw->mac.ops.disable_rx_buff(hw);

	//RNPM_WRITE_REG(hw, RNPM_RXCTRL, regval);

	hw->mac.ops.enable_rx_buff(hw);

	return 0;
}

/**
 *  rnpm_verify_fw_version_vu440 - verify fw version for vu440
 *  @hw: pointer to hardware structure
 *
 *  Verifies that installed the firmware version is 0.6 or higher
 *  for SFI devices. All vu440 SFI devices should have version 0.6 or higher.
 *
 *  Returns RNPM_ERR_EEPROM_VERSION if the FW is not present or
 *  if the FW version is not supported.
 **/
static s32 rnpm_verify_fw_version_vu440(struct rnpm_hw *hw)
{
	// s32 status = RNPM_ERR_EEPROM_VERSION;
	// u16 fw_offset, fw_ptp_cfg_offset;
	// u16 fw_version = 0;

	return 0;
}

/**
 *  rnpm_verify_lesm_fw_enabled_vu440 - Checks LESM FW module state.
 *  @hw: pointer to hardware structure
 *
 *  Returns true if the LESM FW module is present and enabled. Otherwise
 *  returns false. Smart Speed must be disabled if LESM FW module is enabled.
 **/
bool rnpm_verify_lesm_fw_enabled_vu440(struct rnpm_hw *hw)
{
	bool lesm_enabled = false;

	return lesm_enabled;
}

/**
 *  rnpm_read_eeprom_buffer_vu440 - Read EEPROM word(s) using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Retrieves 16 bit word(s) read from EEPROM
 **/
static s32 __maybe_unused rnpm_read_eeprom_buffer_vu440(struct rnpm_hw *hw,
														u16 offset,
														u16 words,
														u16 *data)
{
	s32 ret_val = RNPM_ERR_CONFIG;
#if 0
	struct rnpm_eeprom_info *eeprom = &hw->eeprom;

	/*
	 * If EEPROM is detected and can be addressed using 14 bits,
	 * use EERD otherwise use bit bang
	 */
	if ((eeprom->type == rnpm_eeprom_spi) &&
	    (offset + (words - 1) <= RNPM_EERD_MAX_ADDR))
		ret_val = rnpm_read_eerd_buffer_generic(hw, offset, words,
							 data);
	else
		ret_val = rnpm_read_eeprom_buffer_bit_bang_generic(hw, offset,
								    words,
								    data);

#endif
	return ret_val;
}

/**
 *  rnpm_read_eeprom_vu440 - Read EEPROM word using
 *  fastest available method
 *
 *  @hw: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM
 **/
static s32 __maybe_unused rnpm_read_eeprom_vu440(struct rnpm_hw *hw,
												 u16 offset,
												 u16 *data)
{
	s32 ret_val = RNPM_ERR_CONFIG;

#if 0
	struct rnpm_eeprom_info *eeprom = &hw->eeprom;
	/*
	 * If EEPROM is detected and can be addressed using 14 bits,
	 * use EERD otherwise use bit bang
	 */
	if ((eeprom->type == rnpm_eeprom_spi) &&
	    (offset <= RNPM_EERD_MAX_ADDR))
		ret_val = rnpm_read_eerd_generic(hw, offset, data);
	else
		ret_val = rnpm_read_eeprom_bit_bang_generic(hw, offset, data);
#endif
	return ret_val;
}

/**
 * rnpm_reset_pipeline_vu440 - perform pipeline reset
 *
 * @hw: pointer to hardware structure
 *
 * Reset pipeline by asserting Restart_AN together with LMS change to ensure
 * full pipeline reset.  Note - We must hold the SW/FW semaphore before writing
 * to AUTOC, so this function assumes the semaphore is held.
 **/
s32 rnpm_reset_pipeline_vu440(struct rnpm_hw *hw)
{
	s32 ret_val;
	u32 i;

	/* Enable link if disabled in NVM */

	/* Write AUTOC register with toggled LMS[2] bit and Restart_AN */

	/* Wait for AN to leave state 0 */
	for (i = 0; i < 10; i++) {
		usleep_range(4000, 8000);
			break;
	}

	ret_val = 0;

	// reset_pipeline_out:
	/* Write AUTOC register with original LMS field and Restart_AN */

	return ret_val;
}

static void __maybe_unused upl_init(u8 __iomem* bar2)
{
	int data;

#define SOFT_COMMON11 (0x0007000 + 0xf2c)
#define SOFT_COMMON12 (0x0007000 + 0xf30)
	// config ulh pll
	data = ioread32((void *)(bar2 + SOFT_COMMON11));
	iowrite32(((0x3 << 29) | data), (void *)(bar2 + SOFT_COMMON11)); // ulh pd is 1, bypass is 1
	data = ioread32((void *)(bar2 + SOFT_COMMON11));
	iowrite32(((0x1 << 31) | data), (void *)(bar2 + SOFT_COMMON11)); // ulh reset is 1

	data = ioread32((void *)(bar2 + SOFT_COMMON12));
	iowrite32(((0x3 << 29) | data), (void *) (bar2 + SOFT_COMMON12)); // ulh pd is 1, bypass is 1
	data = ioread32((void *)(bar2 + SOFT_COMMON12));
	iowrite32(((0x1 << 31) | data), (void *) (bar2 + SOFT_COMMON12)); // ulh reset is 1
}


/**
 *  rnpm_reset_hw_vu440 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
static s32 rnpm_reset_hw_vu440(struct rnpm_hw *hw)
{
	// int i;
	s32 status = 0;
	// struct rnpm_adapter *adapter = (struct rnpm_adapter *)hw->back;
	int port = hw->num;

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

	// maybe too early to open rx?
	wr32(hw, RNPM_MAC_RX_CFG(port), rd32(hw, RNPM_MAC_RX_CFG(port)) | 0x01);
	
	/* in this mode close mc filter in mac */
	if (hw->mac.mc_location == rnpm_mc_location_nic)
		wr32(hw, RNPM_MAC_PKT_FLT(port), 0x80000000);
	else
		wr32(hw, RNPM_MAC_PKT_FLT(port), 0x00000400);

	wr32(hw, RNPM_MAC_LPI_CTRL(port), 0x00060000);

	/* Store the permanent mac address only once */
	if (!(hw->mac.mac_flags & RNPM_FLAGS_INIT_MAC_ADDRESS)) {
		rnpm_get_permtion_mac_addr(hw, hw->mac.perm_addr);
		memcpy(hw->mac.addr, hw->mac.perm_addr, ETH_ALEN);
	}

	hw->mac.num_rar_entries = RNPM_vu440_RAR_ENTRIES;
	hw->mac.ops.init_rx_addrs(hw);

	return 0;
}

/**
 *  rnpm_start_hw_vu440 - Prepare hardware for Tx/Rx
 *  @hw: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function
 *  and the generation start_hw function.
 *  Then performs revision-specific operations, if any.
 **/
static s32 rnpm_start_hw_vu440(struct rnpm_hw *hw)
{
	s32 ret_val = 0;

	ret_val = rnpm_start_hw_generic(hw);
	if (ret_val != 0)
		goto out;

	ret_val = rnpm_start_hw_gen2(hw);
	if (ret_val != 0)
		goto out;

	// ETH Registers
	//wr32(hw, RNPM_ETH_ERR_MASK_VECTOR, ~ETH_IGNORE_ALL_ERR);
	wr32(hw, RNPM_ETH_ERR_MASK_VECTOR, 0);
	wr32(hw, RNPM_ETH_BYPASS, 0);
	wr32(hw, RNPM_ETH_DEFAULT_RX_RING, 0);

/*
	wr32(hw, RNPM_TOP_NIC_CONFIG, hw->mode
#ifdef CONFIG_RNPM_FPGA
			| hw->default_rx_queue << 24
#endif
	    );*/

	// DMA common Registers
	wr32(hw, RNPM_DMA_CONFIG, DMA_VEB_BYPASS);

	// enable-dma-axi
	wr32(hw, RNPM_DMA_AXI_EN, (RX_AXI_RW_EN | TX_AXI_RW_EN));


	if (ret_val == 0)
		ret_val = rnpm_verify_fw_version_vu440(hw);
out:
	return ret_val;
}

/**
 *  rnpm_get_media_type_vu440 - Get media type
 *  @hw: pointer to hardware structure
 *
 *  Returns the media type (fiber, copper, backplane)
 **/
static enum rnpm_media_type rnpm_get_media_type_vu440(struct rnpm_hw *hw)
{
	enum rnpm_media_type media_type = rnpm_media_type_fiber;
	return media_type;
}

/**
 *  rnpm_get_supported_physical_layer_vu440 - Returns physical layer type
 *  @hw: pointer to hardware structure
 *
 *  Determines physical layer capabilities of the current configuration.
 **/
static u32 rnpm_get_supported_physical_layer_vu440(struct rnpm_hw *hw)
{
	u32 physical_layer = 0;

	return physical_layer;
}

static s32 rnpm_get_link_capabilities_vu440(struct rnpm_hw *hw,
		rnpm_link_speed *speed,
		bool *autoneg,
		u32 *media_type)
{
	/* fix setup */
	*speed = RNPM_LINK_SPEED_10GB_FULL;
	*autoneg = false;
	*media_type = rnpm_media_type_fiber;

	return 0;
}

static struct rnpm_phy_operations phy_ops_vu440 = {
	.identify		= &rnpm_identify_phy_vu440,
	.identify_sfp		= &rnpm_identify_sfp_module_vu440,
	.init			= &rnpm_init_phy_ops_vu440,
	.reset			= &rnpm_reset_phy_generic,
	.read_reg		= &rnpm_read_phy_reg_generic,
	.write_reg		= &rnpm_write_phy_reg_generic,
	.setup_link		= &rnpm_setup_phy_link_generic,
	.setup_link_speed	= &rnpm_setup_phy_link_speed_generic,
	.read_i2c_byte		= &rnpm_read_i2c_byte_generic,
	.write_i2c_byte		= &rnpm_write_i2c_byte_generic,
	.read_i2c_sff8472	= &rnpm_read_i2c_sff8472_generic,
	.read_i2c_eeprom	= &rnpm_read_i2c_eeprom_generic,
	.write_i2c_eeprom	= &rnpm_write_i2c_eeprom_generic,
	.check_overtemp		= &rnpm_tn_check_overtemp,
};

static struct rnpm_mac_operations mac_ops_vu440 = {
	.init_hw                = &rnpm_init_hw_generic,
	.reset_hw               = &rnpm_reset_hw_vu440,
	.start_hw               = &rnpm_start_hw_vu440,
	.clear_hw_cntrs         = &rnpm_clear_hw_cntrs_generic,
	.get_media_type         = &rnpm_get_media_type_vu440,
	.get_supported_physical_layer = &rnpm_get_supported_physical_layer_vu440,
	.enable_rx_dma          = &rnpm_enable_rx_dma_vu440,
	.disable_rx_buff	= &rnpm_disable_rx_buff_generic,
	.enable_rx_buff		= &rnpm_enable_rx_buff_generic,
	.get_mac_addr           = &rnpm_get_mac_addr_generic,
	.get_device_caps        = &rnpm_get_device_caps_generic,
	//.setup_link             = &ixgbe_setup_mac_link_82599,
	.get_wwn_prefix         = &rnpm_get_wwn_prefix_generic,
	.stop_adapter           = &rnpm_stop_adapter_generic,
	//.set_rxpba		        = &rnpm_set_rxpba_generic,
	.check_link             = &rnpm_check_mac_link_generic,
	.get_link_capabilities  = &rnpm_get_link_capabilities_vu440,
	.led_on                 = &rnpm_led_on_generic,
	.led_off                = &rnpm_led_off_generic,
	.blink_led_start        = &rnpm_blink_led_start_generic,
	.blink_led_stop         = &rnpm_blink_led_stop_generic,
	//.get_bus_info           = &rnpm_get_bus_info_generic,
	.set_rar                = &rnpm_set_rar_generic,
	.set_rar_mac            = &rnpm_set_rar_mac,
	.clear_rar              = &rnpm_clear_rar_generic,
	.clear_rar_mac          = &rnpm_clear_rar_mac,
	.set_vmdq               = &rnpm_set_vmdq_generic,
	//.set_vmdq_san_mac	= &rnpm_set_vmdq_san_mac_generic,
	.clear_vmdq             = &rnpm_clear_vmdq_generic,
	.init_rx_addrs          = &rnpm_init_rx_addrs_generic,
	//.update_mc_addr_list    = &rnpm_update_mc_addr_list_generic,
	.update_mc_addr_list    = &rnpm_update_mutiport_mc_addr_list_generic,
	.enable_mc              = &rnpm_enable_mc_generic,
	.disable_mc             = &rnpm_disable_mc_generic,
	.clear_vfta             = &rnpm_clear_vfta_generic,
	.set_vfta               = &rnpm_set_vfta_generic,
	.set_vfta_mac           = &rnpm_set_vfta_mac_generic,
	.fc_enable              = &rnpm_fc_enable_generic,
	.set_fw_drv_ver         = &rnpm_set_fw_drv_ver_generic,
	.init_uta_tables        = &rnpm_init_uta_tables_generic,
	.setup_sfp              = &rnpm_setup_sfp_modules_440,
	.mng_fw_enabled		= &rnpm_mng_enabled,
};

//==========   vu440 ===============
struct rnpm_info rnpm_vu440_2x10G_info = {
    .one_pf_with_two_dma   = false,
    .total_queue_pair_cnts = RNPM_vu440_MAX_TX_QUEUES,
	.queue_depth     = RNPM_vu440_DEFAULT_TXD,
    .total_msix_table      = 16,
	.coalesce.tx_work_limit = RNPM_DEFAULT_TX_WORK,
	.coalesce.rx_usecs = RNPM_PKT_TIMEOUT,
	.coalesce.rx_frames = RNPM_RX_PKT_POLL_BUDGET,
	.coalesce.tx_usecs = RNPM_PKT_TIMEOUT_TX,
	.coalesce.tx_frames = RNPM_TX_PKT_POLL_BUDGET,
    .total_layer2_count    = RNPM_MAX_LAYER2_FILTERS,
#if NET_FEATURE_TCAM
    .total_tuple5_count = RNPM_MAX_TCAM_FILTERS,
#else
    .total_tuple5_count = RNPM_MAX_TUPLE5_FILTERS,
#endif
    .adapter_cnt = 1,
    .rss_type    = rnpm_rss_uv440,
    // .mac                    = rnpm_mac_2port_10G,
    .get_invariants = &rnpm_get_invariants_vu440,
    .mac_ops        = &mac_ops_vu440,
    .eeprom_ops     = NULL,
    .phy_ops        = &phy_ops_vu440,
    .mbx_ops        = &mbx_ops_generic,
};
struct rnpm_info rnpm_vu440_2x40G_info = {
    .one_pf_with_two_dma   = false,
    .total_queue_pair_cnts = RNPM_vu440_MAX_TX_QUEUES,
	.queue_depth     = RNPM_vu440_DEFAULT_TXD,
    .total_msix_table      = 16,
	.coalesce.tx_work_limit = RNPM_DEFAULT_TX_WORK,
	.coalesce.rx_usecs = RNPM_PKT_TIMEOUT,
	.coalesce.rx_frames = RNPM_RX_PKT_POLL_BUDGET,
	.coalesce.tx_usecs = RNPM_PKT_TIMEOUT_TX,
	.coalesce.tx_frames = RNPM_TX_PKT_POLL_BUDGET,
    .total_layer2_count    = RNPM_MAX_LAYER2_FILTERS,
#if NET_FEATURE_TCAM
    .total_tuple5_count = RNPM_MAX_TCAM_FILTERS,
#else
    .total_tuple5_count = RNPM_MAX_TUPLE5_FILTERS,
#endif
    .adapter_cnt = 1,
    .rss_type    = rnpm_rss_uv440,
    // .mac                    = rnpm_mac_2port_40G,
    .get_invariants = &rnpm_get_invariants_vu440,
    .mac_ops        = &mac_ops_vu440,
    .eeprom_ops     = NULL,
    .phy_ops        = &phy_ops_vu440,
    .mbx_ops        = &mbx_ops_generic,
};

struct rnpm_info rnpm_vu440_4x10G_info = {
    .one_pf_with_two_dma   = false,
    .total_queue_pair_cnts = RNPM_vu440_MAX_TX_QUEUES,
	.queue_depth     = RNPM_vu440_DEFAULT_TXD,
    .total_msix_table      = 64,
	.coalesce.tx_work_limit = RNPM_DEFAULT_TX_WORK,
	.coalesce.rx_usecs = RNPM_PKT_TIMEOUT,
	.coalesce.rx_frames = RNPM_RX_PKT_POLL_BUDGET,
	.coalesce.tx_usecs = RNPM_PKT_TIMEOUT_TX,
	.coalesce.tx_frames = RNPM_TX_PKT_POLL_BUDGET,
    .total_layer2_count    = RNPM_MAX_LAYER2_FILTERS,
#if NET_FEATURE_TCAM
    .total_tuple5_count = RNPM_MAX_TCAM_FILTERS,
#else
    .total_tuple5_count = RNPM_MAX_TUPLE5_FILTERS,
#endif
    .adapter_cnt = 2,
    .rss_type    = rnpm_rss_uv440,
    // .mac                    = rnpm_mac_4port_10G,
    .get_invariants = &rnpm_get_invariants_vu440,
    .mac_ops        = &mac_ops_vu440,
    .eeprom_ops     = NULL,
    .phy_ops        = &phy_ops_vu440,
    .mbx_ops        = &mbx_ops_generic,
};

struct rnpm_info rnpm_vu440_8x10G_info = {
    .one_pf_with_two_dma   = false,
    .total_queue_pair_cnts = RNPM_vu440_MAX_TX_QUEUES,
	.queue_depth     = RNPM_vu440_DEFAULT_TXD,
    .total_msix_table      = 64,
	.coalesce.tx_work_limit = RNPM_DEFAULT_TX_WORK,
	.coalesce.rx_usecs = RNPM_PKT_TIMEOUT,
	.coalesce.rx_frames = RNPM_RX_PKT_POLL_BUDGET,
	.coalesce.tx_usecs = RNPM_PKT_TIMEOUT_TX,
	.coalesce.tx_frames = RNPM_TX_PKT_POLL_BUDGET,
    .total_layer2_count    = RNPM_MAX_LAYER2_FILTERS,
#if NET_FEATURE_TCAM
    .total_tuple5_count = RNPM_MAX_TCAM_FILTERS,
#else
    .total_tuple5_count = RNPM_MAX_TUPLE5_FILTERS,
#endif
    .adapter_cnt = 4,
    .rss_type    = rnpm_rss_uv440,
    // .mac                    = rnpm_mac_8port_10G,
    .get_invariants = &rnpm_get_invariants_vu440,
    .mac_ops        = &mac_ops_vu440,
    .eeprom_ops     = NULL,
    .phy_ops        = &phy_ops_vu440,
    .mbx_ops        = &mbx_ops_generic,
};
