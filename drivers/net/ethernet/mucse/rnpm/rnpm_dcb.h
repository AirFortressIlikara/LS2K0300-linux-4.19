#ifndef __RNPM_DCB_H__
#define __RNPM_DCB_H__
#include "rnpm.h"

enum rnpm_pause_low_thrsh {
	RNPM_PAUSE_4_SLOT_TIME = 0,
	RNPM_PAUSE_28_SLOT_TIME,
	RNPM_PAUSE_36_SLOT_TIME,
	RNPM_PAUSE_144_SLOT_TIME,
	RNPM_PAUSE_256_SLOT_TIME,
};
/* Rx Flow Ctrl */
/* Receive Flow Control Enable */
#define RNPM_RX_RFE                   BIT(0)
/* Unicast Pause Packet Detect */
#define RNPM_UP                       BIT(1)
/*  Priority Based Flow Control Enable. */
#define RNPM_PFCE                     BIT(8)

/*Tx Flow Ctrl */
#define RNPM_TX_FCB                   BIT(0) /* Tx Flow Control Busy. */
#define RNPM_TX_TFE                   BIT(1) /* Transmit Flow Control Enable.*/
#define RNPM_TX_PLT                   GENMASK(6, 4) /* Pause Low Threshold. */
#define RNPM_DZPQ                     BIT(7) /*Disable Zero-Quanta Pause.*/
#define RNPM_PT                       GENMASK(31, 16) /* Pause Time. */

#define RNPM_DEFAULT_PAUSE_TIME      (0x100) /* */
#define RNPM_FC_TX_PLTH_OFFSET       (4)   /* Pause Low Threshold */
#define RNPM_FC_TX_PT_OFFSET         (16)  /* Pause Time */

#define RNPM_DCB_MAX_TCS_NUM          (4)
#define RNPM_DCB_MAX_PFC_NUM          (4)

struct rnpm_adapter;
int rnpm_dcb_init(struct net_device *dev,
		 struct rnpm_adapter *adapter);
#endif
