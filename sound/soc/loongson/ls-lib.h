#ifndef LS_LIB_H
#define LS_LIB_H

#include <linux/platform_device.h>
#include <linux/irqreturn.h>

/* PCM */

struct ls_pcm_dma_params {
	char *name;			/* stream identifier */
	u32 dcmd;			/* DMA descriptor dcmd field */
	volatile u32 *drcmr;		/* the DMA request channel to use */
	u64 dev_addr;			/* device physical address for DMA */
};

#endif
