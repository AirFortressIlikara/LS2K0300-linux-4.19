/**
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LOONGSON_GUTS_H__
#define __LOONGSON_GUTS_H__

#include <linux/types.h>
#include <linux/io.h>

/**
 * Global Utility Registers.
 *
 * Not all registers defined in this structure are available on all chips, so
 * you are expected to know whether a given register actually exists on your
 * chip before you access it.
 *
 * Also, some registers are similar on different chips but have slightly
 * different names.  In these cases, one name is chosen to avoid extraneous
 * #ifdefs.
 */
struct scfg_guts {
	u32	svr;		/* Version Register */
	u8	res0[4];
	u16	feature;	/* Feature Register */
	u32	vendor;		/* Vendor Register */
	u8	res1[6];
	u32	id;
	u8	res2[0x3ff8 - 0x18];
	u32	chip;
} __attribute__ ((packed));

#endif
