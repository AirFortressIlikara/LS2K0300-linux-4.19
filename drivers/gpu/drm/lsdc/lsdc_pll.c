// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Loongson Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 */

/*
 * Authors:
 *      Sui Jingfeng <suijingfeng@loongson.cn>
 */
#include <drm/drm_print.h>

#include "lsdc_drv.h"
#include "lsdc_regs.h"


/* u64 */
struct ls7a1000_pixpll_bitmap {
	/* Byte 0 ~ Byte 3 */
	unsigned div_out      : 7;   /*  0 : 6     output clock divider  */
	unsigned reserved_1   : 14;  /*  7 : 20                          */
	unsigned loopc        : 9;   /* 21 : 29                          */
	unsigned reserved_2   : 2;   /* 30 : 31                          */

	/* Byte 4 ~ Byte 7 */
	unsigned div_ref      : 7;   /*  0 : 6     input clock divider   */
	unsigned locked       : 1;   /*  7         PLL locked flag       */
	unsigned sel_out      : 1;   /*  8         output clk selector   */
	unsigned reserved_3   : 2;   /*  9 : 10    reserved              */
	unsigned set_param    : 1;   /*  11        set pll param         */
	unsigned bypass       : 1;   /*  12                              */
	unsigned powerdown    : 1;   /*  13                              */
	unsigned reserved_4   : 18;  /*  14 : 31                         */
};

/* u128 */
struct ls2k1000_pixpll_bitmap {
	/* Byte 0 ~ Byte 3 */
	unsigned sel_out      :  1;  /*  0      select this PLL          */
	unsigned reserved_1   :  1;  /*  1                               */
	unsigned sw_adj_en    :  1;  /*  2      allow software adjust    */
	unsigned bypass       :  1;  /*  3      bypass L1 PLL            */
	unsigned reserved_2   :  3;  /*  4:6                             */
	unsigned lock_en      :  1;  /*  7      enable lock L1 PLL       */
	unsigned reserved_3   :  2;  /*  8:9                             */
	unsigned lock_check   :  2;  /* 10:11   precision check          */
	unsigned reserved_4   :  4;  /* 12:15                            */

	unsigned locked       :  1;  /* 16      Is L1 PLL locked, RO     */
	unsigned reserved_5   :  2;  /* 17:18                            */
	unsigned powerdown    :  1;  /* 19      powerdown the pll if set */
	unsigned reserved_6   :  6;  /* 20:25                            */
	unsigned div_ref      :  6;  /* 26:31   L1 Prescaler             */

	/* Byte 4 ~ Byte 7 */
	unsigned loopc        : 10;  /* 32:41   Clock Multiplier         */
	unsigned l1_div       :  6;  /* 42:47   not used                 */
	unsigned reserved_7   : 16;  /* 48:63                            */

	/* Byte 8 ~ Byte 15 */
	unsigned div_out      :  6;  /* 0 : 5   output clock divider     */
	unsigned reserved_8   : 26;  /* 6 : 31                           */
	unsigned reserved_9   : 32;  /* 70: 127                          */
};

/* u32 */
struct ls2k0500_pixpll_bitmap {
	/* Byte 0 ~ Byte 1 */
	unsigned sel_out      : 1;
	unsigned reserved_1   : 2;
	unsigned sw_adj_en    : 1;   /* allow software adjust              */
	unsigned bypass       : 1;   /* bypass L1 PLL                      */
	unsigned powerdown    : 1;   /* write 1 to powerdown the PLL       */
	unsigned reserved_2   : 1;
	unsigned locked       : 1;   /*  7     Is L1 PLL locked, read only */
	unsigned div_ref      : 6;   /*  8:13  ref clock divider           */
	unsigned reserved_3   : 2;   /* 14:15                              */
	/* Byte 2 ~ Byte 3 */
	unsigned loopc        : 8;   /* 16:23   Clock Multiplier           */
	unsigned div_out      : 6;   /* 24:29   output clock divider       */
	unsigned reserved_4   : 2;   /* 30:31                              */
};

/* u32 */
struct ls2k0300_pixpll_bitmap {
	unsigned sel_out      : 1;
	unsigned reserved_1   : 2;
	unsigned sw_adj_en    : 1;   /* allow software adjust              */
	unsigned bypass       : 1;   /* bypass L1 PLL                      */
	unsigned powerdown    : 1;   /* write 1 to powerdown the PLL       */
	unsigned reserved_2   : 1;
	unsigned locked       : 1;   /*  7     Is L1 PLL locked, read only */
	unsigned div_ref      : 7;   /*  8:14  ref clock divider           */
	unsigned loopc        : 9;   /* 15:23   Clock Multiplier           */
	unsigned div_out      : 7;   /* 24:30   output clock divider       */
	unsigned reserved_4   : 1;   /* 31                                 */
};

/*
 * NOTE: All loongson's cpu is little endian and  the length of
 * this structure is 8 bytes
 */
union lsdc_pixpll_bitmap {
	struct ls7a1000_pixpll_bitmap ls7a1000;
	struct ls2k1000_pixpll_bitmap ls2k1000;
	struct ls2k0500_pixpll_bitmap ls2k0500;
	struct ls2k0300_pixpll_bitmap ls2k0300;
	u32 dword[4];
	u64 qword[2];
};

/*
 * pixel clock to pll parameters translation table
 */
struct pixclk_to_pll_parm {
	unsigned int clock;   /* kHz */

	unsigned short width;
	unsigned short height;
	unsigned short vrefresh;

	unsigned short div_out;
	unsigned short loopc;
	unsigned short div_ref;
};

/*
 * Small cache to speed up pll parameter calculation
 */
static const struct pixclk_to_pll_parm pll_param_table[] = {
	{148500, 1920, 1080, 60,  11, 49,  3},   /* 1920x1080@60Hz */
						 /* 1920x1080@50Hz */
	{174500, 1920, 1080, 75,  17, 89,  3},   /* 1920x1080@75Hz */
	{181250, 2560, 1080, 75,  8, 58,  4},    /* 2560x1080@75Hz */
	{297000, 2560, 1080, 30,  8, 95,  4},    /* 3840x2160@30Hz */
	{301992, 1920, 1080, 100, 10, 151, 5},   /* 1920x1080@100Hz */
	{146250, 1680, 1050, 60,  16, 117, 5},   /* 1680x1050@60Hz */
	{135000, 1280, 1024, 75,  10, 54,  4},   /* 1280x1024@75Hz */
	{119000, 1680, 1050, 60,  20, 119, 5},   /* 1680x1050@60Hz */
	{108000, 1600, 900,  60,  15, 81,  5},   /* 1600x900@60Hz  */
						 /* 1280x1024@60Hz */
						 /* 1280x960@60Hz */
						 /* 1152x864@75Hz */

	{106500, 1440, 900,  60,  19, 81,  4},   /* 1440x900@60Hz */
	{88750,  1440, 900,  60,  16, 71,  5},   /* 1440x900@60Hz */
	{83500,  1280, 800,  60,  17, 71,  5},   /* 1280x800@60Hz */
	{71000,  1280, 800,  60,  20, 71,  5},   /* 1280x800@60Hz */

	{74250,  1280, 720,  60,  22, 49,  3},   /* 1280x720@60Hz */
						 /* 1280x720@50Hz */

	{78750,  1024, 768,  75,  16, 63,  5},   /* 1024x768@75Hz */
	{75000,  1024, 768,  70,  29, 87,  4},   /* 1024x768@70Hz */
	{65000,  1024, 768,  60,  20, 39,  3},   /* 1024x768@60Hz */

	{51200,  1024, 600,  60,  25, 64,  5},   /* 1024x600@60Hz */

	{57284,  832,  624,  75,  24, 55,  4},   /* 832x624@75Hz */
	{49500,  800,  600,  75,  40, 99,  5},   /* 800x600@75Hz */
	{50000,  800,  600,  72,  44, 88,  4},   /* 800x600@72Hz */
	{40000,  800,  600,  60,  30, 36,  3},   /* 800x600@60Hz */
	{36000,  800,  600,  56,  50, 72,  4},   /* 800x600@56Hz */
	{31500,  640,  480,  75,  40, 63,  5},   /* 640x480@75Hz */
						 /* 640x480@73Hz */

	{30240,  640,  480,  67,  62, 75,  4},   /* 640x480@67Hz */
	{27000,  720,  576,  50,  50, 54,  4},   /* 720x576@60Hz */
	{25175,  640,  480,  60,  85, 107, 5},   /* 640x480@60Hz */
	{25200,  640,  480,  60,  50, 63,  5},   /* 640x480@60Hz */
						 /* 720x480@60Hz */
};

static const struct pixclk_to_pll_parm ls2k0300_pll_param_table[] = {
	{148500, 1920, 1080, 60,  16,  99, 5},	/* 1920x1080@60Hz */
						/* 1920x1080@50Hz */
	{174500, 1920, 1080, 75,  11,  48, 3},	/* 1920x1080@75Hz */
	{181250, 2560, 1080, 75,   9,  68, 5},	/* 2560x1080@75Hz */
	{297000, 2560, 1080, 30,   8,  99, 5},	/* 3840x2160@30Hz */
	{301992, 1920, 1080, 100, 12, 151, 5},	/* 1920x1080@100Hz */
	{146250, 1680, 1050, 60,  16,  78, 4},	/* 1680x1050@60Hz */
	{135000, 1280, 1024, 75,  12,  54, 4},	/* 1280x1024@75Hz */
	{119000, 1680, 1050, 60,  24, 119, 5},	/* 1680x1050@60Hz */
	{108000, 1600, 900,  60,  14,  63, 5},	/* 1600x900@60Hz  */
						/* 1280x1024@60Hz */
						/* 1280x960@60Hz */
						/* 1152x864@75Hz */

	{106500, 1440, 900,  60,  16,  71, 5},	/* 1440x900@60Hz */
	{88750,  1440, 900,  60,  24,  71, 4},	/* 1440x900@60Hz */
	{83500,  1280, 800,  60,  23,  48, 3},	/* 1280x800@60Hz */
	{71000,  1280, 800,  60,  24,  71, 5},	/* 1280x800@60Hz */

	{74250,  1280, 720,  60,  32,  99, 5},	/* 1280x720@60Hz */
						/* 1280x720@50Hz */

	{78750,  1024, 768,  75,  24,  63, 4},	/* 1024x768@75Hz */
	{75000,  1024, 768,  70,  20,  50, 4},	/* 1024x768@70Hz */
	{65000,  1024, 768,  60,  24,  39, 3},	/* 1024x768@60Hz */

	{51200,  1024, 600,  60,  30,  64, 5},	/* 1024x600@60Hz */

	{57284,  832,  624,  75,  31,  74, 5},	/* 832x624@75Hz */
	{49500,  800,  600,  75,  32,  66, 5},	/* 800x600@75Hz */
	{50000,  800,  600,  72,  30,  50, 4},	/* 800x600@72Hz */
	{40000,  800,  600,  60,  36,  36, 3},	/* 800x600@60Hz */
	{36000,  800,  600,  56,  40,  36, 3},	/* 800x600@56Hz */
	{31500,  640,  480,  75,  48,  63, 5},	/* 640x480@75Hz */
						/* 640x480@73Hz */

	{30240,  640,  480,  67,  50,  63, 5},	/* 640x480@67Hz */
	{27000,  720,  576,  50,  56,  63, 5},	/* 720x576@60Hz */
	{25175,  640,  480,  60,  61,  64, 5},	/* 640x480@60Hz */
	{25200,  640,  480,  60,  60,  63, 5},	/* 640x480@60Hz */
						/* 720x480@60Hz */
};

static void lsdc_pixpll_init(struct lsdc_pll * const this, unsigned int id)
{
	this->mmio = ioremap(this->reg_base, this->reg_size);

	DRM_DEBUG_DRIVER("ioremap PIXPLL%u REG[%x, %u] to %llx\n",
		this->index, this->reg_base, this->reg_size, (u64)this->mmio);
}

/*
 * find a usable pll parameter (to generate pixel clock) from a static
 * local table, which save machine cycles to compute the pll parameter
 * everytime a modeset is triggered.
 *
 *  Return true if a parameter is found, otherwise return false.
 */
static bool lsdc_find_pll_param(struct lsdc_pll * const this, unsigned int clk)
{
	const unsigned int num = ARRAY_SIZE(pll_param_table);
	unsigned int i;

	for (i = 0; i < num; i++) {
		if (clk != pll_param_table[i].clock)
			continue;

		this->div_out = pll_param_table[i].div_out;
		this->loopc   = pll_param_table[i].loopc;
		this->div_ref = pll_param_table[i].div_ref;

		DRM_DEBUG_DRIVER("%s: hit: %u: div_out=%u, loopc=%u, div_ref=%u\n",
			__func__, i, this->div_out, this->loopc, this->div_ref);

		return true;
	}

	DRM_DEBUG_DRIVER("%s: %u: miss\n", __func__, clk);

	return false;
}

static bool ls2k0300_dc_find_pll_param(struct lsdc_pll * const this, unsigned int clk)
{
	const unsigned int num = ARRAY_SIZE(ls2k0300_pll_param_table);
	unsigned int i;

	for (i = 0; i < num; i++) {
		if (clk != ls2k0300_pll_param_table[i].clock)
			continue;

		this->div_out = ls2k0300_pll_param_table[i].div_out;
		this->loopc   = ls2k0300_pll_param_table[i].loopc;
		this->div_ref = ls2k0300_pll_param_table[i].div_ref;

		DRM_DEBUG_DRIVER("%s: hit: %u: div_out=%u, loopc=%u, div_ref=%u\n",
			__func__, i, this->div_out, this->loopc, this->div_ref);

		return true;
	}

	DRM_DEBUG_DRIVER("%s: %u: miss\n", __func__, clk);

	return false;
}

static int ls7a1000_config_pixel_pll(struct lsdc_pll * const this)
{
	u32 val;
	unsigned int counter = 0;
	void __iomem *reg = this->mmio;
	bool locked;


	/* clear sel_pll_out0 */
	val = readl(reg + 0x4);
	val &= ~(1 << 8);
	writel(val, reg + 0x4);

	/* set pll_pd */
	val = readl(reg + 0x4);
	val |= (1 << 13);
	writel(val, reg + 0x4);

	/* clear set_pll_param */
	val = readl(reg + 0x4);
	val &= ~(1 << 11);
	writel(val, reg + 0x4);

	/* clear old value & config new value */
	val = readl(reg + 0x04);
	val &= ~0x7F;

	val |= this->div_ref;   /* refc */
	writel(val, reg + 0x4);

	val = readl(reg);
	val &= ~(0x7f << 0);
	val |= (this->div_out << 0);   /* div */
	val &= ~(0x1ffUL << 21);
	val |= (this->loopc << 21);    /* loopc */
	writel(val, reg);

	/* set set_pll_param */
	val = readl(reg + 0x4);
	val |= (1 << 11);
	writel(val, reg + 0x4);

	/* clear pll_pd */
	val = readl(reg + 0x4);
	val &= ~(1 << 13);
	writel(val, reg + 0x4);

	/* wait pll lock */
	do {
		val = readl(reg + 0x4);
		locked = val & 0x80;
		counter++;
	} while (locked == false);

	DRM_DEBUG_DRIVER("%s: %u loop waited\n", __func__, counter);

	/* set sel_pll_out0 */
	val = readl(reg + 0x4);
	val |= (1UL << 8);
	writel(val, reg + 0x4);

	return 0;
}

/*
 * the PIX PLL be software configurable when SYS_CLKSEL[1:0] is 10b
 */
static int ls2k1000_config_pixel_pll(struct lsdc_pll * const this)
{
	void __iomem *reg = this->mmio;
	u64 val = readq(reg);
	bool locked;
	unsigned int counter = 0;

	val &= ~(1 << 0);    /* Bypass the PLL, using refclk directly */
	val |= (1 << 19);    /* powerdown the PLL */
	val &= ~(1 << 2);    /* don't use the software configure param */
	writeq(val, reg);


	val = (1 << 7) | (1L << 42) | (3 << 10);    /* allow L1 PLL locked */
	val |= (unsigned long)this->loopc << 32;    /* set loopc   */
	val |= (unsigned long)this->div_ref << 26;  /* set div_out */
	writeq(val, reg);
	writeq(this->div_out, reg + 8);             /* set div_out */


	val = readq(reg);
	val |= (1 << 2);     /* use the software configure param */
	val &= ~(1 << 19);   /* powerup the PLL */
	writeq(val, reg);

	/* wait pll setup and locked */
	do {
		val = readl(reg);
		locked = val & 0x10000;
		counter++;
	} while (locked == false);

	DRM_DEBUG_DRIVER("%s: %u loop waited\n", __func__, counter);

	val = readq(reg);
	val |= (1 << 0);    /* switch to the software configured pll */
	writeq(val, reg);

	return 0;
}

static int ls2k0500_config_pixel_pll(struct lsdc_pll * const this)
{
	void __iomem *reg = this->mmio;
	unsigned int counter = 0;
	unsigned long val;
	bool locked;

	/* set sel_pll_out0 0 */
	val = readl(reg);
	val &= ~(1UL << 0);
	writel(val, reg);

	/* bypass */
	val |= (1UL << 4);

	/* allow software setting the PLL */
	val |= (1UL << 3);
	writel(val, reg);

	/* pll powerdown */
	val = readl(reg);
	val |= (1UL << 5);
	writel(val, reg);

	val = (this->div_out << 24) |
	      (this->loopc << 16) |
	      (this->div_ref << 8);

	writel(val, reg);
	/* unbypass */
	val &= ~(1UL << 4);
	/* power up */
	val &= ~(1UL << 5);

	writel(val, reg);

	/* wait pll setup and locked */
	do {
		val = readl(reg);
		locked = val & 0x80;
		counter++;
	} while (locked == false);

	DRM_DEBUG_DRIVER("%u loop waited\n", counter);
	/* select PIX0 */
	writel((val | 1), reg);

	return 0;
}

static int ls2k0300_config_pixel_pll(struct lsdc_pll * const this)
{
	void __iomem *reg = this->mmio;
	unsigned int counter = 0;
	unsigned long val;
	bool locked;

	/* set sel_pll_out0 0 */
	val = readl(reg);
	val &= ~(1UL << 0);
	writel(val, reg);

	/* bypass */
	val |= (1UL << 4);

	/* allow software setting the PLL */
	val |= (1UL << 3);
	writel(val, reg);

	/* pll powerdown */
	val = readl(reg);
	val |= (1UL << 5);
	writel(val, reg);

	val = (this->div_out << 24) |
	      (this->loopc << 15) |
	      (this->div_ref << 8);

	writel(val, reg);
	/* unbypass */
	val &= ~(1UL << 4);
	/* power up */
	val &= ~(1UL << 5);

	writel(val, reg);

	/* wait pll setup and locked */
	do {
		val = readl(reg);
		locked = val & 0x80;
		counter++;
	} while (locked == false);

	DRM_DEBUG_DRIVER("%u loop waited\n", counter);
	/* select PIX0 */
	writel((val | 1), reg);

	return 0;
}

/* ls7a1000_print_clock - print clock related parameters
 *
 * clock_out = refclk / div_ref * loopc / divout
 *
 * also print the precision information
 */
static int ls7a1000_print_clock(struct lsdc_pll * const this,
				unsigned int pixclk)
{
	union lsdc_pixpll_bitmap pll_par;
	struct ls7a1000_pixpll_bitmap * const pllp = &pll_par.ls7a1000;
	unsigned int clk_out;

	pll_par.qword[0] = readq(this->mmio);

	clk_out = this->ref_clock / pllp->div_ref * pllp->loopc / this->div_out;

	DRM_DEBUG_DRIVER("\n");

	DRM_DEBUG_DRIVER("div_out=%u, loopc=%u, div_ref=%u\n",
			pllp->div_out, pllp->loopc, pllp->div_ref);

	DRM_DEBUG_DRIVER("PIXPLL%u working at %ukHz: desired=%ukHz, diff=%dkHz\n",
			this->index, clk_out, pixclk, clk_out - pixclk);

	DRM_DEBUG_DRIVER("locked: %s\n", pllp->locked ? "Yes" : "No");
	DRM_DEBUG_DRIVER("bypass: %s\n", pllp->bypass ? "Yes" : "No");
	DRM_DEBUG_DRIVER("powerdown: %s\n", pllp->powerdown ? "Yes" : "No");
	DRM_DEBUG_DRIVER("set_out: %s\n", pllp->sel_out ? "Yes" : "No");

	return 0;
}

static int ls2k0300_print_clock(struct lsdc_pll * const this,
				unsigned int pixclk)
{
	union lsdc_pixpll_bitmap pll_par;
	struct ls2k0300_pixpll_bitmap * const pllp = &pll_par.ls2k0300;
	unsigned int clk_out;

	pll_par.dword[0] = readl(this->mmio);

	DRM_DEBUG_DRIVER("\n");

	DRM_DEBUG_DRIVER("%s: div_out=%u, loopc=%u, div_ref=%u\n",
		__func__, pllp->div_out, pllp->loopc, pllp->div_ref);

	DRM_DEBUG_DRIVER("locked: %s\n", pllp->locked ? "Yes" : "No");
	DRM_DEBUG_DRIVER("bypass: %s\n", pllp->bypass ? "Yes" : "No");

	DRM_DEBUG_DRIVER("powerdown: %s\n", pllp->powerdown ? "Yes" : "No");
	DRM_DEBUG_DRIVER("set_out: %s\n", pllp->sel_out ? "Yes" : "No");
	DRM_DEBUG_DRIVER("sw adjust enable: %u\n", pllp->sw_adj_en);

	clk_out = this->ref_clock / pllp->div_ref * pllp->loopc / pllp->div_out;

	DRM_DEBUG_DRIVER("clock_out=%ukHz\n", clk_out);

	return 0;
}

static int ls2k0500_print_clock(struct lsdc_pll * const this,
				unsigned int pixclk)
{
	union lsdc_pixpll_bitmap pll_par;
	struct ls2k0500_pixpll_bitmap * const pllp = &pll_par.ls2k0500;
	unsigned int clk_out;

	pll_par.dword[0] = readl(this->mmio);

	DRM_DEBUG_DRIVER("\n");

	DRM_DEBUG_DRIVER("%s: div_out=%u, loopc=%u, div_ref=%u\n",
		__func__, pllp->div_out, pllp->loopc, pllp->div_ref);

	DRM_DEBUG_DRIVER("locked: %s\n", pllp->locked ? "Yes" : "No");
	DRM_DEBUG_DRIVER("bypass: %s\n", pllp->bypass ? "Yes" : "No");

	DRM_DEBUG_DRIVER("powerdown: %s\n", pllp->powerdown ? "Yes" : "No");
	DRM_DEBUG_DRIVER("set_out: %s\n", pllp->sel_out ? "Yes" : "No");
	DRM_DEBUG_DRIVER("sw adjust enable: %u\n", pllp->sw_adj_en);

	clk_out = this->ref_clock / pllp->div_ref * pllp->loopc / pllp->div_out;

	DRM_DEBUG_DRIVER("clock_out=%ukHz\n", clk_out);

	return 0;
}

/* lsdc_print_clock() - print clock related parameter only
 *
 * also calculate the precision
 */
static int ls2k1000_print_clock(struct lsdc_pll * const this,
				unsigned int pixclk)
{
	unsigned int clk_out;
	union lsdc_pixpll_bitmap pll_par;
	struct ls2k1000_pixpll_bitmap * const pllp = &pll_par.ls2k1000;

	pll_par.qword[0] = readq(this->mmio);
	pll_par.qword[1] = readq(this->mmio + 8);

	clk_out = this->ref_clock / pllp->div_ref * pllp->loopc / pllp->div_out;

	DRM_DEBUG_DRIVER("\n");

	DRM_DEBUG_DRIVER("div_out=%u, loopc=%u, div_ref=%u\n",
			pllp->div_out, pllp->loopc, pllp->div_ref);

	DRM_DEBUG_DRIVER("PIXPLL%u working at %ukHz: desired=%ukHz, diff=%dkHz\n",
			this->index, clk_out, pixclk, clk_out - pixclk);

	/* infomative */
	DRM_DEBUG_DRIVER("lock enabled: %s\n", pllp->lock_en ? "Yes" : "No");
	DRM_DEBUG_DRIVER("locked: %s\n", pllp->locked ? "Yes" : "No");
	DRM_DEBUG_DRIVER("bypass: %s\n", pllp->bypass ? "Yes" : "No");

	DRM_DEBUG_DRIVER("powerdown: %s\n", pllp->powerdown ? "Yes" : "No");
	DRM_DEBUG_DRIVER("set_out: %s\n", pllp->sel_out ? "Yes" : "No");
	DRM_DEBUG_DRIVER("sw adjust enable: %u\n", pllp->sw_adj_en);

	return 0;
}


/*
 *  clock_out = refclk / div_ref * loopc / div_out
 *
 *   refclk is fixed as 100MHz in ls7a1000, ls2k1000 and ls2k0500
 *   pixclock : kHz
 */
static int lsdc_cal_freq(struct lsdc_pll * const this, unsigned int pixclock)
{
	unsigned int div_out, loopc, div_ref;
	int a, b;
	int diff;
	int min = 1000;

	for (div_out = 6; div_out < 64; div_out++) {
		a = pixclock * div_out;

		for (div_ref = 3; div_ref < 6; div_ref++) {
			for (loopc = 6; loopc < 161; loopc++) {
				if (loopc < 12 * div_ref)
					continue;
				if (loopc > 32 * div_ref)
					continue;

				b = this->ref_clock * loopc / div_ref;

				diff = a - b;

				if (diff < 0)
					diff = -diff;

				if (diff < min) {
					min = diff;

					this->div_out = div_out;
					this->loopc = loopc;
					this->div_ref = div_ref;

					if (min == 0)
						return 1;
				}
			}
		}
	}

	return min < 1000;
}

static unsigned int lsdc_get_clock_rate(struct lsdc_pll * const this,
					struct lsdc_pll_core_values *pout)
{
	struct lsdc_device *ldev = to_lsdc(this->ddev);
	const struct lsdc_platform_desc *desc = ldev->desc;
	unsigned int out;
	union lsdc_pixpll_bitmap parms;

	if ((desc->chip == LSDC_CHIP_7A1000) ||
	    (desc->chip == LSDC_CHIP_7A2000)) {
		struct ls7a1000_pixpll_bitmap *obj = &parms.ls7a1000;

		parms.dword[0] = readl(this->mmio);
		parms.dword[1] = readl(this->mmio + 4);
		out = this->ref_clock / obj->div_ref * obj->loopc / obj->div_out;
		if (pout) {
			pout->div_ref = obj->div_ref;
			pout->loopc = obj->loopc;
			pout->div_out = obj->div_out;
		}
	} else if (desc->chip == LSDC_CHIP_2K1000) {
		struct ls2k1000_pixpll_bitmap *obj = &parms.ls2k1000;

		parms.dword[0] = readl(this->mmio);
		parms.dword[1] = readl(this->mmio + 4);
		parms.dword[2] = readl(this->mmio + 8);
		parms.dword[3] = readl(this->mmio + 12);
		out = this->ref_clock / obj->div_ref * obj->loopc / obj->div_out;
		if (pout) {
			pout->div_ref = obj->div_ref;
			pout->loopc = obj->loopc;
			pout->div_out = obj->div_out;
		}
	} else if (desc->chip == LSDC_CHIP_2K0500) {
		struct ls2k0500_pixpll_bitmap *obj;

		obj = &parms.ls2k0500;

		parms.dword[0] = readl(this->mmio);
		out = this->ref_clock / obj->div_ref * obj->loopc / obj->div_out;
		if (pout) {
			pout->div_ref = obj->div_ref;
			pout->loopc = obj->loopc;
			pout->div_out = obj->div_out;
		}
	} else if (desc->chip == LSDC_CHIP_2K0300) {
		struct ls2k0300_pixpll_bitmap *obj;

		obj = &parms.ls2k0300;

		parms.dword[0] = readl(this->mmio);
		out = this->ref_clock / obj->div_ref * obj->loopc / obj->div_out;
		if (pout) {
			pout->div_ref = obj->div_ref;
			pout->loopc = obj->loopc;
			pout->div_out = obj->div_out;
		}
	} else {
		DRM_ERROR("unknown chip, the driver need update\n");
		return 0;
	}

	return out;
}

static const struct lsdc_pixpll_funcs ls7a1000_pixpll_funcs = {
	.init = lsdc_pixpll_init,
	.find_pll_param = lsdc_find_pll_param,
	.compute_clock = lsdc_cal_freq,
	.config_pll = ls7a1000_config_pixel_pll,
	.print_clock = ls7a1000_print_clock,
	.get_clock_rate = lsdc_get_clock_rate,
};

static const struct lsdc_pixpll_funcs ls2k1000_pixpll_funcs = {
	.init = lsdc_pixpll_init,
	.find_pll_param = lsdc_find_pll_param,
	.compute_clock = lsdc_cal_freq,
	.config_pll = ls2k1000_config_pixel_pll,
	.print_clock = ls2k1000_print_clock,
	.get_clock_rate = lsdc_get_clock_rate,
};

static const struct lsdc_pixpll_funcs ls2k0500_pixpll_funcs = {
	.init = lsdc_pixpll_init,
	.find_pll_param = lsdc_find_pll_param,
	.compute_clock = lsdc_cal_freq,
	.config_pll = ls2k0500_config_pixel_pll,
	.print_clock = ls2k0500_print_clock,
	.get_clock_rate = lsdc_get_clock_rate,
};

static const struct lsdc_pixpll_funcs ls2k0300_pixpll_funcs = {
	.init = lsdc_pixpll_init,
	.find_pll_param = ls2k0300_dc_find_pll_param,
	.compute_clock = lsdc_cal_freq,
	.config_pll = ls2k0300_config_pixel_pll,
	.print_clock = ls2k0300_print_clock,
	.get_clock_rate = lsdc_get_clock_rate,
};

int lsdc_init_pix_pll(struct drm_device *ddev,
		      struct lsdc_pll *pixpll,
		      unsigned int index)
{
	struct lsdc_device *ldev = to_lsdc(ddev);
	const struct lsdc_platform_desc *desc = ldev->desc;

	pixpll->ddev = ddev;
	pixpll->index = index;
	pixpll->ref_clock = LSDC_PLL_REF_CLK;

	if ((desc->chip == LSDC_CHIP_7A1000) ||
	    (desc->chip == LSDC_CHIP_7A2000)) {

		pixpll->funcs = &ls7a1000_pixpll_funcs;

		if (index == 0)
			pixpll->reg_base = LS7A1000_CFG_REG_BASE +
					   LS7A1000_PIX_PLL0_REG;
		else if (index == 1)
			pixpll->reg_base = LS7A1000_CFG_REG_BASE +
					   LS7A1000_PIX_PLL1_REG;
		pixpll->reg_size = 8;
	} else if (desc->chip == LSDC_CHIP_2K1000) {

		pixpll->funcs = &ls2k1000_pixpll_funcs;

		if (index == 0)
			pixpll->reg_base = LS2K1000_CFG_REG_BASE +
					   LS2K1000_PIX_PLL0_REG;
		else if (index == 1)
			pixpll->reg_base = LS2K1000_CFG_REG_BASE +
					   LS2K1000_PIX_PLL1_REG;

		pixpll->reg_size = 16;
	} else if (desc->chip == LSDC_CHIP_2K0500) {

		pixpll->funcs = &ls2k0500_pixpll_funcs;

		if (index == 0)
			pixpll->reg_base = LS2K0500_CFG_REG_BASE +
					   LS2K0500_PIX_PLL0_REG;
		else if (index == 1)
			pixpll->reg_base = LS2K0500_CFG_REG_BASE +
					   LS2K0500_PIX_PLL1_REG;
		pixpll->reg_size = 4;
	} else if (desc->chip == LSDC_CHIP_2K0300) {
		/* 2k300 ref clk is 120MHz */
		pixpll->ref_clock = 120000;

		pixpll->funcs = &ls2k0300_pixpll_funcs;
		pixpll->reg_base = LS2K0300_CFG_REG_BASE +
				   LS2K0300_PIX_PLL0_REG;
		pixpll->reg_size = 4;
	} else {
		DRM_ERROR("unknown chip type\n");
		return -ENOENT;
	}

	/* call ->init only once */
	pixpll->funcs->init(pixpll, index);

	return 0;
}
