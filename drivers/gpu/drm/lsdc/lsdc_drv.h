/* SPDX-License-Identifier: GPL-2.0+ */
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


#ifndef __LSDC_DRV_H__
#define __LSDC_DRV_H__

#include <drm/drm_device.h>
#include <drm/drm_crtc.h>
#include <drm/drm_plane.h>
#include <drm/drm_encoder.h>
#include <drm/drm_connector.h>
#include <drm/drm_bridge.h>

#define LSDC_NUM_CRTC           2
#define LSDC_CLUT_SIZE          256

enum loongson_dc_family {
	LSDC_CHIP_UNKNOWN = 0,
	LSDC_CHIP_2K1000 = 1, /* 2-Core SoC, 64-bit */
	LSDC_CHIP_7A1000 = 2, /* North bridges */
	LSDC_CHIP_7A1000_PLUS = 3, /* bandwidth improved */
	LSDC_CHIP_2K0500 = 4, /* Reduced version of 2k1000, single core */
	LSDC_CHIP_7A2000 = 5, /* North bridges */
	LSDC_CHIP_2K0300 = 6,
	LSDC_CHIP_LAST,
};

/* SoC parameters and platform specific information */
struct lsdc_platform_desc {
	enum loongson_dc_family chip;
	uint32_t num_of_crtc;
	uint32_t max_pixel_clk;
	uint32_t max_width;
	uint32_t max_height;
	uint32_t num_of_hw_cursor;
	uint32_t hw_cursor_w;
	uint32_t hw_cursor_h;
	bool has_builtin_i2c;
	bool has_vram;
	bool broken_gamma;
};

struct lsdc_connector {
	struct drm_connector base;
	struct i2c_adapter *ddc;

	unsigned char edid_data[EDID_LENGTH];
	bool has_edid;

	struct display_timings *disp_tim;
	bool has_disp_tim;

	bool always_connected;
};

#define to_lsdc_connector(x)        \
		container_of(x, struct lsdc_connector, base)

/*
 * PLL structure
 *
 *               L1       Fref                      Fvco     L2
 * refclk   +-----------+      +------------------+      +---------+   outclk
 * -------> | Prescaler | ---> | Clock Multiplier | ---> | divider | --------->
 *    |     +-----------+      +------------------+      +---------+     ^
 *    |           ^                      ^                    ^          |
 *    |           |                      |                    |          |
 *    |           |                      |                    |          |
 *    |        div_ref                 loopc               div_out       |
 *    |                                                                  |
 *    +-------------------------- sel_out -------------------------------+
 *
 *
 *  outclk = refclk / div_ref * loopc / div_out;
 *
 * PLL working requirement:
 *
 *  1) 20 MHz <= refclk / div_ref <= 40Mhz
 *  2) 1.2 GHz <= refclk /div_out * loopc <= 3.2 Ghz
 *
 */
struct lsdc_pll;

struct lsdc_pll_core_values {
	unsigned int div_ref;
	unsigned int loopc;
	unsigned int div_out;
};

struct lsdc_pixpll_funcs {
	void (*init)(struct lsdc_pll * const this, u32 id);

	bool (*find_pll_param)(struct lsdc_pll * const this, unsigned int clk);
	int (*compute_clock)(struct lsdc_pll * const this, unsigned int clk);
	int (*config_pll)(struct lsdc_pll * const this);

	/* dump all pll related paramenter, debug purpose */
	int (*print_clock)(struct lsdc_pll * const this, unsigned int pixclk);
	unsigned int (*get_clock_rate)(struct lsdc_pll * const this,
				       struct lsdc_pll_core_values *pout);
};

struct lsdc_pll {
	const struct lsdc_pixpll_funcs *funcs;

	struct drm_device *ddev;

	unsigned int ref_clock; /* kHz */

	u32 reg_base;
	u32 reg_size;
	void __iomem *mmio;

	unsigned short index;
	unsigned short div_out;
	unsigned short loopc;
	unsigned short div_ref;
};

/*
 * struct lsdc_display_pipe - Abstraction of hardware display pipeline.
 * @crtc: CRTC control structure
 * @primary: Framebuffer plane control structure
 * @cursor: Framebuffer plane control structure
 * @encoder: Encoder control structure
 * @pixpll: Pll control structure
 * @lconn: connector control structure
 * @output: point to output control structure of the display pipe
 * @index: the index corresponding to the hardware display pipe
 * @hw_clone: true if this display pipe working at hardware cloning mode.
 * @available: is this display pipe is available on the motherboard, The
 *  downstream mother board manufacturer may use only one of them.
 *  For example, LEMOTE LX-6901 board has only one VGA output.
 */
struct lsdc_display_pipe {
	struct drm_crtc crtc;
	struct drm_plane primary;
	struct drm_plane cursor;
	struct lsdc_pll pixpll;
	struct drm_encoder encoder;
	/* There is only a 1:1 mapping of encoders and connectors for lsdc */
	struct lsdc_connector *lconn;
	int index;
	bool hw_clone;
	bool available;
};

static inline struct lsdc_display_pipe *
drm_crtc_to_dispipe(struct drm_crtc *crtc)
{
	return container_of(crtc, struct lsdc_display_pipe, crtc);
}

static inline struct lsdc_display_pipe *
lsdc_cursor_to_dispipe(struct drm_plane *plane)
{
	return container_of(plane, struct lsdc_display_pipe, cursor);
}

static inline struct lsdc_display_pipe *
lsdc_primary_to_dispipe(struct drm_plane *plane)
{
	return container_of(plane, struct lsdc_display_pipe, primary);
}

struct lsdc_device {
	struct drm_device ddev;

	void __iomem *reg_base;
	void __iomem *vram;
	resource_size_t vram_base;
	resource_size_t vram_size;

	unsigned int num_output;
	struct lsdc_display_pipe dispipe[LSDC_NUM_CRTC];

	/* platform specific data */
	const struct lsdc_platform_desc *desc;

	/* @reglock: protects concurrent register access */
	spinlock_t reglock;

	/*
	 * @dirty_lock: Serializes framebuffer flushing
	 */
	struct mutex dirty_lock;

	int irq;
	u32 irq_status;

	/*
	 * @shadowfb: is shadow fb layer is enabled.
	 */
	bool shadowfb;

	/*
	 * @ddc0: false if ddc0 is disabled.
	 */
	bool ddc0;
	/*
	 * @ddc1: false if ddc1 is disabled.
	 */
	bool ddc1;
	/*
	 * @enable_gamma: true if hardware gamma is desired.
	 */
	bool enable_gamma;

	/*
	 * @no_vblank: true if the device don't support vsync interrupt
	 */
	bool no_vblank;

	/*
	 * @has_of_graph: true if there are OF graph in the DT
	 */
	bool has_of_graph;
};

static inline struct lsdc_device *to_lsdc(struct drm_device *drm)
{
	return container_of(drm, struct lsdc_device, ddev);
}

int lsdc_mode_config_init(struct lsdc_device *ldev);
void lsdc_mode_config_fini(struct lsdc_device *ldev);

void lsdc_fb_dirty_update_impl(void __iomem *dst,
			       void *vaddr,
			       struct drm_framebuffer * const fb,
			       struct drm_clip_rect * const clip);

int lsdc_crtc_init(struct drm_device *ddev,
		   unsigned int index,
		   struct drm_crtc *crtc,
		   struct drm_plane *fb_plane,
		   struct drm_plane *cursor_plane);

int lsdc_plane_init(struct drm_device *ddev,
		    struct drm_plane *plane,
		    enum drm_plane_type type,
		    unsigned int index);

int lsdc_init_pix_pll(struct drm_device *ddev,
		      struct lsdc_pll *pixpll,
		      unsigned int index);

int lsdc_encoder_init(struct drm_device *ddev,
		      struct drm_encoder *encoder,
		      unsigned int index);

struct lsdc_connector *
lsdc_create_connector(struct drm_device *ddev, unsigned int index);

bool of_graph_is_present(const struct device_node *node);
int lsdc_attach_bridges(struct lsdc_device *ldev, unsigned int num_crtc);

extern int lsdc_shadowfb;

extern struct drm_driver lsdc_drm_driver;

extern struct platform_driver lsdc_platform_driver;

#ifdef CONFIG_DRM_LSDC_PCI_DRIVER
extern struct pci_driver lsdc_pci_driver;
#endif

#endif
