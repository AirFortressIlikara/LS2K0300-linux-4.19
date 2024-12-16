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

#include <drm/drm_device.h>
#include <drm/drm_crtc.h>
#include <drm/drm_plane.h>
#include <drm/drm_atomic_helper.h>

#include <drm/drm_vblank.h>

#include "lsdc_drv.h"
#include "lsdc_regs.h"

static int lsdc_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	unsigned int val = lsdc_reg_read32(ldev, LSDC_INT_REG);

	if (index == 0)
		val |= INT_CRTC0_VS_EN;
	else if (index == 1)
		val |= INT_CRTC1_VS_EN;
	else
		DRM_ERROR("lsdc crtc is no more than 2\n");

	lsdc_reg_write32(ldev, LSDC_INT_REG, val);

	DRM_DEBUG_DRIVER("CRTC%u: vblank enabled\n", index);

	return 0;
}

static void lsdc_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	unsigned int val = lsdc_reg_read32(ldev, LSDC_INT_REG);

	if (index == 0)
		val &= ~INT_CRTC0_VS_EN;
	else if (index == 1)
		val &= ~INT_CRTC1_VS_EN;
	else
		DRM_ERROR("lsdc crtc is no more than 2\n");

	lsdc_reg_write32(ldev, LSDC_INT_REG, val);

	DRM_DEBUG_DRIVER("CRTC%u: vblank disabled\n", index);
}

static void lsdc_crtc_destroy(struct drm_crtc *crtc)
{
	DRM_DEBUG_DRIVER("CRTC%u destroyed\n", drm_crtc_index(crtc));

	drm_crtc_cleanup(crtc);
}

/*
 * CRTC got soft reset if bit 20 of CRTC*_CFG_REG from 1 to 0
 */
void lsdc_crtc_reset(struct drm_crtc *crtc)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	struct lsdc_display_pipe * const dispipe = drm_crtc_to_dispipe(crtc);
	u32 val = CFG_RESET_BIT | CFG_OUTPUT_ENABLE;

	if (ldev->enable_gamma)
		val |= CFG_GAMMAR_EN_BIT;

	if (index == 0) {
		DRM_DEBUG_DRIVER("Reset CRTC0\n");
		if (dispipe->hw_clone) {
			val |= CFG_PANEL_SWITCH;
			DRM_DEBUG_DRIVER("display pipe 0 is clone to display pipe 1\n");
		}
		lsdc_reg_write32(ldev, LSDC_CRTC0_CFG_REG, val);
	} else if (index == 1) {
		DRM_DEBUG_DRIVER("Reset CRTC1\n");
		if (dispipe->hw_clone) {
			val |= CFG_PANEL_SWITCH;
			DRM_DEBUG_DRIVER("display pipe 1 is clone to display pipe 0\n");
		}
		lsdc_reg_write32(ldev, LSDC_CRTC1_CFG_REG, val);
	} else
		DRM_ERROR("lsdc crtc is no more than 2\n");

	drm_atomic_helper_crtc_reset(crtc);
}

/*
 * These provide the minimum set of functions required to handle a CRTC
 * Each driver is responsible for filling out this structure at startup time
 *
 * The drm_crtc_funcs structure is the central CRTC management structure
 * in the DRM. Each CRTC controls one or more connectors
 */
static const struct drm_crtc_funcs lsdc_crtc_funcs = {
	.reset = lsdc_crtc_reset,
	.gamma_set = drm_atomic_helper_legacy_gamma_set,
	.destroy = lsdc_crtc_destroy,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = lsdc_crtc_enable_vblank,
	.disable_vblank = lsdc_crtc_disable_vblank,
};

static enum drm_mode_status
lsdc_crtc_mode_valid(struct drm_crtc *crtc,
		     const struct drm_display_mode *mode)
{
	if (mode->hdisplay % 16)
		return MODE_BAD;

	return MODE_OK;
}

/*
 * @lsdc_crtc_mode_set_nofb:
 *
 * This callback is used to update the display mode of a CRTC without
 * changing anything of the primary plane configuration. This fits the
 * requirement of atomic and hence is used by the atomic helpers.
 * It is also used by the transitional plane helpers to implement a
 * @mode_set hook in drm_helper_crtc_mode_set().
 *
 * Note that the display pipe is completely off when this function is called.
 * Atomic drivers which need hardware to be running before they program the
 * new display mode (e.g. because they implement runtime PM) should not use
 * this hook.
 *
 * This is because the helper library calls this hook only once per mode change
 * and not every time the display pipeline is suspended using either DPMS or
 * the new "ACTIVE" property. Which means register values set in this callback
 * might get reset when the CRTC is suspended, but not restored. Such drivers
 * should instead move all their CRTC setup into the @atomic_enable callback.
 *
 * This callback is optional.
 */
static void lsdc_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	unsigned int index = drm_crtc_index(crtc);
	struct lsdc_display_pipe *dispipe = drm_crtc_to_dispipe(crtc);
	struct lsdc_pll * const pixpll = &dispipe->pixpll;
	unsigned int pixclock = mode->clock;
	u32 val;

	if (index == 0) {
		/* CRTC 0 */
		DRM_DEBUG_DRIVER("CRTC0 mode set no fb\n");

		lsdc_reg_write32(ldev, LSDC_CRTC0_PANCFG_REG, 0x80001311);
		lsdc_reg_write32(ldev, LSDC_CRTC0_PANTIM_REG, 0);

		lsdc_reg_write32(ldev, LSDC_CRTC0_FB_ORIGIN_REG, 0);

		/* hack 256 bytes align issue */
		val = (mode->crtc_hdisplay + 63) & ~63;
		val |= (mode->crtc_htotal << 16);
		lsdc_reg_write32(ldev, LSDC_CRTC0_HDISPLAY_REG, val);

		val = (mode->crtc_vtotal << 16) | mode->crtc_vdisplay;
		lsdc_reg_write32(ldev, LSDC_CRTC0_VDISPLAY_REG, val);

		/* HSync */
		val = CFG_EN_HSYNC;

		/* bits 26:16 hsync end */
		val |= mode->crtc_hsync_end << 16;
		/* bits 10:0 hsync start */
		val |= mode->crtc_hsync_start;

		if (mode->flags & DRM_MODE_FLAG_NHSYNC) {
			val |= CFG_INV_HSYNC;
			DRM_DEBUG_DRIVER("CRTC0 HSync Inverted\n");
		}

		lsdc_reg_write32(ldev, LSDC_CRTC0_HSYNC_REG, val);

		/* VSync */
		val = CFG_EN_VSYNC;

		/* bits 26:16 vsync end */
		val |= mode->crtc_vsync_end << 16;
		/* bits 10:0 vsync start */
		val |= mode->crtc_vsync_start;

		if (mode->flags & DRM_MODE_FLAG_NVSYNC) {
			val |= CFG_INV_VSYNC;
			DRM_DEBUG_DRIVER("CRTC0 VSync Inverted\n");
		}

		lsdc_reg_write32(ldev, LSDC_CRTC0_VSYNC_REG, val);
	} else if (index == 1) {
		/* CRTC 1 */
		DRM_DEBUG_DRIVER("CRTC1 mode set no fb\n");

		lsdc_reg_write32(ldev, LSDC_CRTC1_PANCFG_REG, 0x80001311);
		lsdc_reg_write32(ldev, LSDC_CRTC1_PANTIM_REG, 0);

		lsdc_reg_write32(ldev, LSDC_CRTC1_FB_ORIGIN_REG, 0);

		/* hack 256 byte align issue */
		val = (mode->crtc_hdisplay + 63) & ~63;
		val |= (mode->crtc_htotal << 16);

		lsdc_reg_write32(ldev, LSDC_CRTC1_HDISPLAY_REG, val);

		lsdc_reg_write32(ldev, LSDC_CRTC1_VDISPLAY_REG,
			(mode->crtc_vtotal << 16) | mode->crtc_vdisplay);

		/* HSYNC */
		val = CFG_EN_HSYNC;

		/* bits 26:16 hsync end */
		val |= mode->crtc_hsync_end << 16;
		/* bits 10:0 hsync start */
		val |= mode->crtc_hsync_start;

		if (mode->flags & DRM_MODE_FLAG_NHSYNC) {
			val |= CFG_INV_HSYNC;
			DRM_DEBUG_DRIVER("CRTC1 HSync Inverted\n");
		}

		lsdc_reg_write32(ldev, LSDC_CRTC1_HSYNC_REG, val);

		/* VSYNC */
		val = CFG_EN_VSYNC;

		/* bits 26:16 vsync end */
		val |= mode->crtc_vsync_end << 16;
		/* bits 10:0 vsync start */
		val |= mode->crtc_vsync_start;

		if (mode->flags & DRM_MODE_FLAG_NVSYNC) {
			val |= CFG_INV_VSYNC;
			DRM_DEBUG_DRIVER("CRTC1 VSync Inverted\n");
		}

		lsdc_reg_write32(ldev, LSDC_CRTC1_VSYNC_REG, val);
	} else
		DRM_DEBUG_DRIVER("CRTC is no more than 2\n");

	/* config the pixel pll */

	if (pixpll->funcs->find_pll_param(pixpll, pixclock) == false)
		pixpll->funcs->compute_clock(pixpll, pixclock);

	pixpll->funcs->config_pll(pixpll);
}

static void lsdc_crtc_no_vblank_atomic_enable(struct drm_crtc *crtc,
					      struct drm_crtc_state *old_state)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	u32 val;

	if (index == 0) {
		val = lsdc_reg_read32(ldev, LSDC_CRTC0_CFG_REG);
		val |= CFG_OUTPUT_ENABLE;
		lsdc_reg_write32(ldev, LSDC_CRTC0_CFG_REG, val);
	} else if (index == 1) {
		val = lsdc_reg_read32(ldev, LSDC_CRTC1_CFG_REG);
		val |= CFG_OUTPUT_ENABLE;
		lsdc_reg_write32(ldev, LSDC_CRTC1_CFG_REG, val);
	} else
		DRM_ERROR("CRTC is no more than 2\n");

	DRM_DEBUG_DRIVER("CRTC%u enabled\n", index);
}

static void lsdc_crtc_no_vblank_atomic_disable(struct drm_crtc *crtc,
					       struct drm_crtc_state *old_state)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	u32 val;

	if (index == 0) {
		val = lsdc_reg_read32(ldev, LSDC_CRTC0_CFG_REG);
		val &= ~CFG_OUTPUT_ENABLE;
		lsdc_reg_write32(ldev, LSDC_CRTC0_CFG_REG, val);
	} else if (index == 1) {
		val = lsdc_reg_read32(ldev, LSDC_CRTC1_CFG_REG);
		val &= ~CFG_OUTPUT_ENABLE;
		lsdc_reg_write32(ldev, LSDC_CRTC1_CFG_REG, val);
	} else
		DRM_ERROR("CRTC is no more than 2\n");

	DRM_DEBUG_DRIVER("CRTC%u disabled\n", index);
}

static void lsdc_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_state)
{
	lsdc_crtc_no_vblank_atomic_enable(crtc, old_state);

	drm_crtc_vblank_on(crtc);
}

static void lsdc_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_state)
{
	drm_crtc_vblank_off(crtc);

	lsdc_crtc_no_vblank_atomic_disable(crtc, old_state);
}

static void lsdc_crtc_update_clut(struct drm_crtc *crtc)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	struct drm_color_lut *lut;
	unsigned int i;

	if (!ldev->enable_gamma)
		return;

	if (!crtc->state->color_mgmt_changed || !crtc->state->gamma_lut)
		return;

	lut = (struct drm_color_lut *)crtc->state->gamma_lut->data;

	if (index == 0)
		lsdc_reg_write32(ldev, LSDC_CRTC0_GAMMA_INDEX_REG, 0);
	else if (index == 1)
		lsdc_reg_write32(ldev, LSDC_CRTC1_GAMMA_INDEX_REG, 0);

	for (i = 0; i < LSDC_CLUT_SIZE; i++) {
		u32 val = ((lut->red << 8) & 0xff0000) |
			  (lut->green & 0xff00) |
			  (lut->blue >> 8) ;

		if (index == 0)
			lsdc_reg_write32(ldev, LSDC_CRTC0_GAMMA_DATA_REG, val);
		else if (index == 1)
			lsdc_reg_write32(ldev, LSDC_CRTC1_GAMMA_DATA_REG, val);

		lut++;
	}
}

static void lsdc_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_crtc_state *old_crtc_state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	lsdc_crtc_update_clut(crtc);

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static int lsdc_crtc_no_vblank_atomic_check(struct drm_crtc *crtc,
					    struct drm_crtc_state *state)
{
	state->no_vblank = true;
	return 0;
}

static const struct drm_crtc_helper_funcs lsdc_crtc_no_vblank_helper_funcs = {
	.mode_valid = lsdc_crtc_mode_valid,
	.mode_set_nofb = lsdc_crtc_mode_set_nofb,
	.atomic_check = lsdc_crtc_no_vblank_atomic_check,
	.atomic_enable = lsdc_crtc_no_vblank_atomic_enable,
	.atomic_disable = lsdc_crtc_no_vblank_atomic_disable,
	.atomic_flush = lsdc_crtc_atomic_flush,
};

static const struct drm_crtc_helper_funcs lsdc_crtc_helper_funcs = {
	.mode_valid = lsdc_crtc_mode_valid,
	.mode_set_nofb = lsdc_crtc_mode_set_nofb,
	.atomic_enable = lsdc_crtc_atomic_enable,
	.atomic_disable = lsdc_crtc_atomic_disable,
	.atomic_flush = lsdc_crtc_atomic_flush,
};

/*
 * lsdc_crtc_init
 *
 * @ldev: point to the lsdc_device structure
 *
 * Init CRTC
 */
int lsdc_crtc_init(struct drm_device *ddev,
		   unsigned int index,
		   struct drm_crtc *crtc,
		   struct drm_plane *primary_plane,
		   struct drm_plane *cursor_plane)
{
	struct lsdc_device *ldev = to_lsdc(ddev);
	int ret;

	ret = drm_crtc_init_with_planes(ddev,
					crtc,
					primary_plane,
					cursor_plane,
					&lsdc_crtc_funcs,
					NULL);
	if (ret)
		DRM_ERROR("crtc init with planes failed\n");

	ret = drm_mode_crtc_set_gamma_size(crtc, LSDC_CLUT_SIZE);
	if (ret)
		DRM_WARN("set the gamma table size failed\n");

	drm_crtc_enable_color_mgmt(crtc, 0, false, LSDC_CLUT_SIZE);

	if (ldev->no_vblank)
		drm_crtc_helper_add(crtc, &lsdc_crtc_no_vblank_helper_funcs);
	else
		drm_crtc_helper_add(crtc, &lsdc_crtc_helper_funcs);

	return ret;
}
