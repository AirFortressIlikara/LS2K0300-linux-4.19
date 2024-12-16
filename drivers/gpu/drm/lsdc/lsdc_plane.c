// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Loongson Corporation
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
 *	Sui Jingfeng <suijingfeng@loongson.cn>
 */

#include <drm/drm_atomic.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "lsdc_drv.h"
#include "lsdc_regs.h"

#define LSDC_CURS_MIN_SIZE                      1
#define LSDC_CURS_MAX_SIZE                      64

static const uint32_t lsdc_cursor_formats[] = {
	DRM_FORMAT_ARGB8888,
};

static const uint32_t lsdc_primary_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static int lsdc_plane_get_default_zpos(enum drm_plane_type type)
{
	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		return 0;
	case DRM_PLANE_TYPE_OVERLAY:
		return 1;
	case DRM_PLANE_TYPE_CURSOR:
		return 7;
	}
	return 0;
}

static int lsdc_cursor_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct drm_crtc *crtc = state->crtc;
	struct drm_crtc_state *crtc_state;
	int dst_x, dst_y;
	int src_w, src_h;

	/* no need for further checks if the plane is being disabled */
	if (!crtc || !state->fb)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state->state, crtc);

	dst_x = state->crtc_x;
	dst_y = state->crtc_y;

	/* src_x are in 16.16 format */
	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;

	if (src_w < LSDC_CURS_MIN_SIZE ||
	    src_h < LSDC_CURS_MIN_SIZE ||
	    src_w > LSDC_CURS_MAX_SIZE ||
	    src_h > LSDC_CURS_MAX_SIZE) {
		DRM_ERROR("Invalid cursor size (%dx%d)\n", src_w, src_h);
		return -EINVAL;
	}

	return 0;
}

static void lsdc_cursor_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *oldstate)
{
	struct lsdc_device *ldev = to_lsdc(plane->dev);
	struct drm_plane_state *state = plane->state;
	struct drm_crtc *crtc = state->crtc;
	struct drm_gem_cma_object *cursor_obj;
	int dst_x, dst_y;
	unsigned int width, height;

	u32 cursor_ctrl;

	if (!crtc || !state->fb)
		return;

	/* Left position of visible portion of plane on crtc */
	dst_x = state->crtc_x;
	/* Upper position of visible portion of plane on crtc */
	dst_y = state->crtc_y;

	if (dst_x < 0)
		dst_x = 0;

	if (dst_y < 0)
		dst_y = 0;

	cursor_obj = drm_fb_cma_get_gem_obj(state->fb, 0);

	width = state->src_w >> 16;
	height = state->src_h >> 16;

	lsdc_reg_write32(ldev, LSDC_CURSOR0_ADDR_LO_REG,
			 cursor_obj->paddr);
	lsdc_reg_write32(ldev, LSDC_CURSOR0_ADDR_HI_REG,
			 cursor_obj->paddr);

	/* update the position of the cursor */
	lsdc_reg_write32(ldev, LSDC_CURSOR0_POSITION_REG,
			 (dst_y << 16) | dst_x);

	/* cursor format */
	cursor_ctrl = CURSOR_FORMAT_ARGB8888;

	/* Update the location of the cursor */
	/* Place the hardware cursor on the top of CRTC-1 */
	if (drm_crtc_index(crtc))
		cursor_ctrl |= CURSOR_LOCATION_BIT;

	lsdc_reg_write32(ldev, LSDC_CURSOR0_CFG_REG, cursor_ctrl);
}

static void lsdc_cursor_atomic_disable(struct drm_plane *plane,
				       struct drm_plane_state *oldstate)
{
	struct lsdc_device *ldev = to_lsdc(plane->dev);

	if (!oldstate->crtc) {
		DRM_DEBUG_DRIVER("drm plane:%d not enabled\n",
				 plane->base.id);
		return;
	}

	lsdc_reg_write32(ldev, LSDC_CURSOR0_CFG_REG, 0);

	DRM_DEBUG_KMS("%s disabled\n", plane->name);
}

/*
 * LS7A1000, LS2K1000, LS2K0500
 */
static const struct drm_plane_helper_funcs lsdc_cursor_plane_hpfns = {
	.atomic_check = lsdc_cursor_atomic_check,
	.atomic_update = lsdc_cursor_atomic_update,
	.atomic_disable = lsdc_cursor_atomic_disable,
};

static void ls7a2000_cursor_atomic_update(struct drm_plane *plane,
					  struct drm_plane_state *oldstate)
{
	struct lsdc_device *ldev = to_lsdc(plane->dev);
	struct drm_plane_state *state = plane->state;
	struct drm_crtc *crtc = state->crtc;
	struct drm_gem_cma_object *cursor_obj;
	int dst_x, dst_y;
	unsigned int width, height;
	int index;
	u32 cursor_ctrl;

	if (!crtc || !state->fb)
		return;

	index = drm_crtc_index(crtc);

	/* Left position of visible portion of plane on crtc */
	dst_x = state->crtc_x;
	/* Upper position of visible portion of plane on crtc */
	dst_y = state->crtc_y;

	if (dst_x < 0)
		dst_x = 0;

	if (dst_y < 0)
		dst_y = 0;

	cursor_obj = drm_fb_cma_get_gem_obj(state->fb, 0);

	width = state->src_w >> 16;
	height = state->src_h >> 16;

	if (index == 0) {
		/* update the dma address of the cursor-0 */
		lsdc_reg_write32(ldev, LSDC_CURSOR0_ADDR_LO_REG,
				 cursor_obj->paddr & 0xFFFFFFFF);
		lsdc_reg_write32(ldev, LSDC_CURSOR0_ADDR_HI_REG,
				 cursor_obj->paddr >> 32);

		/* update the position of the cursor-0 */
		lsdc_reg_write32(ldev, LSDC_CURSOR0_POSITION_REG,
				 (dst_y << 16) | dst_x);

		/* update the format, mode and location of the cursor-0 */
		cursor_ctrl = CURSOR_FORMAT_ARGB8888 | CURSOR_SIZE_64X64_BIT;
		/* Assign cursor-0 to CRTC-0 */
		cursor_ctrl &= ~CURSOR_LOCATION_BIT;

		lsdc_reg_write32(ldev, LSDC_CURSOR0_CFG_REG, cursor_ctrl);

	} else if (index == 1) {
		/* update the dma addr of the cursor-1 */
		lsdc_reg_write32(ldev, LSDC_CURSOR1_ADDR_LO_REG,
				 cursor_obj->paddr & 0xFFFFFFFF);
		lsdc_reg_write32(ldev, LSDC_CURSOR1_ADDR_HI_REG,
				 cursor_obj->paddr >> 32);

		/* update the position of the cursor */
		lsdc_reg_write32(ldev, LSDC_CURSOR1_POSITION_REG,
				 (dst_y << 16) | dst_x);

		/* update the format, mode and location of the cursor-0 */
		cursor_ctrl = CURSOR_FORMAT_ARGB8888 | CURSOR_SIZE_64X64_BIT;
		/* Assign cursor-1 to CRTC-1 */
		cursor_ctrl |= CURSOR_LOCATION_BIT;

		lsdc_reg_write32(ldev, LSDC_CURSOR1_CFG_REG, cursor_ctrl);
	}
}

static void ls7a2000_cursor_atomic_disable(struct drm_plane *plane,
					   struct drm_plane_state *oldstate)
{
	struct lsdc_device *ldev = to_lsdc(plane->dev);
	struct drm_crtc *crtc = oldstate->crtc;
	int index = drm_crtc_index(crtc);
	u32 val;

	if (!crtc) {
		DRM_DEBUG_DRIVER("plane-%d is disabled\n", plane->base.id);
		return;
	}

	if (index == 0)
		val = lsdc_reg_read32(ldev, LSDC_CURSOR0_CFG_REG);
	else if (index == 1)
		val = lsdc_reg_read32(ldev, LSDC_CURSOR1_CFG_REG);

	val &= ~CURSOR_ENABLE_MASK;

	if (index == 0)
		lsdc_reg_write32(ldev, LSDC_CURSOR0_CFG_REG, val);
	else if (index == 1)
		lsdc_reg_write32(ldev, LSDC_CURSOR1_CFG_REG, val);

	DRM_DEBUG_KMS("cursor-%d disabled\n", index);
}

static const struct drm_plane_helper_funcs ls7a2000_cursor_hpfns = {
	.atomic_check = lsdc_cursor_atomic_check,
	.atomic_update = ls7a2000_cursor_atomic_update,
	.atomic_disable = ls7a2000_cursor_atomic_disable,
};

static bool lsdc_format_mod_supported(struct drm_plane *plane,
				     uint32_t format,
				     uint64_t modifier)
{
	DRM_DEBUG_KMS("format = %u. modifier=%llu\n", format, modifier);
	return (modifier == DRM_FORMAT_MOD_LINEAR);
}

static void lsdc_update_stride(struct drm_crtc *crtc, unsigned int stride)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);

	if (index == 0)
		lsdc_reg_write32(ldev, LSDC_CRTC0_STRIDE_REG, stride);
	else if (index == 1)
		lsdc_reg_write32(ldev, LSDC_CRTC1_STRIDE_REG, stride);

	DRM_DEBUG_DRIVER("Update the stride of CRTC%u to %u\n", index, stride);
}

static void lsdc_update_fb_format(struct drm_crtc *crtc,
			const struct drm_format_info * const format_info)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	u32 reg_val;

	if (index == 0)
		reg_val = lsdc_reg_read32(ldev, LSDC_CRTC0_CFG_REG);
	else if (index == 1)
		reg_val = lsdc_reg_read32(ldev, LSDC_CRTC1_CFG_REG);

	/* clear the fb format bits first */
	reg_val &= ~CFG_FB_FMT;

	switch (format_info->format) {
	case DRM_FORMAT_RGB565:
		DRM_DEBUG_KMS("FORMAT: RGB565, 16 bit\n");
		reg_val |= 0x3;
		break;
	case DRM_FORMAT_RGB888:
		DRM_DEBUG_KMS("FORMAT: RGB888, 24 bit depth, 24 bit color\n");
		reg_val |= 0x4;
		break;
	case DRM_FORMAT_XRGB8888:
		DRM_DEBUG_KMS("FORMAT: XRGB8888, 24 bit depth, 32 bit color\n");
		reg_val |= 0x4;
		break;
	default:
		reg_val |= 0x4;
		break;
	}

	reg_val |= CFG_OUTPUT_ENABLE;

	if (index == 0)
		lsdc_reg_write32(ldev, LSDC_CRTC0_CFG_REG, reg_val);
	else if (index == 1)
		lsdc_reg_write32(ldev, LSDC_CRTC1_CFG_REG, reg_val);
}

static void lsdc_update_fb_addr(struct drm_crtc *crtc, u64 fb_addr)
{
	struct lsdc_device *ldev = to_lsdc(crtc->dev);
	unsigned int index = drm_crtc_index(crtc);
	unsigned int fb_index;
	u32 reg_val;

	if (index == 0) {
		/* CRTC0 */
		reg_val = lsdc_reg_read32(ldev, LSDC_CRTC0_CFG_REG);
		/* find which framebuffer reg is in use */
		fb_index = reg_val & CFG_FB_IDX_FLAG ? 1 : 0;

		if (fb_index) {
			lsdc_reg_write32(ldev, LSDC_CRTC0_FB_ADDR1_REG, fb_addr);
			lsdc_reg_write32(ldev, LSDC_CRTC0_FB_HI_ADDR1_REG, fb_addr >> 32);
		} else {
			lsdc_reg_write32(ldev, LSDC_CRTC0_FB_ADDR0_REG, fb_addr);
			lsdc_reg_write32(ldev, LSDC_CRTC0_FB_HI_ADDR0_REG, fb_addr >> 32);
		}
	} else if (index == 1) {
		/* CRTC1 */
		reg_val = lsdc_reg_read32(ldev, LSDC_CRTC1_CFG_REG);
		fb_index = reg_val & CFG_FB_IDX_FLAG ? 1 : 0;

		if (fb_index) {
			lsdc_reg_write32(ldev, LSDC_CRTC1_FB_ADDR1_REG, fb_addr);
			lsdc_reg_write32(ldev, LSDC_CRTC1_FB_HI_ADDR1_REG, fb_addr >> 32);
		} else {
			lsdc_reg_write32(ldev, LSDC_CRTC1_FB_ADDR0_REG, fb_addr);
			lsdc_reg_write32(ldev, LSDC_CRTC1_FB_HI_ADDR0_REG, fb_addr >> 32);
		}
	} else
		DRM_ERROR("CRTC is no more than %u.\n", LSDC_NUM_CRTC);

	DRM_DEBUG_DRIVER("CRTC%u_fb%u scantout from DMA addr: 0x%llx\n",
			 index, fb_index, fb_addr);
}

static dma_addr_t lsdc_vram_get_fb_addr(struct drm_framebuffer *fb,
					struct drm_plane_state *state,
					unsigned int plane)
{
	struct lsdc_device *ldev = to_lsdc(fb->dev);
	struct drm_gem_cma_object *obj;
	dma_addr_t paddr;

	/* sanity check */
	obj = drm_fb_cma_get_gem_obj(fb, plane);
	if (!obj)
		return 0;

	paddr = ldev->vram_base + fb->offsets[plane];
	paddr += fb->format->cpp[plane] * (state->src_x >> 16);
	paddr += fb->pitches[plane] * (state->src_y >> 16);

	return paddr;
}

static void lsdc_handle_fullscreen_damage(struct lsdc_device *ldev,
					  struct drm_framebuffer *fb,
					  dma_addr_t fb_addr)
{
	struct drm_clip_rect fullscreen = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);

	DRM_DEBUG_DRIVER("%s: cma: vaddr = 0x%llx, paddr=0x%llx\n",
			__func__, (u64)cma_obj->vaddr, cma_obj->paddr);

	DRM_DEBUG_DRIVER("%s: vram: vaddr = 0x%llx, paddr=0x%llx\n",
			__func__, (u64)ldev->vram, fb_addr);

	lsdc_fb_dirty_update_impl(ldev->vram, cma_obj->vaddr, fb, &fullscreen);
}

static void lsdc_plane_atomic_update(struct drm_plane *plane,
				     struct drm_plane_state *old_state)
{
	struct lsdc_device *ldev = to_lsdc(plane->dev);
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t fb_addr;

	if (!state->crtc || !fb) {
		DRM_DEBUG_KMS("fb or crtc NULL");
		return;
	}

	lsdc_update_fb_format(state->crtc, fb->format);

	lsdc_update_stride(state->crtc, fb->pitches[0]);

	if (ldev->shadowfb) {
		fb_addr = lsdc_vram_get_fb_addr(fb, state, 0);

		lsdc_handle_fullscreen_damage(ldev, fb, fb_addr);
	} else
		fb_addr = drm_fb_cma_get_gem_addr(fb, state, 0);


	lsdc_update_fb_addr(state->crtc, fb_addr);
}

static int lsdc_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	u32 src_x, src_y, src_w, src_h;

	if (!fb)
		return 0;

	/* Convert src_ from 16:16 format */
	src_x = state->src_x >> 16;
	src_y = state->src_y >> 16;
	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;

	/* Reject scaling */
	if ((src_w != state->crtc_w) || (src_h != state->crtc_h)) {
		DRM_ERROR("Scaling is not supported");
		return -EINVAL;
	}

	DRM_DEBUG_KMS("%s: crtc: %dx%d\n", __func__,
			state->crtc_w, state->crtc_h);

	return 0;
}

/**
 * lsdc_plane_prepare_fb:
 *
 * This hook is to prepare a framebuffer for scanout by e.g. pinning
 * it's backing storage or relocating it into a contiguous block of
 * VRAM. Other possible preparatory work includes flushing caches.
 *
 * This function must not block for outstanding rendering, since it is
 * called in the context of the atomic IOCTL even for async commits to
 * be able to return any errors to userspace. Instead the recommended
 * way is to fill out the &drm_plane_state.fence of the passed-in
 * &drm_plane_state. If the driver doesn't support native fences then
 * equivalent functionality should be implemented through private
 * members in the plane structure.
 *
 * Drivers which always have their buffers pinned should use
 * drm_gem_fb_prepare_fb() for this hook.
 *
 * The helpers will call @cleanup_fb with matching arguments for every
 * successful call to this hook.
 *
 * This callback is used by the atomic modeset helpers and by the
 * transitional plane helpers, but it is optional.
 *
 * RETURNS:
 *
 * 0 on success or one of the following negative error codes allowed by
 * the &drm_mode_config_funcs.atomic_commit vfunc. When using helpers
 * this callback is the only one which can fail an atomic commit,
 * everything else must complete successfully.
 */
static int lsdc_plane_prepare_fb(struct drm_plane *plane,
				 struct drm_plane_state *state)
{
	DRM_DEBUG_KMS("PLANE: id=%d, plane->base.id=%d, name=%s\n",
		plane->index, plane->base.id, plane->name);

	/*
	 * Take a reference on the new framebuffer - we want to
	 * hold on to it while the hardware is displaying it.
	 */
	if (state->fb) {
		DRM_DEBUG_KMS("%s: fb->base.id=%d",
			__func__, state->fb->base.id);

		drm_mode_object_get(&state->fb->base);
	}

	return drm_gem_fb_prepare_fb(plane, state);
}

static void lsdc_plane_cleanup_fb(struct drm_plane *plane,
				  struct drm_plane_state *old_state)
{
	if (old_state->fb) {
		drm_framebuffer_put(old_state->fb);
		DRM_DEBUG_KMS("%s: %s: base.is=%d, old fb id:%d\n",
			__func__, plane->name, plane->base.id,
			old_state->fb->base.id);
	}
}

/*
 * Drivers should use this function to unconditionally disable a plane.
 * This hook is called in-between the &drm_crtc_helper_funcs.atomic_begin
 * and drm_crtc_helper_funcs.atomic_flush callbacks.
 */
static void lsdc_primary_plane_atomic_disable(struct drm_plane *plane,
					struct drm_plane_state *old_state)
{
	DRM_DEBUG_KMS("disable plane:%d:%s\n", plane->base.id, plane->name);
}

static const struct drm_plane_helper_funcs lsdc_primary_plane_helper_funcs = {
	.prepare_fb = lsdc_plane_prepare_fb,
	.cleanup_fb = lsdc_plane_cleanup_fb,
	.atomic_check = lsdc_plane_atomic_check,
	.atomic_update = lsdc_plane_atomic_update,
	.atomic_disable = lsdc_primary_plane_atomic_disable,
};

/*
 * @lsdc_plane_reset:
 *
 * Reset plane hardware and software state to off. This function isn't
 * called by the core directly, only through drm_mode_config_reset().
 *
 * Atomic drivers can use drm_atomic_helper_plane_reset() to reset
 * atomic state using this hook.
 */
static void lsdc_plane_reset(struct drm_plane *plane)
{
	drm_atomic_helper_plane_reset(plane);
	plane->state->zpos = lsdc_plane_get_default_zpos(plane->type);

	DRM_DEBUG_DRIVER("%s\n", __func__);
}

static const struct drm_plane_funcs lsdc_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = lsdc_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.format_mod_supported = lsdc_format_mod_supported,
};

int lsdc_plane_init(struct drm_device *ddev,
		    struct drm_plane *plane,
		    enum drm_plane_type type,
		    unsigned int index)
{
	struct lsdc_device *ldev = to_lsdc(ddev);
	const struct lsdc_platform_desc *descp = ldev->desc;
	unsigned int format_count;
	const u32 *supported_formats;
	const char *name;
	int zpos;
	int ret;

	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		supported_formats = lsdc_primary_formats;
		format_count = ARRAY_SIZE(lsdc_primary_formats);
		name = "primary-%u";
		break;
	case DRM_PLANE_TYPE_CURSOR:
		supported_formats = lsdc_cursor_formats;
		format_count = ARRAY_SIZE(lsdc_cursor_formats);
		name = "cursor-%u";
		break;
	case DRM_PLANE_TYPE_OVERLAY:
		DRM_ERROR("overlay plane is not supported\n");
		break;
	}

	ret = drm_universal_plane_init(ddev,
				       plane,
				       BIT(index),
				       &lsdc_plane_funcs,
				       supported_formats,
				       format_count,
				       NULL,
				       type,
				       name,
				       index);

	if (ret) {
		DRM_ERROR("%s failed, %d\n", __func__, ret);
		return ret;
	}

	zpos = lsdc_plane_get_default_zpos(type);

	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		drm_plane_helper_add(plane, &lsdc_primary_plane_helper_funcs);
		drm_plane_create_zpos_property(plane, zpos, 0, 6);
		break;
	case DRM_PLANE_TYPE_CURSOR:
		if (descp->chip == LSDC_CHIP_7A2000)
			drm_plane_helper_add(plane, &ls7a2000_cursor_hpfns);
		else
			drm_plane_helper_add(plane, &lsdc_cursor_plane_hpfns);
		drm_plane_create_zpos_immutable_property(plane, zpos);
		break;
	case DRM_PLANE_TYPE_OVERLAY:
		DRM_ERROR("overlay plane is not supported\n");
		break;
	}

	drm_plane_create_alpha_property(plane);

	return ret;
}
