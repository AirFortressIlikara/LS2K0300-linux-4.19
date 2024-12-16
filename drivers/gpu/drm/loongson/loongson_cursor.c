/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 * Authors:
 *	Zhu Chen <zhuchen@loongson.cn>
 *	Fang Yaling <fangyaling@loongson.cn>
 *	Zhang Dandan <zhangdandan@loongson.cn>
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include "loongson_drv.h"

/*
  Hide the cursor off screen. We can't disable the cursor hardware because it
  takes too long to re-activate and causes momentary corruption
*/
void loongson_hide_cursor(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct loongson_device *ldev =
		(struct loongson_device *)dev->dev_private;
	struct loongson_crtc *lcrtc = to_loongson_crtc(crtc);

	if (lcrtc->crtc_id == 0) {
		int val;

		val = ls_mm_rreg(ldev, LS_FB_CUR_CFG_REG);
		val &= ~0x3;
		ls_mm_wreg(ldev, LS_FB_CUR_CFG_REG, val);
	}
}

void loongson_show_cursor(struct drm_crtc *crtc, int width)
{
	struct drm_device *dev = crtc->dev;
	struct loongson_device *ldev =
		(struct loongson_device *)dev->dev_private;
	struct loongson_crtc *lcrtc = to_loongson_crtc(crtc);

	if (lcrtc->crtc_id == 0) {
		int val;

		val = ls_mm_rreg(ldev, LS_FB_CUR_CFG_REG);
		val |= 0x2;
		ls_mm_wreg(ldev, LS_FB_CUR_CFG_REG, val);
	}
}

void loongson_cursor_set_hot_spot(struct drm_crtc *crtc, int x, int y)
{
	struct loongson_device *ldev = crtc->dev->dev_private;
	struct loongson_crtc *lcrtc = to_loongson_crtc(crtc);
	u32 val;
	if (lcrtc->crtc_id == 0) {

		val = ls_mm_rreg(ldev, LS_FB_CUR_CFG_REG);

		val &= ~0x1F0000;
		val |= ((x & 0x1f) << 16);
		val &= ~0x1F00;
		val |= ((y & 0x1f) << 8);
		ls_mm_wreg(ldev, LS_FB_CUR_CFG_REG, val);
	}
}

void loongson_crtc_cursor_set_local(struct drm_crtc *crtc, int x, int y)
{
	struct loongson_device *ldev = crtc->dev->dev_private;
	struct loongson_crtc *lcrtc = to_loongson_crtc(crtc);

	if (lcrtc->crtc_id == 0) {
		u32 val;

		val = x & 0x7ff;
		val |= (y & 0x7ff) << 16;
		ls_mm_wreg(ldev, LS_FB_CUR_LOC_ADDR_REG, val);
	}
}

void loongson_7a2000_show_cursor(struct drm_crtc *crtc, int width)
{
	struct loongson_device *ldev = crtc->dev->dev_private;
	u32 value;

	if (crtc->index == 0)
		value = ls_mm_rreg(ldev, LS_FB_CUR_CFG_REG);
	else
		value = ls_mm_rreg(ldev, LS_FB_CUR1_CFG_REG);

	value &= ~BIT(2);
	switch (width) {
	case 32:
		value |= DC_CURSOR_MODE_32x32;
		break;
	case 64:
		value |= DC_CURSOR_MODE_64x64;
		break;
	}

	value &= ~(BIT(4) | BIT(1) | BIT(0));
	value |= (crtc->index & DC_CURSOR_DISPLAY_MASK) << DC_CURSOR_DISPLAY_SHIFT;
	value |= (CUR_FORMAT_ARGB8888 & DC_CURSOR_FORMAT_MASK) << DC_CURSOR_FORMAT_SHIFT;

	switch (crtc->index) {
	case 0:
		ls_mm_wreg(ldev, LS_FB_CUR_CFG_REG, value);
		break;
	case 1:
		ls_mm_wreg(ldev, LS_FB_CUR1_CFG_REG, value);
		break;
	}
}

void loongson_7a2000_hide_cursor(struct drm_crtc *crtc)
{
	struct loongson_device *ldev = crtc->dev->dev_private;
	int value;

	switch (crtc->index) {
	case 0:
		value = ls_mm_rreg(ldev, LS_FB_CUR_CFG_REG);
		value &= ~0x3;
		ls_mm_wreg(ldev, LS_FB_CUR_CFG_REG, value);
		break;
	case 1:
		value = ls_mm_rreg(ldev, LS_FB_CUR1_CFG_REG);
		value &= ~0x3;
		ls_mm_wreg(ldev, LS_FB_CUR1_CFG_REG, value);
		break;
	}
}

void loongson_7a2000_crtc_cursor_set_local(struct drm_crtc *crtc, int x, int y)
{
	struct loongson_device *ldev = crtc->dev->dev_private;
	u32 value = 0;

	switch (crtc->index) {
	case 0:
		value = x & 0x7ff;
		value |= (y & 0x7ff) << 16;
		ls_mm_wreg(ldev, LS_FB_CUR_LOC_ADDR_REG, value);
		break;
	case 1:
		value = x & 0x7ff;
		value |= (y & 0x7ff) << 16;
		ls_mm_wreg(ldev, LS_FB_CUR1_LOC_ADDR_REG, value);
		break;
	}
}

void loongson_7a2000_cursor_set_hot_spot(struct drm_crtc *crtc, int x, int y)
{
	struct loongson_device *ldev = crtc->dev->dev_private;
	u32 val;

	if (crtc->index == 0)
		val = ls_mm_rreg(ldev, LS_FB_CUR_CFG_REG);
	else
		val = ls_mm_rreg(ldev, LS_FB_CUR1_CFG_REG);

	val &= ~(DC_CURSOR_POS_HOT_X_MASK << DC_CURSOR_POS_HOT_X_SHIFT);
	val &= ~(DC_CURSOR_POS_HOT_Y_MASK << DC_CURSOR_POS_HOT_Y_SHIFT);

	val |= ((CUR_FORMAT_ARGB8888 & DC_CURSOR_FORMAT_MASK) << DC_CURSOR_FORMAT_SHIFT);
	val |= ((crtc->index & DC_CURSOR_DISPLAY_MASK) << DC_CURSOR_DISPLAY_SHIFT);
	val |= ((x & DC_CURSOR_POS_HOT_X_MASK) << DC_CURSOR_POS_HOT_X_SHIFT);
	val |= ((y & DC_CURSOR_POS_HOT_Y_MASK) << DC_CURSOR_POS_HOT_Y_SHIFT);

	switch (crtc->index) {
	case 0:
		ls_mm_wreg(ldev, LS_FB_CUR_CFG_REG, val);
		break;
	case 1:
		ls_mm_wreg(ldev, LS_FB_CUR1_CFG_REG, val);
		break;
	}
}

int loongson_crtc_cursor_set2(struct drm_crtc *crtc,
			struct drm_file *file_priv,
			uint32_t handle,
			uint32_t width,
			uint32_t height,
			int32_t hot_x,
			int32_t hot_y)
{
	int ret = 0;
	u32 gpu_addr;
	struct drm_device *dev = crtc->dev;
	struct drm_gem_object *obj;
	struct loongson_bo *bo;
	struct loongson_device *ldev = dev->dev_private;
	struct loongson_crtc *lcrtc = to_loongson_crtc(crtc);
	struct loongson_cursor *lcursor = &lcrtc->cursor;

	DRM_DEBUG_KMS("crtc %d  cursor width %d hei %d hotx %d hoty %d\n",
		      crtc->index, width, height, hot_x, hot_y);

	if (!handle) {
		if (ldev->dc_config->hide_cursor)
			ldev->dc_config->hide_cursor(crtc);
		return 0;
	}
	if ((width > dev->mode_config.cursor_width) ||
	    (height > dev->mode_config.cursor_height)) {
		DRM_ERROR("bad cursor width or height %d x %d\n", width, height);
		return -EINVAL;
	}

	obj = drm_gem_object_lookup(file_priv, handle);
	if (!obj)
		return -ENOENT;
	bo = gem_to_loongson_bo(obj);

	ret = loongson_bo_reserve(bo, true);
	if (ret) {
		goto out_unref;
	}

	/* Move cursor buffers into VRAM if they aren't already */
	if (!bo->pin_count) {
		ret = loongson_bo_pin(bo, TTM_PL_FLAG_VRAM, &lcursor->paddr);
		if (ret)
			goto out1;
	} else {
		loongson_bo_unreserve(bo);
		goto out_unref;
	}

	if (width != lcursor->width || height != lcursor->height ||
	    hot_x != lcursor->hot_x || hot_y != lcursor->hot_y) {
		int x, y;

		x = lcursor->x + lcursor->hot_x - hot_x;
		y = lcursor->y + lcursor->hot_y - hot_y;

		loongson_crtc_cursor_move(crtc, x, y);

		lcrtc->cursor.width = width;
		lcrtc->cursor.height = height;
		lcrtc->cursor.hot_x = hot_x;
		lcrtc->cursor.hot_y = hot_y;
	}

	gpu_addr = lower_32_bits(lcursor->paddr);
	ls_mm_wreg(ldev, LS_FB_CUR_ADDR_REG, gpu_addr);
	ls_mm_wreg(ldev, LS_FB_CUR_BACK_REG, 0x00eeeeee);
	ls_mm_wreg(ldev, LS_FB_CUR_FORE_REG, 0x00aaaaaa);

	ls_mm_wreg(ldev, LS_FB_CUR1_ADDR_REG, gpu_addr);
	ls_mm_wreg(ldev, LS_FB_CUR1_BACK_REG, 0x00eeeeee);
	ls_mm_wreg(ldev, LS_FB_CUR1_FORE_REG, 0x00aaaaaa);

	if (ldev->dc_config->show_cursor)
		ldev->dc_config->show_cursor(crtc, width);

 out1:
	if (ret) {
		if (ldev->dc_config->hide_cursor)
			ldev->dc_config->hide_cursor(crtc);
	}
	loongson_bo_unreserve(bo);
	lcursor->bo = bo;

out_unref:
	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

int loongson_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	struct loongson_crtc *lcrtc = to_loongson_crtc(crtc);
	struct loongson_cursor *lcursor = &lcrtc->cursor;
	struct drm_device *dev = crtc->dev;
	struct loongson_device *ldev = dev->dev_private;
	int xorigin = 0, yorigin = 0;

	DRM_DEBUG_KMS("crtc-id %d x %d y %d crtc->x %d crtc->y %d crtc-cursor x %d y %d\n",
		      lcrtc->crtc_id, x, y, crtc->x, crtc->y, crtc->cursor_x, crtc->cursor_y);

	lcursor->x = x;
	lcursor->y = y;

	if (x < 0) {
		xorigin = min(-x, (int)lcursor->width - 1);
		x = 0;
	}
	if (y < 0) {
		yorigin = min(-y, (int)lcursor->height - 1);
		y = 0;
	}

	if (ldev->dc_config->cursor_hot_spot && ldev->dc_config->cursor_local) {
		ldev->dc_config->cursor_hot_spot(crtc, xorigin, yorigin);
		ldev->dc_config->cursor_local(crtc, x, y);
	}

	return 0;
}
