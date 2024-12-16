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

#include <linux/fb.h>
#include "loongson_drv.h"

static void loongson_dirty_update(struct loongson_fbdev *lfbdev, int x, int y,
				  int width, int height)
{
	int i;
	struct drm_gem_object *obj;
	struct loongson_bo *bo;
	int src_offset, dst_offset;
	int bpp = lfbdev->lfb.base.format->cpp[0];
	int ret = -EBUSY;
	bool unmap = false;
	bool store_for_later = false;
	int x2, y2;
	unsigned long flags;

	obj = lfbdev->lfb.obj;
	bo = gem_to_loongson_bo(obj);

	/*
	 * try and reserve the BO, if we fail with busy
	 * then the BO is being moved and we should
	 * store up the damage until later.
	 */
	if (drm_can_sleep())
		ret = loongson_bo_reserve(bo, true);
	if (ret) {
		if (ret != -EBUSY)
			return;

		store_for_later = true;
	}

	x2 = x + width - 1;
	y2 = y + height - 1;
	spin_lock_irqsave(&lfbdev->dirty_lock, flags);

	if (lfbdev->y1 < y)
		y = lfbdev->y1;
	if (lfbdev->y2 > y2)
		y2 = lfbdev->y2;
	if (lfbdev->x1 < x)
		x = lfbdev->x1;
	if (lfbdev->x2 > x2)
		x2 = lfbdev->x2;

	if (store_for_later) {
		lfbdev->x1 = x;
		lfbdev->x2 = x2;
		lfbdev->y1 = y;
		lfbdev->y2 = y2;
		spin_unlock_irqrestore(&lfbdev->dirty_lock, flags);
		return;
	}

	lfbdev->x1 = lfbdev->y1 = INT_MAX;
	lfbdev->x2 = lfbdev->y2 = 0;
	spin_unlock_irqrestore(&lfbdev->dirty_lock, flags);

	if (!bo->kmap.virtual) {
		ret = ttm_bo_kmap(&bo->bo, 0, bo->bo.num_pages, &bo->kmap);
		if (ret) {
			DRM_ERROR("failed to kmap fb updates\n");
			loongson_bo_unreserve(bo);
			return;
		}
		unmap = true;
	}

	for (i = y; i <= y2; i++) {
		/* assume equal stride for now */
		src_offset = dst_offset =
			i * lfbdev->lfb.base.pitches[0] + (x * bpp);
		memcpy_toio(bo->kmap.virtual + src_offset,
		       lfbdev->sysram + src_offset, (x2 - x + 1) * bpp);
	}

	if (unmap)
		ttm_bo_kunmap(&bo->kmap);

	loongson_bo_unreserve(bo);
}

/**
 * loongson_fillrect
 *
 * @info: represent general information and frame buffer device bottom information
 * @rect: point to fb_fillrect structure
 *
 * Draw a rectangle
 */
static void loongson_fillrect(struct fb_info *info,
			      const struct fb_fillrect *rect)
{
	struct loongson_fbdev *lfbdev = info->par;
	drm_fb_helper_sys_fillrect(info, rect);
	loongson_dirty_update(lfbdev, rect->dx, rect->dy, rect->width,
			      rect->height);
}

/**
 * loongson_copyarea
 *
 * @info: fb infomatiom
 * @area: point to fb_copyarea
 *
 * Copy data from area to another
 */
static void loongson_copyarea(struct fb_info *info,
			      const struct fb_copyarea *area)
{
	struct loongson_fbdev *lfbdev = info->par;
	drm_fb_helper_sys_copyarea(info, area);
	loongson_dirty_update(lfbdev, area->dx, area->dy, area->width,
			      area->height);
}

/**
 * loongson_imagablit
 *
 * @info: fb infomation
 * @image: point to fb_image structure
 *
 * Draw a image to the display
 */
static void loongson_imageblit(struct fb_info *info,
			       const struct fb_image *image)
{
	struct loongson_fbdev *lfbdev = info->par;
	drm_fb_helper_sys_imageblit(info, image);
	loongson_dirty_update(lfbdev, image->dx, image->dy, image->width,
			      image->height);
}

/**
 * loongsonfb_ops -- represent fb information
 *
 * @fb_check_var: checks var and eventually tweaks it to something supported
 * @fb_set_par:  set the video mode according to info->var
 * @fb_fillrect: draw a rectangle
 * @fb_copyarea: copy data from area to another
 * @fb_imageblit: draw a image to the display
 * @fb_pan_diaplay: pan display
 * @fb_blank: blank diaplay
 * @fb_setcmap: set color registers in batch
 *
 * Represent frame buffer operations
 */
static struct fb_ops loongsonfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = drm_fb_helper_check_var,
	.fb_set_par = drm_fb_helper_set_par,
	.fb_fillrect = loongson_fillrect,
	.fb_copyarea = loongson_copyarea,
	.fb_imageblit = loongson_imageblit,
	.fb_pan_display = drm_fb_helper_pan_display,
	.fb_blank = drm_fb_helper_blank,
	.fb_setcmap = drm_fb_helper_setcmap,
};

/**
 * loongsonfb_create_object
 *
 * @afbdev: point to loongson_fdev structure
 * @mode_cmd: point to drm_mode_fb_cmd2 structure
 * @gobj_p: address of pointer to dem_gem_object
 *
 * Create frame buffer object
 */
static int loongsonfb_create_object(struct loongson_fbdev *afbdev,
				    const struct drm_mode_fb_cmd2 *mode_cmd,
				    struct drm_gem_object **gobj_p)
{
	struct drm_device *dev = afbdev->helper.dev;
	u32 size;
	struct drm_gem_object *gobj;
	int ret = 0;

	size = mode_cmd->pitches[0] * mode_cmd->height;
	ret = loongson_gem_create(dev, size, true, &gobj);
	if (ret)
		return ret;

	*gobj_p = gobj;
	return ret;
}

/**
 * loongsonfb_create
 *
 * @helper: drm_fb_helper structure
 * @sizes:  describes fbdev size and scanout surface size
 *
 * Allocate and initialize the fbdev info structure.And it also
 * needs to allocate the DRM framebuffer used to back the fbdev
 */
static int loongsonfb_create(struct drm_fb_helper *helper,
			     struct drm_fb_helper_surface_size *sizes)
{
	struct loongson_fbdev *lfbdev =
		container_of(helper, struct loongson_fbdev, helper);
	struct drm_device *dev = lfbdev->helper.dev;
	struct drm_mode_fb_cmd2 mode_cmd;
	struct loongson_device *ldev = dev->dev_private;
	struct fb_info *info;
	struct drm_framebuffer *fb;
	struct drm_gem_object *gobj = NULL;
	int ret;
	void *sysram;
	int size;

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] =
		ALIGN(mode_cmd.width, 64) * ((sizes->surface_bpp + 7) / 8);

	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
							  sizes->surface_depth);
	size = mode_cmd.pitches[0] * mode_cmd.height;

	ret = loongsonfb_create_object(lfbdev, &mode_cmd, &gobj);
	if (ret) {
		DRM_ERROR("failed to create fbcon backing object %d\n", ret);
		return ret;
	}

	sysram = vmalloc(size);
	if (!sysram) {
		ret = -ENOMEM;
		goto err_sysram;
	}

	info = drm_fb_helper_alloc_fbi(helper);
	if (IS_ERR(info)) {
		ret = PTR_ERR(info);
		goto err_alloc_fbi;
	}

	info->par = lfbdev;

	ret = loongson_framebuffer_init(dev, &lfbdev->lfb, &mode_cmd, gobj);
	if (ret)
		goto err_alloc_fbi;

	lfbdev->sysram = sysram;
	lfbdev->size = size;

	fb = &lfbdev->lfb.base;

	/* setup helper */
	lfbdev->helper.fb = fb;

	strcpy(info->fix.id, "loongsondrmfb");

	info->fbops = &loongsonfb_ops;

	/* setup aperture base/size for vesafb takeover */
	info->apertures->ranges[0].base = ldev->dev->mode_config.fb_base;
	info->apertures->ranges[0].size = ldev->mc.vram_size;

	drm_fb_helper_fill_fix(info, fb->pitches[0], fb->format->depth);
	drm_fb_helper_fill_var(info, &lfbdev->helper, sizes->fb_width,
			       sizes->fb_height);

	info->fix.smem_start = ldev->dev->mode_config.fb_base;
	info->fix.smem_len = size;

	info->screen_base = sysram;
	info->screen_size = size;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;

	DRM_INFO("fb mappable at 0x%lX\n", info->fix.smem_start);
	DRM_INFO("vram aper at 0x%lX\n", (unsigned long)info->fix.smem_start);
	DRM_INFO("vram aper size %u Bytes\n", info->fix.smem_len);
	DRM_INFO("fb width is %d\n", fb->width);
	DRM_INFO("fb height is %d\n", fb->height);
	DRM_INFO("fb depth is %d\n", fb->format->depth);
	DRM_INFO("fb pitch is %d\n", fb->pitches[0]);

	return 0;

err_alloc_fbi:
	vfree(sysram);
err_sysram:
	drm_gem_object_unreference_unlocked(gobj);

	return ret;
}

/**
 * loongson_fbdev_destory
 *
 * @dev: drm device
 * @lfbdev: loongson_fbdev member need to destroy
 *
 * Destroy fbdev,including free memory taken by fb_info, release a reference to @obj,
 * remove a framebuffer object and so on
 */
static int loongson_fbdev_destroy(struct drm_device *dev,
				  struct loongson_fbdev *lfbdev)
{
	struct loongson_framebuffer *lfb = &lfbdev->lfb;

	drm_fb_helper_unregister_fbi(&lfbdev->helper);

	if (lfb->obj) {
		drm_gem_object_unreference_unlocked(lfb->obj);
		lfb->obj = NULL;
	}
	drm_fb_helper_fini(&lfbdev->helper);
	vfree(lfbdev->sysram);
	drm_framebuffer_unregister_private(&lfb->base);
	drm_framebuffer_cleanup(&lfb->base);

	return 0;
}

/**
 * loongson_fb_helper_funcs -- drm_fb_helper_funcs structure
 *
 * Driver callbacks used by the fbdev emulation helper library
 */
static const struct drm_fb_helper_funcs loongson_fb_helper_funcs = {
	.fb_probe = loongsonfb_create,
};

/**
 * loongson_fbdev_init
 *
 * @ldev: point to loongson_device
 *
 * Init fbdev
 */
int loongson_fbdev_init(struct loongson_device *ldev)
{
	struct loongson_fbdev *lfbdev;
	int ret;

	/* prefer 16bpp on low end gpus with limited VRAM */
	lfbdev = devm_kzalloc(ldev->dev->dev, sizeof(struct loongson_fbdev),
			      GFP_KERNEL);
	if (!lfbdev)
		return -ENOMEM;

	ldev->lfbdev = lfbdev;
	spin_lock_init(&lfbdev->dirty_lock);

	drm_fb_helper_prepare(ldev->dev, &lfbdev->helper,
			      &loongson_fb_helper_funcs);

	ret = drm_fb_helper_init(ldev->dev, &lfbdev->helper, 1);
	if (ret)
		goto err_fb_helper;

	ret = drm_fb_helper_single_add_all_connectors(&lfbdev->helper);
	if (ret)
		goto err_fb_setup;

	/* disable all the possible outputs/crtcs before entering KMS mode */
	drm_helper_disable_unused_functions(ldev->dev);

	ret = drm_fb_helper_initial_config(&lfbdev->helper, 32);
	if (ret)
		goto err_fb_setup;

	return 0;

err_fb_setup:
	drm_fb_helper_fini(&lfbdev->helper);
err_fb_helper:
	ldev->lfbdev = NULL;

	return ret;
}

/**
 * loongson_fbdev_fini
 *
 * @ldev: point to loongson_device
 *
 * Transfer loongson_fbdev_destroy
 */
void loongson_fbdev_fini(struct loongson_device *ldev)
{
	if (!ldev->lfbdev)
		return;

	loongson_fbdev_destroy(ldev->dev, ldev->lfbdev);
}

/* loongson_fbdev_set_suspend - low level driver signals suspend
 *
 * @ldev: loongson dev supose
 * @state: 0 = resuming, !=0 = suspending
 *
 * This is meant to be used by low level drivers to signal suspend/resume to the core & clients.
 * It must be called with the console semaphore held
 **/
void loongson_fbdev_set_suspend(struct loongson_device *ldev, int state)
{
	if (ldev->lfbdev)
		drm_fb_helper_set_suspend_unlocked(&ldev->lfbdev->helper, state);
}

/**
 * loongson_fbdev_lobj_is_fb
 *
 * @ldev  pointer to loongson_device
 * @lobj  pointer to loongson_bo
 *
 * check if
 * */
bool loongson_fbdev_lobj_is_fb(struct loongson_device *ldev,
			       struct loongson_bo *lobj)
{
	if (!ldev->lfbdev)
		return false;

	if (lobj == gem_to_loongson_bo(ldev->lfbdev->lfb.obj))
		return true;
	return false;
}

void loongson_fbdev_restore_mode(struct loongson_device *ldev)
{
	struct loongson_fbdev *lfbdev = ldev->lfbdev;
	struct drm_fb_helper *fb_helper;
	int ret;

	if (!lfbdev)
		return;

	fb_helper = &lfbdev->helper;

	ret = drm_fb_helper_restore_fbdev_mode_unlocked(fb_helper);
	if (ret)
		DRM_ERROR("failed to restore crtc mode\n");
}

void loongson_fb_output_poll_changed(struct loongson_device *ldev)
{
	u64 gpu_addr;
	struct loongson_bo *lbo;
	struct drm_framebuffer *drm_fb;
	struct loongson_framebuffer *lfb;
	struct drm_device *dev = ldev->dev;

	if (ldev->lfbdev)
		drm_fb_helper_hotplug_event(&ldev->lfbdev->helper);

	DRM_DEBUG_DRIVER("%s\n", __func__);

	mutex_lock(&dev->mode_config.fb_lock);
	drm_for_each_fb (drm_fb, dev) {
		lfb = to_loongson_framebuffer(drm_fb);
		lbo = gem_to_loongson_bo(lfb->obj);
		if (!loongson_fbdev_lobj_is_fb(ldev, lbo))
			continue;

		loongson_bo_reserve(lbo, false);
		loongson_bo_pin(lbo, TTM_PL_FLAG_VRAM, &gpu_addr);
		loongson_bo_unreserve(lbo);
	}
	mutex_unlock(&dev->mode_config.fb_lock);
}
