/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 * Authors:
 *	Li Chenyang <lichenyang@loongson.cn>
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include "loongson_drv.h"
#include "loongson_vbios.h"
/**
 * enable_vblank - enable vblank interrupt events
 * @dev: DRM device
 * @crtc: which irq to enable
 *
 * Enable vblank interrupts for @crtc.  If the device doesn't have
 * a hardware vblank counter, this routine should be a no-op, since
 * interrupts will have to stay on to keep the count accurate.
 *
 * RETURNS
 * Zero on success, appropriate errno if the given @crtc's vblank
 * interrupt cannot be enabled.
 */
int loongson_irq_enable_vblank(struct drm_device *dev, unsigned int crtc_id)
{
	struct loongson_device *ldev = dev->dev_private;
	unsigned int reg_val;

	if (crtc_id) {
		reg_val = ls_mm_rreg(ldev, LS_FB_INT_REG);
		reg_val |= LS_FB_VSYNC1_ENABLE;
		ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val);
	} else {
		reg_val = ls_mm_rreg(ldev, LS_FB_INT_REG);
		reg_val |= LS_FB_VSYNC0_ENABLE;
		ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val);
	}

	return 0;
}

/**
 * disable_vblank - disable vblank interrupt events
 * @dev: DRM device
 * @crtc: which irq to enable
 *
 * Disable vblank interrupts for @crtc.  If the device doesn't have
 * a hardware vblank counter, this routine should be a no-op, since
 * interrupts will have to stay on to keep the count accurate.
 */
void loongson_irq_disable_vblank(struct drm_device *dev, unsigned int crtc_id)
{
	struct loongson_device *ldev = dev->dev_private;
	unsigned int reg_val;

	if (crtc_id) {
		reg_val = ls_mm_rreg(ldev, LS_FB_INT_REG);
		reg_val &= ~LS_FB_VSYNC1_ENABLE;
		ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val);
	} else {
		reg_val = ls_mm_rreg(ldev, LS_FB_INT_REG);
		reg_val &= ~LS_FB_VSYNC0_ENABLE;
		ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val);
	}
}

irqreturn_t loongson_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = (struct drm_device *)arg;
	struct loongson_device *ldev = dev->dev_private;
	struct loongson_crtc *loongson_crtc;
	struct drm_connector *connector;
	unsigned int reg_val;
	unsigned int reg_val_vga;

	/*get irq and clear*/
	reg_val = ls_mm_rreg(ldev, LS_FB_INT_REG);
	if (ldev->chip == dc_7a1000)
		ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val & (0xffff << 16));

	if (ldev->chip == dc_7a2000 || ldev->chip == dc_2k2000) {
		if (reg_val & VGA_INT_HOTPLUG_CTL) {
			reg_val_vga = ls_mm_rreg(ldev, DC_VGA_HOTPULG_CFG);
			if (reg_val_vga & VGA_HOTPLUG_ACCESS) {
				ldev->vga_hpd_status = connector_status_connected;
				reg_val_vga &= ~0x3;
				reg_val_vga |= VGA_HOTPLUG_EXTRACT;
				ls_mm_wreg(ldev, DC_VGA_HOTPULG_CFG, reg_val_vga);
			} else if (reg_val_vga & VGA_HOTPLUG_EXTRACT) {
				ldev->vga_hpd_status = connector_status_disconnected;
				reg_val_vga &= ~0x3;
				reg_val_vga |= VGA_HOTPLUG_ACCESS;
				ls_mm_wreg(ldev, DC_VGA_HOTPULG_CFG, reg_val_vga);
			}
		}
		ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val);
		list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
			if (connector->polled == DRM_CONNECTOR_POLL_HPD)
				schedule_work(&ldev->hotplug_work);
		}
	}

	/*vga crtc1*/
	if (reg_val & 0x1) {
		loongson_pageflip_irq(ldev, 1);
		loongson_crtc = ldev->mode_info[1].crtc;
		drm_crtc_handle_vblank(&loongson_crtc->base);
		ldev->vsync1_count++;
	}
	/*hdmi crtc0*/
	if (reg_val & 0x4) {
		loongson_pageflip_irq(ldev, 0);
		loongson_crtc = ldev->mode_info[0].crtc;
		drm_crtc_handle_vblank(&loongson_crtc->base);
		ldev->vsync0_count++;
	}

	return IRQ_HANDLED;
}

int loongson_pageflip_irq(struct loongson_device *ldev, unsigned int crtc_id)
{
	unsigned long flags;
	struct loongson_crtc *loongson_crtc;
	struct loongson_flip_work *works;
	struct drm_crtc *crtc;
	struct drm_framebuffer *fb;

	loongson_crtc = ldev->mode_info[crtc_id].crtc;
	crtc = &loongson_crtc->base;
	fb = crtc->primary->fb;

	if (loongson_crtc == NULL)
		return 0;

	spin_lock_irqsave(&ldev->dev->event_lock, flags);
	works = loongson_crtc->pflip_works;
	if (loongson_crtc->pflip_status != LOONGSON_FLIP_SUBMITTED) {
		spin_unlock_irqrestore(&ldev->dev->event_lock, flags);
		return 0;
	}

	/* page flip completed. clean up */
	loongson_crtc->pflip_status = LOONGSON_FLIP_NONE;
	loongson_crtc->pflip_works = NULL;

	/* wakeup usersapce */
	if (works->event)
		drm_crtc_send_vblank_event(&loongson_crtc->base, works->event);

	spin_unlock_irqrestore(&ldev->dev->event_lock, flags);
	drm_crtc_vblank_put(&loongson_crtc->base);

	return 0;
}

void loongson_irq_preinstall(struct drm_device *dev)
{
	struct loongson_device *ldev = dev->dev_private;
	unsigned int reg_val;

	/* Clear bits */
	reg_val = ls_mm_rreg(ldev, LS_FB_INT_REG);
	reg_val &= (0xffff << 16);
	ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val);
}

int loongson_irq_postinstall(struct drm_device *dev)
{
	dev->max_vblank_count = 0x00ffffff;
	return 0;
}

void loongson_irq_uninstall(struct drm_device *dev)
{
	struct loongson_device *ldev = dev->dev_private;
	unsigned int reg_val;

	if (ldev == NULL) {
		return;
	}

	/* Disable all interrupts */
	reg_val = (0x0000 << 16);
	ls_mm_wreg(ldev, LS_FB_INT_REG, reg_val);
	loongson_hpd_cancel_work(ldev);
}

int loongson_irq_init(struct loongson_device *ldev)
{
	int r = 0;

	ldev->vsync0_count = 0;
	ldev->vsync1_count = 0;
	ldev->pageflip_count = 0;

	spin_lock_init(&ldev->irq.lock);
	r = drm_vblank_init(ldev->dev, ldev->num_crtc);
	if (r) {
		return r;
	}
	DRM_DEBUG_DRIVER("drm vblank init finished\n");

	ldev->irq.installed = true;
	r = drm_irq_install(ldev->dev, ldev->irq.irq_num);

	if (r) {
		DRM_INFO("drm_irq_install error:%d\n", r);
		ldev->irq.installed = false;
		return r;
	}

	DRM_DEBUG_DRIVER("loongson irq initialized\n");
	return 0;
}

u32 loongson_crtc_vblank_count(struct drm_device *dev, unsigned int pipe)
{
	struct loongson_device *ldev = dev->dev_private;

	if (pipe)
		return ldev->vsync1_count;
	else
		return ldev->vsync0_count;
}

void loongson_flip_work_func(struct work_struct *__work)
{
	struct delayed_work *delayed_work =
		container_of(__work, struct delayed_work, work);
	struct loongson_flip_work *work = container_of(
		delayed_work, struct loongson_flip_work, flip_work);
	struct loongson_device *ldev = work->ldev;
	struct loongson_crtc *loongson_crtc =
		ldev->mode_info[work->crtc_id].crtc;
	struct drm_crtc *crtc = &loongson_crtc->base;
	unsigned long flags;
	unsigned int crtc_address, pitch, y, x;

	y = crtc->y;
	x = crtc->x;
	pitch = crtc->primary->fb->pitches[0];
	crtc_address = (u32)work->base + y * pitch + ALIGN(x, 64) * 4;

	/* We borrow the event spin lock for protecting flip_status */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	if (loongson_crtc->pflip_status == LOONGSON_FLIP_NONE ||
	    ldev->pageflip_count !=
		    loongson_crtc_vblank_count(crtc->dev, work->crtc_id)) {
		loongson_set_start_address(crtc, (u32)crtc_address);
		/* Set the flip status */
		loongson_crtc->pflip_status = LOONGSON_FLIP_SUBMITTED;
		ldev->pageflip_count =
			loongson_crtc_vblank_count(crtc->dev, work->crtc_id);
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	} else {
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		schedule_delayed_work(&work->flip_work, usecs_to_jiffies(1000));
		return;
	}
}
