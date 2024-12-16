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

#include <drm/drm_edid.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include "loongson_drv.h"
#include "loongson_vbios.h"
#include "loongson_i2c.h"
#include "loongson_backlight.h"

/**
 * loongson_connector_best_encoder
 *
 * @connector: point to the drm_connector structure
 *
 * Select the best encoder for the given connector.Used by both the helpers
 * (in the drm_atomic_helper_check_modeset() function)and in the legacy CRTC
 * helpers
 */
static struct drm_encoder *
loongson_connector_best_encoder(struct drm_connector *connector)
{
	int enc_id = connector->encoder_ids[0];
	/* pick the encoder ids */
	if (enc_id)
		return drm_encoder_find(connector->dev, NULL, enc_id);
	return NULL;
}

/**
 * loongson_get_modes
 *
 * @connetcor: central DRM connector control structure
 *
 * Fill in all modes currently valid for the sink into
 * the connector->probed_modes list.
 * It should also update the EDID property by calling
 * drm_mode_connector_update_edid_property().
 */
static int loongson_get_modes(struct drm_connector *connector)
{
	struct loongson_connector *ls_connector =
		to_loongson_connector(connector);
	struct i2c_adapter *adapter = ls_connector->i2c->adapter;
	struct edid *edid = NULL;
	u32 edid_method = ls_connector->edid_method;
	u32 size = sizeof(u8) * EDID_LENGTH * 2;
	int ret = -1;

	switch (edid_method) {
	case via_vbios:
		edid = kmalloc(size, GFP_KERNEL);
		if (edid)
			memcpy(edid, ls_connector->vbios_edid, size);
		break;
	case via_null:
	case via_max:
	case via_encoder:
	case via_i2c:
	default:
		edid = drm_get_edid(connector, adapter);
	}

	if (edid) {
		drm_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	} else {
		ret += drm_add_modes_noedid(connector, 1920, 1080);
		drm_set_preferred_mode(connector, 1024, 768);
	}

	return ret;
}

static bool is_connected(struct drm_connector *connector)
{
	struct loongson_device *ldev = connector->dev->dev_private;
	struct loongson_i2c *i2c = &ldev->i2c_bus[connector->index];
	unsigned char start = 0x0;
	struct i2c_adapter *adapter;
	struct i2c_msg msgs = {
		.addr = DDC_ADDR,
		.flags = 0,
		.len = 1,
		.buf = &start,
	};

	if (!i2c)
		return false;

	adapter = i2c->adapter;
	if (i2c_transfer(adapter, &msgs, 1) != 1) {
		DRM_DEBUG_KMS("display-%d not connect\n", connector->index);
		return false;
	}

	return true;
}

void loongson_hpd_cancel_work(struct loongson_device *ldev)
{
	cancel_work_sync(&ldev->hotplug_work);
}

enum drm_connector_status
loongson_7a2000_detect_config(struct drm_connector *connector,
			      unsigned short edid_method)
{
	struct loongson_device *ldev = connector->dev->dev_private;
	enum drm_connector_status status = connector_status_disconnected;
	int reg_val = ls_mm_rreg(ldev, DC_HDMI_HOTPLUG_STATUS);

	switch (connector->index) {
	case 0:
		if (ldev->vga_hpd_status == connector_status_unknown)
			status = connector_status_unknown;

		if (reg_val & HDMI0_HOTPLUG_STATUS)
			status = connector_status_connected;

		if (connector->polled == (DRM_CONNECTOR_POLL_CONNECT |
		    DRM_CONNECTOR_POLL_DISCONNECT)) {
			if (is_connected(connector))
				ldev->vga_hpd_status = connector_status_connected;
			else
				ldev->vga_hpd_status = connector_status_disconnected;
		}

		if (status != ldev->vga_hpd_status)
			status = connector_status_connected;
		break;
	case 1:
		if (reg_val & HDMI1_HOTPLUG_STATUS)
			status = connector_status_connected;
		break;
	}

	return status;
}

enum drm_connector_status
loongson_2k2000_detect_config(struct drm_connector *connector,
			      unsigned short edid_method)
{
	struct loongson_device *ldev = connector->dev->dev_private;
	enum drm_connector_status status = connector_status_disconnected;
	int reg_val = ls_mm_rreg(ldev, DC_HDMI_HOTPLUG_STATUS);

	switch (connector->index) {
	case 0:
		if (reg_val & HDMI0_HOTPLUG_STATUS)
			status = connector_status_connected;

		if (connector->polled ==
		    (DRM_CONNECTOR_POLL_CONNECT | DRM_CONNECTOR_POLL_DISCONNECT)) {
			if (is_connected(connector))
				status = connector_status_connected;
			else
				status = connector_status_disconnected;
		}
		break;
	case 1:
		if (is_connected(connector))
			status = connector_status_connected;
		else
			status = connector_status_disconnected;
		break;
	}

	return status;
}

/**
 * loongson_connector_detect
 *
 * @connector: point to drm_connector
 * @force: bool
 *
 * Check to see if anything is attached to the connector.
 * The parameter force is set to false whilst polling,
 * true when checking the connector due to a user request
 */
enum drm_connector_status
loongson_7a1000_detect_config(struct drm_connector *connector,
			      unsigned short edid_method)
{
	enum drm_connector_status ret = connector_status_disconnected;
	struct loongson_device *ldev = connector->dev->dev_private;

	if (connector->polled == 0) {
		ldev->connector_active0 = 1;
		ldev->connector_active1 = 1;
		return connector_status_connected;
	}

	DRM_DEBUG_DRIVER("connect%d edid_method:%d\n",
			 connector->index, edid_method);

	switch (edid_method) {
	case via_i2c:
	case via_null:
	case via_max:
		if (is_connected(connector))
			ret = connector_status_connected;
		break;
	case via_vbios:
	case via_encoder:
		ret = connector_status_connected;
		break;
	}

	if (ret == connector_status_connected) {
		if (connector->index == 0)
			ldev->connector_active0 = 1;
		if (connector->index == 1)
			ldev->connector_active1 = 1;
	} else {
		if (connector->index == 0)
			ldev->connector_active0 = 0;
		if (connector->index == 1)
			ldev->connector_active1 = 0;
	}

	return ret;
}

static enum drm_connector_status
loongson_connector_detect(struct drm_connector *connector, bool force)
{
	struct loongson_device *ldev = connector->dev->dev_private;
	struct loongson_connector *ls_connector = to_loongson_connector(connector);
	enum drm_connector_status ret = connector_status_disconnected;
	unsigned short edid_method = ls_connector->edid_method;

	if (ldev->dc_config->detect_config)
		ret = ldev->dc_config->detect_config(connector, edid_method);

	return ret;
}

/**
 * loongson_connector_destroy
 *
 * @connector: point to the drm_connector structure
 *
 * Clean up connector resources
 */
static void loongson_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
	kfree(connector);
}

/**
 * These provide the minimum set of functions required to handle a connector
 *
 * Helper operations for connectors.These functions are used
 * by the atomic and legacy modeset helpers and by the probe helpers.
 */
static const struct drm_connector_helper_funcs loongson_connector_helper = {
	.get_modes = loongson_get_modes,
	.best_encoder = loongson_connector_best_encoder,
};

/**
 * These provide the minimum set of functions required to handle a connector
 *
 * Control connectors on a given device.
 * The functions below allow the core DRM code to control connectors,
 * enumerate available modes and so on.
 */
static const struct drm_connector_funcs loongson_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = loongson_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = loongson_connector_destroy,
	.late_register = loongson_backlight_register,
};

void loongson_hotplug_config(struct loongson_device *ldev)
{
	struct drm_connector *connector;
	struct drm_device *dev = ldev->dev;
	u32 value = ls_mm_rreg(ldev, LS_FB_INT_REG);
	u32 val_vga = ls_mm_rreg(ldev, DC_VGA_HOTPULG_CFG);

	if (ldev->chip == dc_2k2000) {
		list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
			if (connector->index == 0) {
				if (connector->polled)
					value |= DC_INT_HDMI0_HOTPLUG_EN;
				else
					value &= ~DC_INT_HDMI0_HOTPLUG_EN;
			}
		}
		ls_mm_wreg(ldev, LS_FB_INT_REG, value);
		return;
	}

	ldev->vga_hpd_status = connector_status_unknown;
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		switch (connector->index) {
		case 0:
			if (connector->polled)
				value |= DC_INT_HDMI0_HOTPLUG_EN;
			else
				value &= ~DC_INT_HDMI0_HOTPLUG_EN;

			if (connector->polled == DRM_CONNECTOR_POLL_HPD) {
				val_vga &= ~0x3;
				val_vga |= VGA_HOTPLUG_ACCESS;
				value |= DC_INT_VGA_HOTPLUG_EN;
			} else
				value &= ~DC_INT_VGA_HOTPLUG_EN;
			break;
		case 1:
			if (connector->polled)
				value |= DC_INT_HDMI1_HOTPLUG_EN;
			else
				value &= ~DC_INT_HDMI1_HOTPLUG_EN;
			break;
		}
	}

	ls_mm_wreg(ldev, LS_FB_INT_REG, value);
	ls_mm_wreg(ldev, DC_VGA_HOTPULG_CFG, val_vga);
}

int loongson_hdmi_init(struct loongson_device *ldev, int index)
{
	u32 val;
	int reg_offset;

	reg_offset = index * 0x10;
	ls_mm_wreg(ldev, HDMI_CTRL_REG + reg_offset, 0x287);
	ls_mm_wreg(ldev, HDMI_ZONEIDLE_REG + reg_offset, 0x00400040);

	ls_mm_wreg(ldev, HDMI_AUDIO_NCFG_REG + reg_offset, 6272);
	ls_mm_wreg(ldev, HDMI_AUDIO_CTSCFG_REG + reg_offset, 0x80000000);

	ls_mm_wreg(ldev, HDMI_AUDIO_INFOFRAME_REG + reg_offset, 0x11);
	val = ls_mm_rreg(ldev, HDMI_AUDIO_INFOFRAME_REG + reg_offset);
	val |= 0x4;

	ls_mm_wreg(ldev, HDMI_AUDIO_INFOFRAME_REG + reg_offset, val);
	ls_mm_wreg(ldev, HDMI_AUDIO_SAMPLE_REG + reg_offset, 0x1);

	DRM_DEBUG_DRIVER("lOONGSON HDMI init finish.\n");

	return 0;
}

/**
 * loongson_connector_init
 *
 * @ldev: loongson drm device
 * @index obj connector_id:
 *
 * Vga is the interface between host and monitor
 * This function is to init vga
 */
struct loongson_connector *loongson_connector_init(struct loongson_device *ldev,
						   int index)
{
	struct drm_connector *connector;
	struct loongson_connector *ls_connector;

	ls_connector = kzalloc(sizeof(struct loongson_connector), GFP_KERNEL);
	if (!ls_connector)
		return NULL;

	ldev->connector_active0 = 0;
	ldev->connector_active1 = 0;
	ls_connector->ldev = ldev;
	ls_connector->id = index;
	ls_connector->type = get_connector_type(ldev, index);
	ls_connector->i2c_id = get_connector_i2cid(ldev, index);
	ls_connector->hotplug = get_hotplug_mode(ldev, index);
	ls_connector->edid_method = get_edid_method(ldev, index);
	if (ls_connector->edid_method == via_vbios)
		ls_connector->vbios_edid = get_vbios_edid(ldev, index);

	ls_connector->i2c = &ldev->i2c_bus[index];
	if (!ls_connector->i2c)
		DRM_ERROR("connector-%d match i2c-%d err\n", index,
			 ls_connector->i2c_id);

	connector = &ls_connector->base;
	connector->dpms = DRM_MODE_DPMS_OFF;
	connector->connector_type_id = ls_connector->type;

	drm_connector_init(ldev->dev, connector, &loongson_connector_funcs,
				ls_connector->type);

	ldev->mode_info[index].connector = ls_connector;
	ldev->mode_info[index].connector_is_legacy = true;
	ldev->mode_info[index].mode_config_initialized = true;
	drm_connector_helper_add(connector, &loongson_connector_helper);
	drm_connector_register(connector);

	switch (ls_connector->hotplug) {
	case hpd_irq:
		connector->polled = DRM_CONNECTOR_POLL_HPD;
		break;
	case hpd_polling:
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				    DRM_CONNECTOR_POLL_DISCONNECT;
		break;
	case hpd_disable:
	default:
		connector->polled = 0;
		break;
	}

	return ls_connector;
}

/**
 * loongson_connector_resume
 *
 * @ldev loongson drm device
 * */
void loongson_connector_resume(struct loongson_device *ldev)
{
	struct loongson_backlight *ls_bl = NULL;
	int i;

	for (i = 0; i < LS_MAX_MODE_INFO; i++) {
		if (ldev->mode_info[i].mode_config_initialized) {
			ls_bl = ldev->mode_info[i].backlight;
			if (ls_bl && ls_bl->present && ls_bl->power)
				ls_bl->power(ls_bl, true);
		}
	}
}
