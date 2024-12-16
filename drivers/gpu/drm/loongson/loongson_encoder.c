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
#include "loongson_vbios.h"
#include "bridge/bridge_phy.h"

/*
 * The encoder comes after the CRTC in the output pipeline, but before
 * the connector. It's responsible for ensuring that the digital
 * stream is appropriately converted into the output format. Setup is
 * very simple in this case - all we have to do is inform qemu of the
 * colour depth in order to ensure that it displays appropriately
 */

/**
 * loongson_encoder_mode_set
 *
 * @encoder: encoder object
 * @mode: display mode
 * @adjusted_mode: point to the drm_display_mode structure
 *
 * Used to update the display mode of an encoder
 */
static void loongson_encoder_mode_set(struct drm_encoder *encoder,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	return;
}

/**
 * loongson_encoder_dpms
 *
 * @encoder: encoder object
 *
 * Control power levels on the encoder
 *
 */
static void loongson_encoder_dpms(struct drm_encoder *encoder, int state)
{
	/* Automatic backlight adjustment will Callback hear.*/
	struct loongson_encoder *ls_encoder = to_loongson_encoder(encoder);
	struct loongson_device *ldev = ls_encoder->ldev;
	struct bridge_phy *phy = NULL;
	struct loongson_backlight *ls_bl = NULL;

	phy = ldev->mode_info[ls_encoder->encoder_id].bridge_phy;
	DRM_DEBUG("[%s] encoder(id=%d) edpm, state=%d.\n", phy ?
			phy->res->chip_name : "Unknown",
			ls_encoder->encoder_id, state);
	ls_bl = ldev->mode_info[ls_encoder->encoder_id].backlight;
	if (ls_bl && ls_bl->present && ls_bl->power)
		ls_bl->power(ls_bl, state == DRM_MODE_DPMS_ON);
	return;
}

/**
 * loongson_encoder_prepare
 *
 * @encoder: encoder object
 *
 * Prepare the encoder for a subsequent modeset
 */
static void loongson_encoder_prepare(struct drm_encoder *encoder)
{
	struct loongson_encoder *ls_encoder = to_loongson_encoder(encoder);
	struct loongson_device *ldev = ls_encoder->ldev;
	struct bridge_phy *phy;

	phy = ldev->mode_info[ls_encoder->encoder_id].bridge_phy;
	if (phy && phy->cfg_funcs && phy->cfg_funcs->prepare)
		phy->cfg_funcs->prepare(phy);
	return;
}

/**
 * loongson_encoder_commit
 *
 * @encoder: point to tne drm_encoder structure
 *
 * Commit the new mode on the encoder after a modeset
 */
static void loongson_encoder_commit(struct drm_encoder *encoder)
{
	struct loongson_encoder *ls_encoder = to_loongson_encoder(encoder);
	struct loongson_device *ldev = ls_encoder->ldev;
	struct bridge_phy *phy;

	phy = ldev->mode_info[ls_encoder->encoder_id].bridge_phy;
	if (phy && phy->cfg_funcs && phy->cfg_funcs->commit)
		phy->cfg_funcs->commit(phy);
	return;
}

/**
 * loongson_encoder_destroy
 *
 * @encoder: encoder object
 *
 * Clean up encoder resources
 */
static void loongson_encoder_destroy(struct drm_encoder *encoder)
{
	struct loongson_encoder *loongson_encoder =
		to_loongson_encoder(encoder);
	drm_encoder_cleanup(encoder);
	kfree(loongson_encoder);
}

static void loongson_encoder_reset(struct drm_encoder *encoder)
{
	return;
}

/**
 * These provide the minimum set of functions required to handle a encoder
 *
 *  Helper operations for encoders
 */
static const struct drm_encoder_helper_funcs loongson_encoder_helper_funcs = {
	.dpms = loongson_encoder_dpms,
	.mode_set = loongson_encoder_mode_set,
	.prepare = loongson_encoder_prepare,
	.commit = loongson_encoder_commit,
};

/**
 * These provide the minimum set of functions required to handle a encoder
 *
 * Encoder controls,encoder sit between CRTCs and connectors
 */
static const struct drm_encoder_funcs loongson_encoder_encoder_funcs = {
	.reset = loongson_encoder_reset,
	.destroy = loongson_encoder_destroy,
};

static unsigned int encoder_config_type_fix_by_vbios_version(
		struct loongson_encoder *ls_encoder, struct loongson_vbios *vbios)
{
	enum encoder_config config_type_fix;
	enum encoder_config config_type = ls_encoder->config_type;
	enum encoder_type encoder_type = ls_encoder->type;

	if (loongson_vbios_version(vbios) >= VBIOS_VERSION_V1_1) {
		/* Don't need fix in vbios 1.1 or later */
		config_type_fix = config_type;
	} else {
		/* Only edp/lvds do this config at vbios 1.0 and earlier,
		 * make sure to run the bridge kernel driver in this case.
		 */
		if (encoder_type == encoder_lvds)
			config_type_fix = encoder_kernel_driver;
		else if (config_type == encoder_timing_filling)
			config_type_fix = encoder_timing_filling;
		else
			config_type_fix = encoder_kernel_driver;
	}

	if (config_type != config_type_fix)
		DRM_INFO("encoder_config_type_fix: change %s(%s) to %s.\n",
				encoder_config_to_str(config_type), encoder_type_to_str(encoder_type),
				encoder_config_to_str(config_type_fix));

	return config_type_fix;
}

/**
 * loongson_encoder_init
 *
 * @dev: point to the drm_device structure
 *
 * Init encoder
 */
struct loongson_encoder *loongson_encoder_init(struct loongson_device *ldev,
					       int index)
{
	struct drm_encoder *encoder;
	struct loongson_encoder *ls_encoder;

	ls_encoder = kzalloc(sizeof(struct loongson_encoder), GFP_KERNEL);
	if (!ls_encoder)
		return NULL;

	ls_encoder->connector_id = get_encoder_connector_id(ldev, index);
	ls_encoder->type = get_encoder_type(ldev, index);
	ls_encoder->config_type = get_encoder_config_type(ldev, index);
	ls_encoder->config_type = encoder_config_type_fix_by_vbios_version(
			ls_encoder, (struct loongson_vbios *)ldev->vbios);
	ls_encoder->i2c_id = get_encoder_i2c_id(ldev, index);
	ls_encoder->encoder_config = get_encoder_config(ldev, index);
	ls_encoder->cfg_num = get_encoder_cfg_num(ldev, index);
	if (loongson_vbios_version(ldev->vbios) >= VBIOS_VERSION_V1_1)
		ls_encoder->encoder_res = get_encoder_resources(ldev, index);

	ls_encoder->encoder_id = index;
	ls_encoder->ldev = ldev;

	encoder = &ls_encoder->base;
	encoder->possible_crtcs = 1 << index;

	ls_encoder->i2c = &ldev->i2c_bus[index];
	if (!ls_encoder->i2c) {
		DRM_INFO("lson encoder-%d match i2c-%d adap err\n", index,
			 ls_encoder->i2c_id);
	}

	drm_encoder_init(ldev->dev, encoder, &loongson_encoder_encoder_funcs,
			 ls_encoder->type, NULL);

	drm_encoder_helper_add(encoder, &loongson_encoder_helper_funcs);

	if (ldev->chip == dc_7a2000 || ldev->chip == dc_2k2000)
		loongson_hdmi_init(ldev, index);

	return ls_encoder;
}

