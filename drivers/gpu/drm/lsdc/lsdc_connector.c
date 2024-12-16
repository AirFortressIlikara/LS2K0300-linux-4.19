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

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_connector.h>

#include <video/videomode.h>
#include <video/of_display_timing.h>

#include "lsdc_drv.h"
#include "lsdc_i2c.h"

static struct drm_encoder *
lsdc_connector_best_single_encoder(struct drm_connector *conn)
{
	struct drm_encoder *encoder;
	unsigned int i;

	drm_connector_for_each_possible_encoder(conn, encoder, i) {
		/* simply pick the first one */
		return encoder;
	}

	return NULL;
}


/**
 * lsdc_connector_mode_valid: validate a mode for a connector
 *
 * Callback to validate a mode for a connector, irrespective of the
 * specific display configuration.
 *
 * This callback is used by the probe helpers to filter the mode list
 * (which is usually derived from the EDID data block from the sink).
 * See e.g. drm_helper_probe_single_connector_modes().
 *
 * This function is optional.
 *
 * NOTE:
 *
 * This only filters the mode list supplied to userspace in the
 * GETCONNECTOR IOCTL. Compared to &drm_encoder_helper_funcs.mode_valid,
 * &drm_crtc_helper_funcs.mode_valid and &drm_bridge_funcs.mode_valid,
 * which are also called by the atomic helpers from
 * drm_atomic_helper_check_modeset(). This allows userspace to force and
 * ignore sink constraint (like the pixel clock limits in the screen's
 * EDID), which is useful for e.g. testing, or working around a broken
 * EDID. Any source hardware constraint (which always need to be
 * enforced) therefore should be checked in one of the above callbacks,
 * and not this one here.
 *
 * To avoid races with concurrent connector state updates, the helper
 * libraries always call this with the &drm_mode_config.connection_mutex
 * held. Because of this it's safe to inspect &drm_connector->state.
 *
 * RETURNS:
 *
 * Either &drm_mode_status.MODE_OK or one of the failure reasons in &enum
 * drm_mode_status.
 */
static enum drm_mode_status lsdc_connector_mode_valid(
				struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	struct lsdc_device *ldev = to_lsdc(connector->dev);
	uint32_t crtc_max_w, crtc_max_h, max_pixel_clock;

	if (ldev->desc) {
		max_pixel_clock = ldev->desc->max_pixel_clk;
		crtc_max_w = ldev->desc->max_width;
		crtc_max_h = ldev->desc->max_height;
	} else {
		max_pixel_clock = 200000;
		crtc_max_w = 4096;
		crtc_max_h = 4096;
	}

	if (mode->clock > max_pixel_clock) {
		DRM_DEBUG_KMS("%s: mode %dx%d, pixel clock=%d is too high\n",
			__func__, mode->hdisplay, mode->vdisplay, mode->clock);
		return MODE_CLOCK_HIGH;
	}

	if ((mode->hdisplay > crtc_max_w) && (mode->vdisplay > crtc_max_h))
		return MODE_NOMODE;

	return MODE_OK;
}


static int lsdc_get_modes_from_edid(struct drm_connector *connector)
{
	struct lsdc_connector *lcon = to_lsdc_connector(connector);
	struct edid *edid_p = (struct edid *)lcon->edid_data;
	int num = drm_add_edid_modes(connector, edid_p);

	if (num)
		drm_connector_update_edid_property(connector, edid_p);
	else
		DRM_WARN("%s: no valid modes added.\n", __func__);

	return num;
}


static int lsdc_get_modes_from_timings(struct drm_connector *connector)
{
	struct lsdc_connector *lcon = to_lsdc_connector(connector);
	struct display_timings *disp_tim = lcon->disp_tim;
	unsigned int i;
	unsigned int num = 0;

	for (i = 0; i < disp_tim->num_timings; i++) {
		const struct display_timing *dt = disp_tim->timings[i];
		struct drm_display_mode *mode;
		struct videomode vm;

		videomode_from_timing(dt, &vm);
		mode = drm_mode_create(connector->dev);
		if (!mode) {
			DRM_ERROR("%s: failed to add mode %ux%u\n",
				__func__, dt->hactive.typ, dt->vactive.typ);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (i == disp_tim->native_mode)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
		num++;
	}

	DRM_DEBUG_DRIVER("%s: %d modes added.\n", __func__, num);

	return num;
}


static int lsdc_get_modes_from_ddc(struct drm_connector *connector)
{
	struct lsdc_connector *lcon = to_lsdc_connector(connector);
	struct i2c_adapter *ddc = lcon->ddc;
	unsigned int num = 0;

	if (IS_ERR_OR_NULL(ddc) == false) {
		struct edid *edid = drm_get_edid(connector, ddc);

		if (edid) {
			drm_connector_update_edid_property(connector, edid);

			num = drm_add_edid_modes(connector, edid);

			kfree(edid);

			DRM_DEBUG_KMS("%d modes add.\n", num);

			return num;
		}

		DRM_DEBUG_KMS("grab EDID data from ddc failed.\n");
	}

	/*
	 * In case we cannot retrieve the EDIDs (broken or missing i2c
	 * bus), fallback on the XGA standards
	 */
	num = drm_add_modes_noedid(connector, 1920, 1200);

	/* And prefer a mode pretty much anyone can handle */
	drm_set_preferred_mode(connector, 1024, 768);

	return num;
}


static int lsdc_get_modes(struct drm_connector *connector)
{
	struct lsdc_connector *lcon = to_lsdc_connector(connector);

	if (lcon->has_edid)
		return lsdc_get_modes_from_edid(connector);

	if (lcon->has_disp_tim)
		return lsdc_get_modes_from_timings(connector);

	return lsdc_get_modes_from_ddc(connector);
}


static enum drm_connector_status lsdc_connector_detect(
		struct drm_connector *connector, bool force)
{
	struct lsdc_connector *lcon = to_lsdc_connector(connector);

	if (lcon->has_edid == true)
		return connector_status_connected;

	if (lcon->has_disp_tim == true)
		return connector_status_connected;

	if (lcon->ddc == NULL)
		return connector_status_connected;

	if (lcon->ddc && drm_probe_ddc(lcon->ddc))
		return connector_status_connected;

	return connector_status_unknown;
}

/*
 * @connector: point to the drm_connector structure
 *
 * Clean up connector resources
 */
static void lsdc_connector_destroy(struct drm_connector *connector)
{
	struct lsdc_connector *lcon = to_lsdc_connector(connector);

	DRM_INFO("%s: destroy connector\n", __func__);
	if (lcon) {
		lsdc_destroy_i2c(connector->dev, lcon->ddc);
		devm_kfree(connector->dev->dev, lcon);
	}

	drm_connector_cleanup(connector);
}


/*
 * These provide the minimum set of functions required to handle a connector
 *
 * Helper operations for connectors. These functions are used
 * by the atomic and legacy modeset helpers and by the probe helpers.
 */
static const struct drm_connector_helper_funcs lsdc_connector_helpers = {
	.get_modes = lsdc_get_modes,
	.best_encoder = lsdc_connector_best_single_encoder,
	.mode_valid = lsdc_connector_mode_valid,
};

/**
 * These provide the minimum set of functions required to handle a connector
 *
 * Control connectors on a given device.
 *
 * Each CRTC may have one or more connectors attached to it.
 * The functions below allow the core DRM code to control
 * connectors, enumerate available modes, etc.
 */
static const struct drm_connector_funcs lsdc_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = lsdc_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = lsdc_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int lsdc_get_connector_type(struct drm_device *ddev,
				   unsigned int index)
{
	struct device_node *dc_np = ddev->dev->of_node;
	struct device_node *output;
	const char *name;
	int ret;

	output = of_parse_phandle(dc_np, "output-ports", index);
	if (!output)
		goto default_config;

	ret = of_property_read_string(output, "connector", &name);
	of_node_put(output);
	if (ret < 0) {
		DRM_INFO("No connector property in DT, please update dts\n");
		goto default_config;
	}

	if (strncmp(name, "vga-connector", 13) == 0) {
		ret = DRM_MODE_CONNECTOR_VGA;
		DRM_INFO("connector%d is VGA\n", index);
	} else if (strncmp(name, "dvi-connector", 13) == 0) {
		bool analog, digital;

		analog = of_property_read_bool(output, "analog");
		digital = of_property_read_bool(output, "digital");

		if (analog && !digital)
			ret = DRM_MODE_CONNECTOR_DVIA;
		else if (analog && digital)
			ret = DRM_MODE_CONNECTOR_DVII;
		else
			ret = DRM_MODE_CONNECTOR_DVID;

		DRM_INFO("connector%d is DVI\n", index);
	} else if (strncmp(name, "virtual-connector", 17) == 0) {
		ret = DRM_MODE_CONNECTOR_VIRTUAL;
		DRM_INFO("connector%d is virtual\n", index);
	} else if (strncmp(name, "dpi-connector", 13) == 0) {
		ret = DRM_MODE_CONNECTOR_DPI;
		DRM_INFO("connector%d is DPI\n", index);
	} else if (strncmp(name, "hdmi-connector", 14) == 0) {
		int res;
		const char *hdmi_type;

		ret = DRM_MODE_CONNECTOR_HDMIA;

		res = of_property_read_string(output, "type", &hdmi_type);
		if (res == 0 && !strcmp(hdmi_type, "b"))
			ret = DRM_MODE_CONNECTOR_HDMIB;

		DRM_INFO("connector%d is HDMI, type is %s\n", index, hdmi_type);
	} else {
		ret = DRM_MODE_CONNECTOR_Unknown;
		DRM_INFO("The type of connector%d is unknown\n", index);
	}

	return ret;

default_config:

	if (index == 0)
		ret = DRM_MODE_CONNECTOR_VGA;
	else if (index == 1)
		ret = DRM_MODE_CONNECTOR_HDMIA;
	else
		ret = DRM_MODE_CONNECTOR_Unknown;

	return ret;
}


/* Get the EDID data from the device tree, if present */
static void lsdc_get_edid_from_dtb(struct drm_device *ddev,
				   struct lsdc_connector *lconnector,
				   const unsigned int con_id)
{
	struct device_node *dc_np = ddev->dev->of_node;
	struct device_node *output_np;

	output_np = of_parse_phandle(dc_np, "output-ports", con_id);
	if (output_np) {
		int length;
		const void *prop;

		prop = of_get_property(output_np, "edid", &length);
		if (prop && length == EDID_LENGTH) {
			memcpy(lconnector->edid_data, prop, EDID_LENGTH);
			lconnector->has_edid = true;

			DRM_INFO("found edid for connector-%d\n", con_id);
		}

		of_node_put(output_np);
	}
}


static void lsdc_get_display_timings_from_dtb(struct drm_device *ddev,
					      struct lsdc_connector *lcon,
					      const unsigned int con_id)
{
	struct device_node *dc_np = ddev->dev->of_node;
	struct device_node *output_np;

	output_np = of_parse_phandle(dc_np, "output-ports", con_id);
	if (output_np) {
		lcon->disp_tim = of_get_display_timings(output_np);
		of_node_put(output_np);

		if (lcon->disp_tim) {
			DRM_INFO("found %u display timings for connector-%d\n",
				lcon->disp_tim->num_timings, con_id);
			lcon->has_disp_tim = true;
		}
	}
}

struct lsdc_connector *lsdc_create_connector(struct drm_device *ddev,
					     unsigned int index)
{
	struct lsdc_device *ldev = to_lsdc(ddev);
	const struct lsdc_platform_desc * const descp = ldev->desc;
	struct drm_connector *connector;
	struct lsdc_connector *lconnector;
	struct i2c_adapter *i2c;
	int ret;

	lconnector = devm_kzalloc(ddev->dev,
			sizeof(struct lsdc_connector), GFP_KERNEL);
	if (lconnector == NULL)
		return ERR_PTR(-ENOMEM);

	lconnector->ddc = NULL;

	connector = &lconnector->base;

	/* Get the EDID data from the device tree at connector initial time */
	lsdc_get_edid_from_dtb(ddev, lconnector, index);

	if (lconnector->has_edid == false) {
		/* Get the display timmings from the device tree, if present */
		lsdc_get_display_timings_from_dtb(ddev, lconnector, index);
	}

	if ((index == 0) && (ldev->ddc0 == false))
		goto DDC_SKIPED;

	if ((index == 1) && (ldev->ddc1 == false))
		goto DDC_SKIPED;

	if (lconnector->has_edid)
		goto DDC_SKIPED;

	if (lconnector->has_disp_tim)
		goto DDC_SKIPED;

	if ((descp->chip == LSDC_CHIP_7A1000) ||
	    (descp->chip == LSDC_CHIP_7A1000_PLUS) ||
	    (descp->chip == LSDC_CHIP_7A2000)) {
		i2c = ls7a_create_i2c_chan(ddev, index);
	} else if ((descp->chip == LSDC_CHIP_2K1000) ||
		   (descp->chip == LSDC_CHIP_2K0500) ||
		   (descp->chip == LSDC_CHIP_2K0300)) {
		i2c = lsdc_of_retrieve_ddc(ddev, index);
		if (PTR_ERR(i2c) == -ENODEV) {
			DRM_WARN("No ddc-i2c-bus property in the DT\n");
			/* for old device tree */
			i2c = ls2k_get_i2c_chan(ddev, index);
		}

		if (PTR_ERR(i2c) == -EPROBE_DEFER) {
			DRM_INFO("i2c%u driver is not ready, defer probe\n",
				 index);
			return ERR_PTR(-EPROBE_DEFER);
		}
	}

	lconnector->ddc = i2c;

	if (IS_ERR(i2c) == false)
		DRM_INFO("i2c adapter for connector-%d created\n", index);

DDC_SKIPED:

	ret = drm_connector_init(ddev,
				 connector,
				 &lsdc_connector_funcs,
				 lsdc_get_connector_type(ddev, index));

	if (ret) {
		DRM_ERROR("init connector-%d failed\n", index);
		return ERR_PTR(ret);
	}

	drm_connector_helper_add(connector, &lsdc_connector_helpers);

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;
	connector->polled = DRM_CONNECTOR_POLL_CONNECT;

	if (lconnector->ddc)
		connector->polled |= DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_register(connector);

	return lconnector;
}
