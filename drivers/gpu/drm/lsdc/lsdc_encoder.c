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

#include <drm/drm_print.h>
#include <drm/drm_of.h>
#include <drm/drm_bridge.h>

#include "lsdc_regs.h"
#include "lsdc_drv.h"

static void lsdc_hdmi_init(struct lsdc_device *ldev, unsigned int index)
{
	if (index == 0) {
		/* Enable hdmi */
		lsdc_reg_write32(ldev, HDMI0_CTRL_REG, 0x280 | HDMI_EN | HDMI_PACKET_EN);

		/* hdmi zone idle */
		lsdc_reg_write32(ldev, HDMI0_ZONE_REG, 0x00400040);
	} else if (index == 1) {
		/* Enable hdmi */
		lsdc_reg_write32(ldev, HDMI1_CTRL_REG, 0x280 | HDMI_EN | HDMI_PACKET_EN);

		/* hdmi zone idle */
		lsdc_reg_write32(ldev, HDMI1_ZONE_REG, 0x00400040);
	}

	DRM_DEBUG_DRIVER("HDMI%d reset\n", index);
}

static void lsdc_encoder_reset(struct drm_encoder *encoder)
{
	struct lsdc_device *ldev = to_lsdc(encoder->dev);
	int index = encoder->index;

	if (ldev->desc->chip == LSDC_CHIP_7A2000)
		lsdc_hdmi_init(ldev, index);
}

static const struct drm_encoder_funcs lsdc_encoder_funcs = {
	.reset = lsdc_encoder_reset,
	.destroy = drm_encoder_cleanup,
};

static int lsdc_get_encoder_type(struct drm_device *ddev, const unsigned int index)
{
	if (index == 0)
		return DRM_MODE_ENCODER_DAC;

	if (index == 1)
		return DRM_MODE_ENCODER_TMDS;

	DRM_ERROR("%s: encoder index=%d overflow\n", __func__, index);

	return DRM_MODE_ENCODER_NONE;
}

int lsdc_encoder_init(struct drm_device *ddev,
		      struct drm_encoder *encoder,
		      unsigned int index)
{
	int ret;

	encoder->possible_crtcs = BIT(index);
	encoder->possible_clones = BIT(1) | BIT(0);

	ret = drm_encoder_init(ddev, encoder, &lsdc_encoder_funcs,
		lsdc_get_encoder_type(ddev, index), NULL);

	return ret;
}
