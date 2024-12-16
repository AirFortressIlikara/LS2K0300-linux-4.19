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

#include "lsdc_drv.h"

/*
 * of_graph_is_present() - check graph's presence
 * @node: pointer to device_node containing graph port
 *
 * Return: True if @node has a port or ports (with a port) sub-node,
 * false otherwise.
 */
bool of_graph_is_present(const struct device_node *node)
{
	struct device_node *ports, *port;

	ports = of_get_child_by_name(node, "ports");
	if (ports)
		node = ports;

	port = of_get_child_by_name(node, "port");
	of_node_put(ports);
	of_node_put(port);

	return !!port;
}

static const struct drm_encoder_funcs lsdc_bridge_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int lsdc_attach_bridge_impl(struct lsdc_device *ldev,
				   struct device_node *ports,
				   struct drm_encoder *encoder,
				   unsigned int i)
{
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	int ret;

	ret = drm_of_find_panel_or_bridge(ports, i, 0, &panel, &bridge);

	if (panel) {
		bridge = drm_panel_bridge_add(panel, DRM_MODE_CONNECTOR_DPI);
		if (IS_ERR(bridge))
			return PTR_ERR(bridge);

		DRM_INFO("Output-%u is a DPI panel\n", i);
	}

	encoder->possible_crtcs = BIT(i);
	encoder->possible_clones = BIT(1) | BIT(1);

	ret = drm_encoder_init(&ldev->ddev,
			       encoder,
			       &lsdc_bridge_encoder_funcs,
			       DRM_MODE_ENCODER_DPI,
			       NULL);
	if (ret)
		return ret;

	ret = drm_bridge_attach(encoder, bridge, NULL);
	if (ret) {
		DRM_ERROR("Attach bridge %pOF to output-%u failed: %d\n",
			  bridge->of_node, i, ret);
		return ret;
	}

	DRM_INFO("Bridge %pOF attached to %s\n",
		 bridge->of_node, encoder->name);

	return 0;
}

int lsdc_attach_bridges(struct lsdc_device *ldev, unsigned int num_crtc)
{
	struct drm_device *ddev = &ldev->ddev;
	struct device_node *ports;
	struct lsdc_display_pipe *dispipe;
	unsigned int i;
	int ret;

	ldev->num_output = 0;

	ports = of_get_child_by_name(ddev->dev->of_node, "ports");

	/* First, find all available display bridges and mark it */
	for (i = 0; i < num_crtc; i++) {
		struct drm_bridge *b;
		struct drm_panel *p;

		dispipe = &ldev->dispipe[i];

		ret = drm_of_find_panel_or_bridge(ports, i, 0, &p, &b);
		if (ret) {
			if (ret == -ENODEV) {
				DRM_INFO("No panel or bridge on port%u\n", i);
				dispipe->available = false;
				continue;
			}

			if (ret == -EPROBE_DEFER)
				DRM_INFO("port%d is defer probed\n", i);

			goto ERR_RET;
		}

		dispipe->available = true;
		ldev->num_output++;
	}

	if (ldev->num_output == 0) {
		DRM_ERROR("No validate output available, abort\n");
		ret = -ENODEV;
		goto ERR_RET;
	}

	/* Then, attach it */
	for (i = 0; i < num_crtc; i++) {
		dispipe = &ldev->dispipe[i];
		if (dispipe->available) {
			ret = lsdc_attach_bridge_impl(ldev,
						      ports,
						      &dispipe->encoder,
						      i);
			if (ret)
				goto ERR_RET;
		}
	}

	DRM_INFO("number of outputs: %u\n", ldev->num_output);

ERR_RET:
	of_node_put(ports);
	return ret;
}
