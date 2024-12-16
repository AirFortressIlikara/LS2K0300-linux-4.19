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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

#include <drm/drm_print.h>
#include <drm/drm_drv.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>

#include "lsdc_drv.h"

static const struct lsdc_platform_desc dc_in_ls2k1000 = {
	.chip = LSDC_CHIP_2K1000,
	.num_of_crtc = LSDC_NUM_CRTC,
	.max_pixel_clk = 200000,
	.max_width = 4096,
	.max_height = 4096,
	.num_of_hw_cursor = 1,
	.hw_cursor_w = 32,
	.hw_cursor_h = 32,
	.has_builtin_i2c = false,
	.has_vram = false,
	.broken_gamma = true,
};

static const struct lsdc_platform_desc dc_in_ls2k0500 = {
	.chip = LSDC_CHIP_2K0500,
	.num_of_crtc = LSDC_NUM_CRTC,
	.max_pixel_clk = 200000,
	.max_width = 4096,
	.max_height = 4096,
	.num_of_hw_cursor = 1,
	.hw_cursor_w = 32,
	.hw_cursor_h = 32,
	.has_builtin_i2c = false,
	.has_vram = false,
	.broken_gamma = true,
};
static const struct lsdc_platform_desc dc_in_ls2k0300 = {
    	.chip = LSDC_CHIP_2K0300,
	.num_of_crtc = 1,
	.max_pixel_clk = 200000,
	.max_width = 4096,
	.max_height = 4096,
	.num_of_hw_cursor = 1,
	.hw_cursor_w = 32,
	.hw_cursor_h = 32,
	.has_builtin_i2c = false,
	.has_vram = false,
	.broken_gamma = true,
};


static const struct lsdc_platform_desc *lsdc_detect_platform_chip(void)
{
	struct device_node *np;
	const struct lsdc_platform_desc *descp;

	for_each_compatible_node(np, NULL, "loongson,ls2k") {
		const char *model = NULL;

		if (!of_device_is_available(np))
			continue;

		of_property_read_string(np, "model", &model);
		if (!strncmp(model, "loongson,2k500", 15)) {
			descp = &dc_in_ls2k0500;
			DRM_INFO("LS2K0500 found\n");
		} else if (!strncmp(model, "loongson,2k300", 15)) {
		    descp = &dc_in_ls2k0300;
		    DRM_INFO("LS2K0300 found\n");
		} else {
			descp = &dc_in_ls2k1000;
			DRM_INFO("LS2K1000 found\n");
		}

		of_node_put(np);

		break;
	}

	return descp;
}

static const struct of_device_id lsdc_dt_ids[] = {
	{ .compatible = "loongson,ls2k1000-dc", .data = &dc_in_ls2k1000, },
	{ .compatible = "loongson,la2k0500-dc", .data = &dc_in_ls2k0500, },
	{ .compatible = "loongson,la2k0300-dc", .data = &dc_in_ls2k0300, },
	{ .compatible = "loongson,display-subsystem", }, /* must be the last */
	{}
};

static int lsdc_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct lsdc_platform_desc *descp;
	const struct of_device_id *of_id;
	struct lsdc_device *ldev;
	struct resource *res;
	int ret;

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "Failed to set dma mask\n");
		return ret;
	}

	of_id = of_match_device(lsdc_dt_ids, dev);
	if (of_id && of_id->data)
		descp = (const struct lsdc_platform_desc *)of_id->data;
	else
		descp = lsdc_detect_platform_chip();

	if (!descp) {
		dev_err(dev, "unknown dc chip core\n");
		return -ENOENT;
	}

	ldev = kzalloc(sizeof(struct lsdc_device), GFP_KERNEL);
	if (ldev == NULL)
		return -ENOMEM;

	ldev->desc = descp;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ldev->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ldev->reg_base)) {
		dev_err(dev, "Unable to get lsdc registers\n");
		ret = PTR_ERR(ldev->reg_base);
		goto dev_destroy;
	}

	ldev->irq = platform_get_irq(pdev, 0);
	if (ldev->irq < 0) {
		dev_err(dev, "failed to get irq\n");
		ret = ldev->irq;
		goto dev_destroy;
	}

	/* Allocate and initialize the driver private structure.
	 *
	 * The drm_device structure must be at the start of
	 * lsdc_device for drm_dev_put() to work correctly.
	 */
	BUILD_BUG_ON(offsetof(struct lsdc_device, ddev) != 0);

	ret = drm_dev_init(&ldev->ddev, &lsdc_drm_driver, dev);
	if (ret) {
		dev_err(dev, "drm_dev_init failed: %d\n", ret);
		goto dev_destroy;
	}

	dev_set_drvdata(dev, ldev);

	ret = lsdc_mode_config_init(ldev);
	if (ret) {
		goto dev_destroy;
	}

	return 0;

dev_destroy:
	drm_dev_put(&ldev->ddev);

	return ret;
}

static int lsdc_platform_remove(struct platform_device *pdev)
{
	struct lsdc_device *ldev = dev_get_drvdata(&pdev->dev);

	lsdc_mode_config_fini(ldev);

	return 0;
}

#ifdef CONFIG_PM

static int lsdc_drm_suspend(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(ddev);
}

static int lsdc_drm_resume(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(ddev);
}

#endif

static SIMPLE_DEV_PM_OPS(lsdc_pm_ops, lsdc_drm_suspend, lsdc_drm_resume);

struct platform_driver lsdc_platform_driver = {
	.probe = lsdc_platform_probe,
	.remove = lsdc_platform_remove,
	.driver = {
		.name = "lsdc",
		.pm = &lsdc_pm_ops,
		.of_match_table = of_match_ptr(lsdc_dt_ids),
	},
};

MODULE_DEVICE_TABLE(of, lsdc_dt_ids);
