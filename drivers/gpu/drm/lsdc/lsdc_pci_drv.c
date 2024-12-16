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
#include <linux/pci.h>

#include <drm/drm_print.h>
#include <drm/drm_drv.h>
#include <drm/drm_atomic_helper.h>

#include "lsdc_drv.h"
#include "lsdc_regs.h"

#ifdef CONFIG_DRM_LSDC_PCI_DRIVER

static const struct lsdc_platform_desc dc_in_ls7a1000 = {
	.chip = LSDC_CHIP_7A1000,
	.num_of_crtc = LSDC_NUM_CRTC,
	/* ls2k1000 user manual say the pix clock can be about 200MHz */
	.max_pixel_clk = 200000,
	.max_width = 4096,
	.max_height = 4096,
	.hw_cursor_w = 32,
	.hw_cursor_h = 32,
	.has_builtin_i2c = true,
	.has_vram = true,
	.broken_gamma = true,
};

static const struct lsdc_platform_desc dc_in_ls7a2000 = {
	.chip = LSDC_CHIP_7A2000,
	.num_of_crtc = LSDC_NUM_CRTC,
	.max_pixel_clk = 200000,
	.max_width = 4096,
	.max_height = 4096,
	.hw_cursor_w = 64,
	.hw_cursor_h = 64,
	.has_builtin_i2c = true,
	.has_vram = true,
	.broken_gamma = true,
};

static int lsdc_drm_suspend(struct drm_device *ddev, bool suspend)
{
	drm_mode_config_helper_suspend(ddev);

	if (suspend && ddev->pdev) {
		pci_save_state(ddev->pdev);
			pci_set_power_state(ddev->pdev, PCI_D3hot);
			/* Shut down the device */
			pci_disable_device(ddev->pdev);
	}

	return 0;
}

static int lsdc_drm_resume(struct drm_device *ddev, bool resume)
{
	if (resume && ddev->pdev) {
		pci_set_power_state(ddev->pdev, PCI_D0);
		pci_restore_state(ddev->pdev);

		if (pci_enable_device(ddev->pdev))
			return -EIO;
	}

	drm_mode_config_helper_resume(ddev);

	return 0;
}

static int lsdc_drm_pm_suspend(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);

	return lsdc_drm_suspend(ddev, true);
}

static int lsdc_drm_pm_resume(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);

	return lsdc_drm_resume(ddev, true);
}

static int lsdc_drm_pm_freeze(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);

	return lsdc_drm_suspend(ddev, false);
}

static int lsdc_drm_pm_thaw(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);

	return lsdc_drm_resume(ddev, false);
}

static int lsdc_pci_probe(struct pci_dev *pdev,
			  const struct pci_device_id * const ent)
{
	struct device *dev = &pdev->dev;
	struct lsdc_device *ldev;
	const struct lsdc_platform_desc *desc;
	struct resource *bar0;
	struct pci_dev *gpu_dev;
	int ret;
	u8 chip_revision;

	ret = pcim_enable_device(pdev);
	if (ret) {
		DRM_ERROR("Enable pci devive failed.\n");
		return ret;
	}

	pci_set_master(pdev);

	ldev = kzalloc(sizeof(struct lsdc_device), GFP_KERNEL);
	if (ldev == NULL)
		return -ENOMEM;

	desc = (struct lsdc_platform_desc *)ent->driver_data;

	ldev->desc = desc;

	if (desc == NULL) {
		DRM_ERROR("unknown dc chip core\n");
		return -ENOENT;
	}

	if (lsdc_shadowfb == 1)
		ldev->shadowfb = true;

	ldev->irq = pdev->irq;

	/* BAR 0 contains registers */
	bar0 = devm_request_mem_region(dev,
				       pci_resource_start(pdev, 0),
				       pci_resource_len(pdev, 0),
				       dev_name(dev));

	if (bar0 == NULL) {
		DRM_ERROR("Can't reserve mmio registers\n");
		return -ENOMEM;
	}

	ldev->reg_base = pci_iomap_range(pdev, 0, 0, 0);
	if (ldev->reg_base == NULL) {
		DRM_ERROR("Unable to get lsdc registers\n");
		return -ENOMEM;
	}


	if (lsdc_shadowfb > 0) {
		/* LS7A1000 GPU BAR 2 contain VRAM */
		if (desc->chip == LSDC_CHIP_7A1000) {
			gpu_dev = pci_get_device(PCI_VENDOR_ID_LOONGSON, 0x7a15, NULL);
			DRM_INFO("GMEM IN LS7A1000\n");
		} else if (desc->chip == LSDC_CHIP_7A2000) {
			gpu_dev = pci_get_device(PCI_VENDOR_ID_LOONGSON, 0x7a25, NULL);
			DRM_INFO("GMEM IN LS7A2000\n");
		}

		/* Get the revison of this DC */
		pci_read_config_byte(pdev, 0x8, &chip_revision);
		DRM_INFO("DC revision is %d\n", chip_revision);

		ldev->vram_base = pci_resource_start(gpu_dev, 2);
		ldev->vram_size = pci_resource_len(gpu_dev, 2);

		if (!request_mem_region(ldev->vram_base, ldev->vram_size,
					"lsdc_drmfb_vram")) {
			DRM_ERROR("can't reserve VRAM\n");
			return -ENXIO;
		}

		DRM_INFO("vram start: 0x%llx, size: %lluMB\n",
			 ldev->vram_base, ldev->vram_size >> 20);

		ldev->vram = devm_ioremap_wc(dev, ldev->vram_base, ldev->vram_size);
		if (ldev->vram == NULL)
			return -ENOMEM;

		DRM_INFO("vram virtual addr: 0x%llx\n", (u64)ldev->vram);
	}

	/* Allocate and initialize the driver private structure. */
	ret = drm_dev_init(&ldev->ddev, &lsdc_drm_driver, dev);
	if (ret) {
		dev_err(dev, "drm_dev_init failed: %d\n", ret);
		goto err_free;
	}

	ldev->ddev.pdev = pdev;

	dev_set_drvdata(dev, ldev);

	ret = lsdc_mode_config_init(ldev);
	if (ret) {
		goto dev_destroy;
	}

	return ret;

dev_destroy:
	drm_dev_put(&ldev->ddev);
err_free:
	kfree(ldev);

	return ret;
}

static void lsdc_pci_remove(struct pci_dev *pdev)
{
	struct lsdc_device *ldev = dev_get_drvdata(&pdev->dev);

	if (ldev && ldev->vram) {
		devm_iounmap(&pdev->dev, ldev->vram);
		ldev->vram = NULL;
	}

	lsdc_mode_config_fini(ldev);

	pci_clear_master(pdev);

	pci_release_regions(pdev);
}

const struct dev_pm_ops lsdc_drm_pm_ops = {
	.suspend = lsdc_drm_pm_suspend,
	.resume = lsdc_drm_pm_resume,
	.freeze = lsdc_drm_pm_freeze,
	.thaw = lsdc_drm_pm_thaw,
	.poweroff = lsdc_drm_pm_freeze,
	.restore = lsdc_drm_pm_resume,
};

static const struct pci_device_id lsdc_pciid_list[] = {
	{PCI_VENDOR_ID_LOONGSON, 0x7a06, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (kernel_ulong_t)&dc_in_ls7a1000},
	{PCI_VENDOR_ID_LOONGSON, 0x7a36, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (kernel_ulong_t)&dc_in_ls7a2000},
	{0, 0, 0, 0, 0, 0, 0}
};

struct pci_driver lsdc_pci_driver = {
	.name = "lsdc",
	.id_table = lsdc_pciid_list,
	.probe = lsdc_pci_probe,
	.remove = lsdc_pci_remove,
	.driver.pm = &lsdc_drm_pm_ops,
};

MODULE_DEVICE_TABLE(pci, lsdc_pciid_list);
#endif
