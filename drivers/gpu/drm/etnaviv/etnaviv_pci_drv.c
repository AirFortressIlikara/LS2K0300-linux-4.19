// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2018 Etnaviv Project
 */

#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <drm/drm_drv.h>

#include "etnaviv_drv.h"
#include "etnaviv_gpu.h"


#ifdef CONFIG_DRM_ETNAVIV_PCI_DRIVER

static int etnaviv_alloc_gpu(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct etnaviv_gpu *gpu;
	int err;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	gpu->dev = dev;
	mutex_init(&gpu->lock);
	mutex_init(&gpu->fence_lock);

	/* bar 0 contain registers */
	gpu->mmio = devm_ioremap_resource(dev, &pdev->resource[0]);
	if (IS_ERR(gpu->mmio))
		return PTR_ERR(gpu->mmio);

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));

	/* Get Interrupt: */
	err = etnaviv_gpu_register_irq(gpu, pdev->irq);
	if (err)
		return err;

	/* Get Clocks: */
	etnaviv_gpu_get_clock(gpu, dev);

	/* TODO: figure out max mapped size */
	dev_set_drvdata(dev, gpu);

	/*
	 * We treat the device as initially suspended.  The runtime PM
	 * autosuspend delay is rather arbitrary: no measurements have
	 * yet been performed to determine an appropriate value.
	 */
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_enable(dev);

	return 0;
}


static void etnaviv_free_gpu(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);

	pm_runtime_disable(dev);

	if (gpu)
		devm_kfree(dev, gpu);

	dev_set_drvdata(dev, NULL);
}

static int etnaviv_create_private(struct drm_device *ddev)
{
	struct etnaviv_drm_private *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(ddev->dev, "failed to allocate private data\n");
		return -ENOMEM;
	}

	ddev->dev_private = priv;

	priv->num_gpus = 0;

	ddev->dev->dma_parms = &priv->dma_parms;

	mutex_init(&priv->gem_lock);
	INIT_LIST_HEAD(&priv->gem_list);

	dev_info(ddev->dev, "etnaviv drm private created\n");

	return 0;
}

static void etnaviv_destroy_private(struct drm_device *ddev)
{
	struct etnaviv_drm_private *priv = ddev->dev_private;

	ddev->dev_private = NULL;
	ddev->dev->dma_parms = NULL;

	kfree(priv);

	dev_info(ddev->dev, "etnaviv drm private freed\n");
}

int etnaviv_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct drm_device *ddev;
	struct etnaviv_drm_private *priv;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable gpu device\n");
		return ret;
	}

	pci_set_master(pdev);

	/*
	 * Instantiate the DRM device.
	 */
	ddev = drm_dev_alloc(&etnaviv_drm_driver, &pdev->dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	ddev->pdev = pdev;

	ret = etnaviv_create_private(ddev);
	if (ret)
		goto out_unref;

	priv = ddev->dev_private;
	/*
	 * loongson CPU's cache coherency is maintained by the hardware,
	 * include but not limitied: ls3a5000, ls3a4000, ls3a3000, ls2k1000.
	 */
	if (etnaviv_cached_coherent != 0) {
		if (IS_ENABLED(CONFIG_CPU_LOONGSON3) ||
		    IS_ENABLED(CONFIG_CPU_LOONGSON64)) {
			dev_info(&pdev->dev, "using cached coherent mode\n");
			priv->has_cached_coherent = true;
		}
	}

	/* original dma mask is 40 */
	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));

	dev_info(&pdev->dev, "DMA mask and the coherent DMA mask: 32\n");

	dma_set_max_seg_size(&pdev->dev, SZ_2G);

	dev_info(&pdev->dev, "dev->dma_parms->max_segment_size = 2G\n");

	ret = etnaviv_alloc_gpu(pdev);
	if (ret)
		goto out_destroy_private;

	ret = etnaviv_gpu_bind(&pdev->dev, NULL, ddev);
	if (ret)
		goto out_free_gpu;

	dev_info(&pdev->dev, "Number of GPU: %d\n", priv->num_gpus);

	ret = etnaviv_gpu_init(priv->gpu[0]);
	if (ret)
		goto out_unbind_gpu;

	dev_info(&pdev->dev, "GPU Initialized\n");

	ret = drm_dev_register(ddev, 0);
	if (ret)
		goto out_unbind_gpu;

	return 0;

out_unbind_gpu:
	etnaviv_gpu_unbind(&pdev->dev, NULL, ddev);
out_free_gpu:
	etnaviv_free_gpu(pdev);
out_destroy_private:
	etnaviv_destroy_private(ddev);
out_unref:
	drm_dev_put(ddev);

	return ret;
}

void etnaviv_pci_remove(struct pci_dev *pdev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(&pdev->dev);
	struct drm_device *ddev = gpu->drm;

	drm_dev_unregister(ddev);

	etnaviv_gpu_unbind(ddev->dev, NULL, ddev);

	etnaviv_free_gpu(pdev);

	etnaviv_destroy_private(ddev);

	drm_dev_put(ddev);
}

#endif
