/*
 * Driver for the Loongson APB DMA Controller (APB DMAC on 2K or 7A systems)
 *
 * Copyright (C) 2017 Loongson Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "ls_apbdmac.h"
#include "dmaengine.h"

/**
 * apbdma_tx_status - poll for transaction completion
 * @chan: DMA channel
 * @cookie: transaction identifier to check status of
 * @txstate: if not %NULL updated with transaction state
 *
 */
static enum dma_status
apbdma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie,
		struct dma_tx_state *txstate)
{
	return DMA_COMPLETE;
}

/**
 * apbdma_issue_pending - try to finish work
 * @chan: target DMA channel
 */
static void apbdma_issue_pending(struct dma_chan *chan)
{
}

/**
 * apbdma_alloc_chan_resources - allocate resources for DMA channel
 * @chan: allocate descriptor resources for this channel
 *
 * return - the number of allocated descriptors
 */
static int apbdma_alloc_chan_resources(struct dma_chan *chan)
{
	return 0;
}

/**
 * apbdma_free_chan_resources - free all channel resources
 * @chan: DMA channel
 */
static void apbdma_free_chan_resources(struct dma_chan *chan)
{
}

#ifdef CONFIG_OF
static bool apb_dma_filter(struct dma_chan *chan, void *slave)
{
	struct device *dev = slave;
	if (dev == chan->device->dev) {
		return true;
	} else {
		return false;
	}
}

static struct dma_chan *apb_dma_xlate(struct of_phandle_args *dma_spec,
				     struct of_dma *of_dma)
{
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	struct platform_device *dmac_pdev;

	dmac_pdev = of_find_device_by_node(dma_spec->np);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/*
	 * We can fill both SRC_PER and DST_PER, one of these fields will be
	 * ignored depending on DMA transfer direction.
	 */
	chan = dma_request_channel(mask, apb_dma_filter, &(dmac_pdev->dev));
	if (!chan)
		return NULL;

	return chan;
}
#else
static struct dma_chan *apb_dma_xlate(struct of_phandle_args *dma_spec,
				     struct of_dma *of_dma)
{
	return NULL;
}
#endif

#if defined(CONFIG_OF)
static const struct of_device_id ls_apbdma_dt_ids[] = {
	{
		.compatible = "loongson,loongson2-apbdma",
	}, {
		.compatible = "loongson,ls-apbdma-0",
	}, {
		.compatible = "loongson,ls-apbdma-1",
	}, {
		.compatible = "loongson,ls-apbdma-2",
	}, {
		.compatible = "loongson,ls-apbdma-3",
	}, {
		.compatible = "loongson,ls-apbdma-4",
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, ls_apbdma_dt_ids);
#endif

static const struct platform_device_id ls_apbdma_devtypes[] = {
	{
		.name = "ls-apbdma-0",
	}, {
		.name = "ls-apbdma-1",
	}, {
		.name = "ls-apbdma-2",
	}, {
		.name = "ls-apbdma-3",
	}, {
		.name = "ls-apbdma-4",
	}, {
		/* sentinel */
	}
};

/**
 * set_apbdma_config - set up apbdma configuration register
 * @pdata: Controller configuration parameters
 *
 *apbdma sel mapping table:
 *dma_sel0 ------ NAND			dma_sel1 ------ AES  read
 *dma_sel2 ------ AES  write		dma_sel3 ------ DES  read
 *dma_sel4 ------ DES  write		dma_sel5 ------ SDIO
 *dma_sel6 ------ I2S  send		dma_sel7 ------ I2S  receive
 *dma_sel8 ------ AC97  oc		dma_sel9 ------ AC97 ic
 */
static int set_apbdma_config(struct ls_apbdma_platform_data *pdata)
{
	size_t			size;
	void __iomem		*regs;
	struct resource		*io;
	unsigned int		sel, dma_nr, val;
	int			err = 0;

	if (!pdata)
		return -EINVAL;

	if (!IS_ERR_OR_NULL(pdata->regmap)) {
		err = regmap_update_bits(pdata->regmap, 0, pdata->conf_reg_mask,
							pdata->conf_reg_value);
		return err;
	}

	io = platform_get_resource(pdata->common, IORESOURCE_MEM, 0);
	if (!io)
		return -EINVAL;

	size = resource_size(io);
	if (!request_mem_region(io->start, size, pdata->common->dev.of_node->name)) {
		return -EBUSY;
	}
	regs = ioremap(io->start, size);
	if (!regs) {
		err = -ENOMEM;
		goto err_release_r;
	}
	/*set up APB dma sel bits*/
	sel = pdata->dma_sel;
	dma_nr = pdata->apb_dma_nr;

	if (io->start == LS2K0500_APB_DMA_CONF) {
		val = readl(regs) & ~sel;
		val |= dma_nr;
	} else { /* default is LS2K1000 */
		val = readl(regs) & SEL_MASK_SHIFT(sel);
		val |= SEL_VAL(dma_nr, sel);
	}
	writel(val, regs);
err_release_r:
	release_mem_region(io->start, size);

	return err;
}

#ifdef CONFIG_OF
static struct ls_apbdma_platform_data *
ls_apbdma_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ls_apbdma_platform_data *pdata;
	struct of_phandle_args	dma_spec;
	u32 args[2];

	if (!np) {
			dev_err(&pdev->dev, "Missing DT data\n");
			return NULL;
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
			return NULL;

	if (of_property_read_u32(np, "dma-channels", &pdata->nr_channels))
			goto err_parse;

	pdata->regmap = syscon_regmap_lookup_by_phandle_args(np,
					"loongson,apbdma-conf", 2, args);
	if (!IS_ERR(pdata->regmap)) {
		pdata->conf_reg_mask = args[0];
		pdata->conf_reg_value = args[1];

		return pdata;
	}

	if (of_parse_phandle_with_args(np, "apbdma-sel", "#config-nr", 0,
							&dma_spec))
			goto err_parse;

	pdata->common = of_find_device_by_node(dma_spec.np);
	if (!pdata->common)
		goto err_parse;

	pdata->dma_sel = dma_spec.args[0];
	pdata->apb_dma_nr = dma_spec.args[1];

	return pdata;
err_parse:
	devm_kfree(&pdev->dev, pdata);
	return NULL;
}
#else
static inline struct ls_apbdma_platform_data *
ls_apbdma_parse_dt(struct platform_device *pdev)
{
	return NULL;
}
#endif

static int ls_apbdma_probe(struct platform_device *pdev)
{
	struct ls_apbdma_platform_data *pdata;
	struct ls_apbdma		*apbdma = NULL;
	void __iomem		*regs;
	struct resource		*r;
	size_t			size;
	int			err = 0;
	int			i;
	dma_cap_mask_t mask;

	/* get DMA parameters from controller type */
	pdata = ls_apbdma_parse_dt(pdev);
	if (!pdata)
		return -ENOMEM;
	platform_device_add_data(pdev, pdata, sizeof(*pdata));

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -EINVAL;
	if (!request_mem_region(r->start, r->end - r->start + 1, pdev->name))
		return -EBUSY;

	regs = ioremap(r->start, r->end - r->start + 1);
	if (!regs) {
		err = -ENOMEM;
		goto err_release_r;
	}

#ifndef CONFIG_LOONGARCH /* 2k500 don't have this feature */
	/* 2k1000LA default enable coherentio so this code useless */
	if (hw_coherentio == 1)
		writel(readl(regs) & (~0x2), regs);
	else
		writel(readl(regs) | 0x2, regs);
#endif

	err = set_apbdma_config(pdata);
	if (err)
		goto err_kfree;
	size = sizeof(struct ls_apbdma);
	size += pdata->nr_channels * sizeof(struct ls_dma_chan);
	apbdma = kzalloc(size, GFP_KERNEL);
	if (!apbdma)
		return -ENOMEM;

	platform_set_drvdata(pdev, apbdma);
	/* initialize channels related values */
	INIT_LIST_HEAD(&apbdma->dma_common.channels);
	for (i = 0; i < pdata->nr_channels; i++) {
		struct ls_dma_chan	*lschan = &apbdma->chan[i];
		lschan->chan_common.device = &apbdma->dma_common;
		list_add_tail(&lschan->chan_common.device_node,
				&apbdma->dma_common.channels);
		lschan->mask = 1 << i;
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	/* set base routines */
	apbdma->dma_common.cap_mask = mask;
	apbdma->dma_common.device_alloc_chan_resources = apbdma_alloc_chan_resources;
	apbdma->dma_common.device_free_chan_resources = apbdma_free_chan_resources;
	apbdma->dma_common.device_tx_status = apbdma_tx_status;
	apbdma->dma_common.device_issue_pending = apbdma_issue_pending;
	apbdma->dma_common.dev = &pdev->dev;
	dev_info(&pdev->dev, "Loongson APB DMA Controller ( %s%s), %d channels\n",
	dma_has_cap(DMA_MEMCPY, apbdma->dma_common.cap_mask) ? "cpy " : "",
	dma_has_cap(DMA_SLAVE, apbdma->dma_common.cap_mask)  ? "slave " : "",
	pdata->nr_channels);

	dma_async_device_register(&apbdma->dma_common);

	/*
	 * Do not return an error if the dmac node is not present in order to
	 * not break the existing way of requesting channel with
	 * dma_request_channel().
	 */
	if (pdev->dev.of_node) {
		err = of_dma_controller_register(pdev->dev.of_node,
						 apb_dma_xlate, apbdma);
		if (err) {
			dev_err(&pdev->dev, "could not register of_dma_controller\n");
			goto err_of_dma_controller_register;
		}
	}

	return 0;

err_of_dma_controller_register:
	dma_async_device_unregister(&apbdma->dma_common);
err_kfree:
	devm_kfree(&pdev->dev, pdata);
	kfree(apbdma);
err_release_r:
	iounmap(regs);
	release_mem_region(r->start, r->end - r->start + 1);
	return err;
}

static int ls_apbdma_remove(struct platform_device *pdev)
{
	struct ls_apbdma	*apbdma = platform_get_drvdata(pdev);
	struct dma_chan		*chan, *_chan;

	dma_async_device_unregister(&apbdma->dma_common);

	list_for_each_entry_safe(chan, _chan, &apbdma->dma_common.channels,
			device_node) {
		list_del(&chan->device_node);
	}

	if (pdev->dev.of_node)
			of_dma_controller_free(pdev->dev.of_node);

	kfree(apbdma);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ls_apbdma_resume_early(struct device *dev)
{
	return set_apbdma_config(dev_get_platdata(dev));
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops ls_apbdma_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(NULL, ls_apbdma_resume_early)
};

static struct platform_driver ls_apbdma_driver = {
	.probe		= ls_apbdma_probe,
	.remove		= ls_apbdma_remove,
	.id_table	= ls_apbdma_devtypes,
	.driver = {
		.name	= "ls_apbdma",
		.pm	= &ls_apbdma_pm_ops,
		.of_match_table	= of_match_ptr(ls_apbdma_dt_ids),
	},
};

static int __init ls_apbdma_init(void)
{
	return platform_driver_register(&ls_apbdma_driver);
}
subsys_initcall(ls_apbdma_init);

static void __exit ls_apbdma_exit(void)
{
	platform_driver_unregister(&ls_apbdma_driver);
}
module_exit(ls_apbdma_exit);

MODULE_DESCRIPTION("Loongson APB DMA Controller driver");
MODULE_AUTHOR("Xuefeng Li <lixuefeng@loongson.cn>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ls_apbdmac");
