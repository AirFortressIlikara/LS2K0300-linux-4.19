// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2016
 * Author: Benjamin Gaignard <benjamin.gaignard@st.com>
 *
 * Copyright (C) 2025 Ilikara <3435193369@qq.com>
 * Modified by: Ilikara <3435193369@qq.com>
 */

#include <linux/bitfield.h>
#include <linux/mfd/loongson-timers.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/reset.h>

#define LS_TIMERS_MAX_REGISTERS 0x3fc

/* DIER register DMA enable bits */
static const u32 loongson_timers_dier_dmaen[LS_TIMERS_MAX_DMAS] = {
	TIM_DIER_CC1DE,
	TIM_DIER_CC2DE,
	TIM_DIER_CC3DE,
	TIM_DIER_CC4DE,
	TIM_DIER_UIE,
	TIM_DIER_TDE,
	TIM_DIER_COMDE
};

static void loongson_timers_dma_done(void *p)
{
	struct loongson_timers_dma *dma = p;
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(dma->chan, dma->chan->cookie, &state);
	if (status == DMA_COMPLETE)
		complete(&dma->completion);
}

/**
 * loongson_timers_dma_burst_read - Read from timers registers using dual DMA channels.
 * 
 * @dev: Loongson timers MFD device
 * @buf: Destination buffer (must hold bursts * num_reg * 2 elements)
 * @id: DMA event pair identifier (ch[1/2/3/4]_pair)
 * @reg: Base register offset for DMA transfers
 * @num_reg: Number of consecutive registers per burst
 * @bursts: Number of bursts to capture (each burst triggers dual DMA transfers)
 * @tmo_ms: Operation timeout in milliseconds
 */
int loongson_timers_dma_burst_read(struct device *dev, u32 *buf,
				   enum loongson_timers_dmas id, u32 reg,
				   unsigned int num_reg, unsigned int bursts,
				   unsigned long tmo_ms)
{
	struct loongson_timers *ddata = dev_get_drvdata(dev);
	unsigned long timeout = msecs_to_jiffies(tmo_ms);
	struct regmap *regmap = ddata->regmap;
	struct loongson_timers_dma *dma = &ddata->dma;
	size_t len = num_reg * bursts * sizeof(u32);
	struct dma_async_tx_descriptor *descs[2];
	struct dma_slave_config configs[2];
	dma_cookie_t cookie;
	dma_addr_t dma_bufs[2];
	long err;
	int ret;
	int i;

	/* Sanity check */
	if (id < LS_TIMERS_DMA_CH1 || id >= LS_TIMERS_MAX_DMAS - 1)
		return -EINVAL;

	if (!num_reg || !bursts || reg > LS_TIMERS_MAX_REGISTERS ||
	    (reg + num_reg * sizeof(u32)) > LS_TIMERS_MAX_REGISTERS)
		return -EINVAL;

	if (!dma->chans[id] || !dma->chans[id + 1])
		return -ENODEV;
	mutex_lock(&dma->lock);

	/* Select DMA channel in use */
	dma->chan = dma->chans[id];
	for (i = 0; i < 2; i++) {
		dma_bufs[i] = dma_map_single(dev, buf + i * bursts, len / 2,
					     DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, dma_bufs[i])) {
			ret = -ENOMEM;
			goto unmap;
		}
	}

	/* Configure DMA channels */
	for (i = 0; i < 2; i++) {
		memset(&configs[i], 0, sizeof(configs[i]));
		configs[i].src_addr =
			(dma_addr_t)dma->phys_base + reg + i * sizeof(u32);
		configs[i].src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

		ret = dmaengine_slave_config(dma->chans[id + i], &configs[i]);
		if (ret)
			goto unmap;

		descs[i] = dmaengine_prep_slave_single(dma->chans[id + i],
						       dma_bufs[i], len / 2,
						       DMA_DEV_TO_MEM,
						       DMA_PREP_INTERRUPT);
		if (!descs[i]) {
			ret = -EBUSY;
			goto unmap;
		}

		descs[i]->callback = loongson_timers_dma_done;
		descs[i]->callback_param = dma;
	}
	for (i = 0; i < 2; i++) {
		cookie = dmaengine_submit(descs[i]);
		ret = dma_submit_error(cookie);
		if (ret)
			goto dma_term;
	}

	reinit_completion(&dma->completion);
	dma_async_issue_pending(dma->chans[id + 0]);
	dma_async_issue_pending(dma->chans[id + 1]);

	/* Clear pending flags before enabling DMA request */
	ret = regmap_write(regmap, TIM_SR, 0);
	if (ret)
		goto dma_term;

	regmap_update_bits(regmap, TIM_DIER,
			   loongson_timers_dier_dmaen[id] |
				   loongson_timers_dier_dmaen[id + 1],
			   loongson_timers_dier_dmaen[id] |
				   loongson_timers_dier_dmaen[id + 1]);
	if (ret)
		goto dma_term;

	err = wait_for_completion_interruptible_timeout(&dma->completion,
							timeout);
	if (err == 0)
		ret = -ETIMEDOUT;
	else if (err < 0)
		ret = err;

	regmap_update_bits(regmap, TIM_DIER,
			   loongson_timers_dier_dmaen[id] |
				   loongson_timers_dier_dmaen[id + 1],
			   0);
	regmap_write(regmap, TIM_SR, 0);
dma_term:
	dmaengine_terminate_all(dma->chans[id + 0]);
	dmaengine_terminate_all(dma->chans[id + 1]);
unmap:
	for (i = 0; i < 2; i++) {
		if (!dma_mapping_error(dev, dma_bufs[i]))
			dma_unmap_single(dev, dma_bufs[i], len / 2,
					 DMA_FROM_DEVICE);
	}
unlock:
	dma->chan = NULL;
	mutex_unlock(&dma->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(loongson_timers_dma_burst_read);

static const struct regmap_config loongson_timers_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = sizeof(u32),
	.max_register = LS_TIMERS_MAX_REGISTERS,
};

static void loongson_timers_get_arr_size(struct loongson_timers *ddata)
{
	u32 arr;

	/* Backup ARR to restore it after getting the maximum value */
	regmap_read(ddata->regmap, TIM_ARR, &arr);

	/*
	 * Only the available bits will be written so when readback
	 * we get the maximum value of auto reload register
	 */
	regmap_write(ddata->regmap, TIM_ARR, ~0L);
	regmap_read(ddata->regmap, TIM_ARR, &ddata->max_arr);
	regmap_write(ddata->regmap, TIM_ARR, arr);
}

static int loongson_timers_dma_probe(struct device *dev,
				     struct loongson_timers *ddata)
{
	int i;
	int ret = 0;
	char name[4];

	init_completion(&ddata->dma.completion);
	mutex_init(&ddata->dma.lock);

	/* Optional DMA support: get valid DMA channel(s) or NULL */
	for (i = LS_TIMERS_DMA_CH1; i <= LS_TIMERS_DMA_CH4; i++) {
		snprintf(name, ARRAY_SIZE(name), "ch%1d", i + 1);
		ddata->dma.chans[i] = dma_request_chan(dev, name);
	}
	ddata->dma.chans[LS_TIMERS_DMA_UP] = dma_request_chan(dev, "up");
	ddata->dma.chans[LS_TIMERS_DMA_TRIG] = dma_request_chan(dev, "trig");
	ddata->dma.chans[LS_TIMERS_DMA_COM] = dma_request_chan(dev, "com");

	for (i = LS_TIMERS_DMA_CH1; i < LS_TIMERS_MAX_DMAS; i++) {
		if (IS_ERR(ddata->dma.chans[i])) {
			/* Save the first error code to return */
			if (PTR_ERR(ddata->dma.chans[i]) != -ENODEV && !ret)
				ret = PTR_ERR(ddata->dma.chans[i]);

			ddata->dma.chans[i] = NULL;
		}
	}

	return ret;
}

static void loongson_timers_dma_remove(struct device *dev,
				       struct loongson_timers *ddata)
{
	int i;

	for (i = LS_TIMERS_DMA_CH1; i < LS_TIMERS_MAX_DMAS; i++)
		if (ddata->dma.chans[i])
			dma_release_channel(ddata->dma.chans[i]);
}

static int loongson_timers_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_timers *ddata;
	struct resource *res;
	void __iomem *mmio;
	int ret;
	u32 clk;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	/* Timer physical addr for DMA */
	ddata->dma.phys_base = res->start;

	ddata->regmap =
		devm_regmap_init_mmio(dev, mmio, &loongson_timers_regmap_cfg);
	if (IS_ERR(ddata->regmap))
		return PTR_ERR(ddata->regmap);

	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clk);
	if(ret) 
		return ret;
	ddata->clock_frequency = clk;

	loongson_timers_get_arr_size(ddata);

	ret = loongson_timers_dma_probe(dev, ddata);
	if (ret) {
		loongson_timers_dma_remove(dev, ddata);
		return ret;
	}

	platform_set_drvdata(pdev, ddata);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret)
		loongson_timers_dma_remove(dev, ddata);

	return ret;
}

static int loongson_timers_remove(struct platform_device *pdev)
{
	struct loongson_timers *ddata = platform_get_drvdata(pdev);

	/*
	 * Don't use devm_ here: enfore of_platform_depopulate() happens before
	 * DMA are released, to avoid race on DMA.
	 */
	of_platform_depopulate(&pdev->dev);
	loongson_timers_dma_remove(&pdev->dev, ddata);

	return 0;
}

static const struct of_device_id loongson_timers_of_match[] = {
	{
		.compatible = "loongson,loongson-timers",
	},
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, loongson_timers_of_match);

static struct platform_driver loongson_timers_driver = {
	.probe = loongson_timers_probe,
	.remove = loongson_timers_remove,
	.driver	= {
		.name = "loongson-timers",
		.of_match_table = loongson_timers_of_match,
	},
};
module_platform_driver(loongson_timers_driver);

MODULE_DESCRIPTION("Loongson Timers");
MODULE_LICENSE("GPL v2");
