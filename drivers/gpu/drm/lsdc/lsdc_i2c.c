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

#include <linux/string.h>
#include <linux/i2c.h>

#include <drm/drm_print.h>
#include <drm/drm_edid.h>

#include "lsdc_drv.h"
#include "lsdc_regs.h"
#include "lsdc_i2c.h"

/*
 *  mask : gpio pin mask
 */
static void ls7a_gpio_i2c_set(struct lsdc_i2c * const i2c, int mask, int state)
{
	struct lsdc_device *ldev = to_lsdc(i2c->ddev);
	u8 val;
	unsigned long flags;

	spin_lock_irqsave(&ldev->reglock, flags);

	if (state) {
		val = readb(i2c->dir_reg);
		val |= mask;
		writeb(val, i2c->dir_reg);
	} else {
		val = readb(i2c->dir_reg);
		val &= ~mask;
		writeb(val, i2c->dir_reg);

		val = readb(i2c->dat_reg);
		val &= ~mask;
		writeb(val, i2c->dat_reg);
	}

	spin_unlock_irqrestore(&ldev->reglock, flags);
}

/*
 *  mask : gpio pin mask
 */
static int ls7a_gpio_i2c_get(struct lsdc_i2c * const i2c, int mask)
{
	struct lsdc_device *ldev = to_lsdc(i2c->ddev);
	u8 val;
	unsigned long flags;

	spin_lock_irqsave(&ldev->reglock, flags);

	/* first set this pin as input */
	val = readb(i2c->dir_reg);
	val |= mask;
	writeb(val, i2c->dir_reg);

	/* then get level state from this pin */
	val = readb(i2c->dat_reg);

	spin_unlock_irqrestore(&ldev->reglock, flags);

	return (val & mask) ? 1 : 0;
}


/* set the state on the i2c->sda pin */
static void ls7a_i2c_set_sda(void *i2c, int state)
{
	struct lsdc_i2c * const li2c = (struct lsdc_i2c *)i2c;

	return ls7a_gpio_i2c_set(li2c, li2c->sda, state);
}

/* set the state on the i2c->scl pin */
static void ls7a_i2c_set_scl(void *i2c, int state)
{
	struct lsdc_i2c * const li2c = (struct lsdc_i2c *)i2c;

	return ls7a_gpio_i2c_set(li2c, li2c->scl, state);
}

/* read the value from the i2c->sda pin */
static int ls7a_i2c_get_sda(void *i2c)
{
	struct lsdc_i2c * const li2c = (struct lsdc_i2c *)i2c;

	return ls7a_gpio_i2c_get(li2c, li2c->sda);
}

/* read the value from the i2c->scl pin */
static int ls7a_i2c_get_scl(void *i2c)
{
	struct lsdc_i2c * const li2c = (struct lsdc_i2c *)i2c;

	return ls7a_gpio_i2c_get(li2c, li2c->scl);
}


/*
 * Get i2c id from connector id
 *
 * TODO: get it from dtb
 */
int lsdc_get_i2c_id(struct drm_device *ddev, const unsigned int index)
{
	return index;
}


/*
 * @index : output channel index,
 * 0 stand for DVO0, 1 stand for DVO1
 */
struct i2c_adapter *
ls7a_create_i2c_chan(struct drm_device *ddev, unsigned int index)
{
	struct lsdc_device *ldev = to_lsdc(ddev);
	struct i2c_adapter *adapter;
	struct lsdc_i2c *li2c;
	int ret;

	li2c = devm_kzalloc(ddev->dev, sizeof(*li2c), GFP_KERNEL);
	if (li2c == NULL)
		return ERR_PTR(-ENOMEM);

	li2c->ddev = ddev;

	if (index == 0) {
		li2c->sda = 0x01;
		li2c->scl = 0x02;
	} else if (index == 1) {
		li2c->sda = 0x04;
		li2c->scl = 0x08;
	} else {
		DRM_ERROR("%s: index=%u out of range\n", __func__, index);
		return NULL;
	}

	li2c->dir_reg = ldev->reg_base + LS7A_DC_GPIO_DIR_REG;
	li2c->dat_reg = ldev->reg_base + LS7A_DC_GPIO_DAT_REG;

	li2c->bit.setsda = ls7a_i2c_set_sda;
	li2c->bit.setscl = ls7a_i2c_set_scl;
	li2c->bit.getsda = ls7a_i2c_get_sda;
	li2c->bit.getscl = ls7a_i2c_get_scl;
	li2c->bit.udelay = 5;
	li2c->bit.timeout = usecs_to_jiffies(2200); /* from VESA */
	li2c->bit.data = li2c;

	adapter = &li2c->adapter;

	adapter->algo_data = &li2c->bit;
	adapter->owner = THIS_MODULE;
	adapter->class = I2C_CLASS_DDC;
	adapter->dev.parent = ddev->dev;
	adapter->nr = -1;

	snprintf(adapter->name, sizeof(adapter->name),
		"%s-%d", "lsdc_gpio_i2c", index);

	i2c_set_adapdata(adapter, li2c);

	ret = i2c_bit_add_numbered_bus(adapter);
	if (ret) {
		DRM_ERROR("Failed to create i2c adaptor for output-%u\n",
			  index);
		devm_kfree(ddev->dev, li2c);
		return NULL;
	}

	return adapter;
}

struct i2c_adapter *
lsdc_of_retrieve_ddc(struct drm_device *ddev, unsigned int index)
{
	struct device_node *dc_np = ddev->dev->of_node;
	struct device_node *output_np;
	struct device_node *phandle;
	struct i2c_adapter *ddc;

	output_np = of_parse_phandle(dc_np, "output-ports", index);
	if (!output_np)
		return ERR_PTR(-ENODEV);

	phandle = of_parse_phandle(output_np, "ddc-i2c-bus", 0);
	of_node_put(output_np);
	if (!phandle)
		return ERR_PTR(-ENODEV);

	ddc = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!ddc)
		return ERR_PTR(-EPROBE_DEFER);

	return ddc;
}

struct i2c_adapter *ls2k_get_i2c_chan(struct drm_device *ddev,
				      unsigned int con_id)
{
	struct lsdc_device *ldev = to_lsdc(ddev);
	const struct lsdc_platform_desc *descp = ldev->desc;
	struct i2c_adapter *adapter;
	unsigned int i2c_id;

	if (descp->chip == LSDC_CHIP_2K1000) {
		i2c_id = lsdc_get_i2c_id(ddev, con_id);

		adapter = i2c_get_adapter(i2c_id);
		if (adapter == NULL) {
			DRM_WARN("%s: failed to get i2c-%d adapter\n",
				__func__, i2c_id);

			return ERR_PTR(-EPROBE_DEFER);
		}
	}

	if (descp->chip == LSDC_CHIP_2K0500) {
		i2c_id = lsdc_get_i2c_id(ddev, con_id + 2);

		adapter = i2c_get_adapter(i2c_id);
		if (adapter == NULL) {
			DRM_WARN("%s: failed to get i2c-%d adapter\n",
				__func__, i2c_id);

			return ERR_PTR(-EPROBE_DEFER);
		}
	}

	return adapter;
}

void lsdc_destroy_i2c(struct drm_device *ddev, struct i2c_adapter *adapter)
{
	i2c_put_adapter(adapter);
}
