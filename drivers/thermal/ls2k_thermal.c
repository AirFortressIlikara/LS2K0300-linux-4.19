// SPDX-License-Identifier: GPL-2.0
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/thermal.h>
#include "thermal_hwmon.h"

#define LS2K_SOC_DEFAULT_SENSOR	0
#define LS2K_SOC_MAX_SENSOR_NUM	3

#define LS2K_TSENSOR_CTRL_HI		(0x0)
#define LS2K_TSENSOR_CTRL_LO		(0x8)
#define LS2K_TSENSOR_STATUS			(0x10)
#define LS2K_TSENSOR_OUT			(0x14)

struct ls2k_thermal_data {
	struct thermal_zone_device *tzd;
	int irq;
	int id;
	void __iomem *regs;
	struct platform_device *pdev;
	u16 ctrl_low_val;
	u16 ctrl_hi_val;
};

/**
 * @low : temperature in degree
 * @high: temperature in degree
 */
static int ls2k_tsensor_set(struct ls2k_thermal_data *data,
					int low, int high, bool enable)
{
	u64 reg_ctrl = 0;
	int reg_off = data->id * 2;

	if (low > high)
		return -EINVAL;

	low = low < -100 ? -100 : low;
	high = high > 155 ? 155 : high;

	low += 100;
	high += 100;

	reg_ctrl |= low;
	reg_ctrl |= enable ? 0x100 : 0;
	writew(reg_ctrl, data->regs + LS2K_TSENSOR_CTRL_LO + reg_off);

	reg_ctrl = 0;
	reg_ctrl |= high;
	reg_ctrl |= enable ? 0x100 : 0;
	writew(reg_ctrl, data->regs + LS2K_TSENSOR_CTRL_HI + reg_off);

	return 0;
}

static int ls2k_thermal_get_temp(void *__data, int *temp)
{
	struct ls2k_thermal_data *data = __data;
	u32 reg_val;

	reg_val = readl(data->regs + LS2K_TSENSOR_OUT);
	*temp = ((reg_val & 0xff) - 100) * 1000;

	return 0;
}

static irqreturn_t ls2k_thermal_alarm_irq(int irq, void *dev)
{
	struct ls2k_thermal_data *data = dev;

	/* clear interrupt */
	writeb(0x3, data->regs + LS2K_TSENSOR_STATUS);

	disable_irq_nosync(irq);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t ls2k_thermal_irq_thread(int irq, void *dev)
{
	struct ls2k_thermal_data *data = dev;

	thermal_zone_device_update(data->tzd,
				   THERMAL_EVENT_UNSPECIFIED);
	enable_irq(data->irq);

	return IRQ_HANDLED;
}

static int ls2k_thermal_set_trips(void *data, int low, int high)
{
	return ls2k_tsensor_set(data, low/1000, high/1000, true);
}

static const struct thermal_zone_of_device_ops ls2k_of_thermal_ops = {
	.get_temp = ls2k_thermal_get_temp,
	.set_trips = ls2k_thermal_set_trips,
};

static int ls2k_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct ls2k_thermal_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->pdev = pdev;
	platform_set_drvdata(pdev, data);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(data->regs)) {
		dev_err(dev, "failed to get io address\n");
		return PTR_ERR(data->regs);
	}

	/* get irq */
	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0)
		return data->irq;

	/* get id */
	if (of_property_read_u32(dev->of_node, "id", &data->id)) {
		dev_err(dev, "not found id property!\n");
		data->id = LS2K_SOC_DEFAULT_SENSOR;
	}

	if (data->id > LS2K_SOC_MAX_SENSOR_NUM) {
		dev_err(dev, "sensor id error,must be in <0 ~ %d>\n",
				LS2K_SOC_MAX_SENSOR_NUM);
		return -EINVAL;
	}

	writeb(0xff, data->regs + LS2K_TSENSOR_STATUS);

	ls2k_tsensor_set(data, 0, 0, false);

	data->tzd = devm_thermal_zone_of_sensor_register(&pdev->dev,
							   data->id, data,
							   &ls2k_of_thermal_ops);
	if (IS_ERR(data->tzd)) {
		ret = PTR_ERR(data->tzd);
		data->tzd = NULL;
		dev_err(&pdev->dev, "failed to register %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, data->irq,
			ls2k_thermal_alarm_irq, ls2k_thermal_irq_thread,
			IRQF_ONESHOT, "ls2k_thermal", data);
	if (ret < 0) {
		dev_err(dev, "failed to request alarm irq: %d\n", ret);
		return ret;
	}

	/*
	 * Thermal_zone doesn't enable hwmon as default,
	 * enable it here
	 */
	data->tzd->tzp->no_hwmon = false;
	ret = thermal_add_hwmon_sysfs(data->tzd);
	if (ret) {
		dev_err(dev, "failed to add hwmon sysfs interface %d\n", ret);
		return ret;
	}

	return 0;
}

int ls2k_thermal_remove(struct platform_device *pdev)
{
	struct ls2k_thermal_data *data = platform_get_drvdata(pdev);
	int reg_off = data->id * 2;

	/* disable interrupt */
	writew(0, data->regs + LS2K_TSENSOR_CTRL_LO + reg_off);
	writew(0, data->regs + LS2K_TSENSOR_CTRL_HI + reg_off);

	return 0;
}

static const struct of_device_id of_ls2k_thermal_match[] = {
	{ .compatible = "loongson,ls2k-tsensor", },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, of_ls2k_thermal_match);

#ifdef CONFIG_PM_SLEEP
static int ls2k_thermal_suspend(struct device *dev)
{
	struct ls2k_thermal_data *data = dev_get_drvdata(dev);
	int reg_off = data->id * 2;

	data->ctrl_low_val = readw(data->regs + LS2K_TSENSOR_CTRL_LO + reg_off);
	data->ctrl_hi_val = readw(data->regs + LS2K_TSENSOR_CTRL_HI + reg_off);

	writew(0, data->regs + LS2K_TSENSOR_CTRL_LO + reg_off);
	writew(0, data->regs + LS2K_TSENSOR_CTRL_HI + reg_off);

	return 0;
}

static int ls2k_thermal_resume(struct device *dev)
{
	struct ls2k_thermal_data *data = dev_get_drvdata(dev);
	int reg_off = data->id * 2;

	writew(data->ctrl_low_val, data->regs + LS2K_TSENSOR_CTRL_LO + reg_off);
	writew(data->ctrl_hi_val, data->regs + LS2K_TSENSOR_CTRL_HI + reg_off);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ls2k_thermal_pm_ops,
			 ls2k_thermal_suspend, ls2k_thermal_resume);

static struct platform_driver ls2k_thermal_driver = {
	.driver = {
		.name		= "ls2k_thermal",
		.pm = &ls2k_thermal_pm_ops,
		.of_match_table = of_ls2k_thermal_match,
	},
	.probe	= ls2k_thermal_probe,
	.remove	= ls2k_thermal_remove,
};
module_platform_driver(ls2k_thermal_driver);

MODULE_DESCRIPTION("lonngson-2k thermal driver");
MODULE_LICENSE("GPL v2");
