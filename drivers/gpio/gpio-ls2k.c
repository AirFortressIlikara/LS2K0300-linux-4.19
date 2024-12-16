// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define GPIO_IO_CONF(x)	(x->base + 0)
#ifdef LOONGSON_2K1000
#define GPIO_OUT(x)	(x->base + 0x10)
#define GPIO_IN(x)	(x->base + 0x20)
#else
#define GPIO_OUT(x)	(x->base + 0x8)
#define GPIO_IN(x)	(x->base + 0xc)
#endif
#define LS2K_UNDEFINE_GPIO0  15
#define LS2K_UNDEFINE_GPIO1  31
#define LS2K_UNDEFINE_GPIO2  42
#define LS2K_UNDEFINE_GPIO3  43

struct ls2_gpio_chip {
	struct gpio_chip	chip;
	spinlock_t		lock;
	void __iomem		*base;
};

static inline void
__set_direction(struct ls2_gpio_chip *lgpio, unsigned int pin, int input)
{
	u64 u;

	u = readq(GPIO_IO_CONF(lgpio));
	if (input)
		u |= 1UL << pin;
	else
		u &= ~(1UL << pin);
	writeq(u, GPIO_IO_CONF(lgpio));
}

static void __set_level(struct ls2_gpio_chip *lgpio, unsigned int pin, int high)
{
	u64 u;

	u = readq(GPIO_OUT(lgpio));
	if (high)
		u |= 1UL << pin;
	else
		u &= ~(1UL << pin);
	writeq(u, GPIO_OUT(lgpio));
}

/*
 * GPIO primitives.
 */
static int ls2_gpio_request(struct gpio_chip *chip, unsigned int pin)
{
	if (pin >= chip->ngpio)
		return -EINVAL;
	else
		return 0;
}

static void ls2_gpio_free(struct gpio_chip *chip, unsigned int pin)
{
	return;
}

static int ls2_gpio_direction_input(struct gpio_chip *chip, unsigned int pin)
{
	unsigned long flags;
	struct ls2_gpio_chip *lgpio =
		container_of(chip, struct ls2_gpio_chip, chip);

	spin_lock_irqsave(&lgpio->lock, flags);
	__set_direction(lgpio, pin, 1);
	spin_unlock_irqrestore(&lgpio->lock, flags);

	return 0;
}

static int ls2_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct ls2_gpio_chip *lgpio =
		container_of(chip, struct ls2_gpio_chip, chip);
	u64 val;

	val = readq(GPIO_IN(lgpio));

	return (val >> pin) & 1;
}

static int
ls2_gpio_direction_output(struct gpio_chip *chip, unsigned int pin, int value)
{
	struct ls2_gpio_chip *lgpio =
		container_of(chip, struct ls2_gpio_chip, chip);
	unsigned long flags;

	spin_lock_irqsave(&lgpio->lock, flags);
	__set_level(lgpio, pin, value);
	__set_direction(lgpio, pin, 0);
	spin_unlock_irqrestore(&lgpio->lock, flags);

	return 0;
}

static void ls2_gpio_set(struct gpio_chip *chip, unsigned int pin, int value)
{
	struct ls2_gpio_chip *lgpio =
		container_of(chip, struct ls2_gpio_chip, chip);
	unsigned long flags;

	spin_lock_irqsave(&lgpio->lock, flags);
	__set_level(lgpio, pin, value);
	spin_unlock_irqrestore(&lgpio->lock, flags);
}

static int ls2_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct platform_device *pdev =
		container_of(chip->parent, struct platform_device, dev);

	if (offset >= chip->ngpio || offset == LS2K_UNDEFINE_GPIO0 ||
			offset == LS2K_UNDEFINE_GPIO1 ||
			offset == LS2K_UNDEFINE_GPIO2 ||
			offset == LS2K_UNDEFINE_GPIO3)
		return -EINVAL;

	return platform_get_irq(pdev, offset);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static void ls2_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{

}
#else
#define ls2_gpio_dbg_show NULL
#endif

static struct ls2_gpio_chip ls_gpio;
static void ls2_gpio_init(struct device *dev, struct device_node *np,
			    int gpio_base, int ngpio,
			    void __iomem *base)
{
	struct ls2_gpio_chip *lgpio;

	lgpio = &ls_gpio;
	lgpio->chip.label = kstrdup("ls2-gpio", GFP_KERNEL);
	lgpio->chip.request = ls2_gpio_request;
	lgpio->chip.free = ls2_gpio_free;
	lgpio->chip.direction_input = ls2_gpio_direction_input;
	lgpio->chip.get = ls2_gpio_get;
	lgpio->chip.direction_output = ls2_gpio_direction_output;
	lgpio->chip.set = ls2_gpio_set;
	lgpio->chip.base = gpio_base;
	lgpio->chip.ngpio = ngpio;
	lgpio->chip.can_sleep = 0;
	lgpio->chip.of_node = np;
	lgpio->chip.dbg_show = ls2_gpio_dbg_show;
	lgpio->chip.to_irq = ls2_gpio_to_irq;
	lgpio->chip.parent = dev;

	spin_lock_init(&lgpio->lock);
	lgpio->base = (void __iomem *)base;

	gpiochip_add(&lgpio->chip);
}

static int ls2_gpio_probe(struct platform_device *pdev)
{
	struct resource *iores;
	int gpio_base;
	unsigned int ngpio;
	void __iomem *base;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int ret = 0;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iores) {
		ret = -ENODEV;
		goto out;
	}

	if (!request_mem_region(iores->start, resource_size(iores),
				pdev->name)) {
		ret = -EBUSY;
		goto out;
	}

	gpio_base = 0;

	base = ioremap(iores->start, resource_size(iores));

	if (!base) {
		ret = -ENOMEM;
		goto out;
	}

	if (of_property_read_u32(pdev->dev.of_node, "ngpios", &ngpio)) {
		ret = -ENODEV;
		goto out;
	}

	ls2_gpio_init(dev, np, gpio_base, ngpio, base);

	return 0;
out:
	pr_err("%s: %s: missing mandatory property\n", __func__, np->name);
	return ret;
}

static int ls2_gpio_remove(struct platform_device *pdev)
{
	iounmap(ls_gpio.base);

	if (ls_gpio.chip.label)
		kfree(ls_gpio.chip.label);

	return 0;
}

static const struct of_device_id ls2_gpio_dt_ids[] = {
	{ .compatible = "ls,ls-gpio"},
	{ .compatible = "ls,ls2k-gpio"},
	{ .compatible = "loongson,ls2k-gpio"},
	{}
};
MODULE_DEVICE_TABLE(of, ls2_gpio_dt_ids);

static struct platform_driver ls2_gpio_driver = {
	.driver = {
		.name	= "gpio-ls",
		.owner	= THIS_MODULE,
		.of_match_table = ls2_gpio_dt_ids,
	},
	.probe		= ls2_gpio_probe,
	.remove 	= ls2_gpio_remove,
};

static int __init gpio_ls_init(void)
{
	return platform_driver_register(&ls2_gpio_driver);
}
postcore_initcall(gpio_ls_init);

static void __exit gpio_ls2_exit(void)
{
	platform_driver_unregister(&ls2_gpio_driver);
}
module_exit(gpio_ls2_exit);


MODULE_AUTHOR("Huang Pei <huangpei@loongson.cn>");
MODULE_DESCRIPTION("loongson 2K GPIO Driver");
MODULE_LICENSE("GPL v2");
