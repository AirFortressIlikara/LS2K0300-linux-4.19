// SPDX-License-Identifier: GPL-2.0
/*
 * Loongson-2H I2C master mode driver
 *
 * Copyright (C) 2013 Loongson Technology Corporation Limited
 * Copyright (C) 2014-2017 Lemote, Inc.
 *
 * Originally written by liushaozong
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <loongson-pch.h>

#define DRIVER_NAME "ls_i2c"

#define LS2X_I2C_PRER_LO_REG	0x0
#define LS2X_I2C_PRER_HI_REG	0x1
#define LS2X_I2C_CTR_REG	0x2
#define LS2X_I2C_TXR_REG	0x3
#define LS2X_I2C_RXR_REG	0x3
#define LS2X_I2C_CR_REG		0x4
#define LS2X_I2C_SR_REG		0x4
#define LS2X_I2C_BLTOP_REG	0x5
#define LS2X_I2C_SADDR_REG	0x7

#define CTR_EN			0x80
#define CTR_IEN			0x40
#define CTR_TXROK		0x90
#define CTR_RXROK		0x88

#define CR_START		0x81
#define CR_STOP			0x41
#define CR_READ			0x21
#define CR_WRITE		0x11
#define CR_ACK			0x8
#define CR_IACK			0x1

#define SR_NOACK		0x80
#define SR_BUSY			0x40
#define SR_AL			0x20
#define SR_SLAVE_ADDRESSED	0x10
#define SR_SLAVE_RW		0x8
#define SR_TIP			0x2
#define SR_IF			0x1


#define I2C_HOST_MODE           0x0
#define I2C_SLAVE_MODE          0x1
#define I2C_CLIENT_ADDR 0x12

#define i2c_readb(addr)		readb(dev->base + addr)
#define i2c_writeb(val, addr)	writeb(val, dev->base + addr)

#ifdef LS2X_I2C_DEBUG
#define i2c_debug(fmt, args...)	printk(KERN_CRIT fmt, ##args)
#else
#define i2c_debug(fmt, args...)
#endif

/* I2C clock frequency 50M */
#define LS_I2C_CLK_RATE_50M	(50 * 1000000)

static bool repeated_start = 1;
module_param(repeated_start, bool, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(repeated_start, "Compatible with devices that support repeated start");

enum ls_i2c_slave_state {
	LS_I2C_SLAVE_STOP,
	LS_I2C_SLAVE_START,
	LS_I2C_SLAVE_READ_REQUESTED,
	LS_I2C_SLAVE_READ_PROCESSED,
	LS_I2C_SLAVE_WRITE_REQUESTED,
	LS_I2C_SLAVE_WRITE_RECEIVED,
};

struct ls2x_i2c_dev {
	spinlock_t		lock;
	unsigned int		suspended:1;
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	u32			speed_hz;
	struct completion	cmd_complete;
	struct resource		*ioarea;
	struct i2c_adapter	adapter;
	int                   i2c_mode;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client	*slave;
	enum ls_i2c_slave_state	slave_state;
#endif /* CONFIG_I2C_SLAVE */
};

static int i2c_stop(struct ls2x_i2c_dev *dev)
{
	unsigned long time_left;

again:
	i2c_writeb(CR_STOP, LS2X_I2C_CR_REG);
	time_left = wait_for_completion_timeout(
		&dev->cmd_complete,
		(&dev->adapter)->timeout);
	if (!time_left) {
		pr_info("Timeout abort message cmd\n");
		return -1;
	}

	i2c_readb(LS2X_I2C_SR_REG);
	while (i2c_readb(LS2X_I2C_SR_REG) & SR_BUSY)
		goto again;

	return 0;
}

static int i2c_start(struct ls2x_i2c_dev *dev,
		int dev_addr, int flags)
{
	unsigned long time_left;
	int retry = 5;
	unsigned char addr = (dev_addr & 0x7f) << 1;
	addr |= (flags & I2C_M_RD)? 1:0;

start:
	mdelay(1);
	i2c_writeb(addr, LS2X_I2C_TXR_REG);
	i2c_debug("%s <line%d>: i2c device address: 0x%x\n",
			__func__, __LINE__, addr);
	i2c_writeb((CR_START | CR_WRITE), LS2X_I2C_CR_REG);
	time_left = wait_for_completion_timeout(
		&dev->cmd_complete,
		(&dev->adapter)->timeout);
	if (!time_left) {
		pr_info("Timeout abort message cmd\n");
		return -1;
	}

	if (i2c_readb(LS2X_I2C_SR_REG) & SR_NOACK) {
		if (i2c_stop(dev) < 0)
			return -1;
		while (retry--)
			goto start;
		pr_debug("There is no i2c device ack\n");
		return 0;
	}
	return 1;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static void __ls_i2c_reg_slave(struct ls2x_i2c_dev *dev, u16 slave_addr)
{

	/* Turn on slave mode. */
	i2c_writeb(0xc0, LS2X_I2C_CTR_REG);
	/* Set slave addr. */
	i2c_writeb((slave_addr & 0x7f)|0x80, LS2X_I2C_SADDR_REG);
}

static int ls_i2c_reg_slave(struct i2c_client *client)
{
	struct ls2x_i2c_dev *dev = i2c_get_adapdata(client->adapter);
	unsigned long flags;

	if (dev->slave) {
		return -EINVAL;
	}

	__ls_i2c_reg_slave(dev, client->addr);

	dev->slave = client;
	dev->slave_state = LS_I2C_SLAVE_STOP;

	return 0;
}

static int ls_i2c_unreg_slave(struct i2c_client *client)
{
	struct ls2x_i2c_dev *dev = i2c_get_adapdata(client->adapter);
	unsigned long flags;

	if (!dev->slave) {
		return -EINVAL;
	}

	/* Turn off slave mode. */
	i2c_writeb(0xa0, LS2X_I2C_CTR_REG);

	dev->slave = NULL;

	return 0;
}
#endif /* CONFIG_I2C_SLAVE */

static void ls2x_i2c_reginit(struct ls2x_i2c_dev *dev)
{
	u16 prer_val;
	u32 ls_pclk;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	if (dev->slave) {
		__ls_i2c_reg_slave(dev, dev->slave->addr);
		return;
	}
#endif /* CONFIG_I2C_SLAVE */
	if (!dev->speed_hz) {
		prer_val = 0x12c;
	} else {
                if(of_property_read_u32((dev->dev)->of_node, "clock_in", &ls_pclk))
		    ls_pclk = LS_I2C_CLK_RATE_50M;
		prer_val = ls_pclk / (5 * dev->speed_hz) - 1;
	}

	i2c_writeb(i2c_readb(LS2X_I2C_CR_REG) | 0x01, LS2X_I2C_CR_REG);
	i2c_writeb(i2c_readb(LS2X_I2C_CTR_REG) & ~0x80, LS2X_I2C_CTR_REG);
	i2c_writeb(prer_val & 0xFF, LS2X_I2C_PRER_LO_REG);
	i2c_writeb((prer_val & 0xFF00) >> 8, LS2X_I2C_PRER_HI_REG);
	i2c_writeb(i2c_readb(LS2X_I2C_CTR_REG) | 0xe0, LS2X_I2C_CTR_REG);
}

static int i2c_read(struct ls2x_i2c_dev *dev,
		unsigned char *buf, int count)
{
	int i;
	unsigned long time_left;

	for (i = 0; i < count; i++) {
		i2c_writeb((i == count - 1)?
				(CR_READ | CR_ACK) : CR_READ,
				LS2X_I2C_CR_REG);
		time_left = wait_for_completion_timeout(
			&dev->cmd_complete,
			(&dev->adapter)->timeout);
		if (!time_left) {
			pr_info("Timeout abort message cmd\n");
			return -1;
		}

		buf[i] = i2c_readb(LS2X_I2C_RXR_REG);
		i2c_debug("%s <line%d>: read buf[%d] <= %02x\n",
				__func__, __LINE__, i, buf[i]);
        }

        return i;
}

static int i2c_write(struct ls2x_i2c_dev *dev,
		unsigned char *buf, int count)
{
        int i;
	unsigned long time_left;

        for (i = 0; i < count; i++) {
		i2c_writeb(buf[i], LS2X_I2C_TXR_REG);
		i2c_debug("%s <line%d>: write buf[%d] => %02x\n",
				__func__, __LINE__, i, buf[i]);
		i2c_writeb(CR_WRITE, LS2X_I2C_CR_REG);
		time_left = wait_for_completion_timeout(
			&dev->cmd_complete,
			(&dev->adapter)->timeout);
		if (!time_left) {
			pr_info("Timeout abort message cmd\n");
			return -1;
		}

		if (i2c_readb(LS2X_I2C_SR_REG) & SR_NOACK) {
			i2c_debug("%s <line%d>: device no ack\n",
					__func__, __LINE__);
			if (i2c_stop(dev) < 0)
				return -1;
			return 0;
		}
        }

        return i;
}

static int i2c_doxfer(struct ls2x_i2c_dev *dev,
		struct i2c_msg *msgs, int num)
{
	struct i2c_msg *m = msgs;
	int i, err;

	for (i = 0; i < num; i++) {
		reinit_completion(&dev->cmd_complete);
		err = i2c_start(dev, m->addr, m->flags);
		if (err <= 0)
			return err;

		if (m->flags & I2C_M_RD) {
			if (i2c_read(dev, m->buf, m->len) < 0)
				return -1;
		} else {
			if (i2c_write(dev, m->buf, m->len) < 0)
				return -1;
		}
		++m;
		if (!repeated_start && i2c_stop(dev) < 0)
			return -1;
	}
	if (repeated_start && i2c_stop(dev) < 0)
		return -1;
	return i;
}

static int i2c_xfer(struct i2c_adapter *adap,
                        struct i2c_msg *msgs, int num)
{
	int ret;
	int retry;
	struct ls2x_i2c_dev *dev;

	dev = i2c_get_adapdata(adap);
	for (retry = 0; retry < adap->retries; retry++) {
		ret = i2c_doxfer(dev, msgs, num);
		if (ret != -EAGAIN)
			return ret;

		udelay(100);
	}

	return -EREMOTEIO;
}

static unsigned int i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ls2x_i2c_algo = {
	.master_xfer	= i2c_xfer,
	.functionality	= i2c_func,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave	= ls_i2c_reg_slave,
	.unreg_slave	= ls_i2c_unreg_slave,
#endif /* CONFIG_I2C_SLAVE */
};

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static bool ls_i2c_slave_irq(struct ls2x_i2c_dev *dev)
{
	u32 stat;
	struct i2c_client *slave = dev->slave;
	u8 value;

	stat = i2c_readb(LS2X_I2C_SR_REG);

	/* Slave was requested, restart state machine. */
	if (stat & SR_SLAVE_ADDRESSED) {
		dev->slave_state = LS_I2C_SLAVE_START;
		i2c_writeb(CTR_RXROK | CTR_IEN, LS2X_I2C_CTR_REG);
	}

	/* Slave is not currently active, irq was for someone else. */
	if (dev->slave_state == LS_I2C_SLAVE_STOP) {
		return IRQ_NONE;
	}

	/* Handle address frame. */
	if (dev->slave_state == LS_I2C_SLAVE_START) {
		if (stat & SR_SLAVE_RW)	//slave be read
			dev->slave_state =
				LS_I2C_SLAVE_READ_REQUESTED;
		else
			dev->slave_state =
				LS_I2C_SLAVE_WRITE_REQUESTED;
	}

	/* Slave was asked to stop. */
	if (stat & SR_NOACK) {
		dev->slave_state = LS_I2C_SLAVE_STOP;
	}

	value = i2c_readb(LS2X_I2C_RXR_REG);
	switch (dev->slave_state) {
	case LS_I2C_SLAVE_READ_REQUESTED:
		dev->slave_state = LS_I2C_SLAVE_READ_PROCESSED;
		i2c_slave_event(slave, I2C_SLAVE_READ_REQUESTED, &value);
		i2c_writeb(value, LS2X_I2C_TXR_REG);
		i2c_writeb(CTR_TXROK | CTR_IEN, LS2X_I2C_CTR_REG);
		break;
	case LS_I2C_SLAVE_READ_PROCESSED:
		i2c_slave_event(slave, I2C_SLAVE_READ_PROCESSED, &value);
		i2c_writeb(value, LS2X_I2C_TXR_REG);
		i2c_writeb(CTR_TXROK | CTR_IEN, LS2X_I2C_CTR_REG);
		break;
	case LS_I2C_SLAVE_WRITE_REQUESTED:
		dev->slave_state = LS_I2C_SLAVE_WRITE_RECEIVED;
		i2c_slave_event(slave, I2C_SLAVE_WRITE_REQUESTED, &value);
		break;
	case LS_I2C_SLAVE_WRITE_RECEIVED:
		i2c_slave_event(slave, I2C_SLAVE_WRITE_RECEIVED, &value);
		i2c_writeb(CTR_RXROK | CTR_IEN, LS2X_I2C_CTR_REG);
		break;
	case LS_I2C_SLAVE_STOP:
		i2c_slave_event(slave, I2C_SLAVE_STOP, &value);
		i2c_writeb(0, LS2X_I2C_TXR_REG);
		i2c_writeb(CTR_TXROK | CTR_IEN, LS2X_I2C_CTR_REG);
		break;
	default:
		dev_err(dev->dev, "unhandled slave_state: %d\n",
			dev->slave_state);
		break;
	}

out:
	return IRQ_HANDLED;
}
#endif /* CONFIG_I2C_SLAVE */

/*
 * Interrupt service routine. This gets called whenever an I2C interrupt
 * occurs.
 */
static irqreturn_t i2c_ls2x_isr(int this_irq, void *dev_id)
{
	unsigned char iflag;
	struct ls2x_i2c_dev *dev = dev_id;

	iflag = i2c_readb(LS2X_I2C_SR_REG);

	if (iflag & SR_IF) {
		i2c_writeb(CR_IACK, LS2X_I2C_CR_REG);
#if IS_ENABLED(CONFIG_I2C_SLAVE)
		if (dev->slave) {
			ls_i2c_slave_irq(dev);
		}
#endif
		if (!(iflag & SR_TIP))
			complete(&dev->cmd_complete);
	} else
		return IRQ_NONE;

	return IRQ_HANDLED;
}



#if IS_ENABLED(CONFIG_I2C_SLAVE)
int  get_i2c_mode(struct device *dev)
{
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  
    return m_dev->i2c_mode;
}


int  set_i2c_mode(struct device *dev, char *s)
{
    int l = strlen(s);
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  

    if(!strncmp("slave\n", s, l)){         //set mode to slave mode
	if(m_dev->i2c_mode == I2C_SLAVE_MODE)
	    return 1;
        struct i2c_client *cl = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
        if(!cl)
	    return -1;
	m_dev->slave = cl;
        u32 client_addr;
        if(!of_property_read_u32(dev->of_node, "client_addr", &client_addr))
	    (m_dev->slave)->addr = client_addr;
        else
            (m_dev->slave)->addr = I2C_CLIENT_ADDR;
        ls2x_i2c_reginit(m_dev);
	m_dev->i2c_mode = I2C_SLAVE_MODE;

	return 1;
    }
    else if(0 == strncmp("host\n", s, l)){
	if(m_dev->i2c_mode == I2C_HOST_MODE)
	    return 1;
	kfree(m_dev->slave);
	m_dev->i2c_mode = I2C_HOST_MODE;
	m_dev->slave = NULL;
        writeb(0x00, m_dev->base + LS2X_I2C_SADDR_REG);
	ls2x_i2c_reginit(m_dev);
	return 1;
    }
    else
	return 0;
}

unsigned char  get_client_addr(struct device *dev)
{
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  
    return (m_dev->slave)->addr;
}

int set_client_addr(struct device *dev, unsigned char addr)
{
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  
    (m_dev->slave)->addr = addr;
    ls2x_i2c_reginit(m_dev);
    return 1;
}
#endif

u32  get_speed(struct device *dev)
{
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  
    return m_dev->speed_hz;
}

void set_speed(struct device *dev, u32 speed)
{
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  
    m_dev->speed_hz = speed;
    ls2x_i2c_reginit(m_dev);

}



static int ls2x_i2c_probe(struct platform_device *pdev)
{
	struct ls2x_i2c_dev	*dev;
	struct i2c_adapter	*adap;
	struct resource		*mem, *ioarea;
	int r, irq;
	u32 bus_speed;

	bus_speed = i2c_acpi_find_bus_speed(&pdev->dev);
	if (!bus_speed)
		device_property_read_u32(&pdev->dev, "clock-frequency",
					 &bus_speed);

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	dev = kzalloc(sizeof(struct ls2x_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto err_release_region;
	}

	init_completion(&dev->cmd_complete);

	dev->dev = &pdev->dev;
	dev->irq = irq;
	dev->speed_hz = bus_speed;
	dev->base = ioremap(mem->start, resource_size(mem));
	if (!dev->base) {
		r = -ENOMEM;
		goto err_free_mem;
	}

	platform_set_drvdata(pdev, dev);

        dev->i2c_mode = I2C_HOST_MODE;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct property *i2c_mode_p = of_find_property(pdev->dev.of_node, "i2c-mode", NULL);
     	if(i2c_mode_p){
	    if(!strcmp("slave", i2c_mode_p->value)){
		dev->i2c_mode = I2C_SLAVE_MODE;
		struct i2c_client *cl = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
		if(!cl){
		    r = -ENOMEM;
		    goto err_free_mem;
		}
                dev->slave = cl;
		u32 client_addr;
		if(!of_property_read_u32(pdev->dev.of_node, "client_addr", &client_addr))
		    (dev->slave)->addr = client_addr;
		else
                    (dev->slave)->addr = I2C_CLIENT_ADDR;
	    }
	}
#endif


	ls2x_i2c_reginit(dev);

	r = request_irq(dev->irq, i2c_ls2x_isr, IRQF_SHARED, DRIVER_NAME, dev);
	if (r)
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->nr = pdev->id;
	strlcpy(adap->name, pdev->name, sizeof(adap->name));
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	adap->retries = 5;
	adap->algo = &ls2x_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	ACPI_COMPANION_SET(&adap->dev, ACPI_COMPANION(&pdev->dev));
	adap->timeout = msecs_to_jiffies(100);

	/* i2c device drivers may be active on return from add_adapter() */
	r = i2c_add_adapter(adap);
	if (r) {
		dev_err(dev->dev, "failure adding adapter\n");
		goto err_iounmap;
	}

	return 0;

err_iounmap:
	iounmap(dev->base);
err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(dev);
err_release_region:
	release_mem_region(mem->start, resource_size(mem));

	return r;
}

static int ls2x_i2c_remove(struct platform_device *pdev)
{
	struct ls2x_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	platform_set_drvdata(pdev, NULL);
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	kfree(dev->slave);
#endif
	i2c_del_adapter(&dev->adapter);
	free_irq(dev->irq, dev);
	iounmap(dev->base);
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));
	return 0;
}

#ifdef CONFIG_PM
static int ls2x_i2c_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ls2x_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_dev->suspended = 1;

	return 0;
}

static int ls2x_i2c_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ls2x_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_dev->suspended = 0;
	ls2x_i2c_reginit(i2c_dev);

	return 0;
}

static const struct dev_pm_ops ls2x_i2c_dev_pm_ops = {
	.suspend_noirq	= ls2x_i2c_suspend_noirq,
	.resume		= ls2x_i2c_resume,
};

#define LS2X_DEV_PM_OPS (&ls2x_i2c_dev_pm_ops)
#else
#define LS2X_DEV_PM_OPS NULL
#endif

#ifdef CONFIG_OF
static struct of_device_id ls2x_i2c_id_table[] = {
	{.compatible = "loongson,ls2h-i2c"},
	{.compatible = "loongson,ls2k-i2c"},
	{.compatible = "loongson,ls7a-i2c"},
	{.compatible = "loongson,ls-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, ls2x_i2c_id_table);
#endif
static const struct acpi_device_id loongson_i2c_acpi_match[] = {
	{"LOON0004"},
	{}
};
MODULE_DEVICE_TABLE(acpi, loongson_i2c_acpi_match);

static struct platform_driver ls2x_i2c_driver = {
	.probe		= ls2x_i2c_probe,
	.remove		= ls2x_i2c_remove,
	.driver		= {
		.name	= "ls2x-i2c",
		.owner	= THIS_MODULE,
		.pm	= LS2X_DEV_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ls2x_i2c_id_table),
#endif
		.acpi_match_table = ACPI_PTR(loongson_i2c_acpi_match),
	},
};

static int __init ls2x_i2c_init_driver(void)
{
	return platform_driver_register(&ls2x_i2c_driver);
}
subsys_initcall(ls2x_i2c_init_driver);

static void __exit ls2x_i2c_exit_driver(void)
{
	platform_driver_unregister(&ls2x_i2c_driver);
}
module_exit(ls2x_i2c_exit_driver);

MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_DESCRIPTION("Loongson LS2X I2C bus adapter");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ls2x-i2c");
