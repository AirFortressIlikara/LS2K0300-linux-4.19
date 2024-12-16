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

#define DRIVER_NAME "ls300_i2c"

#define I2C_DutyCycle 0
/*regs*/
#define CR1   0x0
#define CR2   0x4
#define OAR   0x8
#define DR    0x10
#define SR1   0x14
#define SR2   0x18
#define CCR   0x1c
#define TRISE 0x20

#define CR1_PE      0x1
#define CR1_RECOVER 0x4000
#define CR1_START   0x100
#define CR1_STOP    0x200
#define CR1_SWRST   0x8000
#define CR1_ACK     0x400
#define CR1_POS     0x800



#define CR2_DMAEN   0x800
#define CR2_ITBUFEN 0x400
#define CR2_ITEVTEN 0x200
#define CR2_ITERREN 0x100


#define SR1_SB      0x1
#define SR1_ADDR    0x2
#define SR1_BTF     0x4
#define SR1_STOPF   0x10
#define SR1_RXNE    0x40
#define SR1_TXE     0x80
#define SR1_BERR    0x100
#define SR1_ARLO    0x200
#define SR1_AF      0x400
#define SR1_OVR     0x800

#define SR2_MSL     0x1
#define SR2_BUSY    0x2
#define SR2_TRA     0x4
#define SR2_GENCALL 0x10




#define I2C_HOST_MODE           0x0
#define I2C_SLAVE_MODE          0x1
#define I2C_CLIENT_ADDR 0x12
#define ee_inw(reg)         (*(volatile unsigned short*)(reg))
#define ee_outw(reg, val)	(*(volatile unsigned short*)(reg) = (val))
#define i2c_readw(addr)		readw(dev->base + addr)
#define i2c_writew(val, addr)	writew(val, dev->base + addr)

#ifdef LS2X_I2C_DEBUG
#define i2c_debug(fmt, args...)	printk(KERN_CRIT fmt, ##args)
#else
#define i2c_debug(fmt, args...)
#endif

#define I2C_ClockSpeed 100000
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
	reinit_completion(&dev->cmd_complete);
	ee_outw(CR1, CR1_PE | CR1_STOP);
	while (!(ee_inw(CR1) & CR1_STOP));
	return 0;
}

static int i2c_start(struct ls2x_i2c_dev *dev,
		int dev_addr, int flags)
{
	unsigned long time_left;
	int retry = 5;
	unsigned char addr = (dev_addr & 0x7f) << 1;
	addr |= (flags & I2C_M_RD)? 1:0;
	if(ee_inw(SR2) & SR2_BUSY){
		pr_info("bus busy\n");
		return 0;
	}
start:
	mdelay(1);
	pr_info("2k300 i2c start\n");
	ee_outw(CR1, (CR1_PE | CR1_START));
	i2c_debug("%s <line%d>: i2c device address: 0x%x\n",
			__func__, __LINE__, addr);
	time_left = wait_for_completion_timeout(
		&dev->cmd_complete,
		(&dev->adapter)->timeout);
	if (!time_left) {
		pr_info("Timeout abort message cmd\n");
		return -1;
	}
	if(!(ee_inw(SR1) & SR1_SB)){
		while (retry--)
			goto start;
		return 0;
	}
	ee_outw(DR, addr);
	time_left = wait_for_completion_timeout(
		&dev->cmd_complete,
		(&dev->adapter)->timeout);
	if (!time_left) {
		pr_info("Timeout abort message cmd\n");
		return -1;
	}
	if(!(ee_inw(SR1) & SR1_ADDR)){
		pr_info("There is no i2c device ack");
		return 0;
	}
	ee_inw(SR2);
	if((flags & I2C_M_RD) && (!(ee_inw(SR1) & SR1_TXE))){
		pr_info("TXE error\n");
		return 0;
	}
	if(!(flags & I2C_M_RD) && (!(ee_inw(SR1) & SR1_RXNE))){
		pr_info("RXNE error\n");
		return 0;
	}
	return 1;
}

#if 0//IS_ENABLED(CONFIG_I2C_SLAVE)
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

static void ls300_i2c_reginit(struct ls2x_i2c_dev *dev)
{
	u16 tmpreg = 0, freqrange = 0;
    u16 result = 0x04;
    u32 pclk1 = 10000000;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	if (dev->slave) {
		__ls_i2c_reg_slave(dev, dev->slave->addr);
		return;
	}
#endif /* CONFIG_I2C_SLAVE */
	
    /* Clear frequency FREQ[5:0] bits */
    ee_inw(CR2) &= ~0x3f;
    /* Set frequency bits depending on pclk1 value */
    freqrange = (u16)(pclk1 / 1000000);
    /* Write to I2Cx CR2 */
    ee_inw(CR2) |= freqrange;
    /* Disable the selected I2C peripheral to configure TRISE */
    ee_inw(CR1) &= (~0x1);

    tmpreg = 0;

    /* Configure speed in standard mode */
    if (I2C_ClockSpeed <= 100000) {
        /* Standard mode speed calculate */
        result = (u16)(pclk1 / (I2C_ClockSpeed << 1));
        /* Test if CCR value is under 0x4*/
        if (result < 0x04) {
            /* Set minimum allowed value */
            result = 0x04;  
        }
        /* Set speed value for standard mode */
        tmpreg |= result;	  
        /* Set Maximum Rise Time for standard mode */
        ee_outw(TRISE, freqrange + 1);  
    } else {/* Configure speed in fast mode */
        if (I2C_DutyCycle == 0x0) {
            /* Fast mode speed calculate: Tlow/Thigh = 2 */
            result = (u16)(pclk1 / (I2C_ClockSpeed * 3));
        }
        else /*I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_16_9*/
        {
            /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
            result = (u16)(pclk1 / (I2C_ClockSpeed * 25));
            /* Set DUTY bit */
            result |= 0x4000;
        }
        /* Test if CCR value is under 0x1*/
        if ((result & 0xfff) == 0)
        {
            /* Set minimum allowed value */
            result |= (u16)0x0001;  
        }
        /* Set speed value and set F/S bit for fast mode */
        tmpreg |= result;
        /* Set Maximum Rise Time for fast mode */
        ee_outw(TRISE, (u16)(((freqrange * 300) / 1000) + 1));  
    }
    /* Write to I2Cx CCR */
    ee_outw(CCR, tmpreg);
	ee_outw(CR1, 0x401);
	ee_inw(CR2) |= 0x700;
    

}
static int i2c_read(struct ls2x_i2c_dev *dev,
		unsigned char *buf, int count)
{
	int i;
	unsigned long time_left;
		for (i = 0; i < count; i++) {
		buf[i] = ee_inw(DR);
		if(i == count - 1){
			ee_inw(CR1) &= (~0x400);
		}
		time_left = wait_for_completion_timeout(
				&dev->cmd_complete,
				(&dev->adapter)->timeout);
		if (!time_left) {
			pr_info("Timeout abort message cmd\n");
			return -1;
		}
		if(!(ee_inw(SR1) & SR1_RXNE)){
			pr_info("SR1_RXNE error\n");
			return -1;
		}
	}
	ee_inw(CR1) |= (0x400);
	return i;
}

static int i2c_write(struct ls2x_i2c_dev *dev,
		unsigned char *buf, int count)
{
        int i;
		unsigned long time_left;

        for (i = 0; i < count; i++) {
			ee_outw(DR, buf[i]);
			i2c_debug("%s <line%d>: write buf[%d] => %02x\n",
				__func__, __LINE__, i, buf[i]);
			time_left = wait_for_completion_timeout(
				&dev->cmd_complete,
				(&dev->adapter)->timeout);
			if (!time_left) {
				pr_info("Timeout abort message cmd\n");
				return -1;
			}
			if(!(ee_inw(SR1) & SR1_BTF)){
				i2c_debug("%s <line%d>: device no ack\n",
					__func__, __LINE__);
				i2c_stop(dev);
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

static const struct i2c_algorithm ls300_i2c_algo = {
	.master_xfer	= i2c_xfer,
	.functionality	= i2c_func,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave	= ls_i2c_reg_slave,
	.unreg_slave	= ls_i2c_unreg_slave,
#endif /* CONFIG_I2C_SLAVE */
};

#if 0//IS_ENABLED(CONFIG_I2C_SLAVE)
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
static irqreturn_t i2c_ls300_isr(int this_irq, void *dev_id)
{
	unsigned short iflag;
	struct ls2x_i2c_dev *dev = dev_id;

	iflag = ee_inw(SR1);

	if (iflag != 0) {
#if IS_ENABLED(CONFIG_I2C_SLAVE)
		if (dev->slave) {
			ls_i2c_slave_irq(dev);
		}
#endif
			complete(&dev->cmd_complete);
	} else
		return IRQ_NONE;

	return IRQ_HANDLED;
}



#if 0 //IS_ENABLED(CONFIG_I2C_SLAVE)
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

/*u32  get_speed(struct device *dev)
{
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  
    return m_dev->speed_hz;
}*/

/*void set_speed(struct device *dev, u32 speed)
{
    struct ls2x_i2c_dev* m_dev = dev->driver_data;  
    m_dev->speed_hz = speed;
    ls2x_i2c_reginit(m_dev);

}*/



static int ls300_i2c_probe(struct platform_device *pdev)
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


	ls300_i2c_reginit(dev);

	r = request_irq(dev->irq, i2c_ls300_isr, IRQF_SHARED, DRIVER_NAME, dev);
	if (r)
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->nr = pdev->id;
	strlcpy(adap->name, pdev->name, sizeof(adap->name));
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	adap->retries = 5;
	adap->algo = &ls300_i2c_algo;
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

static int ls300_i2c_remove(struct platform_device *pdev)
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

#define LS300_DEV_PM_OPS NULL

#ifdef CONFIG_OF
static struct of_device_id ls300_i2c_id_table[] = {
	{.compatible = "loongson,ls300-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, ls300_i2c_id_table);
#endif

static struct platform_driver ls300_i2c_driver = {
	.probe		= ls300_i2c_probe,
	.remove		= ls300_i2c_remove,
	.driver		= {
		.name	= "ls300-i2c",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ls300_i2c_id_table),
#endif
	},
};

static int __init ls300_i2c_init_driver(void)
{
	return platform_driver_register(&ls300_i2c_driver);
}
subsys_initcall(ls300_i2c_init_driver);

static void __exit ls300_i2c_exit_driver(void)
{
	platform_driver_unregister(&ls300_i2c_driver);
}
module_exit(ls300_i2c_exit_driver);

MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_DESCRIPTION("Loongson LS2K300 I2C bus adapter");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ls300-i2c");
