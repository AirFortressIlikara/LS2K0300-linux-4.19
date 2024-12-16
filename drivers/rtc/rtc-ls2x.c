// SPDX-License-Identifier: GPL-2.0
/*
 * Loongson-2H Real Time Clock interface for Linux
 *
 * Author: Shaozong Liu <liushaozong@loongson.cn>
 *	   Huacai Chen <chenhc@lemote.com>
 *
 */

#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <asm/io.h>
#include <asm/time.h>
#include <linux/mfd/syscon.h>

#ifdef CONFIG_CPU_LOONGSON2K
#include <loongson-2k.h>
#else
#include <loongson-pch.h>
#endif

static u32 ver = 0;

/**
 * Loongson-2H rtc register
 */

#define TOY_TRIM_REG   0x20
#define TOY_WRITE0_REG 0x24
#define TOY_WRITE1_REG 0x28
#define TOY_READ0_REG  0x2c
#define TOY_READ1_REG  0x30
#define TOY_MATCH0_REG 0x34
#define TOY_MATCH1_REG 0x38
#define TOY_MATCH2_REG 0x3c
#define RTC_CTRL_REG   0x40
#define RTC_TRIM_REG   0x60
#define RTC_WRITE0_REG 0x64
#define RTC_READE0_REG 0x68
#define RTC_MATCH0_REG 0x6c
#define RTC_MATCH1_REG 0x70
#define RTC_MATCH2_REG 0x74

/**
 * shift bits and filed mask
 */
#define TOY_MON_MASK   0x3f
#define TOY_DAY_MASK   0x1f
#define TOY_HOUR_MASK  0x1f
#define TOY_MIN_MASK   0x3f
#define TOY_SEC_MASK   0x3f
#define TOY_MSEC_MASK  0xf

#define TOY_MON_SHIFT  26
#define TOY_DAY_SHIFT  21
#define TOY_HOUR_SHIFT 16
#define TOY_MIN_SHIFT  10
#define TOY_SEC_SHIFT  4
#define TOY_MSEC_SHIFT 0

/* shift bits for TOY_MATCH */
#define TOY_MATCH_YEAR_SHIFT 26
#define TOY_MATCH_MON_SHIFT  22
#define TOY_MATCH_DAY_SHIFT  17
#define TOY_MATCH_HOUR_SHIFT 12
#define TOY_MATCH_MIN_SHIFT  6
#define TOY_MATCH_SEC_SHIFT  0

/* Filed mask bits for TOY_MATCH */
#define TOY_MATCH_YEAR_MASK  0x3f
#define TOY_MATCH_MON_MASK   0xf
#define TOY_MATCH_DAY_MASK   0x1f
#define TOY_MATCH_HOUR_MASK  0x1f
#define TOY_MATCH_MIN_MASK   0x3f
#define TOY_MATCH_SEC_MASK   0x3f

/* TOY Enable bits */
#define RTC_ENABLE_BIT		(1UL << 13)
#define TOY_ENABLE_BIT		(1UL << 11)
#define OSC_ENABLE_BIT		(1UL << 8)

#define LS7A_REVISION_R1	0x0080
#define LS7A_REVISION_R2	0x0181

/* ACPI and RTC offset */
#ifdef CONFIG_CPU_LOONGSON2K
#define ACPI_RTC_OFFSET		0x800//0x0
#else
#define ACPI_RTC_OFFSET		0x100//0x0
#endif

/* support rtc wakeup */
static struct regmap *regmap;
static void __iomem *acpi_reg_base;
#define PM1_STS_FOR_RTC		0x10
#define PM1_STS_STATUS_REG	0x0c
#define RTC_STS_WAKEUP_BIT	(0x1 << 10)
#define RTC_STS_STATUS_BIT	(0x1 << 10)
/* interface for rtc read and write */
#define rtc_write(val, addr)   writel(val, rtc_reg_base + (addr))
#define rtc_read(addr)         readl(rtc_reg_base + (addr))
#define rtc_pm_write(val, offset)   writel(val, acpi_reg_base + (offset))//writel(val, rtc_reg_base + (offset))
#define rtc_pm_read(offset)        readl(acpi_reg_base + (offset)) //readl(rtc_reg_base + (offset))

struct ls2x_rtc_info {
	struct platform_device *pdev;
	struct rtc_device *rtc_dev;
	struct resource *mem_res;
	void __iomem *rtc_base;
	int irq_base;
#ifdef CONFIG_CPU_LOONGSON2K
	unsigned int acpi_pm1_sts;
#endif
};

static void __iomem *rtc_reg_base;

static int rtc_pm_regmap_write(struct regmap *regmap, unsigned int offset,
			       unsigned int val)
{
	if(ver){
		rtc_write(val,offset);
	}else{
		if (!IS_ERR_OR_NULL(regmap))
			regmap_write(regmap, offset, val);
		else
			rtc_pm_write(val, offset);
	}
	return 0;
}

static int rtc_pm_regmap_read(struct regmap *regmap, unsigned int offset,
			      unsigned int *val)
{
	if(ver){
		*val = rtc_read(offset); 
	}else{
		if (!IS_ERR_OR_NULL(regmap))
			regmap_read(regmap, offset, val);
		else
			*val = rtc_pm_read(offset);
	}
	return 0;
}

/* IRQ Handlers */
static irqreturn_t ls2x_rtc_alarmirq(int irq, void *id)
{
	struct ls2x_rtc_info *info = (struct ls2x_rtc_info *)id;

	rtc_update_irq(info->rtc_dev, 1, RTC_AF | RTC_IRQF);
	return IRQ_HANDLED;
}

/* fix_event Handler */
#if defined(CONFIG_ACPI) && defined(CONFIG_LOONGARCH)
static u32 ls2x_rtc_handler(void *id)
{
	unsigned int temp;
	struct ls2x_rtc_info *info = (struct ls2x_rtc_info *)id;

	/* Disable acpi rtc enabled */
	rtc_pm_regmap_read(regmap, PM1_STS_FOR_RTC, &temp);
	temp &= ~RTC_STS_WAKEUP_BIT;
	rtc_pm_regmap_write(regmap, PM1_STS_FOR_RTC, temp);

	/*
	 * The TOY_MATCH0_REG should be cleared 0 here,
	 * otherwise the interrupt cannot be cleared.
	 * Because the match condition is still satisfied
	 */
	rtc_write(0, TOY_MATCH0_REG);

	/* Clear acpi rtc interrupt Status */
	temp = RTC_STS_STATUS_BIT;
	rtc_pm_regmap_write(regmap, PM1_STS_STATUS_REG, temp);

	rtc_update_irq(info->rtc_dev, 1, RTC_AF | RTC_IRQF);
	return 0;
}
#endif

/* Update control registers */
static int ls2x_rtc_setaie(struct device *dev, unsigned int enabled)
{
	unsigned int temp, temp1;
	int retry = 10;

	/* set acpi rtc wakeup */
	rtc_pm_regmap_read(regmap, PM1_STS_FOR_RTC, &temp);
	if (enabled)
		temp |= RTC_STS_WAKEUP_BIT;
	else
		temp &= ~RTC_STS_WAKEUP_BIT;

	do {
		rtc_pm_regmap_write(regmap, PM1_STS_FOR_RTC, temp);
		rtc_pm_regmap_read(regmap, PM1_STS_FOR_RTC, &temp1);
	} while (temp1 != temp && !!(retry--));

	return 0;
}

static int ls2x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	val = rtc_read(TOY_READ1_REG);
	tm->tm_year = val;
	val = rtc_read(TOY_READ0_REG);
	tm->tm_sec = (val >> TOY_SEC_SHIFT) & TOY_SEC_MASK;
	tm->tm_min = (val >> TOY_MIN_SHIFT) & TOY_MIN_MASK;
	tm->tm_hour = (val >> TOY_HOUR_SHIFT) & TOY_HOUR_MASK;
	tm->tm_mday = (val >> TOY_DAY_SHIFT) & TOY_DAY_MASK;
	tm->tm_mon = ((val >> TOY_MON_SHIFT) & TOY_MON_MASK) - 1;
	spin_unlock_irqrestore(&rtc_lock, flags);

	return 0;
}

static int ls2x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned int val = 0;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);

	val |= (tm->tm_sec << TOY_SEC_SHIFT);
	val |= (tm->tm_min << TOY_MIN_SHIFT);
	val |= (tm->tm_hour << TOY_HOUR_SHIFT);
	val |= (tm->tm_mday << TOY_DAY_SHIFT);
	val |= ((tm->tm_mon + 1) << TOY_MON_SHIFT);
	rtc_write(val, TOY_WRITE0_REG);
	val = tm->tm_year;
	rtc_write(val, TOY_WRITE1_REG);

	spin_unlock_irqrestore(&rtc_lock, flags);

	return 0;
}

static int ls2x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned int val;
	unsigned long flags;
	unsigned int alrm_en;
	struct rtc_time *tm = &alrm->time;
	rtc_pm_regmap_read(regmap, PM1_STS_FOR_RTC, &alrm_en);
	alrm_en &= RTC_STS_WAKEUP_BIT;

	alrm->enabled = (alrm_en) ? 1 : 0;

	spin_lock_irqsave(&rtc_lock, flags);

	val = rtc_read(TOY_MATCH0_REG);
	tm->tm_sec = (val >> TOY_MATCH_SEC_SHIFT) & TOY_MATCH_SEC_MASK;
	tm->tm_min = (val >> TOY_MATCH_MIN_SHIFT) & TOY_MATCH_MIN_MASK;
	tm->tm_hour = (val >> TOY_MATCH_HOUR_SHIFT) & TOY_MATCH_HOUR_MASK;
	tm->tm_mday = (val >> TOY_MATCH_DAY_SHIFT) & TOY_MATCH_DAY_MASK;
	tm->tm_mon = ((val >> TOY_MATCH_MON_SHIFT) & TOY_MATCH_MON_MASK) - 1;
	/* The 7a1000 and 2k1000 need add 64 to fix invalid alarm value issue. */
	tm->tm_year = ((val >> TOY_MATCH_YEAR_SHIFT) & TOY_MATCH_YEAR_MASK) + 64;

	spin_unlock_irqrestore(&rtc_lock, flags);

	return 0;
}

static int ls2x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned int val = 0;
	unsigned long flags;
	struct rtc_time *tm = &alrm->time;
	unsigned int retry;

	spin_lock_irqsave(&rtc_lock, flags);

	val |= (tm->tm_sec << TOY_MATCH_SEC_SHIFT);
	val |= (tm->tm_min << TOY_MATCH_MIN_SHIFT);
	val |= (tm->tm_hour << TOY_MATCH_HOUR_SHIFT);
	val |= (tm->tm_mday << TOY_MATCH_DAY_SHIFT);
	val |= ((tm->tm_mon + 1) << TOY_MATCH_MON_SHIFT);
	val |= ((tm->tm_year & TOY_MATCH_YEAR_MASK) << TOY_MATCH_YEAR_SHIFT);
	retry = 10;
	do {
		rtc_write(val, TOY_MATCH0_REG);
	} while (rtc_read(TOY_MATCH0_REG) != val && !!(retry--));
	if (!retry) {
		spin_unlock_irqrestore(&rtc_lock, flags);
		return -EIO;
	}


	spin_unlock_irqrestore(&rtc_lock, flags);

	ls2x_rtc_setaie(dev, alrm->enabled);
	return 0;
}

#ifdef CONFIG_LOONGSON_PM_ACPI
static int rtc_pm_acpi_notify(struct notifier_block *nb, unsigned long value,
			   void *arg)
{
	rtc_write(0, TOY_MATCH0_REG);

	return NOTIFY_DONE;
}

static struct notifier_block rtc_pm_acpi_notifier = {
	.notifier_call = rtc_pm_acpi_notify,
};

extern int register_ls_acpi_pm_notifier(struct notifier_block *);
extern int unregister_ls_acpi_pm_notifier(struct notifier_block *);
#endif

static struct rtc_class_ops ls2x_rtc_ops = {
	.read_time = ls2x_rtc_read_time,
	.set_time = ls2x_rtc_set_time,
	.read_alarm = ls2x_rtc_read_alarm,
	.set_alarm = ls2x_rtc_set_alarm,
	.alarm_irq_enable = ls2x_rtc_setaie,
};

static int ls2x_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct resource *res, *mem;
	struct rtc_device *rtc;
	struct ls2x_rtc_info *info;
	unsigned int __maybe_unused revision;
	unsigned int reg_val;

	info = kzalloc(sizeof(struct ls2x_rtc_info), GFP_KERNEL);
	if (!info) {
		pr_debug("%s: no enough memory\n", pdev->name);
		return -ENOMEM;
	}
	info->pdev = pdev;
	info->irq_base = platform_get_irq(pdev, 0);
	if (info->irq_base <= 0) {
		pr_debug("%s: no irq?\n", pdev->name);
		return -ENOENT;
	}
	if(pdev->dev.of_node && of_device_is_compatible(pdev->dev.of_node, "loongson,ls300-rtc"))
		ver = 1;
	else
		ver = 0;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_debug("%s: RTC resource data missing\n", pdev->name);
		return -ENOENT;
	}

	mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!mem) {
		pr_debug("%s: RTC registers at %x are not free\n",
			 pdev->name, (unsigned int)res->start);
		return -EBUSY;
	}
	info->mem_res = mem;
	info->rtc_base = ioremap(res->start, resource_size(res));
	if (!info->rtc_base) {
		pr_debug("%s: RTC registers can't be mapped\n", pdev->name);
		goto fail1;
	}
	rtc_reg_base = info->rtc_base;
	if(!ver){	
		regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "regmap");
		if (IS_ERR(regmap))
			acpi_reg_base = rtc_reg_base - ACPI_RTC_OFFSET;
	}
	/* enable rtc */
	rtc_write(0, RTC_TRIM_REG);
	rtc_write(0, TOY_TRIM_REG);
	reg_val = rtc_read(RTC_CTRL_REG);
	reg_val |= TOY_ENABLE_BIT | OSC_ENABLE_BIT;
	rtc_write(reg_val, RTC_CTRL_REG);
	if(!ver)
		ret = device_init_wakeup(dev, 1);
	if(ver){
		ls2x_rtc_ops.read_alarm = NULL;
		ls2x_rtc_ops.set_alarm = NULL;
		ls2x_rtc_ops.alarm_irq_enable = NULL;
	}
	rtc = info->rtc_dev = rtc_device_register(pdev->name, &pdev->dev,
						  &ls2x_rtc_ops, THIS_MODULE);
	if(!ver){
#if defined(CONFIG_CPU_LOONGSON3) || defined(CONFIG_CPU_LOONGSON64)
#if defined(CONFIG_LOONGARCH)
		if (!loongson_sysconf.is_soc_cpu) {
#endif
			/* get 7A pch revision number */
			revision = readl((void *)TO_UNCAC(LS7A_CHIPCFG_REG_BASE + 0x3ffc)) >> 16U;
			if (revision == LS7A_REVISION_R1 || revision == LS7A_REVISION_R2) {
				rtc->set_offset_nsec = -900000000;
			} else {
				rtc->set_offset_nsec = 0;
			}
#if defined(CONFIG_LOONGARCH)
		}
#endif
#endif
	}
	/* There don't need alarm interrupt */
	info->rtc_dev->uie_unsupported = 1;
	if(!ver){
		ret = devm_request_irq(dev, info->irq_base, ls2x_rtc_alarmirq,
			       IRQF_TRIGGER_RISING, "ls2x-rtc alarm", info);
		if (ret) {
			dev_err(&pdev->dev, "IRQ%d error %d\n", info->irq_base, ret);
			goto fail0;
		}
		if (has_acpi_companion(dev))
			acpi_install_fixed_event_handler(ACPI_EVENT_RTC, ls2x_rtc_handler, info);
		}
		if (IS_ERR(info->rtc_dev)) {
			pr_debug("%s: can't register RTC device, err %ld\n",
				 pdev->name, PTR_ERR(rtc));
		goto fail0;
	}

	platform_set_drvdata(pdev, info);
	dev_set_drvdata(&rtc->dev, info);
	if(!ver){
#ifdef CONFIG_LOONGSON_PM_ACPI
		register_ls_acpi_pm_notifier(&rtc_pm_acpi_notifier);
#endif
	}
	if (ret)
		dev_warn(dev, "init wakeup fail, ERRNO is %d\n", ret);
	return ret;

fail0:
	iounmap(info->rtc_base);
fail1:
	release_resource(mem);
	kfree(info);

	return -EIO;
}

static int ls2x_rtc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ls2x_rtc_info *info = platform_get_drvdata(pdev);
	struct rtc_device *rtc = info->rtc_dev;

	ls2x_rtc_setaie(dev, 0);
	iounmap(info->rtc_base);
	release_resource(dev_get_drvdata(&rtc->dev));
	release_resource(info->mem_res);
	rtc_device_unregister(rtc);
#ifdef CONFIG_LOONGSON_PM_ACPI
	unregister_ls_acpi_pm_notifier(&rtc_pm_acpi_notifier);
#endif
	kfree(info);

	return 0;
}

static int ls2x_rtc_suspend(struct device *dev)
{
#ifdef CONFIG_CPU_LOONGSON2K
	unsigned long flags;
	struct ls2x_rtc_info *info = dev_get_drvdata(dev);

	spin_lock_irqsave(&rtc_lock, flags);

	rtc_pm_regmap_read(regmap, PM1_STS_STATUS_REG, &info->acpi_pm1_sts);

	spin_unlock_irqrestore(&rtc_lock, flags);
#endif
	return 0;
}

static int ls2x_rtc_resume(struct device *dev)
{
#ifdef CONFIG_CPU_LOONGSON2K
	unsigned long flags;
	unsigned int temp;
	struct ls2x_rtc_info *info = dev_get_drvdata(dev);

	spin_lock_irqsave(&rtc_lock, flags);

	/* clear acpi RTC_STS (RTC Status) */
	temp = info->acpi_pm1_sts | RTC_STS_STATUS_BIT;

	rtc_pm_regmap_write(regmap, PM1_STS_STATUS_REG, temp);

	spin_unlock_irqrestore(&rtc_lock, flags);

	dev_dbg(dev, "resume, acpi PM1_STS: %x\n", temp);
#endif
	return 0;
}

static SIMPLE_DEV_PM_OPS(ls2x_rtc_pm_ops, ls2x_rtc_suspend, ls2x_rtc_resume);

#ifdef CONFIG_OF
static struct of_device_id ls2x_rtc_id_table[] = {
	{.compatible = "loongson,ls2h-rtc"},
	{.compatible = "loongson,ls2k-rtc"},
	{.compatible = "loongson,ls7a-rtc"},
	{.compatible = "loongson,ls-rtc"},
	{.compatible = "loongson,ls300-rtc"},
	{},
};
#endif

static const struct acpi_device_id loongson_rtc_acpi_match[] = {
	{"LOON0001"},
	{}
};
MODULE_DEVICE_TABLE(acpi, loongson_rtc_acpi_match);

static struct platform_driver ls2x_rtc_driver = {
	.probe		= ls2x_rtc_probe,
	.remove		= ls2x_rtc_remove,
	.driver		= {
		.name	= "ls2x-rtc",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ls2x_rtc_id_table),
#endif
		.acpi_match_table = ACPI_PTR(loongson_rtc_acpi_match),
		.pm = &ls2x_rtc_pm_ops,
	},
};

static int __init rtc_init(void)
{
	return platform_driver_register(&ls2x_rtc_driver);
}

static void __exit rtc_exit(void)
{
	platform_driver_unregister(&ls2x_rtc_driver);
}

module_init(rtc_init);
module_exit(rtc_exit);

MODULE_AUTHOR("Liu Shaozong");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ls2x-rtc");
