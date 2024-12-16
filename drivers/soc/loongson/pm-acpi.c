// SPDX-License-Identifier: GPL-2.0

#include <linux/io.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeirq.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <asm/suspend.h>

#define PM1_CNT_REG	0x14
#define PM1_EVT_REG	0x0c
#define PM1_ENA_REG	0x10
#define GPE0_STS_REG	0x28
#define GPE0_ENA_REG	0x2c

/* Power Management Status */
#define ACPI_PWRBT_STATUS		(1 << 8)
#define ACPI_RTC_STATUS			(1 << 10)
#define ACPI_PCIE_WAKEUP_STATUS		(1 << 14)
#define ACPI_WAKE_STATUS		(1 << 15)

/* Power Management Enbale */
#define ACPI_PWRBT_EN		ACPI_PWRBT_STATUS

/* General Purpose Event0 Status */
#define GPE0_GMAC_STATUS	(0x3 << 5)
#define GPE0_USB_STATUS		(0x3f << 10)

/* General Purpose Event0 Enable */
#define GPE0_GMAC_EN	GPE0_GMAC_STATUS
#define GPE0_USB_EN	GPE0_USB_STATUS

static struct loongson_acpi_pm {
	void __iomem *base;
	struct input_dev *button;
	bool button_suspend;
} acpi_pm;

static ATOMIC_NOTIFIER_HEAD(acpi_pm_notif_list);

#define acpi_pm_readw(offset)		readw(acpi_pm.base + offset)
#define acpi_pm_readl(offset)		readl(acpi_pm.base + offset)
#define acpi_pm_writew(val, offset)	writew(val, acpi_pm.base + offset)
#define acpi_pm_writel(val, offset)	writel(val, acpi_pm.base + offset)

static void acpi_hw_clear_status(void)
{
	u16 value;

	/* PMStatus: Clear WakeStatus/PwrBtnStatus */
	value = acpi_pm_readw(PM1_EVT_REG);
	value |= (ACPI_PWRBT_STATUS | ACPI_PCIE_WAKEUP_STATUS |
						ACPI_WAKE_STATUS);
	acpi_pm_writew(value, PM1_EVT_REG);

	/* GPEStatus: Clear all generated events */
	acpi_pm_writel(acpi_pm_readl(GPE0_STS_REG), GPE0_STS_REG);
}

static void acpi_enable_sci(void)
{
	u16 value;

	value = acpi_pm_readw(PM1_CNT_REG);
	value |= 1;
	acpi_pm_writew(value, PM1_CNT_REG);
}

static void acpi_registers_setup(void)
{
	u16 value;

	acpi_enable_sci();

	/* setup PM EN reg */
	value = acpi_pm_readw(PM1_ENA_REG);

	/* PMEnable: Enable PwrBtn always */
	value |= ACPI_PWRBT_EN;

	acpi_pm_writew(value, PM1_ENA_REG);
}

static void mach_resume(void)
{
	arch_common_resume();
}

static void mach_suspend(void)
{
	acpi_hw_clear_status();
	arch_common_suspend();
}

static int loongson_pm_enter(suspend_state_t state)
{
	mach_suspend();

	/* processor specific suspend */
	loongarch_suspend_enter();
	pm_set_resume_via_firmware();

	mach_resume();

	return 0;
}

static int loongson_pm_begin(suspend_state_t state)
{
	pm_set_suspend_via_firmware();
	return 0;
}

static int loongson_pm_valid_state(suspend_state_t state)
{
	if (state == PM_SUSPEND_ON)
		return 1;
	else if (state == PM_SUSPEND_MEM)
		return !!loongson_sysconf.suspend_addr;

	return 0;
}

static const struct platform_suspend_ops loongson_pm_ops = {
	.valid	= loongson_pm_valid_state,
	.begin	= loongson_pm_begin,
	.enter	= loongson_pm_enter,
};

static void __init loongson_pm_init(void)
{
	if (loongson_sysconf.suspend_addr)
		suspend_set_ops(&loongson_pm_ops);
}

static irqreturn_t acpi_int_handler(int irq, void *dev_id)
{
	u16 status = acpi_pm_readw(PM1_EVT_REG);

	if (!acpi_pm.button_suspend && (status & ACPI_PWRBT_STATUS)) {
		pr_info("Power Button pressed...\n");
		input_report_key(acpi_pm.button, KEY_POWER, 1);
		input_sync(acpi_pm.button);
		input_report_key(acpi_pm.button, KEY_POWER, 0);
		input_sync(acpi_pm.button);
	}

	if (status & ACPI_RTC_STATUS)
		atomic_notifier_call_chain(&acpi_pm_notif_list,	status, NULL);

	acpi_hw_clear_status();

	return IRQ_HANDLED;
}

int register_ls_acpi_pm_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&acpi_pm_notif_list, nb);
}
EXPORT_SYMBOL_GPL(register_ls_acpi_pm_notifier);

int unregister_ls_acpi_pm_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&acpi_pm_notif_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_ls_acpi_pm_notifier);

#ifdef CONFIG_PM_SLEEP
static int acpi_pmc_pm_suspend(struct device *dev)
{
	acpi_pm.button_suspend = true;
	return 0;
}

static int acpi_pmc_pm_resume(struct device *dev)
{
	acpi_pm.button_suspend = false;
	acpi_hw_clear_status();
	return 0;
}
#endif
static SIMPLE_DEV_PM_OPS(acpi_pmc_pm_ops,
				acpi_pmc_pm_suspend, acpi_pmc_pm_resume);

static int acpi_pmc_pm_notify(struct notifier_block *nb,
				unsigned long action, void *ptr)
{
	switch (action) {
	case PM_POST_SUSPEND:
		acpi_enable_sci();
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static int __init power_button_init(struct device *dev, int irq)
{
	int ret;
	struct input_dev *button;

	button = input_allocate_device();
	if (!button)
		return -ENOMEM;

	button->name = "ACPI Power Button";
	button->phys = "acpi/button/input0";
	button->id.bustype = BUS_HOST;
	button->dev.parent = NULL;
	input_set_capability(button, EV_KEY, KEY_POWER);

	ret = request_irq(irq, acpi_int_handler, IRQF_SHARED, "acpi-pmc",
				acpi_int_handler);
	if (ret) {
		dev_err(dev, "Power Button: Request irq %d failed!\n", irq);
		goto free_button;
	}

	ret = input_register_device(button);
	if (ret)
		goto free_button;

	dev_pm_set_wake_irq(&button->dev, irq);
	device_set_wakeup_capable(&button->dev, true);
	device_set_wakeup_enable(&button->dev, true);

	acpi_pm.button = button;
	dev_info(dev, "Power Button: Init successful!\n");

	return 0;
free_button:
	input_free_device(button);
	return ret;
}

static void __init loongson_acpi_init(void)
{
	acpi_registers_setup();
	acpi_hw_clear_status();
}

static int __init loongson_acpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *regs;
	u32 suspend_addr;
	int irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "get irq failed!\n");
		return -EINVAL;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(dev, "missing regs resources!\n");
		return -EINVAL;
	}

	if (!request_mem_region(regs->start, resource_size(regs),
				pdev->name)) {
		dev_err(dev, "can't request memmem region!\n");
		return -EBUSY;
	}
	acpi_pm.base = ioremap(regs->start, resource_size(regs));
	if (!acpi_pm.base) {
		release_mem_region(regs->start, resource_size(regs));
		dev_err(dev, "map memmem region failed!\n");
		return -ENOMEM;
	}

	if (device_property_read_u32(dev, "suspend-address", &suspend_addr))
		dev_err(dev, "No suspend-address, could not support S3!\n");
	else
		loongson_sysconf.suspend_addr = TO_UNCAC(suspend_addr);

	if (power_button_init(dev, irq))
		goto release_mem;

	loongson_acpi_init();
	loongson_pm_init();
	pm_notifier(acpi_pmc_pm_notify, 0);

	return 0;
release_mem:
	release_mem_region(regs->start, resource_size(regs));
	return -EINVAL;
}

static int ls_acpi_remove(struct platform_device *pdev)
{
	return -EPERM;
}

static const struct of_device_id ls_acpi_pmc_match[] = {
	{ .compatible = "loongson,acpi-pmc", },
	{},
};

static struct platform_driver ls_acpi_pmc_driver __refdata = {
	.driver = {
		.name = "acpi-pmc",
		.pm = &acpi_pmc_pm_ops,
		.of_match_table = ls_acpi_pmc_match,
	},
	.probe = loongson_acpi_probe,
	.remove = ls_acpi_remove,
};

builtin_platform_driver(ls_acpi_pmc_driver);
