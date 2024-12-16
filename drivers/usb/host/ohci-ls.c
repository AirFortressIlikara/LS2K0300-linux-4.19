/*
* OHCI HCD (Host Controller Driver) for USB.
*
* Based on various ohci-*.c drivers
*
* This file is licensed under the terms of the GNU General Public
* License version 2. This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ohci.h"

#define DRIVER_DESC "OHCI ls2k driver"

static const char hcd_name[] = "ls2k-ohci";

static struct hc_driver __read_mostly ohci_ls2k_hc_driver;

static void ls2k_reset_hc(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ohci_regs __iomem *regs = hcd->regs;
	/*
	 * Put the USB host controller into reset.
	 */
	writel(0, &regs->control);

}

static int ls2k_ohci_hcd_drv_probe(struct platform_device *pdev)
{
	const struct hc_driver *driver = &ohci_ls2k_hc_driver;
	struct usb_hcd *hcd = NULL;
	struct resource *res;
	int retval, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		retval = irq;
		goto fail;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	retval = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (retval)
		goto fail;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		retval = -ENODEV;
		goto err_put_hcd;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	if (!devm_request_mem_region(&pdev->dev, hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		retval = -EBUSY;
		goto err_put_hcd;
	}

	hcd->regs = devm_ioremap(&pdev->dev, hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_dbg(&pdev->dev, "Failed to ioremap registers.\n");
		retval = -ENOMEM;
		goto err_put_hcd;
	}

	ls2k_reset_hc(pdev);
	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto err_put_hcd;

	return retval;

err_put_hcd:
	usb_put_hcd(hcd);
fail:
	dev_err(&pdev->dev, "init fail, %d\n", retval);

	return retval;
}

static int ls2k_ohci_hcd_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	ls2k_reset_hc(pdev);

	usb_put_hcd(hcd);
	devm_release_mem_region(&pdev->dev, hcd->rsrc_start, hcd->rsrc_len);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int ohci_hcd_restore(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	ohci_resume(hcd, true);
	return 0;
}

static const struct dev_pm_ops usb_ohci_hcd_pm_ops = {
	.restore = ohci_hcd_restore,
};
#endif

static const struct of_device_id ls2k_ohci_id_table[] = {
	{ .compatible = "loongson,ls2k-ohci", },
	{ },
};
MODULE_DEVICE_TABLE(of, ls2k_ohci_id_table);

/* Driver definition to register with the platform bus */
static struct platform_driver ls2k_ohci_hcd_driver = {
	.probe =	ls2k_ohci_hcd_drv_probe,
	.remove =	ls2k_ohci_hcd_drv_remove,
	.driver = {
		.name = "ls2k-ohci",
		.bus = &platform_bus_type,
		.of_match_table = ls2k_ohci_id_table,
#ifdef CONFIG_PM
		.pm =	&usb_ohci_hcd_pm_ops
#endif
	},
};

static int __init ohci_ls2k_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);

	ohci_init_driver(&ohci_ls2k_hc_driver, NULL);
	return platform_driver_register(&ls2k_ohci_hcd_driver);
}
module_init(ohci_ls2k_init);

static void __exit ohci_ls2k_cleanup(void)
{
	platform_driver_unregister(&ls2k_ohci_hcd_driver);
}
module_exit(ohci_ls2k_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ls2k-ohci");
