/*
* Driver for EHCI HCD on ls2k SOC
*
* Copyright (C) 2010 ST Micro Electronics,
* Deepak Sikri <deepak.sikri@st.com>
*
* Based on various ehci-*.c drivers
*
* This file is subject to the terms and conditions of the GNU General Public
* License. See the file COPYING in the main directory of this archive for
* more details.
*/

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ehci_pdriver.h>
#include <loongson-2k.h>
#include "ehci.h"

#define DRIVER_DESC "EHCI LS2K driver"

static const char hcd_name[] = "ls2k-ehci";

static int ehci_platform_reset(struct usb_hcd *hcd)
{
	struct platform_device *pdev = to_platform_device(hcd->self.controller);
	struct usb_ehci_pdata *pdata = pdev->dev.platform_data;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	hcd->has_tt = pdata->has_tt;
	ehci->has_synopsys_hc_bug = pdata->has_synopsys_hc_bug;
	ehci->big_endian_desc = pdata->big_endian_desc;
	ehci->big_endian_mmio = pdata->big_endian_mmio;

	if (pdata->pre_setup) {
		retval = pdata->pre_setup(hcd);

		if (retval < 0)
			return retval;
	}

	retval = ehci_setup(hcd);
	if (retval)
		return retval;

	if (pdata->no_io_watchdog)
		ehci->need_io_watchdog = 0;

	return 0;
}

static struct hc_driver __read_mostly ehci_ls2k_hc_driver;

static const struct ehci_driver_overrides platform_overrides __initconst = {
    .reset =    ehci_platform_reset,
};

static struct usb_ehci_pdata ehci_platform_defaults;

static int ls2k_ehci_hcd_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd ;
	struct resource *res;
	const struct hc_driver *driver = &ehci_ls2k_hc_driver;
	int irq, retval;
	 __be32 *dma_mask_p = NULL;

	if (usb_disabled())
		return -ENODEV;

	if (pdev->dev.of_node) {
		dma_mask_p = (__be32 *)of_get_property(pdev->dev.of_node, "dma-mask", NULL);
		if (dma_mask_p != 0) {
			if (of_read_number(dma_mask_p, 2) == DMA_BIT_MASK(64))
				ls2k_writeq(ls2k_readq(LS2K_GEN_CONFIG0_REG) | (0x1UL << 36), LS2K_GEN_CONFIG0_REG);
			else
				ls2k_writeq(ls2k_readq(LS2K_GEN_CONFIG0_REG) & ~(0x1UL << 36), LS2K_GEN_CONFIG0_REG);
		}
	}

	if (!pdev->dev.platform_data)
		pdev->dev.platform_data = &ehci_platform_defaults;

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
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -ENOMEM;
		goto err_put_hcd;
	}


	/* registers start at offset 0x0 */
	hcd_to_ehci(hcd)->caps = hcd->regs;

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto err_put_hcd;

	return retval;

err_put_hcd:
	usb_put_hcd(hcd);
fail:
	dev_err(&pdev->dev, "init fail, %d\n", retval);

	return retval ;
}

static int ls2k_ehci_hcd_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);

	usb_put_hcd(hcd);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int ehci_hcd_restore(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	ehci_resume(hcd, true);
	return 0;
}

static const struct dev_pm_ops usb_ehci_hcd_pm_ops = {
	.restore = ehci_hcd_restore,
};
#endif

static struct of_device_id ls2k_ehci_id_table[] = {
	{ .compatible = "loongson,ls2k-ehci", },
	{ },
};
MODULE_DEVICE_TABLE(of, ls2k_ehci_id_table);

static struct platform_driver ls2k_ehci_hcd_driver = {
	.probe		= ls2k_ehci_hcd_drv_probe,
	.remove		= ls2k_ehci_hcd_drv_remove,
	.driver		= {
		.name = "ls2k-ehci",
		.bus = &platform_bus_type,
		.of_match_table = of_match_ptr(ls2k_ehci_id_table),
#ifdef CONFIG_PM
		.pm =	&usb_ehci_hcd_pm_ops
#endif
	}
};


static int __init ehci_ls2k_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);

	ehci_init_driver(&ehci_ls2k_hc_driver, &platform_overrides);
	return platform_driver_register(&ls2k_ehci_hcd_driver);
}
module_init(ehci_ls2k_init);

static void __exit ehci_ls2k_cleanup(void)
{
	platform_driver_unregister(&ls2k_ehci_hcd_driver);
}
module_exit(ehci_ls2k_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ls2k-ehci");
