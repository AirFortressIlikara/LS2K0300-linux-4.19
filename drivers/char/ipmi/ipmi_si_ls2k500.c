// SPDX-License-Identifier: GPL-2.0+
/*
 * ipmi_si_pci.c
 *
 * Handling for IPMI devices on the PCI bus.
 */

#define pr_fmt(fmt) "ipmi_pci: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include "ipmi_si.h"
#include "kcs_bmc_ls2k500.h"

#define KCS_STATUS_CMD_DAT      BIT(3)

static unsigned char intf_sim_inb(const struct si_sm_io *io,
				  unsigned int offset)
{
	IPMIKCS *ik = io->addr_source_data;
	uint32_t ret;

	btlock_lock(&ik->lock, 0, 1);
	switch (offset & 1) {
	case 0:
		ret = ik->data_out_reg;
		IPMI_KCS_SET_OBF(ik->status_reg, 0);
		break;
	case 1:
		ret = ik->status_reg;
		break;
	}
	btlock_unlock(&ik->lock, 0);
	return ret;
}

static void intf_sim_outb(const struct si_sm_io *io, unsigned int offset,
			  unsigned char val)
{
	IPMIKCS *ik = io->addr_source_data;

	btlock_lock(&ik->lock, 0, 1);
	if (IPMI_KCS_GET_IBF(ik->status_reg)) {
		goto out;
	}

	switch (offset & 1) {
	case 0:
		ik->data_in_reg = val;
		ik->status_reg &= ~KCS_STATUS_CMD_DAT;
		break;

	case 1:
		ik->cmd_reg = val;
		ik->status_reg |= KCS_STATUS_CMD_DAT;
		break;
	}
	IPMI_KCS_SET_IBF(ik->status_reg, 1);
	ik->write_req++;
out:
	btlock_unlock(&ik->lock, 0);
}

static void ipmi_ls2k500_cleanup(struct si_sm_io *io)
{
}

int ipmi_si_sim_setup(struct si_sm_io *io)
{
	io->inputb = intf_sim_inb;
	io->outputb = intf_sim_outb;
	io->io_cleanup = ipmi_ls2k500_cleanup;
	return 0;
}

#define platform_resource_start(dev, bar)   ((dev)->resource[(bar)].start)
#define platform_resource_end(dev, bar)     ((dev)->resource[(bar)].end)
static int of_ipmi_ls2k500_probe(struct platform_device *pdev)
{
	int rv;
	struct si_sm_io io;
	memset(&io, 0, sizeof(io));
	io.addr_source = SI_PLATFORM;
	dev_info(&pdev->dev, "probing via ls2k500 platform");
	io.si_type = SI_KCS;

	io.addr_source_cleanup = ipmi_ls2k500_cleanup;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
	io.addr_space = IPMI_MEM_ADDR_SPACE;
#else
	io.addr_type = IPMI_MEM_ADDR_SPACE;
#endif
	io.io_setup = ipmi_si_sim_setup;
	io.addr_data = pdev->resource[0].start;
	io.addr_source_data = ioremap(pdev->resource[0].start,
					pdev->resource[0].end -
					pdev->resource[0].start + 1);
	io.dev = &pdev->dev;
	io.regspacing = 4;
	io.regsize = DEFAULT_REGSIZE;
	io.regshift = 0;
	io.irq = 0;
	if (io.irq)
		io.irq_setup = ipmi_std_irq_setup;

	dev_info(&pdev->dev, "%pR regsize %d spacing %d irq %d\n",
		&pdev->resource[0], io.regsize, io.regspacing, io.irq);

	rv = ipmi_si_add_smi(&io);

	return rv;
}

static int ipmi_ls2k500_remove(struct platform_device *pdev)
{
	return ipmi_si_remove_by_dev(&pdev->dev);
}

#define SI_DEVICE_NAME "ipmi_ls2k500_si"
struct platform_driver ipmi_ls2k500_platform_driver = {
	.driver = {
		.name = SI_DEVICE_NAME,
	},
	.probe		= of_ipmi_ls2k500_probe,
	.remove		= ipmi_ls2k500_remove,
};

static bool platform_registered;
void ipmi_si_ls2k500_init(void)
{
	struct pci_dev *pdev = NULL;
	resource_size_t start;
	int i, rv;
	struct resource res[1] = {
		[0] = {.flags = IORESOURCE_MEM,},
	};
	do
		pdev = pci_get_device(0x14, 0x1a05, pdev);
	while (pdev && pdev->bus->number == 0);
	if (!pdev)
		return;

	pci_enable_device(pdev);
	start = pci_resource_end(pdev, 0) + 1 - 0x1000000 + 0xf00000;
	for (i = 0; i < 5; i++) {
		res[0].start = start + sizeof(IPMIKCS)*i;
		res[0].end = start + sizeof(IPMIKCS)*(i+1) - 1;
		platform_device_register_simple(SI_DEVICE_NAME, i, res, 1);
	}
	rv = platform_driver_register(&ipmi_ls2k500_platform_driver);
	if (rv)
		pr_err("Unable to register driver: %d\n", rv);
	else
		platform_registered = true;
}

void ipmi_si_ls2k500_shutdown(void)
{
	if (platform_registered)
		platform_driver_unregister(&ipmi_ls2k500_platform_driver);
}
