/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/*
 * Copyright (C) 2019-2020 Loongson Technology Ltd.
 * Author:	Zhu Chen <zhuchen@loongson.cn>
 *		Lv Chen <lvchen@loongson.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/iommu.h>
#include <linux/sizes.h>
#include <asm/ptrace.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pci_regs.h>


#define LS_IOMMU_PGSIZE	(SZ_4K | SZ_16K | SZ_64K | SZ_1M | SZ_16M | SZ_32M)


#define IOMMU_VENDOR_BASE		0x90000efdfe000100ULL
#define IOMMU_FUNC_CHECK		0x90000efdfe000108ULL
#define IOMMU_MEMORY_WRITEABLE		0x90000efdfb0001c0ULL
#define IOMMU_HT_LO_ADDRESS		LS_HIGHMEM_BASE10010424ULL
#define LOONGSON_PCI_MSI_ADDR		0xFDF8000000ULL
#define LS_IOMMU_ENTRYLO_0		0x0
#define LS_IOMMU_ENTRYLO_1		0x8
#define LS_IOMMU_ENTRYHI		0x10
#define LS_IOMMU_INDEX			0x18
#define LS_IOMMU_PAGEMASK		0x28
#define LS_IOMMU_COMMAND		0x68
#define LS_IOMMU_BDF			0x78
#define LS_IOMMU_TLBNUM_REG		0x100
#define LS_IOMMU_PAGEMASK_1M		0x1FE000
#define LS_IOMMU_PAGEMASK_16M		0x1FFE000

#define LS_IOMMU_ERROR_ID		0x80
#define LS_IOMMU_ERROR_ADDR		0x88
#define IOMMU_COMMAND_WR		0
#define IOMMU_COMMAND_READ		2
#define LS_HIGHMEM_BASE			0x90000000


#define LS_IOMMU_VENDOR			0x7a100014
#define LS_IOMMU_FUNC			0x6000001


#define LS_IOMMU_TLB_NUM	32
#define LS_IOMMU_TLB_BITMAP	32

/*only use 32MB hugepage*/
#define LS_IOMMU_PAGE_SHIFT		25
#define LS_IOMMU_PAGE_SIZE		(_AC(1, UL) << LS_IOMMU_PAGE_SHIFT)
#define LS_IOMMU_PAGE_MASK		(~((1ULL << LS_IOMMU_PAGE_SHIFT) - 1))
#define DELAY	1

/*virtio page use 16k*/
#define LS_VIRTIO_PAGE_SHIFT	14
#define LS_VIRTIO_PAGE_SIZE	(_AC(1, UL) << LS_VIRTIO_PAGE_SHIFT)
#define LS_VIRTIO_PAGE_MASK	(~((1ULL << LS_VIRTIO_PAGE_SHIFT) - 1))
#define LS_VIRTIO_PGD_SHIFT	25
#define LS_VIRTIO_PGD_MASK	(((1ULL << LS_VIRTIO_PGD_SHIFT) - 1))

#define IOMMU_ADDRESS_BASE		0xEFDFE000300ULL
#define PHYS_IOMMU_MEMORY_ADDR		0x49600000ULL
#define IOMMU_CONFBUS			0xe0010010414ULL

#define UNCAC_IOMMU_BASE		(UNCAC_BASE | IOMMU_ADDRESS_BASE)
#define UNCAC_IOMMU_CONFBUS		(UNCAC_BASE | IOMMU_CONFBUS)
#define IOMMU_MEMORY_ADDRESS		(UNCAC_BASE + PHYS_IOMMU_MEMORY_ADDR)

/* List of all available dev_data structures */
static LIST_HEAD(global_dev_data_list);
DEFINE_SPINLOCK(loongson_iommu_lock);
static struct iommu_ops loongson_iommu_ops;

/*iommu iotlb */
typedef struct iommu_entry_str {
	unsigned long	entryLo0;
	unsigned long	entryLo1;
	unsigned long	entryHi;
	unsigned long	pagemask;
	bool		valid;
	unsigned int 	bdf;
} iommu_entry;

#define INVALID_IOMMU_OP		0
#define FLUSH_IOMMU			1
#define ADD_IOMMU			2
struct iommu_table {
	iommu_entry		*pgtable;
	unsigned long		len;
	unsigned long		*bitmap;
	unsigned long		pending;
	unsigned int		effect_count;
	unsigned int		op;
};

/*One vm is equal to a domain,one domain has a priv*/
typedef struct loongson_iommu_priv {
	/* for list of all protection domains */
	struct list_head 	list;
	/* List of all devices in this domain */
	struct list_head	list_attached;
	struct iommu_domain	domain;
	iommu_entry		*pgtable;
	unsigned long		len;
	unsigned long		*bitmap;
	unsigned long		memslot[LS_IOMMU_TLB_NUM];
	unsigned long		*virtio_pgtable;
	unsigned long		virtio_pgtable_len;
	struct device		*dev;
	spinlock_t		pgtlock; /* pagetable lock */
	struct iommu_table	*hwtable;
} loongson_iommu_priv;

/*a device for passthrough*/
struct ls_iommu_dev_data {
	struct list_head list;	/* For domain->dev_list */
	struct list_head glist;	/* For global dev_data_list */
	loongson_iommu_priv	*priv;
	unsigned short bdf;
	int count;
};

struct global_io_pgtable{
	struct ls_io_pgtable	*g_pgtable;
	unsigned long		g_io_pgtable_len;
	unsigned long		*g_io_pgtable_bitmap;
};

struct iotlb_data {
	unsigned long iova;
	unsigned long entrylo0;
	unsigned long entrylo1;
	unsigned long bdf;
	unsigned long pagemask;
	int index;
	int reserved;
};

static struct iommu_table *g_io_pgtable;

int loongson_iommu_disable;

static void iommu_write_regl(unsigned long off, unsigned long val)
{
	*(unsigned long *)(IOMMU_MEMORY_ADDRESS + off) = val;
	mb();
}

static unsigned long iommu_read_regl(unsigned long off)
{
	unsigned long val;

	val = *(unsigned long *)(IOMMU_MEMORY_ADDRESS + off);
	mb();
	return val;
}

static void iommu_memory_writeable(void)
{
	/* write enable */
	*(unsigned long *)IOMMU_MEMORY_WRITEABLE |= (0x1 << 26);
	mb();
}

static void iommu_memory_writedisable(void)
{
	/*write disable*/
	*(unsigned long *)IOMMU_MEMORY_WRITEABLE &= ~(0x1 << 26);
	mb();
}

static loongson_iommu_priv *to_loongson_iommu_priv(struct iommu_domain *dom)
{
	return container_of(dom, loongson_iommu_priv, domain);
}

/* write to real hardware iommu table */
static void sync_hwiotlb(loongson_iommu_priv *priv)
{
	int i;
	struct iommu_table *hwtable;

	hwtable = priv->hwtable;
	/*enable iommu memory*/
	iommu_memory_writeable();
	for (i = 0; i < hwtable->len; i++) {
		if ((hwtable->pending & (0x1ULL << i)) == 0)
			continue;

		if (hwtable->op == FLUSH_IOMMU) {
			/* invalid real iommu */
			iommu_write_regl(LS_IOMMU_ENTRYLO_0, 0);
			iommu_write_regl(LS_IOMMU_ENTRYLO_1, 0);
			iommu_write_regl(LS_IOMMU_ENTRYHI, 0);
			iommu_write_regl(LS_IOMMU_PAGEMASK, 0);
			iommu_write_regl(LS_IOMMU_BDF, 0);
			iommu_write_regl(LS_IOMMU_INDEX, i);
			mb();
			iommu_write_regl(LS_IOMMU_COMMAND, IOMMU_COMMAND_WR);
			mb();
			mdelay(DELAY);
		} else if (hwtable->op == ADD_IOMMU) {
			iommu_write_regl(LS_IOMMU_ENTRYLO_0, hwtable->pgtable[i].entryLo0);
			iommu_write_regl(LS_IOMMU_ENTRYLO_1, hwtable->pgtable[i].entryLo1);
			iommu_write_regl(LS_IOMMU_ENTRYHI, hwtable->pgtable[i].entryHi);
			iommu_write_regl(LS_IOMMU_PAGEMASK, hwtable->pgtable[i].pagemask);
			iommu_write_regl(LS_IOMMU_BDF, hwtable->pgtable[i].bdf);
			iommu_write_regl(LS_IOMMU_INDEX, i);
			mb();
			/*0 write 0x2 read*/
			iommu_write_regl(LS_IOMMU_COMMAND, IOMMU_COMMAND_WR);
			mb();
			mdelay(DELAY);
		}
	}

	hwtable->op = INVALID_IOMMU_OP;
	hwtable->pending = 0;
	iommu_memory_writedisable();
	mb();
}

static void flush_shadowtlb(loongson_iommu_priv *priv,
				unsigned long va, unsigned short bdf)
{
	int i;
	struct iommu_table	*hwtable;
	iommu_entry		*entry;

	hwtable = priv->hwtable;
	for (i = 0; i < hwtable->len; i++) {
		entry = &hwtable->pgtable[i];
		if (entry->valid == false)
			continue;

		if ((entry->entryHi == va) && (entry->bdf == bdf)) {
			/* found matched */
			hwtable->pgtable[i].entryLo0	= 0;
			hwtable->pgtable[i].entryLo1	= 0;
			hwtable->pgtable[i].entryHi	= 0;
			hwtable->pgtable[i].valid	= false;
			hwtable->pgtable[i].pagemask	= 0;
			hwtable->pgtable[i].bdf		= 0;
			bitmap_clear(hwtable->bitmap, i, 1);

			hwtable->pending |= 0x1ULL << i;
			hwtable->op	 = FLUSH_IOMMU;
			hwtable->effect_count--;
		}
	}
}

static void add_shadowtlb(loongson_iommu_priv *priv, iommu_entry *entry)
{
	struct iommu_table *hwtable;
	int i;

	hwtable = priv->hwtable;
	i = find_first_zero_bit(hwtable->bitmap, LS_IOMMU_TLB_BITMAP);
	if (i == LS_IOMMU_TLB_BITMAP) {
		printk(KERN_ERR"%s:%d iommu entry is full\n",
						__func__, __LINE__);
		return;
	}

	hwtable->pgtable[i].entryLo0 = entry->entryLo0;
	hwtable->pgtable[i].entryLo1 = entry->entryLo1;
	hwtable->pgtable[i].pagemask = entry->pagemask;
	hwtable->pgtable[i].entryHi = entry->entryHi;
	hwtable->pgtable[i].valid = true;
	hwtable->pgtable[i].bdf = entry->bdf;
	bitmap_set(hwtable->bitmap, i, 1);

	hwtable->pending |= 0x1ULL << i;
	hwtable->op	= ADD_IOMMU;
	hwtable->effect_count++;
}

static bool loongson_iommu_capable(enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		return true;
	default:
		return false;
	}
}

static loongson_iommu_priv *loongson_iommu_alloc_priv(void)
{
	loongson_iommu_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		printk("%s:%d iommu debug error --- priv addr 0x%lx\n",
				__func__, __LINE__, (unsigned long)priv);
		goto fail_nomem;
	}

	spin_lock_init(&priv->pgtlock);
	INIT_LIST_HEAD(&priv->list_attached);
	priv->pgtable = kzalloc(sizeof(iommu_entry) * LS_IOMMU_TLB_NUM,
							GFP_KERNEL);
	priv->bitmap = (unsigned long *)kzalloc(LS_IOMMU_TLB_BITMAP,
							GFP_KERNEL);
	priv->virtio_pgtable = (unsigned long *)__get_free_pages(GFP_KERNEL, 6);
	if ((!priv->pgtable) || (!priv->bitmap) || (!priv->virtio_pgtable)) {
		printk("%s:%d pgtable addr 0x%lx bitmap addr 0x%lx\n",
		__func__, __LINE__, (unsigned long)priv->pgtable,
		(unsigned long)priv->bitmap);
		goto fail_nomem;
	}

	memset(priv->pgtable, 0x0, sizeof(iommu_entry) * LS_IOMMU_TLB_NUM);
	memset(priv->bitmap, 0x0, LS_IOMMU_TLB_BITMAP);
	memset(priv->virtio_pgtable, 0x0, 16384*64);
	iommu_memory_writeable();
	priv->len = iommu_read_regl(LS_IOMMU_TLBNUM_REG);
	iommu_memory_writedisable();
	priv->hwtable = g_io_pgtable;
	return priv;

fail_nomem:
	kfree(priv);
	return NULL;
}

static struct iommu_domain *loongson_iommu_domain_alloc(unsigned type)
{
	loongson_iommu_priv *priv;

	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	priv = loongson_iommu_alloc_priv();
	if (!priv)
		return NULL;
	priv->domain.geometry.aperture_start	= 0;
	priv->domain.geometry.aperture_end	= ~0ULL;
	priv->domain.geometry.force_aperture	= true;

	printk("%s:%d iommu alloc domain --- type %d\n", __func__,
							__LINE__, type);
	return &priv->domain;
}


static void loongson_iommu_domain_free(struct iommu_domain *domain)
{
	loongson_iommu_priv *priv;

	priv = to_loongson_iommu_priv(domain);
	if (priv) {
		if (priv->virtio_pgtable) {
			free_pages(priv->virtio_pgtable, 6);
			priv->virtio_pgtable = NULL;
		}
		if (priv->pgtable) {
			kfree(priv->pgtable);
			priv->pgtable = NULL;
		}
		if (priv->bitmap) {
			kfree(priv->bitmap);
			priv->bitmap = NULL;
		}
		kfree(priv);
	}
}

static int iommu_init_device(struct device *dev)
{
	struct ls_iommu_dev_data *dev_data;
	struct pci_dev	*pdev = to_pci_dev(dev);
	struct pci_bus	*bus = pdev->bus;
	unsigned short bdf;
	unsigned char busnum;

	bdf = pdev->devfn & 0xff;
	busnum = bus->number;
	if (busnum != 0) {
		while (bus->parent->parent)
			bus = bus->parent;
		bdf = bus->self->devfn & 0xff;
	}

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		printk("kzalloc dev_data error\n");
		return -1;
	}

	dev_data->bdf = bdf;
	dev_data->count = 1;
	dev->archdata.iommu = dev_data;
	list_add_tail(&dev_data->glist, &global_dev_data_list);
#ifdef LOONGSON_IOMMU_DEBUG
	printk(KERN_ERR"%s:%d iommu debug--- bdf 0x%x\n", __func__,
							__LINE__, bdf);
#endif

	return 0;
}

static int loongson_iommu_add_device(struct device *dev)
{
	struct iommu_group *group;

	iommu_init_device(dev);
	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group))
		return PTR_ERR(group);

	iommu_group_put(group);
	return 0;
}

static struct iommu_group *loongson_iommu_device_group(struct device *dev)
{
	struct iommu_group *group;

	/*
	 * We don't support devices sharing stream IDs other than PCI RID
	 * aliases, since the necessary ID-to-device lookup becomes rather
	 * impractical given a potential sparse 32-bit stream ID space.
	 */
	if (dev_is_pci(dev))
		group = pci_device_group(dev);
	else
		group = generic_device_group(dev);

	return group;
}


static void loongson_iommu_remove_device(struct device *dev)
{
	struct ls_iommu_dev_data *dev_data;

	iommu_group_remove_device(dev);

	dev_data = dev->archdata.iommu;
	if (dev_data) {
		list_del(&dev_data->glist);
		kfree(dev_data);
	}
}

static struct ls_iommu_dev_data *iommu_get_devdata(loongson_iommu_priv *priv,
							unsigned long bdf)
{
	struct ls_iommu_dev_data *dev_data;

	list_for_each_entry(dev_data, &priv->list_attached, list) {
		if (dev_data->bdf == bdf) {
			return dev_data;
		}
	}

	return NULL;
}

static int loongson_iommu_attach_dev(struct iommu_domain *domain,
							struct device *dev)
{
	loongson_iommu_priv *priv = to_loongson_iommu_priv(domain);
	struct pci_dev	*pdev = to_pci_dev(dev);
	struct pci_bus	*bus = pdev->bus;
	unsigned char busnum = pdev->bus->number;
	struct ls_iommu_dev_data *dev_data;
	unsigned short bdf;
	unsigned long iova, pa;
	int ret = 0, i, j;

	bdf = pdev->devfn & 0xff;
	if (busnum != 0) {
		while (bus->parent->parent)
			bus = bus->parent;
		bdf = bus->self->devfn & 0xff;
	}

	printk("%s:%d iommu debug--- 0x%x busnum 0x%x devfn %x\n",
				__func__, __LINE__, bdf, busnum, pdev->devfn);
	dev_data = iommu_get_devdata(priv, bdf);
	if (dev_data) {
		printk("%s:%d iommu debug--- bdf 0x%x devfn %x has attached\n",
					__func__, __LINE__, bdf, pdev->devfn);
		dev_data->count++;
		return 0;
	}

	dev_data = dev->archdata.iommu;
	dev_data->priv = priv;
	list_add(&dev_data->list, &priv->list_attached);

	bdf	= bdf << 8;
	iova	= LOONGSON_PCI_MSI_ADDR;
	pa	= LOONGSON_PCI_MSI_ADDR;
	j	= find_first_zero_bit(priv->bitmap, LS_IOMMU_TLB_BITMAP);
	priv->pgtable[j].pagemask = LS_IOMMU_PAGEMASK_16M;
	priv->pgtable[j].entryLo0 = (((pa >> 12) << 6)) | 0x6ULL;
	priv->pgtable[j].entryLo1 = ((((pa + SZ_16M) >> 12) << 6)) | 0x6ULL;
	priv->pgtable[j].entryHi = iova;
	priv->pgtable[j].valid = true;
	priv->pgtable[j].bdf = bdf;
	bitmap_set(priv->bitmap, j, 1);
	add_shadowtlb(priv, &priv->pgtable[j]);
	for (i = 0; i < LS_IOMMU_TLB_BITMAP; i++) {
		pa = priv->memslot[i];
		if (pa == 0)
			continue;
		iova = ((unsigned long)i) << LS_IOMMU_PAGE_SHIFT;
		if (iova >= SZ_256M)
			iova = iova - SZ_256M + LS_HIGHMEM_BASE;
		j = find_first_zero_bit(priv->bitmap, LS_IOMMU_TLB_BITMAP);
		priv->pgtable[j].pagemask = LS_IOMMU_PAGEMASK_16M;
		priv->pgtable[j].entryLo0 = (((pa >> 12) << 6)) | 0x6ULL;
		priv->pgtable[j].entryLo1 =
				((((pa + 0x1000000ULL) >> 12) << 6)) | 0x6ULL;
		priv->pgtable[j].entryHi = iova;
		priv->pgtable[j].valid = true;
		priv->pgtable[j].bdf = bdf;
		bitmap_set(priv->bitmap, j, 1);
		add_shadowtlb(priv, &priv->pgtable[j]);
	}

	if (priv->hwtable->pending)
		sync_hwiotlb(priv);

	return ret;
}

static void loongson_iommu_detach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	//fix me disable mmu
	loongson_iommu_priv *priv = to_loongson_iommu_priv(domain);
	struct pci_dev *pdev = to_pci_dev(dev);
	struct pci_bus *bus = pdev->bus;
	unsigned char busnum = pdev->bus->number;
	struct ls_iommu_dev_data *dev_data;
	unsigned short bdf;
	unsigned long iova;
	int i;


	bdf = pdev->devfn & 0xff;
	if (busnum != 0) {
		while (bus->parent->parent)
			bus = bus->parent;
		bdf = bus->self->devfn & 0xff;
	}

	dev_data = iommu_get_devdata(priv, bdf);
	if (dev_data == NULL) {
		printk(KERN_INFO "%s:%d iommu debug--- bdf 0x%x has attach\n",
						__func__, __LINE__, bdf);
		return;
	}

	dev_data->count--;
	if (dev_data->count) {
		printk(KERN_INFO "iommu %s:%d bdf 0x%x has attach devfn %x\n",
					__func__, __LINE__, bdf, pdev->devfn);
		return;
	}
	dev_data->priv = NULL;

	list_del(&dev_data->list);
	bdf = bdf << 8;
	for (i = 0; i < priv->len ; i++) {
		if (priv->pgtable[i].valid == false)
			continue;
		if (priv->pgtable[i].bdf != bdf)
			continue;
		iova = priv->pgtable[i].entryHi;
		priv->pgtable[i].valid = false;
		priv->pgtable[i].pagemask = 0;
		priv->pgtable[i].entryLo0 = 0;
		priv->pgtable[i].entryLo1 = 0;
		priv->pgtable[i].entryHi = 0;
		priv->pgtable[i].bdf = 0;
		bitmap_clear(priv->bitmap, i, 1);
		flush_shadowtlb(priv, iova, bdf);
	}

	if (priv->hwtable->pending)
		sync_hwiotlb(priv);
}

static int loongson_iommu_fill_pgtable(loongson_iommu_priv *priv,
					unsigned long iova, phys_addr_t pa)
{
	struct ls_iommu_dev_data *dev_data;
	int i;

	list_for_each_entry(dev_data, &priv->list_attached, list) {
/*0x0x01FFE000 --> size of each output page 16M and entryLo0 + entryLo1 is 32M*/
		i = find_first_zero_bit(priv->bitmap, LS_IOMMU_TLB_BITMAP);
		if (i == LS_IOMMU_TLB_BITMAP) {
			printk(KERN_INFO "%s fail to find empty iotlb entry\n",
								__func__);
			return 0;
		}
		priv->pgtable[i].pagemask = LS_IOMMU_PAGEMASK_16M;
		priv->pgtable[i].entryLo0 = (((pa >> 12) << 6)) | 0x6ULL;
		priv->pgtable[i].entryLo1 =
			((((pa + 0x1000000ULL) >> 12) << 6)) | 0x6ULL;
		priv->pgtable[i].entryHi = iova;
		priv->pgtable[i].valid = true;
		priv->pgtable[i].bdf = dev_data->bdf << 8;
		bitmap_set(priv->bitmap, i, 1);
		add_shadowtlb(priv, &priv->pgtable[i]);
	}
	return 0;

}

static int loongson_iommu_map(struct iommu_domain *domain, unsigned long iova,
			 phys_addr_t pa, size_t len, int prot)
{
	loongson_iommu_priv *priv = to_loongson_iommu_priv(domain);
	int i;
	unsigned long *pte;
	long size, mask;

	size = len;
	if ((iova >= SZ_256M) && (iova < LS_HIGHMEM_BASE)) {
		iova -= SZ_256M;
		pte = (unsigned long *)(priv->virtio_pgtable);
		while (size > 0) {
			pte[iova >> LS_VIRTIO_PAGE_SHIFT] =
					pa & LS_VIRTIO_PAGE_MASK;
			size -= 0x4000;
			iova += 0x4000;
			pa += 0x4000;
		}
		return 0;
	}
	mask = LS_IOMMU_PAGE_SIZE - 1;
	if ((iova & mask) != (pa & mask)) {
		printk(KERN_INFO
			"%s:%d iova %lx pa %llx not 32M aligned len 0x%lx\n",
					__func__, __LINE__, iova, pa, len);
		return -1;
	}

	iova = iova & LS_IOMMU_PAGE_MASK;
	pa = pa & LS_IOMMU_PAGE_MASK;
	while (size > 0) {
		i = iova >> LS_IOMMU_PAGE_SHIFT;
		if (i > (SZ_256M >> LS_IOMMU_PAGE_SHIFT))
			i = i - ((LS_HIGHMEM_BASE - SZ_256M) >> LS_IOMMU_PAGE_SHIFT);
		if (priv->memslot[i] == 0) {
			priv->memslot[i] = pa;
			printk("%s:%d iova %lx pa %llx Adding 32M page 0x%lx\n",
					 __func__, __LINE__, iova, pa, len);
			loongson_iommu_fill_pgtable(priv, iova, pa);
		}

		iova	+= SZ_32M;
		size	-= SZ_32M;
		pa	+= SZ_32M;
	}

	if (priv->hwtable->pending) {
		sync_hwiotlb(priv);
	}
	return 0;
}

static size_t loongson_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
				size_t len)
{
	loongson_iommu_priv *priv = to_loongson_iommu_priv(domain);
	unsigned long i;
	size_t unmap_len = 0;
	unsigned long *pte;
	long size;
	unsigned short bdf;

	size = len;
	if ((iova >= SZ_256M) && (iova < LS_HIGHMEM_BASE)) {
		iova -= SZ_256M;
		pte = (unsigned long *)priv->virtio_pgtable;
		while (size > 0) {
			//pte[iova >> 14] = 0;
			size -= 0x4000;
			unmap_len += 0x4000;
			iova += 0x4000;
		}
		unmap_len += size;
		return unmap_len;
	}

#ifdef LOONGSON_IOMMU_DEBUG
	printk(KERN_ERR
	"%s:%d iommu debug--- domain addr 0x%lx iova 0x%lx len 0x%lx\n",
					__func__, __LINE__, domain, iova, len);
#endif
	iova = iova & LS_IOMMU_PAGE_MASK;
	while (size > 0) {
		i = iova >> LS_IOMMU_PAGE_SHIFT;
		if (i > (SZ_256M >> LS_IOMMU_PAGE_SHIFT))
			i = i - ((LS_HIGHMEM_BASE - SZ_256M) >> LS_IOMMU_PAGE_SHIFT);
		if (priv->memslot[i] != 0) {
			//priv->memslot[i] = 0;
			for (i = 0; i < priv->len; i++) {
				if (priv->pgtable[i].valid == false)
					continue;
				if (priv->pgtable[i].entryHi != iova)
					continue;

				bdf = priv->pgtable[i].bdf;
				priv->pgtable[i].valid = false;
				priv->pgtable[i].pagemask = 0;
				priv->pgtable[i].entryLo0 = 0;
				priv->pgtable[i].entryLo1 = 0;
				priv->pgtable[i].entryHi = 0;
				priv->pgtable[i].bdf = 0;
				bitmap_clear(priv->bitmap, i, 1);
				flush_shadowtlb(priv, iova, bdf);
			}
		}

		iova += SZ_32M;
		size -= SZ_32M;
		unmap_len += SZ_32M;
		if (size < 0)
			unmap_len += size;
	}

	if (priv->hwtable->pending)
		sync_hwiotlb(priv);

	return unmap_len;
}

static phys_addr_t loongson_iommu_iova_to_phys(struct iommu_domain *domain,
					dma_addr_t va)
{
	loongson_iommu_priv *priv = to_loongson_iommu_priv(domain);
	unsigned long offset, i, pa, iova;
	unsigned long *pte;

	if ((va >= SZ_256M) && (va < LS_HIGHMEM_BASE)) {
		iova = va & LS_VIRTIO_PAGE_MASK;
		pte = (unsigned long *)priv->virtio_pgtable;
		offset = va & ((1ULL << LS_VIRTIO_PAGE_SHIFT) - 1);
		pa = pte[(iova - SZ_256M) >> 14] + offset;
	} else {

		iova = va & LS_IOMMU_PAGE_MASK;
		i = iova >> LS_IOMMU_PAGE_SHIFT;
		if (i > (SZ_256M >> LS_IOMMU_PAGE_SHIFT))
			i = i - ((LS_HIGHMEM_BASE - SZ_256M) >> LS_IOMMU_PAGE_SHIFT);
		pa = priv->memslot[i];
	}

	if (pa == 0)
		printk("%s:%d iommu find iova to pa--va 0x%llx pa %lx\n",
						__func__, __LINE__, va, pa);
	return pa;
}

static struct iommu_ops loongson_iommu_ops = {
	.capable = loongson_iommu_capable,
	.domain_alloc = loongson_iommu_domain_alloc,
	.domain_free = loongson_iommu_domain_free,
	.attach_dev = loongson_iommu_attach_dev,
	.detach_dev = loongson_iommu_detach_dev,
	.map = loongson_iommu_map,
	.unmap = loongson_iommu_unmap,
	.iova_to_phys = loongson_iommu_iova_to_phys,
	.add_device = loongson_iommu_add_device,
	.remove_device = loongson_iommu_remove_device,
	.device_group = loongson_iommu_device_group,
	.pgsize_bitmap = LS_IOMMU_PGSIZE,
};

static inline int loongson_iommu_init_api(void)
{
	int ret;

	ret = bus_set_iommu(&pci_bus_type, &loongson_iommu_ops);
	return ret;
}

static int loongson_iommu_probe(struct platform_device *pdev)
{
	*(unsigned long *)(UNCAC_IOMMU_BASE | PCI_BASE_ADDRESS_0) =
		PHYS_IOMMU_MEMORY_ADDR;
	*(unsigned long *)(UNCAC_IOMMU_BASE | PCI_COMMAND) =
		(PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	*(unsigned int *)(UNCAC_IOMMU_CONFBUS) = 0x0;

	g_io_pgtable = kzalloc(sizeof(*g_io_pgtable), GFP_KERNEL);
	if (!g_io_pgtable) {
		printk(KERN_ERR"%s:%d iommu debug error\n", __func__, __LINE__);
		return -1;
	}

	memset(g_io_pgtable, 0x0, sizeof(*g_io_pgtable));
	g_io_pgtable->pgtable =
		kzalloc(sizeof(iommu_entry) * LS_IOMMU_TLB_NUM, GFP_KERNEL);
	g_io_pgtable->bitmap = kzalloc(LS_IOMMU_TLB_BITMAP, GFP_KERNEL);

	memset(g_io_pgtable->pgtable, 0x0,
			sizeof(iommu_entry) * LS_IOMMU_TLB_NUM);
	memset(g_io_pgtable->bitmap, 0x0, LS_IOMMU_TLB_BITMAP);
	g_io_pgtable->len = LS_IOMMU_TLB_NUM;

	loongson_iommu_init_api();
	return 0;
}

static int loongson_iommu_remove(struct platform_device *pdev)
{
	kfree(g_io_pgtable->pgtable);
	kfree(g_io_pgtable->bitmap);
	kfree(g_io_pgtable);
	return 0;
}



static int __init loongson_iommu_setup(char *str)
{
	if (!str)
		return -EINVAL;
	while (*str) {
		if (!strncmp(str, "on", 2)) {
			loongson_iommu_disable = 0;
			pr_info("IOMMU enabled\n");
		} else if (!strncmp(str, "off", 3)) {
			loongson_iommu_disable = 1;
			pr_info("IOMMU disabled\n");
		}
		str += strcspn(str, ",");
		while (*str == ',')
			str++;
	}
	return 0;
}
__setup("loongson_iommu=", loongson_iommu_setup);



static struct platform_driver loongson_iommu_driver = {
	.driver = {
		.name = "loongson-iommu",
	},
	.probe	= loongson_iommu_probe,
	.remove	= loongson_iommu_remove,
};

static int __init loongson_iommu_driver_init(void)
{
	int ret = 0;

	if (loongson_iommu_disable == 0) {
		if ((*(unsigned long *)IOMMU_VENDOR_BASE == LS_IOMMU_VENDOR)
		&& (*(unsigned long *)IOMMU_FUNC_CHECK == LS_IOMMU_FUNC)) {
			ret = platform_driver_register(&loongson_iommu_driver);
		}
	}
	if (ret != 0) {
		pr_err("Failed to register IOMMU driver\n");
		goto error;
	}
error:
	return ret;
}

static void __exit loongson_iommu_driver_exit(void)
{
	platform_driver_unregister(&loongson_iommu_driver);
}

subsys_initcall(loongson_iommu_driver_init);
module_exit(loongson_iommu_driver_exit);
