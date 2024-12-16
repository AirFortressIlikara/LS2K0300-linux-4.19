/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 * Authors:
 *	Zhu Chen <zhuchen@loongson.cn>
 *	Fang Yaling <fangyaling@loongson.cn>
 *	Zhang Dandan <zhangdandan@loongson.cn>
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/console.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/property.h>
#include "loongson_drv.h"
#include "loongson_vbios.h"
#include "loongson_prime.h"

#include "bridge/bridge_phy.h"

#define DEVICE_NAME "loongson-drm"
#define DRIVER_NAME "loongson-drm"
#define DRIVER_DESC "Loongson DRM Driver"
#define DRIVER_AUTHOR "Loongson graphics driver team"
#define DRIVER_DATE "20180328"
#define DRIVER_MAJOR 0
#define DRIVER_MINOR 4
#define DRIVER_PATCHLEVEL 0

int loongson_lg100_support;
MODULE_PARM_DESC(LG100_support, "LG100 support (1 = enabled, 0 = disabled)");
module_param_named(LG100_support, loongson_lg100_support, int, 0444);

int loongson_ttm_enable = 1;
MODULE_PARM_DESC(
	enable,
	"enable of loongson-drm (1 = enabled (default) , 0 = disabled)");
module_param_named(enable, loongson_ttm_enable, int, 0444);

int loongson_gamma = -1;
MODULE_PARM_DESC(gamma,
	"enable gamma (-1 = disabled (default) , >0 = enabled)");
module_param_named(gamma, loongson_gamma, int, 0444);


u32 ls_mm_rreg(struct loongson_device *ldev, u32 offset)
{
	return readl(ldev->mmio + offset);
}

void ls_mm_wreg(struct loongson_device *ldev, u32 offset, u32 val)
{
	writel(val, ldev->mmio + offset);
}

void ls_mm_wreg_check(struct loongson_device *ldev, u32 offset, u32 val)
{
	u32 val_reg, count = 10000;

	writel(val, ldev->mmio + offset);
	do {
		val_reg = readl(ldev->mmio + offset);
	} while ((val_reg != val) && (count--));
}

u32 ls_io_rreg(struct loongson_device *ldev, u32 offset)
{
	return readl(ldev->io + offset);
}

void ls_io_wreg(struct loongson_device *ldev, u32 offset, u32 val)
{
	writel(val, ldev->io + offset);
}

u32 ls_mm_rreg_locked(struct loongson_device *ldev, u32 offset)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&ldev->mmio_lock, flags);
	val = readl(ldev->mmio + offset);
	spin_unlock_irqrestore(&ldev->mmio_lock, flags);

	return val;
}

void ls_mm_wreg_locked(struct loongson_device *ldev, u32 offset, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&ldev->mmio_lock, flags);
	writel(val, ldev->mmio + offset);
	spin_unlock_irqrestore(&ldev->mmio_lock, flags);
}

u32 ls_io_rreg_locked(struct loongson_device *ldev, u32 offset)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&ldev->mmio_lock, flags);
	val = readl(ldev->io + offset);
	spin_unlock_irqrestore(&ldev->mmio_lock, flags);

	return val;
}

void ls_io_wreg_lcoked(struct loongson_device *ldev, u64 offset, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&ldev->mmio_lock, flags);
	writeq(val, ldev->io + offset);
	spin_unlock_irqrestore(&ldev->mmio_lock, flags);
}

/**
 * loongson_user_framebuffer_destroy -- release framebuffer, clean up framebuffer resource
 *
 * @fb pointer to drm_framebuffer
 */
static void loongson_user_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct loongson_framebuffer *loongson_fb = to_loongson_framebuffer(fb);
	if (loongson_fb->obj)
		drm_gem_object_unreference_unlocked(loongson_fb->obj);
	drm_framebuffer_cleanup(fb);
	kfree(fb);
}

/**
 * loongson_fb_funcs --framebuffer hooks sturcture
 */
static const struct drm_framebuffer_funcs loongson_fb_funcs = {
	.destroy = loongson_user_framebuffer_destroy,
};

/**
 *  loongson_framebuffer_init ---registers the framebuffer and
 *   makes it accessible to other threads
 *
 *  @dev      pointer to drm_device structure
 * @lfb      pointer to loongson_framebuffer structure
 * @mode_cmd drm mode framebuffer command
 * @obj      pointer to drm_gem_object structure
 *
 * RETURN
 *  init result
 */
int loongson_framebuffer_init(struct drm_device *dev,
			      struct loongson_framebuffer *lfb,
			      const struct drm_mode_fb_cmd2 *mode_cmd,
			      struct drm_gem_object *obj)
{
	int ret;

	drm_helper_mode_fill_fb_struct(dev, &lfb->base, mode_cmd);
	lfb->obj = obj;
	ret = drm_framebuffer_init(dev, &lfb->base, &loongson_fb_funcs);
	if (ret) {
		DRM_ERROR("drm_framebuffer_init failed: %d\n", ret);
		return ret;
	}
	return 0;
}

/**
 * loongson_user_framebuffer_create
 *
 * @dev       the pointer to drm_device structure
 * @filp      the pointer to drm_file structure
 * @mode_cmd  drm mode framebuffer cmd structure
 *
 * RETURN
 * A new framebuffer with an initial reference count of 1 or a negative
 * error code encoded with ERR_PTR()
 */
static struct drm_framebuffer *
loongson_user_framebuffer_create(struct drm_device *dev, struct drm_file *filp,
				 const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *obj;
	struct loongson_framebuffer *loongson_fb;
	int ret;

	obj = drm_gem_object_lookup(filp, mode_cmd->handles[0]);
	if (obj == NULL)
		return ERR_PTR(-ENOENT);

	loongson_fb = kzalloc(sizeof(*loongson_fb), GFP_KERNEL);
	if (!loongson_fb) {
		drm_gem_object_unreference_unlocked(obj);
		return ERR_PTR(-ENOMEM);
	}

	ret = loongson_framebuffer_init(dev, loongson_fb, mode_cmd, obj);
	if (ret) {
		drm_gem_object_unreference_unlocked(obj);
		kfree(loongson_fb);
		return ERR_PTR(ret);
	}
	return &loongson_fb->base;
}

static void loongson_output_poll_changed(struct drm_device *dev)
{
	struct loongson_device *ldev = dev->dev_private;
	loongson_fb_output_poll_changed(ldev);
}

/**
 * loongson_mode_funcs---basic driver provided mode setting functions
 *
 * Some global (i.e. not per-CRTC, connector, etc) mode setting functions that
 * involve drivers.
 */
static const struct drm_mode_config_funcs loongson_mode_funcs = {
	.fb_create = loongson_user_framebuffer_create,
	.output_poll_changed = loongson_output_poll_changed,
};

/**
 * loongson_mc_init --Map the framebuffer from the card and configure the core
 *
 * @ldev pointer to loongson_device
 *
 * RETURN
 *  vram init result
 */
static int loongson_mc_init(struct loongson_device *ldev)
{
	u64 vram_base;
	u64 vram_size;
	int ret = 0;

	struct apertures_struct *aper = alloc_apertures(1);
	if (!aper)
		return -ENOMEM;

	switch (ldev->chip) {
	case dc_7a1000:
	case dc_7a2000:
		/* BAR 2 is VRAM in 7A-gpu*/
		ldev->mc.aper_base = pci_resource_start(ldev->gpu_pdev, 2);
		ldev->mc.aper_size = pci_resource_len(ldev->gpu_pdev, 2);
		break;
	case dc_2k2000:
		ret = device_property_read_u64(&ldev->gpu_pdev->dev,
					       "vram-base", &vram_base);
		if (ret)
			DRM_ERROR("2k2000 acpi get vmem base error!\n");
		ret = device_property_read_u64(&ldev->gpu_pdev->dev,
					       "vram-size", &vram_size);
		if (ret)
			DRM_ERROR("2k2000 acpi get vmem size error!\n");

		ldev->mc.aper_base = vram_base;
		ldev->mc.aper_size = vram_size;
		DRM_DEBUG_DRIVER("2k2000 mc init address 0x%llx size 0x%llx\n",
				 vram_base, vram_size);
		break;
	default:
		break;
	}

	/* ls7a1000 only support 40 bit physicl address */
	ldev->mc.mc_mask = 0xFFFFFFFFFFULL; /* 40 bit MC */

	/* Setup GPU vram space */
	ldev->mc.vram_start = ldev->mc.aper_base;
	ldev->mc.vram_size = ldev->mc.aper_size;
	ldev->mc.vram_end = ldev->mc.vram_start + ldev->mc.vram_size - 1;

	/* Setup GPU GTT space */
	ldev->mc.gtt_start = ldev->mc.vram_end + 1;
	ldev->mc.gtt_size = 0x10000000;
	ldev->mc.gtt_end = ldev->mc.gtt_start + ldev->mc.gtt_size - 1;

	aper->ranges[0].base = 0;
	aper->ranges[0].size = ~0;

	drm_fb_helper_remove_conflicting_framebuffers(aper, "loongsondrmfb", true);
	kfree(aper);

	return 0;
}

static int plat_loongson_device_init(struct drm_device *dev, uint32_t flags)
{
	struct loongson_device *ldev = dev->dev_private;
	struct resource *res;
	struct resource res2;
	struct platform_device *pdev;
	struct device_node *io_node;
	struct page *vram_mem;
	resource_size_t mmio_base;
	resource_size_t mmio_size;
	u32 addr;

	ldev->gpu_pdev = NULL;
	pdev = to_platform_device(dev->dev);

	/* DC IO */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		DRM_ERROR("Unable to get DC device memory resources \n");
		return -ENOMEM;
	}
	mmio_base = res->start;
	mmio_size = resource_size(res);

	ldev->mmio = devm_ioremap(dev->dev, mmio_base, mmio_size);
	if (!ldev->mmio) {
		DRM_ERROR("Cannot map mmio region\n");
		return -ENOMEM;
	}

	if (!devm_request_mem_region(ldev->dev->dev, mmio_base,
				mmio_size, "loongson_mmio")) {
		DRM_ERROR("Can't reserve mmio registers\n");
		return -ENOMEM;
	}

	vram_mem = alloc_pages(GFP_KERNEL |  __GFP_NORETRY, 11);
	if (!vram_mem) {
		DRM_ERROR("Failed request memory ! \n");
		return -ENOMEM;
	}

	addr = page_to_phys(vram_mem);
	ldev->mc.vram_start = addr;
	ldev->mc.vram_size = VRAM_SIZE;

	/*CHIP IO*/
	io_node = of_find_compatible_node(NULL, NULL, "loongson,ls2k-dc_io");
	if (!io_node)
		DRM_ERROR("GET deivce tree io node  error! \n");

	of_address_to_resource(io_node, 0, &res2);
	ldev->io = devm_ioremap(dev->dev, res2.start, 0xf);

	DRM_INFO("DC mmio_base 0x%llx mmio_size 0x%llx io 0x%x\n",
			mmio_base, mmio_size, *(int *)ldev->io);

	DRM_INFO("VRAM vram_start 0x%llx vram_size 0x%llx\n",
		ldev->mc.vram_start, ldev->mc.vram_size);

	if (!loongson_vbios_init(ldev)) {
		DRM_ERROR("Get vbios failed!\n");
		return -ENOMEM;
	}

	ldev->irq.irq_num = platform_get_irq(to_platform_device(ldev->dev->dev), 0);

	return 0;
}

static int pci_loongson_device_init(struct drm_device *dev, uint32_t flags)
{
	struct loongson_device *ldev = dev->dev_private;
	int ret;

	/* pci get dc revision */
	pci_read_config_byte(ldev->dev->pdev, 0x8, &ldev->chip_revision);
	DRM_DEBUG_DRIVER("7A DC revision %d\n", ldev->chip_revision);

	/* We need get 7A-gpu pci device information for ldev->gpu_pdev */
	/* dev->pdev save 7A-dc pci device information */
	switch (ldev->chip) {
	case dc_7a1000:
		ldev->gpu_pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON,
					PCI_DEVICE_ID_LOONGSON_GPU, NULL);
		break;
	case dc_7a2000:
	case dc_2k2000:
		ldev->gpu_pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON,
						PCI_DEVICE_ID_GSGPU, NULL);
		break;
	default:
		return -ENODEV;
	}

	ret = pci_enable_device(ldev->gpu_pdev);
	if (ret)
		return ret;
	pci_set_drvdata(ldev->gpu_pdev, dev);

	ldev->fb_vram_base = 0x0;

	/*BAR 0 contains registers */
	ldev->mmio_base = pci_resource_start(ldev->dev->pdev, 0);
	ldev->mmio_size = pci_resource_len(ldev->dev->pdev, 0);

	if (!devm_request_mem_region(ldev->dev->dev, ldev->mmio_base,
				     ldev->mmio_size, "loongsonfb_mmio")) {
		DRM_ERROR("can't reserve mmio registers\n");
		return -ENOMEM;
	}

	/* Mem */
	ldev->mmio = pcim_iomap(dev->pdev, 0, 0);
	if (ldev->mmio == NULL)
		return -ENOMEM;

	/* IO */
	ldev->io = ioremap(LS7A_CHIPCFG_REG_BASE, 0xf);
	if (ldev->io == NULL)
		return -ENOMEM;

	DRM_INFO("io: 0x%lx, mmio: 0x%llx, size: 0x%llx\n",
		 (unsigned long)ldev->io, ldev->mmio_base, ldev->mmio_size);

	ret = loongson_dc_gpio_init(ldev);
	if (ret) {
		DRM_ERROR("Failed to initialize dc gpios\n");
		return ret;
	}

	ret = loongson_mc_init(ldev);
	if (ret)
		return ret;
	DRM_INFO("vram start: 0x%llx, size: %lldM\n",
		 ldev->mc.vram_start, ldev->mc.vram_size/1024/1024);

	if (!loongson_vbios_init(ldev)) {
		DRM_ERROR("Get vbios failed!\n");
		return -ENOMEM;
	}

	ldev->irq.irq_num = ldev->dev->pdev->irq;
	return 0;
}

/**
 * loongson_gem_create  -- allocate GEM object
 *
 * @dev: pointer to drm_device structure
 * @size: Requested size of buffer object.
 * @iskernel
 * @obj: address of pointer to drm_gem_object
 *
 * RETURN
 *   the result of alloc gem object
 */
int loongson_gem_create(struct drm_device *dev, u32 size, bool iskernel,
			struct drm_gem_object **obj)
{
	struct loongson_bo *astbo;
	int ret;

	*obj = NULL;

	size = roundup(size, PAGE_SIZE);
	if (size == 0)
		return -EINVAL;

	ret = loongson_bo_create(dev, size, 0, 0, &astbo);
	if (ret) {
		if (ret != -ERESTARTSYS)
			DRM_ERROR("Failed to allocate GEM object\n");
		return ret;
	}
	*obj = &astbo->gem;
	return 0;
}

int loongson_general_kms_init(struct loongson_device *ldev)
{
	struct loongson_config *dc_config  = ldev->dc_config;

	switch (ldev->chip) {
	case dc_7a1000:
	case dc_2k0500:
		dc_config->mode_filter = loongson_7a1000_mode_filter;
		dc_config->detect_config = loongson_7a1000_detect_config;
		dc_config->show_cursor = loongson_show_cursor;
		dc_config->hide_cursor = loongson_hide_cursor;
		dc_config->cursor_local = loongson_crtc_cursor_set_local;
		dc_config->cursor_hot_spot = loongson_cursor_set_hot_spot;
		break;
	case dc_7a2000:
		dc_config->mode_filter = loongson_7a2000_mode_filter;
		dc_config->detect_config = loongson_7a2000_detect_config;
		dc_config->show_cursor = loongson_7a2000_show_cursor;
		dc_config->hide_cursor = loongson_7a2000_hide_cursor;
		dc_config->cursor_local = loongson_7a2000_crtc_cursor_set_local;
		dc_config->cursor_hot_spot = loongson_7a2000_cursor_set_hot_spot;
		break;
	case dc_2k2000:
		dc_config->mode_filter = loongson_2k2000_mode_filter;
		dc_config->detect_config = loongson_2k2000_detect_config;
		dc_config->show_cursor = loongson_7a2000_show_cursor;
		dc_config->hide_cursor = loongson_7a2000_hide_cursor;
		dc_config->cursor_local = loongson_7a2000_crtc_cursor_set_local;
		dc_config->cursor_hot_spot =
					loongson_7a2000_cursor_set_hot_spot;
		break;
	}

	return 0;
}

int get_gpu_resource_from_vbios(struct loongson_device *ldev)
{
	struct pci_dev *pdev = NULL;
	char *vram_type[] = {"DDR3", "DDR4", "DDR5"};
	u32 cap;

	cap = get_cap(ldev);
	if (cap) {
		pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_GSGPU_DC, NULL);
		if (pdev) {
			dev_info(&pdev->dev, "VRAM: %dM %s %dbit %dMhz.\n", cap,
				vram_type[get_vram_type(ldev)], get_bit_width(ldev), get_freq(ldev));
			dev_info(&pdev->dev, "GSGPU: shaders_num: %d, shaders_freq: %d, freq_count: %d.\n",
				get_shaders_num(ldev), get_shaders_freq(ldev), get_count_freq(ldev));
		}
	} else
		DRM_WARN("The video memory and gpu information is not obtained from the vbios! \n");

	return 0;
}

/**
 * loongson_modeset_init --- init kernel mode setting
 *
 * @ldev: pointer to loongson_device structure
 *
 * RETURN
 *  return init result
 */
static int loongson_modeset_init(struct loongson_device *ldev)
{
	struct loongson_encoder *ls_encoder;
	struct loongson_crtc *ls_crtc;
	struct pci_dev *pdev = NULL;
	int ret, i;

	ldev->dev->mode_config.max_width = LOONGSON_MAX_FB_WIDTH;
	ldev->dev->mode_config.max_height = LOONGSON_MAX_FB_HEIGHT;

	ldev->dev->mode_config.cursor_width = 32;
	ldev->dev->mode_config.cursor_height = 32;

	ldev->dev->mode_config.fb_base = ldev->mc.vram_start;
	ldev->dc_config = kzalloc(sizeof(struct loongson_config), GFP_KERNEL);
	if (!ldev->dc_config) {
		DRM_ERROR("Failed to alloc config memory\n");
		return ret;
	}

	loongson_general_kms_init(ldev);
	ret = loongson_i2c_init(ldev);
	if (ret) {
		DRM_ERROR("Failed to initialize dc i2c\n");
		return ret;
	}

	for (i = 0; (i < ldev->num_crtc && i < LS_MAX_MODE_INFO); i++) {
		ls_crtc = loongson_crtc_init(ldev, i);
		ldev->mode_info[i].crtc = ls_crtc;

		ls_encoder = loongson_encoder_init(ldev, ls_crtc->encoder_id);
		if (ls_encoder == NULL)
			continue;

		ldev->mode_info[i].encoder = ls_encoder;
		ret = loongson_bridge_bind(ldev, i);
		if (ret) {
			DRM_ERROR("loongson bridge bind error\n");
			return ret;
		}
	}

	if (ldev->chip == dc_7a2000)
		get_gpu_resource_from_vbios(ldev);
	else if (ldev->chip == dc_7a1000) {
		pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_DC, NULL);
		if (pdev)
			dev_info(&pdev->dev, "VRAM: 128M\n");
	}

	DRM_INFO("loongson modeset init success\n");

	return 0;
}

/**
 * loongson_modeset_fini --- deinit kernel mode setting
 *
 * @ldev: pointer to loongson_device structure
 *
 * RETURN
 */
void loongson_modeset_fini(struct loongson_device *ldev)
{
}

/*
 * Userspace get information ioctl
 */
/**
 *ioctl_get_fb_vram_base - answer a device specific request.
 *
 * @rdev: drm device pointer
 * @data: request object
 * @filp: drm filp
 *
 * This function is used to pass device specific parameters to the userspace drivers.
 * Returns 0 on success, -EINVAL on failure.
 */
static int ioctl_get_fb_vram_base(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	struct loongson_device *ldev = dev->dev_private;
	struct drm_loongson_param *args = data;

	args->value = ldev->fb_vram_base;

	return 0;
}

/**
 *ioctl_get_bo_vram_base - answer a device specific request.
 *
 * @rdev: drm device pointer
 * @data: request object
 * @filp: drm filp
 *
 * This function is used to pass device specific parameters to the userspace drivers.
 * Returns 0 on success, -EINVAL on failure.
 */
static int ioctl_get_bo_vram_base(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	struct loongson_device *ldev = dev->dev_private;
	struct drm_loongson_param *args = data;
	struct drm_gem_object *obj;
	struct loongson_bo *bo;
	int ret;
	u64 gpu_addr;

	obj = drm_gem_object_lookup(file_priv, args->value);
	if (obj == NULL)
		return -ENOENT;
	bo = gem_to_loongson_bo(obj);
	ret = loongson_bo_reserve(bo, false);
	if (ret)
		return ret;

	ret = loongson_bo_pin(bo, TTM_PL_FLAG_VRAM, &gpu_addr);
	if (ret) {
		loongson_bo_unreserve(bo);
		return ret;
	}

	loongson_bo_unreserve(bo);
	ldev->fb_vram_base = gpu_addr;
	args->value = gpu_addr;
	DRM_DEBUG("loongson_get_bo_vram_base bo=%p, fb_vram_base=%llx\n", bo,
		  gpu_addr);
	return 0;
}

static struct drm_ioctl_desc loongson_ioctls_kms[DRM_COMMAND_END -
						 DRM_COMMAND_BASE] = {
	DRM_IOCTL_DEF_DRV(LOONGSON_GET_FB_VRAM_BASE, ioctl_get_fb_vram_base,
			  DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(LOONGSON_GET_BO_VRAM_BASE, ioctl_get_bo_vram_base,
			  DRM_UNLOCKED | DRM_AUTH),
};

#define DRM_LOONGSON_KMS_MAX_IOCTLS 2


/**
 * loongson_load_kms - setup chip and create an initial config
 * @dev: DRM device
 * @flags: startup flags
 *
 * The driver load routine has to do several things:
 *   - initialize the memory manager
 *   - allocate initial config memory
 *   - setup the DRM framebuffer with the allocated memory
 */
static int loongson_load_kms(struct drm_device *dev, unsigned long flags)
{
	struct loongson_device *ldev;
	struct pci_dev *pdev = NULL;
	int ret;

	ldev = devm_kzalloc(dev->dev, sizeof(struct loongson_device),
			    GFP_KERNEL);
	if (!ldev)
		return -ENOMEM;

	dev->dev_private = (void *)ldev;
	ldev->dev = dev;
	spin_lock_init(&ldev->mmio_lock);

	pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_DC, NULL);
	if (pdev) {
		ldev->chip = dc_7a1000;
		DRM_INFO("Set 7A1000 DC device\n");
	} else if ((pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_GSGPU_DC, NULL))) {
		pci_read_config_byte(ldev->dev->pdev, 0x8,
				     &ldev->chip_revision);
		if (ldev->chip_revision < 0x10) {
			ldev->chip = dc_7a2000;
			DRM_INFO("Set 7A2000 DC device\n");
		} else if (ldev->chip_revision == 0x10) {
			ldev->chip = dc_2k2000;
			DRM_INFO("Set 2K2000 DC device\n");
		}
	} else {
		ldev->chip = dc_2k0500;
		DRM_INFO("Set 2K0500 DC device\n");
	}

	ldev->enable_gamma = loongson_gamma > 0 ? true : false;

	if (pdev)
		ret = pci_loongson_device_init(dev, flags);
	else
		ret = plat_loongson_device_init(dev, flags);

	if (ret)
		goto err;

	ret = loongson_ttm_init(ldev);
	if (ret) {
		drm_err("Error initializing TTM %d\n", ret);
		goto err;
	}

	drm_mode_config_init(dev);
	dev->mode_config.funcs = (void *)&loongson_mode_funcs;
	dev->mode_config.preferred_depth = 24;
	dev->mode_config.prefer_shadow = 1;

	ret = loongson_modeset_init(ldev);
	if (ret) {
		drm_err("Fatal error during modeset init: %d\n", ret);
		goto err;
	}

	drm_kms_helper_poll_init(dev);

	/*Enable IRQ*/
	loongson_hpd_init_work(ldev);
	ret = loongson_irq_init(ldev);
	if (ret) {
		drm_err("Fatal error during irq init: %d\n", ret);
		goto err;
	}

	if (ldev->chip == dc_7a2000 || ldev->chip == dc_2k2000)
		loongson_hotplug_config(ldev);

	ret = loongson_fbdev_init(ldev);
	if (ret) {
		DRM_ERROR("loongson_fbdev_init failed\n");
		return ret;
	}
	ldev->inited = true;

	return 0;

err:
	drm_err("failed to initialize drm driver: %d\n", ret);
	kfree(ldev);
	return ret;
}

static void loongson_hotplug_work_func(struct work_struct *work)
{
	bool changed = false;
	enum drm_connector_status old_status;
	struct loongson_device *ldev =
		container_of(work, struct loongson_device, hotplug_work);
	struct drm_device *dev = ldev->dev;
	struct drm_connector_list_iter conn_iter;
	struct drm_connector *connector;

	mutex_lock(&dev->mode_config.mutex);
	drm_connector_list_iter_begin(dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		struct loongson_connector *ls_connector = to_loongson_connector(connector);

		old_status = ls_connector->base.status;
		ls_connector->base.status =
			drm_helper_probe_detect(&ls_connector->base, NULL, false);
		if (old_status != ls_connector->base.status)
			changed = true;
	}
	drm_connector_list_iter_end(&conn_iter);
	mutex_unlock(&dev->mode_config.mutex);

	if (changed)
		drm_kms_helper_hotplug_event(dev);

}

void loongson_hpd_init_work(struct loongson_device *ldev)
{
	INIT_WORK(&ldev->hotplug_work, loongson_hotplug_work_func);
}

/**
 * loongson_unload_kms--release drm resource
 *
 * @dev: pointer to drm_device
 *
 * RETURN
 *  release success or fail
 */
static void loongson_unload_kms(struct drm_device *dev)
{
	struct loongson_device *ldev = dev->dev_private;

	if (ldev == NULL)
		return;

	loongson_modeset_fini(ldev);
	loongson_fbdev_fini(ldev);
	drm_mode_config_cleanup(dev);
	loongson_ttm_fini(ldev);
	dev->dev_private = NULL;
	ldev->inited = false;
}


/**
 * loongson's display controller require the pitch be a multiple
 * of 256 bytes, which is for optimize for dma data transfer
 * @param width  buffer width
 * @param bpp  bits per pixel
 * @return int  calculated pitch
 */
static int loongson_compute_aligned_pitch(int width, int bpp)
{
	int pitch = width * ((bpp + 7) / 8);

	pitch = (pitch + 255) & ~255;

	return pitch;
}


/**
 *  loongson_dumb_create --dump alloc support
 *
 * @file: pointer to drm_file structure,DRM file private date
 * @dev:  pointer to drm_device structure
 * @args: a dumb scanout buffer param
 *
 * RETURN
 *  dum alloc result
 */
int loongson_dumb_create(struct drm_file *file, struct drm_device *dev,
			 struct drm_mode_create_dumb *args)
{
	struct drm_gem_object *gobj;
	u32 handle;
	int ret;

	args->pitch = loongson_compute_aligned_pitch(args->width, args->bpp);
	args->size = args->pitch * args->height;

	ret = loongson_gem_create(dev, args->size, false, &gobj);
	if (ret)
		return ret;

	ret = drm_gem_handle_create(file, gobj, &handle);
	drm_gem_object_unreference_unlocked(gobj);
	if (ret)
		return ret;

	args->handle = handle;

	return 0;
}

/**
 * loongson_bo_unref -- reduce ttm object buffer refer
 *
 * @bo: the pointer to loongson ttm buffer object
 */
void loongson_bo_unref(struct loongson_bo **bo)
{
	struct ttm_buffer_object *tbo;

	if ((*bo) == NULL)
		return;

	tbo = &((*bo)->bo);
	ttm_bo_unref(&tbo);
	*bo = NULL;
}

/**
 * loongson_gem_free_object --free gem object
 *
 * @obj: the pointer to drm_gem_object
 */
void loongson_gem_free_object(struct drm_gem_object *obj)
{
	struct loongson_bo *loongson_bo = gem_to_loongson_bo(obj);

	loongson_bo_unref(&loongson_bo);
}

/**
 * loongson_bo_mmap_offset -- Return sanitized offset for user-space mmaps
 *
 * @bo: the pointer to loongson ttm buffer object
 *
 * RETURN
 *  Offset of @node for byte-based addressing
 * 0 if the node does not have an object allocatedS
 */
static inline u64 loongson_bo_mmap_offset(struct loongson_bo *bo)
{
	return drm_vma_node_offset_addr(&bo->bo.vma_node);
}

/**
 * loongson_dumb_mmap_offset --return sanitized offset for userspace mmaps
 *
 * @file: the pointer to drm_file structure,DRM file private date
 * @dev: the pointer to drm_device structrure
 * @handle: user space handle
 * @offset: return value pointer
 *
 * RETRUN
 *  return sainitized offset
 */
int loongson_dumb_mmap_offset(struct drm_file *file, struct drm_device *dev,
			      uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	struct loongson_bo *bo;

	obj = drm_gem_object_lookup(file, handle);
	if (obj == NULL)
		return -ENOENT;

	bo = gem_to_loongson_bo(obj);
	*offset = loongson_bo_mmap_offset(bo);

	drm_gem_object_unreference_unlocked(obj);
	return 0;
}

/**
 * loongson_open_kms -Driver callback when a new struct drm_file is opened.
 * Useful for setting up driver-private data structures like buffer allocators,
 *  execution contexts or similar things.
 *
 * @dev DRM device
 * @file DRM file private date
 *
 * RETURN
 * 0 on success, a negative error code on failure, which will be promoted to
 *  userspace as the result of the open() system call.
 */
static int loongson_open_kms(struct drm_device *dev, struct drm_file *file)
{
	file->driver_priv = NULL;

	DRM_DEBUG("open: dev=%p, file=%p", dev, file);

	return 0;
}

/**
 * loongson_drm_driver_fops - file operations structure
 *
 * .open: file open
 * .release : file close
 * .unlocked_ioctl:
 * .mmap: map device memory to process address space
 * .poll: device status: POLLIN POLLOUT POLLPRI
 * .compat_ioctl: used in 64 bit
 * .read: sync and read data from device
 */
static const struct file_operations loongson_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = loongson_drm_mmap,
	.poll = drm_poll,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.read = drm_read,
};

/**
 * loongson_kms_driver - DRM device structure
 *
 * .load: driver callback to complete initialization steps after the driver is registered
 * .unload:Reverse the effects of the driver load callback
 * .open:Driver callback when a new struct drm_file is opened
 * .fops:File operations for the DRM device node.
 * .gem_free_object:deconstructor for drm_gem_objects
 * .dumb_create:This creates a new dumb buffer in the driver’s backing storage manager
 *  (GEM, TTM or something else entirely) and returns the resulting buffer handle.
 *  This handle can then be wrapped up into a framebuffer modeset object
 * .dumb_map_offset:Allocate an offset in the drm device node’s address space
 *  to be able to memory map a dumb buffer
 * .dump_destory:This destroys the userspace handle for the given dumb backing storage buffer
 */
static struct drm_driver loongson_kms_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME | DRIVER_HAVE_IRQ,
	.load = loongson_load_kms,
	.unload = loongson_unload_kms,
	.open = loongson_open_kms,
	.fops = &loongson_drm_driver_fops,
	.gem_free_object = loongson_gem_free_object,
	.dumb_create = loongson_dumb_create,
	.dumb_map_offset = loongson_dumb_mmap_offset,

	.ioctls = loongson_ioctls_kms,
	.num_ioctls = DRM_LOONGSON_KMS_MAX_IOCTLS,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
	/*vblank*/
	.enable_vblank = loongson_irq_enable_vblank,
	.disable_vblank = loongson_irq_disable_vblank,
	.get_vblank_counter = loongson_crtc_vblank_count,
	/*IRQ*/
	.irq_preinstall = loongson_irq_preinstall,
	.irq_postinstall = loongson_irq_postinstall,
	.irq_uninstall = loongson_irq_uninstall,
	.irq_handler = loongson_irq_handler,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,

	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_export = drm_gem_prime_export,

	.gem_prime_get_sg_table = loongson_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = loongson_gem_prime_import_sg_table,
	.gem_prime_vmap = loongson_gem_prime_vmap,
	.gem_prime_vunmap = loongson_gem_prime_vunmap,
	.gem_prime_pin = loongson_gem_prime_pin,
	.gem_prime_unpin = loongson_gem_prime_unpin,
	.gem_prime_res_obj = loongson_gem_prime_res_obj,
	.gem_prime_mmap = loongson_gem_prime_mmap,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init = loongson_debugfs_init,
#endif
};

/**
 * pciidlist  -- loongson pci device id
 *
 * __u32 vendor, device            Vendor and device ID or PCI_ANY_ID
 * __u32 subvendor, subdevice     Subsystem ID's or PCI_ANY_ID
 * __u32 class, class_mask        (class,subclass,prog-if) triplet
 * kernel_ulong_t driver_data     Data private to the driver
 */
static struct pci_device_id pciidlist[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_DC) },
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_GSGPU_DC) },
	{ 0, 0, 0, 0, 0, 0, 0 }
};

MODULE_DEVICE_TABLE(pci, pciidlist);

/**
 * loongson_pci_probe -- add pci device
 *
 * @pdev PCI device
 * @ent pci device id
 */
static int loongson_pci_probe(struct pci_dev *pdev,
			      const struct pci_device_id *ent)
{
	if (ent->device == PCI_DEVICE_ID_GSGPU_DC && !loongson_lg100_support)
		return -ENODEV;
	else
		return drm_get_pci_dev(pdev, ent, &loongson_kms_driver);
}

/**
 * loongson_pci_remove -- release drm device
 *
 * @pdev PCI device
 */
static void loongson_pci_remove(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	drm_put_dev(dev);
}

/*
 * loongson_drm_suspend - initiate device suspend *
 *
 * @pdev: drm dev pointer
 * @state: suspend state
 *
 * Puts the hw in the suspend state (all asics).
 * Returns 0 for success or an error on failure.
 * Called at driver suspend.
 */
int loongson_drm_suspend(struct drm_device *dev)
{
	struct loongson_bo *lbo;
	struct drm_framebuffer *drm_fb;
	struct loongson_framebuffer *lfb;
	struct loongson_device *ldev = dev->dev_private;

	if (dev->switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	if (ldev->chip == dc_7a1000)
		loongson_bridge_suspend(ldev);

	drm_kms_helper_poll_disable(dev);
	pci_save_state(dev->pdev);

	loongson_fbdev_set_suspend(ldev, 1);

	mutex_lock(&dev->mode_config.fb_lock);
	drm_for_each_fb (drm_fb, dev) {
		lfb = to_loongson_framebuffer(drm_fb);
		lbo = gem_to_loongson_bo(lfb->obj);
		if (!lbo->pin_count)
			continue;
		/* don't unpin kernel fb objects */
		if (!loongson_fbdev_lobj_is_fb(ldev, lbo)) {
			loongson_bo_reserve(lbo, true);
			loongson_bo_unpin(lbo);
			loongson_bo_unreserve(lbo);
		}
	}
	mutex_unlock(&dev->mode_config.fb_lock);

	/* evict vram memory. copy fb from vram to ttm pages*/
	ttm_bo_evict_mm(&ldev->ttm.bdev, TTM_PL_VRAM);

	return 0;
}

/*
 *  * loongson_drm_resume - initiate device suspend
 *
 * @pdev: drm dev pointer
 * @state: suspend state
 *
 * Puts the hw in the suspend state (all asics).
 * Returns 0 for success or an error on failure.
 * Called at driver suspend.
 */
int loongson_drm_resume(struct drm_device *dev)
{
	struct loongson_bo *lbo;
	struct drm_framebuffer *drm_fb;
	struct loongson_framebuffer *lfb;
	struct loongson_device *ldev = dev->dev_private;
	u64 gpu_addr;
	int index;

	if (dev->switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	if (pci_enable_device(dev->pdev))
		return -EIO;

	mutex_lock(&dev->mode_config.fb_lock);
	drm_for_each_fb (drm_fb, dev) {
		lfb = to_loongson_framebuffer(drm_fb);
		lbo = gem_to_loongson_bo(lfb->obj);
		if (!loongson_fbdev_lobj_is_fb(ldev, lbo)) {
			loongson_bo_reserve(lbo, true);
			loongson_bo_pin(lbo, TTM_PL_FLAG_VRAM, &gpu_addr);
			loongson_bo_unreserve(lbo);
		}
	}
	mutex_unlock(&dev->mode_config.fb_lock);

	switch (ldev->chip) {
	case dc_7a1000:
		loongson_bridge_resume(ldev);
		break;
	case dc_7a2000:
	case dc_2k2000:
		loongson_hotplug_config(ldev);
		for (index = 0; index < 2; index++)
			loongson_hdmi_init(ldev, index);
		break;
	default:
		break;
	}

	drm_mode_config_reset(dev);
	drm_helper_resume_force_mode(dev);

	loongson_fbdev_set_suspend(ldev, 0);
	loongson_connector_resume(ldev);
	drm_kms_helper_poll_enable(dev);

	return 0;
}

/**
 * loongson_pmops_suspend
 *
 * @dev   pointer to the device
 *
 * Executed before putting the system into a sleep state in which the
 * contents of main memory are preserved.
 */
static int loongson_pmops_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);
	int ret;

	ret = loongson_drm_suspend(drm_dev);
	if (ret)
		return ret;

	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
	return 0;
}

/**
 * loongson_pmops_resume
 *
 * @dev pointer to the device
 *
 * Executed after waking the system up from a sleep state in which the
 * contents of main memory were preserved.
 */
static int loongson_pmops_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	return loongson_drm_resume(drm_dev);
}

/**
 * loongson_pmops_freeze
 *
 * @dev pointer to the device
 *
 * Executed after waking the system up from a freezz state in which the
 * contents of main memory were preserved.
 */
static int loongson_pmops_freeze(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	loongson_drm_suspend(drm_dev);

	return 0;
}
static int loongson_pmops_restore_early(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	/*diabsle Dipaly controllor output*/
	return drm_crtc_force_disable_all(drm_dev);
}
/*
 * * struct dev_pm_ops - device PM callbacks
 *
 *@suspend:  Executed before putting the system into a sleep state in which the
 *           contents of main memory are preserved.
 *@resume:   Executed after waking the system up from a sleep state in which the
 *           contents of main memory were preserved.
 *@freeze:   Hibernation-specific, executed before creating a hibernation image.
 *           Analogous to @suspend(), but it should not enable the device to signal
 *           wakeup events or change its power state.  The majority of subsystems
 *           (with the notable exception of the PCI bus type) expect the driver-level
 *           @freeze() to save the device settings in memory to be used by @restore()
 *           during the subsequent resume from hibernation.
 *@thaw:     Hibernation-specific, executed after creating a hibernation image OR
 *           if the creation of an image has failed.  Also executed after a failing
 *           attempt to restore the contents of main memory from such an image.
 *           Undo the changes made by the preceding @freeze(), so the device can be
 *           operated in the same way as immediately before the call to @freeze().
 *@poweroff: Hibernation-specific, executed after saving a hibernation image.
 *           Analogous to @suspend(), but it need not save the device's settings in
 *           memory.
 *@restore:  Hibernation-specific, executed after restoring the contents of main
 *           memory from a hibernation image, analogous to @resume().
 */
static const struct dev_pm_ops loongson_pmops = {
	.suspend = loongson_pmops_suspend,
	.resume = loongson_pmops_resume,
	.freeze = loongson_pmops_freeze,
	.poweroff = loongson_pmops_freeze,
	.restore_early = loongson_pmops_restore_early,
	.restore = loongson_pmops_resume,
};

/**
 * loongson_pci_driver -- pci driver structure
 *
 * .id_table : must be non-NULL for probe to be called
 * .probe: New device inserted
 * .remove: Device removed
 * .resume: Device suspended
 * .suspend: Device woken up
 */
static struct pci_driver loongson_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pciidlist,
	.probe = loongson_pci_probe,
	.remove = loongson_pci_remove,
	.driver.pm = &loongson_pmops,
};

static int loongson_drm_plat_register(struct platform_device *pdev)
{

	int ret;
	struct drm_device *dev;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	dev = drm_dev_alloc(&loongson_kms_driver, &pdev->dev);

	if (IS_ERR(dev)) {
		DRM_ERROR("Failed to allocate drm_device \n");
		return PTR_ERR(dev);
	}

	dev_set_drvdata(&pdev->dev, dev);

	ret = drm_dev_register(dev, 0);
	if (ret) {
		DRM_ERROR("Failed to register drv for userspace access: %d\n", ret);
		goto err_pdev;
	}

	return 0;

err_pdev:
	drm_dev_unref(dev);
	return ret;

}

static int loongson_drm_plat_unregister(struct platform_device *pdev)
{
	struct drm_device *dev = platform_get_drvdata(pdev);
	struct loongson_device *ldev = dev->dev_private;
	u64 vaddr = (unsigned long)phys_to_virt(ldev->mc.vram_start);
	free_pages(vaddr, 11);
	drm_put_dev(dev);
	return 0;
}

const static struct of_device_id loongson_drm_ids[] = {
	{ .compatible = "loongson,ls2k0500-dc",},
};

static struct platform_driver loongson_drm_plat_driver = {
	.probe		= loongson_drm_plat_register,
	.remove		= loongson_drm_plat_unregister,
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(loongson_drm_ids),
	},
};

/**
 * loongson_init()  -- kernel module init function
 */
static int __init loongson_init(void)
{
	struct pci_dev *pdev = NULL;
	/* Discrete card prefer */
	while ((pdev = pci_get_class(PCI_CLASS_DISPLAY_VGA << 8, pdev))) {
		if (pdev->vendor != PCI_VENDOR_ID_LOONGSON || pdev->device == 0x1a05)
			return 0;
	}

	DRM_INFO("loongson kernel modesetting driver enable\n");

	if (!loongson_ttm_enable) {
		DRM_INFO("loongson-drm kernel modesetting driver disabled\n");
		return -ENODEV;
	}

	pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_DC, NULL);
	if (pdev == NULL)
		pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_GSGPU_DC, NULL);

	if (pdev)
		return pci_register_driver(&loongson_pci_driver);
	else
		return platform_driver_register(&loongson_drm_plat_driver);
}

/**
 * loongson_exit()  -- kernel module exit function
 */
static void __exit loongson_exit(void)
{
	struct pci_dev *pdev = NULL;
	if (!loongson_ttm_enable)
		return;

	pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_DC, NULL);
	if (pdev == NULL)
		pdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_GSGPU_DC, NULL);

	if (pdev)
		pci_unregister_driver(&loongson_pci_driver);
	else
		platform_driver_unregister(&loongson_drm_plat_driver);
}

module_init(loongson_init);
module_exit(loongson_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
