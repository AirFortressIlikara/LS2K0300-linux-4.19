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

#include "loongson_drv.h"
#include "loongson_vbios.h"

DEFINE_SPINLOCK(loongson_crtc_lock);

/**
 * @loongson_crtc_mode_match
 *
 * @hdisplay
 * @vdisplay
 * @ls_crtc
 * @Returns   struct crtc_vbios_timing *
 */
static struct loongson_timing *
loongson_crtc_get_timing(u32 hdisplay, u32 vdisplay,
			 struct loongson_crtc *ls_crtc)
{
	int match_index = 0;
	struct crtc_timing *timing = ls_crtc->timing;
	struct loongson_timing *config = NULL, *table = NULL;

	table = timing->tables;
	if (!table) {
		DRM_INFO("crtc mode table null \n");
		return config;
	}

	while (match_index < timing->num) {
		if (vdisplay == table->vdisplay &&
		    hdisplay == table->hdisplay) {
			config = table;
			break;
		}

		match_index++;
		table++;
	}

	return config;
}

/*
 * loongson_set_start_address
 *
 * @crtc: point to a drm_crtc structure
 * @offset: framebuffer base address
 */
void loongson_set_start_address(struct drm_crtc *crtc, u64 addr)
{
	struct loongson_device *ldev;
	struct loongson_crtc *loongson_crtc;
	u32 crtc_offset;
	u32 addr_h;
	u32 addr_l;

	ldev = crtc->dev->dev_private;
	loongson_crtc = to_loongson_crtc(crtc);
	crtc_offset = loongson_crtc->crtc_offset;
	addr_h = (addr >> 32) & 0xff;
	addr_l = addr & 0xffffffff;

	DRM_DEBUG_DRIVER("crtc-%d, addr: %llx\n", crtc->index, addr);

	ls_mm_wreg(ldev, LS_FB_HADDR0_REG + crtc_offset, addr_h);
	ls_mm_wreg(ldev, LS_FB_HADDR1_REG + crtc_offset, addr_h);
	ls_mm_wreg(ldev, LS_FB_LADDR0_REG + crtc_offset, addr_l);
	ls_mm_wreg(ldev, LS_FB_LADDR1_REG + crtc_offset, addr_l);
}

/**
 * loongson_crtc_do_set_base
 *
 * @crtc: point to a drm_crtc structure
 * @fb: point to a drm_framebuffer structure
 * @x: x position on screen
 * @y: y position on screen
 * @atomic: int variable
 *
 * Ast is different - we will force move buffers out of VRAM
 */
static int loongson_crtc_do_set_base(struct drm_crtc *crtc,
				     struct drm_framebuffer *fb, int x, int y,
				     int atomic)
{
	struct loongson_device *ldev = crtc->dev->dev_private;
	struct loongson_crtc *loongson_crtc = to_loongson_crtc(crtc);
	struct drm_gem_object *obj;
	struct loongson_framebuffer *loongson_fb;
	struct loongson_bo *bo;
	u32 depth, ret;
	u64 crtc_address;
	u32 width, pitch;
	u32 cfg_reg = 0;
	u64 gpu_addr;
	u32 crtc_offset;

	crtc_offset = loongson_crtc->crtc_offset;
	ldev = crtc->dev->dev_private;
	width = crtc->primary->fb->width;
	depth = crtc->primary->fb->format->cpp[0] << 3;

	if (!atomic && fb) {
		loongson_fb = to_loongson_framebuffer(fb);
		obj = loongson_fb->obj;
		bo = gem_to_loongson_bo(obj);
		ret = loongson_bo_reserve(bo, false);
		if (ret)
			return ret;
		loongson_bo_push_sysram(bo);
		loongson_bo_unreserve(bo);
	}

	DRM_DEBUG_DRIVER("crtc width = %d,height = %d\n", width,
		  crtc->primary->fb->height);
	DRM_DEBUG_DRIVER("crtc pitches[0]=%d\n", crtc->primary->fb->pitches[0]);
	loongson_fb = to_loongson_framebuffer(crtc->primary->fb);

	obj = loongson_fb->obj;
	bo = gem_to_loongson_bo(obj);
	ret = loongson_bo_reserve(bo, false);
	if (ret)
		return ret;

	ret = loongson_bo_pin(bo, TTM_PL_FLAG_VRAM, &gpu_addr);
	if (ret) {
		loongson_bo_unreserve(bo);
		return ret;
	}

	DRM_DEBUG_DRIVER("bo addr in gpu addr space: 0x%llx\n", gpu_addr);

	ldev->fb_vram_base = gpu_addr;
	if (&ldev->lfbdev->lfb == loongson_fb) {
		/* if pushing console in kmap it */
		ret = ttm_bo_kmap(&bo->bo, 0, bo->bo.num_pages, &bo->kmap);
		if (ret)
			DRM_ERROR("failed to kmap fbcon\n");
	}
	loongson_bo_unreserve(bo);

	cfg_reg |= LS_FB_CFG_RESET | LS_FB_CFG_ENABLE;
	switch (depth) {
	case 16:
		cfg_reg |= LS_FB_CFG_FORMAT16;
		break;
	case 15:
		cfg_reg |= LS_FB_CFG_FORMAT15;
		break;
	case 12:
		cfg_reg |= LS_FB_CFG_FORMAT12;
		break;
	case 32:
	case 24:
	default:
		cfg_reg |= LS_FB_CFG_FORMAT32;
		break;
	}

	pitch = crtc->primary->fb->pitches[0];
	switch (depth) {
	case 12 ... 16:
		crtc_address = gpu_addr + y * pitch + ALIGN(x, 64) * 2;
		break;
	case 24 ... 32:
	default:
		crtc_address = gpu_addr + y * pitch + ALIGN(x, 64) * 4;
		break;
	}

	ls_mm_wreg_check(ldev, LS_FB_CFG_REG + crtc_offset, cfg_reg);
	ls_mm_wreg(ldev, LS_FB_STRI_REG + crtc_offset, pitch);
	loongson_set_start_address(crtc, crtc_address);

	return 0;
}

/**
 * config_pll
 *
 * @pll_base: represent a long type
 * @pll_cfg: point to the pix_pll srtucture
 *
 * Config pll apply to 7a
 */
static void pci_config_pll(struct loongson_device *ldev, unsigned long pll_base,
		       struct pix_pll *pll_cfg)
{
	unsigned long val;
	u32 count = 0;

	/* clear sel_pll_out0 */
	val = ls_io_rreg(ldev, pll_base + 0x4);
	val &= ~(1UL << 8);
	ls_io_wreg(ldev, pll_base + 0x4, val);
	/* set pll_pd */
	val = ls_io_rreg(ldev, pll_base + 0x4);
	val |= (1UL << 13);
	ls_io_wreg(ldev, pll_base + 0x4, val);
	/* clear set_pll_param */
	val = ls_io_rreg(ldev, pll_base + 0x4);
	val &= ~(1UL << 11);
	ls_io_wreg(ldev, pll_base + 0x4, val);
	/* clear old value & config new value */
	val = ls_io_rreg(ldev, pll_base + 0x4);
	val &= ~(0x7fUL << 0);
	val |= (pll_cfg->l1_frefc << 0); /* refc */
	ls_io_wreg(ldev, pll_base + 0x4, val);
	val = ls_io_rreg(ldev, pll_base + 0x0);
	val &= ~(0x7fUL << 0);
	val |= (pll_cfg->l2_div << 0); /* div */
	val &= ~(0x1ffUL << 21);
	val |= (pll_cfg->l1_loopc << 21); /* loopc */
	ls_io_wreg(ldev, pll_base + 0x0, val);
	/* set set_pll_param */
	val = ls_io_rreg(ldev, pll_base + 0x4);
	val |= (1UL << 11);
	ls_io_wreg(ldev, pll_base + 0x4, val);
	/* clear pll_pd */
	val = ls_io_rreg(ldev, pll_base + 0x4);
	val &= ~(1UL << 13);
	ls_io_wreg(ldev, pll_base + 0x4, val);
	/* wait pll lock */
	while (!(ls_io_rreg(ldev, pll_base + 0x4) & 0x80)) {
		cpu_relax();
		count++;
		if (count >= 1000) {
			DRM_ERROR("loongson-7A PLL lock failed\n");
			break;
		}
	}
	/* set sel_pll_out0 */
	val = ls_io_rreg(ldev, pll_base + 0x4);
	val |= (1UL << 8);
	ls_io_wreg(ldev, pll_base + 0x4, val);
}

void hdmi_phy_pll_config(struct loongson_device *ldev, int index, int clock)
{
	int val;
	int count = 0;
	int reg_offset = index * 0x10;

	if (ldev->chip == dc_2k2000 && index == 1)
		return;

	ls_mm_wreg(ldev, HDMI_PHY_PLLCFG_REG + reg_offset, 0x0);
	ls_mm_wreg(ldev, HDMI_PHY_CTRL_REG + reg_offset, 0xf02);
	while ((ls_mm_rreg(ldev, HDMI_PHY_CTRL_REG + reg_offset) & 0x1)) {
		count++;
		if (count >= 1000) {
			DRM_ERROR("LOONGSON HDMI PHY CTRL close failed\n");
			return;
		}
	}

	count = 0;
	if (clock >= 170000)
		val = (0x0 << 13) | (0x28 << 6) | (0x10 << 1) | (0 << 0);
	else if (clock >= 85000 && clock < 170000)
		val = (0x1 << 13) | (0x28 << 6) | (0x8 << 1) | (0 << 0);
	else if (clock >= 42500 && clock < 85000)
		val = (0x2 << 13) | (0x28 << 6) | (0x4 << 1) | (0 << 0);
	else if (clock >= 21250 && clock < 42500)
		val = (0x3 << 13) | (0x28 << 6) | (0x2 << 1) | (0 << 0);

	ls_mm_wreg(ldev, HDMI_PHY_PLLCFG_REG + reg_offset, val);
	val |= (1 << 0);
	ls_mm_wreg(ldev, HDMI_PHY_PLLCFG_REG + reg_offset, val);

	/* wait pll lock */
	while (!(ls_mm_rreg(ldev, HDMI_PHY_PLLCFG_REG + reg_offset) & 0x10000)) {
		count++;
		if (count >= 1000) {
			DRM_ERROR("LOONGSON HDMI PHY PLL lock failed\n");
			return;
		}
	}

	ls_mm_wreg(ldev, HDMI_PHY_CTRL_REG + reg_offset, 0xf03);
}

void plat_config_pll(struct loongson_device *ldev, u32 pll_base,
		       struct pix_pll *pll_cfg)
{
	unsigned int out;
	u32 count = 0;

	out = (pll_cfg->l2_div << 24) | ((pll_cfg->l1_loopc) << 16) |
		((pll_cfg->l1_frefc) << 8);

	writel(0, ldev->io + pll_base);
	writel(1 << 5, ldev->io + pll_base);
	writel(out, ldev->io + pll_base);
	out = (out | (1 << 3));
	writel(out, ldev->io + pll_base);

	while (!(readl(ldev->io + pll_base) & 0x80)) {
		count++;
		if (count >= 1000) {
			DRM_ERROR("loongson-2K500 PLL lock failed\n");
			break;
		}
	}

	writel((out | 1), ldev->io + pll_base);
}

/**
 * cal_freq
 *
 * @pixclock_khz: unsigned int
 * @pll_config: point to the pix_pll structure
 *
 * Calculate frequency
 */
static unsigned int cal_freq(unsigned int pixclock_khz,
			     struct pix_pll *pll_config)
{
	unsigned int pstdiv, loopc, frefc;
	unsigned long a, b, c;
	unsigned long min = 50;

	for (pstdiv = 1; pstdiv < 64; pstdiv++) {
		a = (unsigned long)pixclock_khz * pstdiv;
		for (frefc = 3; frefc < 6; frefc++) {
			for (loopc = 24; loopc < 161; loopc++) {
				if ((loopc < 12 * frefc) ||
				    (loopc > 32 * frefc))
					continue;

				b = 100000L * loopc / frefc;
				c = (a > b) ? (a * 10000 / b - 10000) :
					(b * 10000 / a - 10000);
				if (c < min) {
					min = c;
					pll_config->l2_div = pstdiv;
					pll_config->l1_loopc = loopc;
					pll_config->l1_frefc = frefc;
				}
			}
		}
	}

	if (min < 50)
		return 1;

	return 0;
}

/**
 * loongson_crtc_mode_set
 *
 * @crtc: point to the drm_crtc structure
 * @mode: represent a display mode
 * @adjusted_mode: point to the drm_display_mode structure
 * @old_fb: point to the drm_framebuffer structure
 *
 * Used by the legacy CRTC helpers to set a new mode
 */
static int loongson_crtc_mode_set(struct drm_crtc *crtc,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode, int x,
				  int y, struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	struct loongson_device *ldev = dev->dev_private;
	struct loongson_crtc *ls_crtc = to_loongson_crtc(crtc);
	struct loongson_timing *crtc_mode;
	struct pix_pll pll_cfg;
	u32 pix_val;

	u32 hr, hss, hse, hfl;
	u32 vr, vss, vse, vfl;
	u32 pix_freq;
	u32 depth;
	u32 crtc_id;
	u32 crtc_offset;
	u32 reg_val = 0;
	u32 ret;

	crtc_offset = ls_crtc->crtc_offset;
	crtc_id = ls_crtc->crtc_id;
	depth = crtc->primary->fb->format->cpp[0] << 3;
	pix_freq = mode->clock;

	hr = mode->hdisplay;
	hss = mode->hsync_start;
	hse = mode->hsync_end;
	hfl = mode->htotal;
	vr = mode->vdisplay;
	vss = mode->vsync_start;
	vse = mode->vsync_end;
	vfl = mode->vtotal;

	if (ls_crtc->is_vb_timing) {
		crtc_mode = loongson_crtc_get_timing(hr, vr, ls_crtc);
		if (!crtc_mode) {
			DRM_ERROR("match  vdisp = %d, hdisp %d no support\n",
				  vr, hr);
			return 1;
		}

		hr = crtc_mode->hdisplay;
		hss = crtc_mode->hsync_start;
		hse = crtc_mode->hsync_start + crtc_mode->hsync_width;
		hfl = crtc_mode->htotal;
		vr = crtc_mode->vdisplay;
		vss = crtc_mode->vsync_start;
		vse = crtc_mode->vsync_start + crtc_mode->vsync_width;
		vfl = crtc_mode->vtotal;
		pix_freq = crtc_mode->clock;
	}

	/* Hack to support non 64 aligned resolution, say 800x600, 1680x1050
	 * Align the hactive to 64.
	 */
	if (ldev->chip == dc_7a1000)
		hr = (hr + 63) & ~63;

	DRM_DEBUG_KMS("id = %d, depth = %d, pix_freq = %d, x = %d, y = %d\n",
		      crtc_id, depth, pix_freq, x, y);
	DRM_DEBUG_KMS("hr = %d, hss = %d, hse = %d, hfl = %d\n", hr, hss, hse,
		      hfl);
	DRM_DEBUG_KMS("vr = %d, vss = %d, vse = %d, vfl = %d\n", vr, vss, vse,
		      vfl);

	DRM_DEBUG_KMS("fb width = %d, height = %d\n", crtc->primary->fb->width,
		      crtc->primary->fb->height);

	ls_crtc->width = hr;
	ls_crtc->height = vr;
	ret = cal_freq(pix_freq, &pll_cfg);

	if (ret) {
		if (ldev->gpu_pdev)
			pci_config_pll(ldev, LS_PIX_PLL + crtc_offset, &pll_cfg);
		else {
			of_property_read_u32(ls_crtc->pix_node, "offset", &pix_val);
			plat_config_pll(ldev, pix_val, &pll_cfg);
		}
	}

	loongson_crtc_do_set_base(crtc, old_fb, x, y, 0);
	if (ldev->chip == dc_7a2000 || ldev->chip == dc_2k2000)
		hdmi_phy_pll_config(ldev, crtc_id, pix_freq);

	/* these 4 lines cause out of range, because
	 * the hfl hss vfl vss are different with PMON vgamode cfg.
	 * So the refresh freq in kernel and refresh freq in PMON are different.
	 */
	ls_mm_wreg(ldev, LS_FB_DITCFG_REG + crtc_offset, 0);
	ls_mm_wreg(ldev, LS_FB_DITTAB_LO_REG + crtc_offset, 0);
	ls_mm_wreg(ldev, LS_FB_DITTAB_HI_REG + crtc_offset, 0);

	reg_val = LS_FB_PANCFG_BASE | LS_FB_PANCFG_DE | LS_FB_PANCFG_CLKEN |
		  LS_FB_PANCFG_CLKPOL;

	ls_mm_wreg(ldev, LS_FB_PANCFG_REG + crtc_offset, reg_val);
	ls_mm_wreg(ldev, LS_FB_PANTIM_REG + crtc_offset, 0);

	ls_mm_wreg(ldev, LS_FB_HDISPLAY_REG + crtc_offset, (hfl << 16) | hr);
	ls_mm_wreg(ldev, LS_FB_HSYNC_REG + crtc_offset,
		   LS_FB_HSYNC_POLSE | (hse << 16) | hss);

	ls_mm_wreg(ldev, LS_FB_VDISPLAY_REG + crtc_offset, (vfl << 16) | vr);
	ls_mm_wreg(ldev, LS_FB_VSYNC_REG + crtc_offset,
		   LS_FB_VSYNC_POLSE | (vse << 16) | vss);

	if (ls_crtc->cursor.bo)
		loongson_crtc_cursor_move(crtc, ls_crtc->cursor.x, ls_crtc->cursor.y);

	return 0;
}

/**
 * loongson_crtc_dpms
 *
 * @crtc: point to the drm_crtc structure
 * @mode: represent mode
 *
 * According to mode,represent the power levels on the CRTC
 */
static void loongson_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct drm_device *dev = crtc->dev;
	struct loongson_device *ldev = dev->dev_private;
	struct loongson_crtc *loongson_crtc = to_loongson_crtc(crtc);
	u32 val;
	u32 crtc_offset;

	crtc_offset = loongson_crtc->crtc_offset;

	if (ldev->inited == false) {
		return;
	}

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		val = ls_mm_rreg(ldev, LS_FB_CFG_REG + crtc_offset);
		val |= LS_FB_CFG_ENABLE;
		ls_mm_wreg_check(ldev, LS_FB_CFG_REG + crtc_offset, val);

		val = ls_mm_rreg(ldev, LS_FB_PANCFG_REG + crtc_offset);
		val |= LS_FB_PANCFG_DE | LS_FB_PANCFG_CLKEN;
		ls_mm_wreg(ldev, LS_FB_PANCFG_REG + crtc_offset, val);

		val = ls_mm_rreg(ldev, LS_FB_HSYNC_REG + crtc_offset);
		val |= LS_FB_HSYNC_POLSE;
		ls_mm_wreg(ldev, LS_FB_HSYNC_REG + crtc_offset, val);
		val = ls_mm_rreg(ldev, LS_FB_VSYNC_REG + crtc_offset);
		val |= LS_FB_VSYNC_POLSE;
		ls_mm_wreg(ldev, LS_FB_VSYNC_REG + crtc_offset, val);
		drm_crtc_vblank_on(crtc);

		if (ldev->chip == dc_7a2000) {
			val = ls_mm_rreg(ldev, HDMI_CTRL_REG + crtc_offset);
			val |= BIT(0);
			ls_mm_wreg(ldev, HDMI_CTRL_REG + crtc_offset, val);

			val = ls_mm_rreg(ldev, HDMI_PHY_CTRL_REG + crtc_offset);
			val |= 0x3;
			ls_mm_wreg(ldev, HDMI_PHY_CTRL_REG + crtc_offset, val);

			val = ls_io_rreg(ldev, GENERAL_PURPOSE_REG + 0x4);
			val &= ~BIT(28);
			ls_io_wreg(ldev, GENERAL_PURPOSE_REG + 0x4, val);

			val = ls_io_rreg(ldev, GENERAL_PURPOSE_REG + 0x4);
			val &= ~BIT(29);
			ls_io_wreg(ldev, GENERAL_PURPOSE_REG + 0x4, val);
		}
		loongson_crtc->enabled = true;
		break;
	case DRM_MODE_DPMS_OFF:
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
		drm_crtc_vblank_off(crtc);
		val = ls_mm_rreg(ldev, LS_FB_PANCFG_REG + crtc_offset);
		val &= ~LS_FB_PANCFG_DE;
		val &= ~LS_FB_PANCFG_CLKEN;
		ls_mm_wreg(ldev, LS_FB_PANCFG_REG + crtc_offset, val);

		val = ls_mm_rreg(ldev, LS_FB_CFG_REG + crtc_offset);
		val &= ~LS_FB_CFG_ENABLE;
		ls_mm_wreg_check(ldev, LS_FB_CFG_REG + crtc_offset, val);

		val = ls_mm_rreg(ldev, LS_FB_HSYNC_REG + crtc_offset);
		val &= ~LS_FB_HSYNC_POLSE;
		ls_mm_wreg(ldev, LS_FB_HSYNC_REG + crtc_offset, val);
		val = ls_mm_rreg(ldev, LS_FB_VSYNC_REG + crtc_offset);
		val &= ~LS_FB_VSYNC_POLSE;
		ls_mm_wreg(ldev, LS_FB_VSYNC_REG + crtc_offset, val);

		if (ldev->chip == dc_7a2000) {
			val = ls_mm_rreg(ldev, HDMI_CTRL_REG + crtc_offset);
			val &= ~BIT(0);
			ls_mm_wreg(ldev, HDMI_CTRL_REG + crtc_offset, val);

			val = ls_mm_rreg(ldev, HDMI_PHY_CTRL_REG + crtc_offset);
			val &= ~0x3;
			ls_mm_wreg(ldev, HDMI_PHY_CTRL_REG + crtc_offset, val);

			val = ls_io_rreg(ldev, GENERAL_PURPOSE_REG + 0x4);
			val |= BIT(28);
			ls_io_wreg(ldev, GENERAL_PURPOSE_REG + 0x4, val);

			val = ls_io_rreg(ldev, GENERAL_PURPOSE_REG + 0x4);
			val |= BIT(29);
			ls_io_wreg(ldev, GENERAL_PURPOSE_REG + 0x4, val);
		}

		loongson_crtc->enabled = false;
		break;
	}
}

/**
 * loongson_crtc_prepare
 *
 * @crtc: point to a drm_crtc structure
 *
 * This is called before a mode is programmed. A typical use might be to
 * enable DPMS during the programming to avoid seeing intermediate stages
 */
static void loongson_crtc_prepare(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_crtc *crtci;
	/*
	 * The hardware wedges sometimes if you reconfigure one CRTC
	 * whilst another is running
	 */

	DRM_DEBUG_DRIVER("loongson_crtc_prepare\n");
	list_for_each_entry (crtci, &dev->mode_config.crtc_list, head)
		loongson_crtc_dpms(crtci, DRM_MODE_DPMS_OFF);
}

/**
 * loongson_crtc_commit
 *
 * @crtc: point to the drm_crtc structure
 *
 * Commit the new mode on the CRTC after a modeset.This is called after
 * a mode is programmed. It should reverse anything done by the prepare function
 */
static void loongson_crtc_commit(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_crtc *crtci;

	DRM_DEBUG_DRIVER("loongson_crtc_commit\n");
	list_for_each_entry (crtci, &dev->mode_config.crtc_list, head) {
		if (crtci->enabled)
			loongson_crtc_dpms(crtci, DRM_MODE_DPMS_ON);
	}
}

/**
 * loongson_crtc_destroy
 *
 * @crtc: pointer to a drm_crtc struct
 *
 * Destory the CRTC when not needed anymore,and transfer the drm_crtc_cleanup
 * function,the function drm_crtc_cleanup() cleans up @crtc and removes it
 * from the DRM mode setting core.Note that the function drm_crtc_cleanup()
 * does not free the structure itself.
 */
static void loongson_crtc_destroy(struct drm_crtc *crtc)
{
	struct loongson_crtc *loongson_crtc = to_loongson_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(loongson_crtc);
}

/**
 * loongosn_crtc_disable
 *
 * @crtc: DRM CRTC
 *
 * Used to shut down CRTC
 */
static void loongson_crtc_disable(struct drm_crtc *crtc)
{
	int ret;

	loongson_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
	if (crtc->primary->fb) {
		struct loongson_framebuffer *lfb =
			to_loongson_framebuffer(crtc->primary->fb);
		struct drm_gem_object *obj = lfb->obj;
		struct loongson_bo *bo = gem_to_loongson_bo(obj);
		ret = loongson_bo_reserve(bo, false);
		if (ret)
			return;
		loongson_bo_unpin(bo);
		loongson_bo_unreserve(bo);
	}
	crtc->primary->fb = NULL;
}

int loongson_crtc_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *fb,
			    struct drm_pending_vblank_event *event,
			    uint32_t page_flip_flags,
			    struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_device *dev = crtc->dev;
	struct loongson_device *ldev = dev->dev_private;
	struct loongson_crtc *loongson_crtc = to_loongson_crtc(crtc);
	struct loongson_framebuffer *old_fb;
	struct loongson_framebuffer *new_fb;
	struct drm_gem_object *obj;
	struct loongson_flip_work *work;
	struct loongson_bo *new_bo;
	u32 ret;
	u64 base;

	work = kzalloc(sizeof(*work), GFP_KERNEL);
	if (work == NULL)
		return -ENOMEM;

	INIT_DELAYED_WORK(&work->flip_work, loongson_flip_work_func);

	old_fb = to_loongson_framebuffer(crtc->primary->fb);
	obj = old_fb->obj;
	work->old_bo = gem_to_loongson_bo(obj);

	work->event = event;
	work->ldev = ldev;
	work->crtc_id = loongson_crtc->crtc_id;

	new_fb = to_loongson_framebuffer(fb);
	obj = new_fb->obj;
	new_bo = gem_to_loongson_bo(obj);

	/* pinthenewbuffer */
	ret = loongson_bo_reserve(new_bo, false);
	if (unlikely(ret != 0)) {
		DRM_ERROR("failed to reserve new bo buffer before flip\n");
		goto cleanup;
	}

	ret = loongson_bo_pin(new_bo, TTM_PL_FLAG_VRAM, &base);
	if (unlikely(ret != 0)) {
		ret = -EINVAL;
		DRM_ERROR("failed to pin new bo buffer before flip\n");
		goto unreserve;
	}

	loongson_bo_unreserve(new_bo);
	work->base = base;

	ret = drm_crtc_vblank_get(crtc);
	if (ret)
		goto cleanup;

	loongson_crtc->pflip_works = work;
	crtc->primary->fb = fb;

	loongson_flip_work_func(&work->flip_work.work);

	return 0;

unreserve:
	loongson_bo_unreserve(new_bo);

cleanup:
	loongson_bo_unref(&work->old_bo);
	kfree(work);

	return 0;
}

static int loongson_crtc_gamma_set(struct drm_crtc *crtc,
				   u16 *red, u16 *green, u16 *blue,
				   uint32_t size,
				   struct drm_modeset_acquire_ctx *ctx)
{
	struct loongson_device *ldev;
	unsigned int i;
	u32 reg_offset;
	u32 val;

	if (!crtc->enabled)
		return 0;

	ldev = crtc->dev->dev_private;

	if (!ldev->enable_gamma)
		return 0;

	reg_offset = crtc->index * 0x10;

	val = ls_mm_rreg(ldev, LS_FB_CFG_REG + reg_offset);

	val |= LS_FB_CFG_GAMMA;

	ls_mm_wreg_check(ldev, LS_FB_CFG_REG + reg_offset, val);

	/* init gamma */
	ls_mm_wreg(ldev, GAMMA_INDEX_REG + reg_offset, 0x0);

	for (i = 0; i < 256; i++) {
		val = (((red[i] << 8) & 0xFF0000) |
		       (green[i] & 0x00FF00) |
		       ((blue[i] >> 8) & 0x0000FF));

		ls_mm_wreg(ldev, GAMMA_DATA_REG + reg_offset, val);
	}

	DRM_DEBUG_DRIVER("hardware gamma table updated\n");

	return 0;
}

/**
 * These provide the minimum set of functions required to handle a CRTC
 * Each driver is responsible for filling out this structure at startup time
 *
 * The drm_crtc_funcs structure is the central CRTC management structure
 * in the DRM. Each CRTC controls one or more connectors
 */
static const struct drm_crtc_funcs loongson_crtc_funcs = {
	.cursor_set2 = loongson_crtc_cursor_set2,
	.cursor_move = loongson_crtc_cursor_move,
	.gamma_set = loongson_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = loongson_crtc_destroy,
	.page_flip = loongson_crtc_page_flip,
};

enum drm_mode_status
loongson_7a1000_mode_filter(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode)
{
	struct loongson_device *ldev = crtc->dev->dev_private;

	if (mode->hdisplay > 1920)
		return MODE_BAD;
	if (mode->vdisplay > 1080)
		return MODE_BAD;
	if (mode->hdisplay < 800)
		return MODE_BAD;
	if (mode->vdisplay < 600)
		return MODE_BAD;
	if (ldev->connector_active0 == 1 &&
	    ldev->connector_active1 == 1 &&
	    mode->hdisplay % 64)
		return MODE_BAD;
	if (mode->clock >= 173000)
		return MODE_CLOCK_HIGH;
	if (mode->hdisplay == 1152)
		return MODE_BAD;
	if (mode->clock == 141750 && mode->hdisplay == 1920)
		return MODE_BAD;

	return MODE_OK;
}

enum drm_mode_status
loongson_7a2000_mode_filter(struct drm_crtc *crtc,
			      const struct drm_display_mode *mode)
{
	if (mode->hdisplay > 4096)
		return MODE_BAD;
	if (mode->vdisplay > 2160)
		return MODE_BAD;
	if (mode->clock > 340000)
		return MODE_CLOCK_HIGH;
	if (mode->hdisplay % 8)
		return MODE_BAD;
	if (mode->vdisplay < 480)
		return MODE_BAD;

	return MODE_OK;
}

enum drm_mode_status
loongson_2k2000_mode_filter(struct drm_crtc *crtc,
			      const struct drm_display_mode *mode)
{
	if (mode->hdisplay > 1920 && crtc->index == 1)
		return MODE_BAD;
	else if (mode->hdisplay > 4096)
		return MODE_BAD;
	if (mode->vdisplay > 1080 && crtc->index == 1)
		return MODE_BAD;
	else if (mode->vdisplay > 2160)
		return MODE_BAD;
	if (mode->clock > 340000)
		return MODE_CLOCK_HIGH;
	if (mode->hdisplay % 8)
		return MODE_BAD;
	if (mode->vdisplay < 480)
		return MODE_BAD;

	return MODE_OK;
}

static enum drm_mode_status
loongson_crtc_mode_valid(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode)
{
	struct loongson_device *ldev = crtc->dev->dev_private;

	if (ldev->dc_config->mode_filter)
		return ldev->dc_config->mode_filter(crtc, mode);

	return MODE_ERROR;
}

/**
 * These provide the minimum set of functions required to handle a CRTC
 *
 * The drm_crtc_helper_funcs is a helper operations for CRTC
 */
static const struct drm_crtc_helper_funcs loongson_helper_funcs = {
	.disable = loongson_crtc_disable,
	.dpms = loongson_crtc_dpms,
	.mode_set = loongson_crtc_mode_set,
	.prepare = loongson_crtc_prepare,
	.commit = loongson_crtc_commit,
	.mode_valid = loongson_crtc_mode_valid,
};

/**
 * loongosn_crtc_init
 *
 * @ldev: point to the loongson_device structure
 *
 * Init CRTC
 */
struct loongson_crtc *loongson_crtc_init(struct loongson_device *ldev,
					 int index)
{
	struct loongson_crtc *ls_crtc;
	struct device_node *node = NULL;

	ls_crtc = kzalloc(sizeof(struct loongson_crtc), GFP_KERNEL);
	if (!ls_crtc)
		return NULL;

	ls_crtc->ldev = ldev;
	ls_crtc->crtc_offset = index * REG_OFFSET;
	ls_crtc->crtc_id = get_crtc_id(ldev, index);
	ls_crtc->max_freq = get_crtc_max_freq(ldev, index);
	ls_crtc->max_width = get_crtc_max_width(ldev, index);
	ls_crtc->max_height = get_crtc_max_height(ldev, index);
	ls_crtc->encoder_id = get_crtc_encoder_id(ldev, index);
	ls_crtc->is_vb_timing = get_crtc_is_vb_timing(ldev, index);

	if (!ldev->gpu_pdev) {
		if (ls_crtc->crtc_id)
			node = of_find_compatible_node(NULL, NULL, "loongson,ls2k-pix1_pll");
		else
			node = of_find_compatible_node(NULL, NULL, "loongson,ls2k-pix0_pll");
		ls_crtc->pix_node = node;
	}

	if (ls_crtc->is_vb_timing)
		ls_crtc->timing = get_crtc_timing(ldev, index);

	drm_crtc_init(ldev->dev, &ls_crtc->base, &loongson_crtc_funcs);
	drm_mode_crtc_set_gamma_size(&ls_crtc->base, 256);
	drm_crtc_helper_add(&ls_crtc->base, &loongson_helper_funcs);

	return ls_crtc;
}
