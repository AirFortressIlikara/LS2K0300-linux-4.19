// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (c) 2021 Loongson Technology Co., Ltd.
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/*
 * LCD_BKLT_EN is connected to GPIO46
 * LCD_VDD_EN is connected to GPIO47
 * LVDS_BKLT_CTRL is connected to PWM3
 * The panel is NL192108AC18-02D
 */
#include <asm/delay.h>

#include <linux/pwm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <drm/drm_edid.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>

#include "../loongson_drv.h"
#include "../loongson_vbios.h"
#include "bridge_phy.h"
#include "bridge_phy_interface.h"

/*
 * LT8619 Device Address : 0x64
 */
#define LT8619_CHIP_NAME                        "LT8619C"
#define LT8619_CHIP_ADDR                        0x32

#define LT8619_REG_START                        0x0000
#define LT8619_REG_END                          0x80FE
#define LT8619_REG_PAGE_SELECT                  0xFF

#define EDID_DATA_SIZE                          256

#define WAIT_TIMES                              10000

/* REG 0x6000 */
#define LT8619_REG_CHIP_ID                      0x6000

/* REG 0x6004 */

/* Analog register clock enable(Address range: 0x6080 ~ 0x60FE) */
#define ANALOG_REG_CLK_EN_BIT                   BIT(7)
/* Control register clock enable(Address range: 0x6010 ~ 0x607F) */
#define CONTROL_REG_CLK_EN_BIT                  BIT(6)
/* HDMI register clock enable(Address range: 0x8000 ~ 0x809F) */
#define HDMI_REG_CLK_EN_BIT                     BIT(5)
/* HDCP register clock enable(Address range: 0x80A0 ~ 0x80CF) */
#define HDCP_REG_CLK_EN_BIT                     BIT(4)
/* LVDS PLL lock detect module clock enable */
#define LVDS_PLL_LOCK_DETECT_CLK_EN_BIT         BIT(1)
/* GPIO lt8619_mode_validtest module clock enable. */
#define GPIO_TEST_CLK_EN_BIT                    BIT(0)


/* REG 0x6006 */
#define LVDS_TX_CLK_EN_BIT                      BIT(7)
#define BT_TX_CLK_EN_BIT                        BIT(6)
#define EDID_SHADOW_CLK_EN_BIT                  BIT(5)
#define DUAL_PORT_CLK_EN_BIT                    BIT(4)  /* ? */
#define CEC_CTL_SYS_CLK_EN_BIT                  BIT(3)
#define VIDEO_CHECK_SYS_CLK_EN_BIT              BIT(2)
#define VIDEO_CHECK_PIX_CLK_EN_BIT              BIT(1)
#define INTERRUPT_PROCESS_SYS_CLK_EN_BIT        BIT(0)


/* REG 0x6009 */
#define HDMI_RX_CTL_BIT                         BIT(7)
#define HDMI_RX_CDR0_BIT                        BIT(6)
#define HDMI_RX_CDR1_BIT                        BIT(5)
#define HDMI_RX_CDR2_BIT                        BIT(4)
#define HDMI_RX_CDR_BW_BIT                      BIT(3)
#define HDMI_RX_CDR_INDEX_BIT                   BIT(2)
#define HDMI_RX_PLL_LOCK_DETECT_BIT             BIT(1)
#define HDMI_RX_PLL_HW_RST                      BIT(0)

/* REG 0x600A */
#define LVDS_PLL_LOCK_DETECT_CTL_SW_RST         BIT(6)

/* REG 0x600C */
#define EDID_SHADOW_SW_RST_BIT                  BIT(6)
#define VIDEO_CHK_SW_RST_BIT                    BIT(2)

/* REG 0x600D */
#define LVDS_TX_CTL_SW_RST                      BIT(2)
#define BT_TX_CTL_SW_RST                        BIT(1)
#define BT_TX_AFIFO_SW_RST                      BIT(0)

/* REG 0x600E */
#define HDMI_RX_CALIBRATION_BIT                 BIT(7)
#define HDMI_RX_PLL_BIT                         BIT(6)
#define HDMI_RX_PI0_BIT                         BIT(5)
#define HDMI_RX_PI1_BIT                         BIT(4)
#define HDMI_RX_PI2_BIT                         BIT(3)
#define HDMI_RX_AUDIO_PLL_BIT                   BIT(2)
#define LVDS_PLL_SW_RST_BIT                     BIT(1)
#define LVDS_TX_CLK_GEN_RESET                   BIT(0)


/* hsync width, unit: number for pix_clk
 * +------+--------+--------+
 * | REG  | 0x6014 | 0x6015 |
 * +------+--------+--------+
 * | bits |  11:8  |  7:0   |
 * +------+--------+--------+
 */

/* REG 0x6059 */
#define LVDS_OE_EN_BIT                          BIT(6)
#define LVDS_SYNC_DE_MODE_BIT                   BIT(5)
#define LVDS_6_BIT_COlOR                        BIT(4)
#define LVDS_HSYNC_POL_INV_BIT                  BIT(3)
#define LVDS_VSYNC_POL_INV_BIT                  BIT(2)
#define LVDS_PORT_SWAP_EN_BIT                   BIT(1)
#define LVDS_RGB_RB_SWAP_BIT                    BIT(0)

/* REG 0x60A0 */
/* 1 = Power down, 0 = Power up */
#define LVDS_PLL_PD                             BIT(7)
/* 1 = Adaptive BW tracking PLL, 0 = Second order passive LPF PLL */
#define LVDS_PLL_ADAPRT_BW_EN                   BIT(6)
/* 1 = High BW, 0 = Low BW */
#define LVDS_PLL_CP_CUR_SEL                     BIT(5)
/* This bit controls the operation of PLL locking EN */
#define LVDS_PLL_LOCK_EN                        BIT(4)

/* Reference clock selection:
 * 1 = Pixel clock as reference, 0 = Double clock as reference
 */
#define LVDS_PLL_PIX_CLK_SEL                    BIT(3)
/* Pixel clock selection: 1 = 2 x lvds clock, 0 = 1 x lvds clock */
#define LVDS_PLL_DUAL_MODE_EN                   BIT(2)
/* Pixel clock selection: 1 = 4 x lvds clock, 0 = 2 x lvds clock */
#define LVDS_PLL_HALF_MODE_EN                   BIT(1)
/* BT clock clock selection */
#define LVDS_PLL_DOUB_MODE_EN                   BIT(0)


/* REG 0x60A8 */
#define RGB888_TTL_EN_BIT                       BIT(3)

/* REG 0x8006 */

/* REG 0x8013 */

/* 1 = Input is HDMI, 0 = Input is DVI */
#define RX_IS_HDMI_BIT                          BIT(1)
/* 1 = Hsync is detected and is stable, 0 = No hsync detected or not stable */
#define RX_HDMI_HSYNC_STABLE_BIT                BIT(0)

/* REG 0x802C */

/* Set to enable clock data recovery bandwidth adaptation */
#define CDR_BW_ADAP_EN_BIT                      BIT(3)

/* REG 0x8044 */
/* Clock stable status indicator */
#define RX_CLK_STABLE_BIT                       BIT(3)

/* REG 0x8071 */
/* Packet byte 1 of AVI information
 * {Y1, Y0} of {0,Y1,Y0,A0,B1,B0,S1,S0}:
 */

#define COLOR_SPACE_MASK                        GENMASK(6, 5)
#define CSC_RGB                                 0x00
#define CSC_YCBCR_422                           0x20
#define CSC_YCBCR_444                           0x40
#define CSC_FUTURE                              0x60


/* TMDS clock frequency, unit: kHz
 * +------+--------+--------+--------+
 * | REG  | 0x8044 | 0x8045 | 0x8046 |
 * +------+--------+--------+--------+
 * | bits |  18:16 |  15:8  |  7:0   |
 * +------+--------+--------+--------+
 */

/* REG 0x8087 */
#define LVDS_PLL_LOCKED_BIT                     BIT(5)
#define RX_PLL_LOCKED_BIT                       BIT(4)

/* REG 0x808E */
#define DDC_SHADOW_EDID_DISABLE                 BIT(2)
#define DDC_I2C_SLAVE_0X60_DISABLE              BIT(1)
#define EDID_SHADOW_BUFFER_ENABLE               BIT(0)


static const struct regmap_range_cfg lt8619_ranges[] = {
	{
		.name = "lt8619_register_range",
		.range_min = LT8619_REG_START,
		.range_max = LT8619_REG_END,
		.selector_reg = LT8619_REG_PAGE_SELECT,
		.selector_mask = 0xFF,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt8619_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = LT8619_REG_END,
	.ranges = lt8619_ranges,
	.num_ranges = ARRAY_SIZE(lt8619_ranges),
};

struct lt8619_bridge {
	struct drm_bridge bridge;
	struct drm_connector connector;

	struct regmap *regmap;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct loongson_backlight *bklt;

	u8 edid_buf[EDID_DATA_SIZE];

	union lt_chip_id {
		unsigned int i;
		unsigned char c[4];
	} chip_id;
};

static inline struct lt8619_bridge *drm_bridge_to_ltb(struct drm_bridge *b)
{
	return container_of(b, struct lt8619_bridge, bridge);
}

static inline struct lt8619_bridge *
drm_connector_to_ltb(struct drm_connector *c)
{
	return container_of(c, struct lt8619_bridge, connector);
}

static int lt8619_read_revision(struct lt8619_bridge *ltb, unsigned int *dst)
{
	unsigned int chip_id;
	int ret;

	ret = regmap_bulk_read(ltb->regmap, LT8619_REG_CHIP_ID, &chip_id, 4);
	if (ret) {
		DRM_ERROR("failed to read revision: %d\n", ret);
		return ret;
	}

	if (dst)
		*dst = chip_id;

	DRM_DEV_INFO(&ltb->client->dev, "LT8619 vision: 0x%x\n", chip_id);

	return ret;
}

static int lt8619_config_hpd(struct lt8619_bridge *ltb, bool enable)
{
	int ret;
	unsigned int val;

	regmap_read(ltb->regmap, 0x8006, &val);

	val = enable ? (val | 0x08) : (val & 0xF7);

	ret = regmap_write(ltb->regmap, 0x8006, val);
	if (ret) {
		DRM_ERROR("error lt8619 config hpd\n");
		return ret;
	}

	DRM_DEBUG_DRIVER("lt8619 config hpd: 0x%x\n", val);

	return ret;
}

static int lt8619_config_rgb888_phy(struct lt8619_bridge *ltb, bool enable)
{
	int ret;
	unsigned int val;

	val = enable ? RGB888_TTL_EN_BIT : 0;

	ret = regmap_update_bits(ltb->regmap, 0x60A8, RGB888_TTL_EN_BIT, val);

	DRM_DEBUG_DRIVER("%s: reg=0x%x, val=0x%x\n", __func__, 0x60A8, val);

	return ret;
}

static void lt8619_edid_shadow_clk_enable(struct lt8619_bridge *ltb)
{
	unsigned int val;

	regmap_read(ltb->regmap, 0x6006, &val);

	if (val & EDID_SHADOW_CLK_EN_BIT)
		DRM_DEBUG_DRIVER("edid shadow clk is enabled\n");
	else
		DRM_DEBUG_DRIVER("edid shadow clk is not enabled\n");

	regmap_update_bits(ltb->regmap, 0x6006,
			   EDID_SHADOW_CLK_EN_BIT,
			   EDID_SHADOW_CLK_EN_BIT);
}

static void lt8619_lvds_tx_clk_enable(struct lt8619_bridge *ltb)
{
	regmap_update_bits(ltb->regmap, 0x6006,
			   LVDS_TX_CLK_EN_BIT,
			   LVDS_TX_CLK_EN_BIT);

	regmap_update_bits(ltb->regmap, 0x6006,
			   BT_TX_CLK_EN_BIT,
			   BT_TX_CLK_EN_BIT);

	DRM_DEBUG_DRIVER("LVDS TX controller module clock enabled\n");
}

static int lt8619_enable_sys_clk(struct lt8619_bridge *ltb, bool is_lvds)
{
	u32 val = ANALOG_REG_CLK_EN_BIT |
		  CONTROL_REG_CLK_EN_BIT |
		  HDMI_REG_CLK_EN_BIT |
		  HDCP_REG_CLK_EN_BIT;
	int ret;

	if (is_lvds)
		val |= LVDS_PLL_LOCK_DETECT_CLK_EN_BIT;

	ret = regmap_write(ltb->regmap, 0x6004, val);

	return ret;
}

static int lt8619_enable_cdr_bandwidth_adaptation(struct lt8619_bridge *ltb)
{
	int ret;

	ret = regmap_update_bits(ltb->regmap, 0x802C,
				 CDR_BW_ADAP_EN_BIT,
				 CDR_BW_ADAP_EN_BIT);

	return ret;
}

static const unsigned char LT8619_SIMPLE_EDID[][EDID_DATA_SIZE] = {
	{/* 1920x1080@60Hz and 1280x720@60Hz  */
		0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
		0x32, 0x8d, 0x19, 0x86, 0x01, 0x00, 0x00, 0x00,
		0x18, 0x19, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78,
		0x0a, 0x0d, 0xc9, 0xa0, 0x57, 0x47, 0x98, 0x27,
		0x12, 0x48, 0x4c, 0x00, 0x00, 0x00, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a,
		0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
		0x45, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x1e,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x32,
		0x46, 0x1e, 0x50, 0x0f, 0x00, 0x0a, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc,
		0x00, 0x4c, 0x4f, 0x4e, 0x54, 0x49, 0x55, 0x4d,
		0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x18,

		0x02, 0x03, 0x22, 0x70, 0x23, 0x09, 0x7f, 0x07,
		0x83, 0x01, 0x00, 0x00, 0x41, 0x90, 0x66, 0x03,
		0x0c, 0x00, 0x10, 0x00, 0xb8, 0x10, 0x08, 0x60,
		0x22, 0x00, 0x12, 0x8e, 0x21, 0x08, 0x08, 0x18,
		0x8c, 0x0a, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38,
		0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0xc4, 0x8e,
		0x21, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc8,
	},
	{	/* 1920*1080 */
		0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
		0x0e, 0xd4, 0x32, 0x31, 0x00, 0x88, 0x88, 0x88,
		0x20, 0x1c, 0x01, 0x03, 0x80, 0x0c, 0x07, 0x78,
		0x0a, 0x0d, 0xc9, 0xa0, 0x57, 0x47, 0x98, 0x27,
		0x12, 0x48, 0x4c, 0x00, 0x00, 0x00, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a,
		0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
		0x45, 0x00, 0x80, 0x38, 0x74, 0x00, 0x00, 0x1e,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x0a,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
		0x00, 0x4C, 0x6F, 0x6E, 0x74, 0x69, 0x75, 0x6D,
		0x20, 0x73, 0x65, 0x6D, 0x69, 0x20, 0x01, 0xf5,

		0x02, 0x03, 0x12, 0xf1, 0x23, 0x09, 0x04, 0x01,
		0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0c, 0x00,
		0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf,
	}
};

static int lt8619_download_edid_to_chip(struct lt8619_bridge *ltb,
					const unsigned char *edid_blob,
					unsigned int edid_len)
{
	unsigned int i;
	unsigned int val;

	regmap_write(ltb->regmap, 0x808E, 0x07);

	for (i = 0; i < edid_len; i++) {
		regmap_write(ltb->regmap, 0x808F, i);
		regmap_write(ltb->regmap, 0x8090, edid_blob[i]);
	}

	regmap_write(ltb->regmap, 0x808E, 0x01);

	regmap_read(ltb->regmap, 0x8091, &val);
	if ((val & 0x01) == 0) {
		DRM_ERROR("%s: EDID is NOT valid\n", __func__);
		return -1;
	}

	return edid_len;
}

static int lt8619_read_edid_from_chip(struct lt8619_bridge *ltb,
				      unsigned char *dst)
{
	unsigned int i;

	regmap_write(ltb->regmap, 0x808E, 0x07);

	for (i = 0; i < EDID_DATA_SIZE; i++) {
		regmap_write(ltb->regmap, 0x808F, i);
		regmap_read(ltb->regmap, 0x8090, (unsigned int *)&dst[i]);
	}

	regmap_write(ltb->regmap, 0x808E, 0x01);

	DRM_DEBUG_DRIVER("Read EDID from eeprom on chip\n");

	return 0;
}

static int lt8619_get_modes(struct drm_connector *connector)
{
	struct lt8619_bridge *ltb = drm_connector_to_ltb(connector);
	struct i2c_adapter *adapter = ltb->adapter;
	struct edid *edid = drm_get_edid(connector, adapter);
	unsigned int count = 0;

	if (edid) {
		drm_connector_update_edid_property(connector, edid);

		count = drm_add_edid_modes(connector, edid);

		kfree(edid);

		DRM_DEBUG_DRIVER("get %u modes\n", count);

		return count;
	}

	edid = (struct edid *)ltb->edid_buf;
	count = drm_add_edid_modes(connector, edid);
	if (count) {
		drm_connector_update_edid_property(connector, edid);
		return count;
	}

	/* Give a default */
	count = drm_add_modes_noedid(connector, 1920, 1080);
	drm_set_preferred_mode(connector, 1920, 1080);

	return count;
}

static enum drm_mode_status lt8619_mode_valid(struct drm_connector *connector,
					      struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder *lt8619_best_encoder(struct drm_connector *conn)
{
	struct lt8619_bridge *ltb = drm_connector_to_ltb(conn);
	struct drm_bridge *bridge = &ltb->bridge;
	struct drm_encoder *encoder = bridge->encoder;

	DRM_DEBUG_DRIVER("%s: %u\n", encoder->name, encoder->index);

	return encoder;
}

static const struct drm_connector_helper_funcs lt8619_connector_helpers = {
	.get_modes = lt8619_get_modes,
	.mode_valid = lt8619_mode_valid,
	.best_encoder = lt8619_best_encoder,
};

static const struct drm_connector_funcs lt8619_lvds_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
};

static void lt8619_wait_rx_pll_locked(struct lt8619_bridge *ltb)
{
	unsigned int val = 0;
	unsigned int count = 0;

	do {
		regmap_read(ltb->regmap, 0x8087, &val);
		count++;
		udelay(5);
	} while (((val & RX_PLL_LOCKED_BIT) == 0) && (count < WAIT_TIMES));

	if (count >= WAIT_TIMES)
		DRM_ERROR("HDMI RX PLL IS NOT LOCKED");
	else
		DRM_DEBUG_DRIVER("HDMI RX PLL LOCKED\n");
}

/*
 * When HDMI signal is stable, LVDS PLL lock status needs to be detected.
 * If it is not locked, LVDS PLL needs to be reset.
 */
static void lt8619_wait_hdmi_stable(struct lt8619_bridge *ltb)
{
	unsigned int val = 0;
	unsigned int count = 0;

	do {
		regmap_read(ltb->regmap, 0x8044, &val);
		count++;
		udelay(5);
	} while ((!(val & RX_CLK_STABLE_BIT)) && (count < WAIT_TIMES));

	DRM_DEBUG_DRIVER("HDMI clock signal stabled\n");

	lt8619_wait_rx_pll_locked(ltb);

	count = 0;
	DRM_DEBUG_DRIVER("Wait HDMI HSync detect and stable\n");

	do {
		regmap_read(ltb->regmap, 0x8013, &val);
		count++;
		udelay(5);
	} while ((!(val & RX_HDMI_HSYNC_STABLE_BIT)) && (count < WAIT_TIMES));

	DRM_DEBUG_DRIVER("HDMI HSync stabled\n");
}

/* TMDS clock frequency indicator */
static void lt8619_read_hdmi_clock_frequency(struct lt8619_bridge *ltb,
					     unsigned int *pfreq)
{
	unsigned int up, mid, low;
	unsigned int freq;

	regmap_read(ltb->regmap, 0x8044, &up);
	regmap_read(ltb->regmap, 0x8045, &mid);
	regmap_read(ltb->regmap, 0x8046, &low);

	freq = ((up & 0x07) << 16) + (mid << 8) + low;

	if (pfreq)
		*pfreq = freq;
	else
		DRM_DEBUG_DRIVER("HDMI clock frequency: %dkHz\n", freq);
}

static ssize_t hdmi_clk_freq_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;
	unsigned int kHz;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	lt8619_read_hdmi_clock_frequency(ltb, &kHz);

	return snprintf(buf, PAGE_SIZE, "%ukHz\n", kHz);
}
static DEVICE_ATTR_RO(hdmi_clk_freq);

/* LVDS TX controller module soft reset */
static void lt8619_lvds_tx_ctl_soft_reset(struct lt8619_bridge *ltb)
{
	regmap_update_bits(ltb->regmap, 0x600D, LVDS_TX_CTL_SW_RST, 0);
	udelay(10000);
	regmap_update_bits(ltb->regmap, 0x600D,
			   LVDS_TX_CTL_SW_RST,
			   LVDS_TX_CTL_SW_RST);
	udelay(10000);
	DRM_DEBUG_DRIVER("LVDS TX controller module soft reset finished\n");
}

static void lt8619_edid_shadow_soft_reset(struct lt8619_bridge *ltb)
{
	regmap_update_bits(ltb->regmap, 0x600C, EDID_SHADOW_SW_RST_BIT, 0);
	udelay(10000);
	regmap_update_bits(ltb->regmap, 0x600C,
			   EDID_SHADOW_SW_RST_BIT,
			   EDID_SHADOW_SW_RST_BIT);
	udelay(10000);
	DRM_DEBUG_DRIVER("%s\n", __func__);
}

/* Video check logic soft reset */
static void lt8619_vid_chk_soft_reset(struct lt8619_bridge *ltb)
{
	regmap_update_bits(ltb->regmap, 0x600C, VIDEO_CHK_SW_RST_BIT, 0);
	udelay(10000);
	regmap_update_bits(ltb->regmap, 0x600C,
			   VIDEO_CHK_SW_RST_BIT,
			   VIDEO_CHK_SW_RST_BIT);
	udelay(10000);
}

/*
 * Read HDMI Timing information
 */
static ssize_t lt8619_read_hdmi_timings(struct lt8619_bridge *ltb, char *buf)
{
	unsigned int len = 0;
	unsigned int pos = 0;
	unsigned int high, low;
	unsigned int polarity;
	unsigned int h_front_porch, h_back_porch, h_total, h_active;
	unsigned int v_front_porch, v_back_porch, v_total, v_active;
	unsigned int v_sync, h_sync;

	/* horizontal sync width */
	regmap_read(ltb->regmap, 0x6014, &high);
	regmap_read(ltb->regmap, 0x6015, &low);
	h_sync = (high << 8) + low;
	/* horizontal back porch */
	regmap_read(ltb->regmap, 0x6018, &high);
	regmap_read(ltb->regmap, 0x6019, &low);
	h_back_porch = (high << 8) + low;
	/* horizontal front porch */
	regmap_read(ltb->regmap, 0x601A, &high);
	regmap_read(ltb->regmap, 0x601B, &low);
	h_front_porch = (high << 8) + low;
	/* horizontal total */
	regmap_read(ltb->regmap, 0x601C, &high);
	regmap_read(ltb->regmap, 0x601D, &low);
	h_total = (high << 8) + low;
	/* horizontal active */
	regmap_read(ltb->regmap, 0x6020, &high);
	regmap_read(ltb->regmap, 0x6021, &low);
	h_active = (high << 8) + low;

	/* vertical total */
	regmap_read(ltb->regmap, 0x601E, &high);
	regmap_read(ltb->regmap, 0x601F, &low);
	v_total = (high << 8) + low;
	/* vertical active */
	regmap_read(ltb->regmap, 0x6022, &high);
	regmap_read(ltb->regmap, 0x6023, &low);
	v_active = (high << 8) + low;
	/* vertical back porch */
	regmap_read(ltb->regmap, 0x6016, &v_back_porch);
	/* vertical front porch */
	regmap_read(ltb->regmap, 0x6017, &v_front_porch);
	/* vertical sync width */
	regmap_read(ltb->regmap, 0x6013, &v_sync);

	/* The vsync polarity and hsync polarity */
	regmap_read(ltb->regmap, 0x6024, &polarity);

	if (buf) {
		len = snprintf(&buf[pos], PAGE_SIZE, "%ux%u\n",
			       h_active, v_active);
		pos += len;

		len = snprintf(&buf[pos], PAGE_SIZE, "%u, %u, %u, %u\n",
			       h_front_porch, h_back_porch, h_total, h_sync);
		pos += len;

		len = snprintf(&buf[pos], PAGE_SIZE, "%u, %u, %u, %u\n",
			       v_front_porch, v_back_porch, v_total, v_sync);
		pos += len;
	} else {
		DRM_DEBUG_DRIVER("%ux%u\n", h_active, v_active);

		DRM_DEBUG_DRIVER("%u, %u, %u, %u\n",
				 h_front_porch, h_back_porch, h_total, h_sync);
		DRM_DEBUG_DRIVER("%u, %u, %u, %u\n",
				 v_front_porch, v_back_porch, v_total, v_sync);
	}

	return pos;
}

static ssize_t hdmi_timings_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	return lt8619_read_hdmi_timings(ltb, buf);
}
static DEVICE_ATTR_RO(hdmi_timings);

static void lt8619_turn_on_lvds(struct lt8619_bridge *ltb)
{
	/* bit2 = 0 => turn on LVDS C */
	regmap_write(ltb->regmap, 0x60BA, 0x18);

	/* bit2 = 0 => turn on LVDS D */
	regmap_write(ltb->regmap, 0x60C0, 0x18);
}

static void lt8619_turn_off_lvds(struct lt8619_bridge *ltb)
{
	/* bit2= 1 => turn off LVDS C */
	regmap_write(ltb->regmap, 0x60BA, 0x44);

	/* bit2= 1 => turn off LVDS D */
	regmap_write(ltb->regmap, 0x60C0, 0x44);
}

static void lt8619_hdmi_rx_reset(struct lt8619_bridge *ltb)
{
	regmap_write(ltb->regmap, 0x600E, 0xFF);
	regmap_write(ltb->regmap, 0x6009, 0xFF);

	regmap_update_bits(ltb->regmap, 0x600E, HDMI_RX_CALIBRATION_BIT, 0);
	regmap_update_bits(ltb->regmap, 0x6009, HDMI_RX_PLL_LOCK_DETECT_BIT, 0);

	udelay(10000);

	regmap_write(ltb->regmap, 0x600E, 0xFF);
	regmap_write(ltb->regmap, 0x6009, 0xFF);

	udelay(10000);

	regmap_update_bits(ltb->regmap, 0x600E, HDMI_RX_PI0_BIT, 0);
	regmap_update_bits(ltb->regmap, 0x600E, HDMI_RX_PI1_BIT, 0);
	regmap_update_bits(ltb->regmap, 0x600E, HDMI_RX_PI2_BIT, 0);
	regmap_update_bits(ltb->regmap, 0x6009, HDMI_RX_CTL_BIT, 0);
	regmap_update_bits(ltb->regmap, 0x6009, HDMI_RX_CDR0_BIT, 0);
	regmap_update_bits(ltb->regmap, 0x6009, HDMI_RX_CDR1_BIT, 0);
	regmap_update_bits(ltb->regmap, 0x6009, HDMI_RX_CDR2_BIT, 0);

	udelay(10000);

	regmap_write(ltb->regmap, 0x600E, 0xFF);
	regmap_write(ltb->regmap, 0x6009, 0xFF);

	udelay(10000);
}


/* LVDS PLL lock detect control logic Soft Reset */
static void lt8619_lvds_pll_lock_ctl_soft_reset(struct lt8619_bridge *ltb)
{
	regmap_update_bits(ltb->regmap, 0x600A,
			   LVDS_PLL_LOCK_DETECT_CTL_SW_RST, 0);
	udelay(10000);
	regmap_update_bits(ltb->regmap, 0x600A,
			   LVDS_PLL_LOCK_DETECT_CTL_SW_RST,
			   LVDS_PLL_LOCK_DETECT_CTL_SW_RST);
}

/* LVDS PLL Soft Reset */
static void lt8619_lvds_pll_soft_reset(struct lt8619_bridge *ltb)
{
	regmap_update_bits(ltb->regmap, 0x600E, LVDS_PLL_SW_RST_BIT, 0);
	udelay(10000);
	regmap_update_bits(ltb->regmap, 0x600E, LVDS_PLL_SW_RST_BIT,
						LVDS_PLL_SW_RST_BIT);

	DRM_DEBUG_DRIVER("%s\n", __func__);
}

static void lt8619_wait_lvds_pll_locked(struct lt8619_bridge *ltb)
{
	unsigned int val = 0;
	unsigned int count = 0;

	do {
		regmap_read(ltb->regmap, 0x8087, &val);
		udelay(5);
		count++;
	} while ((!(val & LVDS_PLL_LOCKED_BIT)) && (count < WAIT_TIMES));

	if (count >= WAIT_TIMES)
		DRM_ERROR("LVDS PLL is NOT locked\n");
	else
		DRM_DEBUG_DRIVER("LVDS PLL locked\n");
}

static void lt8619_config_lvds_pll(struct lt8619_bridge *ltb)
{
	/* LVDS TX controller module clock enable */
	regmap_write(ltb->regmap, 0x60A0,
		     LVDS_PLL_ADAPRT_BW_EN |
		     LVDS_PLL_LOCK_EN |
		     LVDS_PLL_PIX_CLK_SEL);
}

static void lt8619_config_color_space(struct lt8619_bridge *ltb)
{
	unsigned int temp_csc;

	regmap_read(ltb->regmap, 0x8071, &temp_csc);

	/* if the color space is not RGB, we need convert it */
	if ((temp_csc & COLOR_SPACE_MASK) == CSC_YCBCR_422) {
		/* enable YCbCr to RGB clk */
		regmap_write(ltb->regmap, 0x6007, 0x8C);
		/* YUV422 to YUV444 enable */
		regmap_write(ltb->regmap, 0x6052, 0x01);
		/* 0x40:YUV to RGB enable; */
		regmap_write(ltb->regmap, 0x6053, 0x40 + 0x30);

		DRM_DEBUG_DRIVER("HDMI Color Space is YUV422\n");
	} else if ((temp_csc & COLOR_SPACE_MASK) == CSC_YCBCR_444) {
		/* enable YCbCr to RGB clk */
		regmap_write(ltb->regmap, 0x6007, 0x8C);
		/* YUV444 */
		regmap_write(ltb->regmap, 0x6052, 0x00);
		/* 0x40:YUV to RGB enable; */
		regmap_write(ltb->regmap, 0x6053, 0x40 + 0x30);

		DRM_DEBUG_DRIVER("HDMI Color Space is YUV444\n");
	} else if ((temp_csc & COLOR_SPACE_MASK) == CSC_RGB) {
		/* 0x00: bypass ColorSpace conversion */
		regmap_write(ltb->regmap, 0x6007, 0x80);
		regmap_write(ltb->regmap, 0x6052, 0x00);
		regmap_write(ltb->regmap, 0x6053, 0x00);
		DRM_DEBUG_DRIVER("HDMI Color Space is RGB");
	}
}

static ssize_t color_space_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;
	unsigned int cs;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	regmap_read(ltb->regmap, 0x8071, &cs);

	if ((cs & COLOR_SPACE_MASK) == CSC_RGB)
		return snprintf(buf, PAGE_SIZE, "RGB\n");

	if ((cs & COLOR_SPACE_MASK) == CSC_YCBCR_422)
		return snprintf(buf, PAGE_SIZE, "YCbCr 422\n");

	if ((cs & COLOR_SPACE_MASK) == CSC_YCBCR_444)
		return snprintf(buf, PAGE_SIZE, "YCbCr 444\n");

	return snprintf(buf, PAGE_SIZE, "Future\n");
}
static DEVICE_ATTR_RO(color_space);

static int lt8619_lvds_config(struct lt8619_bridge *ltb,
			      bool double_port,
			      bool is_6_bit,
			      bool de_mode,
			      bool hsync_inv,
			      bool vsync_inv,
			      bool port_swap,
			      bool rgb_swap)
{
	unsigned int val = LVDS_OE_EN_BIT;
	int ret;

	if (is_6_bit)
		val |= LVDS_6_BIT_COlOR;

	if (de_mode)
		val |= LVDS_SYNC_DE_MODE_BIT;

	if (hsync_inv)
		val |= LVDS_HSYNC_POL_INV_BIT;

	if (vsync_inv)
		val |= LVDS_VSYNC_POL_INV_BIT;

	if (port_swap)
		val |= LVDS_PORT_SWAP_EN_BIT;

	if (rgb_swap)
		val |= LVDS_RGB_RB_SWAP_BIT;

	regmap_write(ltb->regmap, 0x6059, val);

	if (is_6_bit) {
		regmap_write(ltb->regmap, 0x605F, 0x38);
		DRM_DEBUG_DRIVER("LVDS display color depth is 6 bit\n");
	}

	if (double_port) {
		regmap_write(ltb->regmap, 0x60A4, 0x01);
		regmap_write(ltb->regmap, 0x605C, 0x01);
		DRM_DEBUG_DRIVER("LVDS is double port\n");
	}

	/* LVDS channel output current settings */

	/* LVDS C channel d0[7:4] and d1[3:0] lane */
	regmap_write(ltb->regmap, 0x60B0, 0x66);
	/* LVDS C channel d2[7:4] lane and clk[3:0] */
	regmap_write(ltb->regmap, 0x60B1, 0x66);
	/* LVDS C channel d3[7:4] lane and LVDS D d0[3:0] */
	regmap_write(ltb->regmap, 0x60B2, 0x66);
	/* [7:4]:LVDS D channel d1 lane; [3:0] LVDS D channel d2 lane */
	regmap_write(ltb->regmap, 0x60B3, 0x66);
	/* [7:4]:LVDS D channel clk lane;[3:0] LVDS D channel d3 lane */
	regmap_write(ltb->regmap, 0x60B4, 0x66);

	/* LVDS C channel d0 lane common mode voltage */
	/* DC0 ; bit1 = 1 => DC0 PN swap */
	regmap_write(ltb->regmap, 0x60B5, 0x41);
	/* DC1 ; bit1 = 1 => DC1 PN swap */
	regmap_write(ltb->regmap, 0x60B6, 0x41);
	/* DC2 ; bit1 = 1 => DC2 PN swap */
	regmap_write(ltb->regmap, 0x60B7, 0x41);
	/* DCC ; bit7 = 1 => DCC PN swap */
	regmap_write(ltb->regmap, 0x60B8, 0x4c);
	/* DC3 ; bit1 = 1 => DC3 PN swap */
	regmap_write(ltb->regmap, 0x60B9, 0x41);

	/* DD0 ; bit1 = 1 => DD0 PN swap */
	regmap_write(ltb->regmap, 0x60BB, 0x41);
	/* DD1 ; bit1 = 1 => DD1 PN swap */
	regmap_write(ltb->regmap, 0x60BC, 0x41);
	/* DD2 ; bit1 = 1 => DD2 PN swap */
	regmap_write(ltb->regmap, 0x60BD, 0x41);
	/* DDC ; bit7 = 1 => DDC PN swap */
	regmap_write(ltb->regmap, 0x60BE, 0x4c);
	/* DD3 ; bit1 = 1 => DD3 PN swap */
	regmap_write(ltb->regmap, 0x60BF, 0x41);

	/* System clk from ring? */
	ret = regmap_write(ltb->regmap, 0x6080, 0x08);
	if (ret)
		return ret;

	ret = regmap_write(ltb->regmap, 0x6089, 0x88);
	if (ret)
		return ret;

	/* DC mode clk detector comparator is enable */
	ret = regmap_write(ltb->regmap, 0x608B, 0x90);
	if (ret)
		return ret;

	regmap_write(ltb->regmap, 0x60A1, 0xb0);
	regmap_write(ltb->regmap, 0x60A2, 0x10);

	return 0;
}

static void loongson_backlight_enable(struct loongson_backlight *bklt)
{
	gpio_set_value(bklt->gpio_vdd, 1);

	gpio_set_value(bklt->gpio_enable, 1);
}

static void loongson_backlight_disable(struct loongson_backlight *bklt)
{
	gpio_set_value(bklt->gpio_vdd, 0);

	gpio_set_value(bklt->gpio_enable, 0);
}

static void lt8619_bridge_disable(struct drm_bridge *bridge)
{
	struct lt8619_bridge *ltb = drm_bridge_to_ltb(bridge);

	loongson_backlight_disable(ltb->bklt);

	lt8619_turn_off_lvds(ltb);
}

static void lt8619_bridge_enable(struct drm_bridge *bridge)
{
	struct lt8619_bridge *ltb = drm_bridge_to_ltb(bridge);

	loongson_backlight_enable(ltb->bklt);

	lt8619_hdmi_rx_reset(ltb);

	lt8619_vid_chk_soft_reset(ltb);

	lt8619_edid_shadow_soft_reset(ltb);

	lt8619_edid_shadow_clk_enable(ltb);

	lt8619_enable_cdr_bandwidth_adaptation(ltb);

	lt8619_wait_hdmi_stable(ltb);

	lt8619_read_hdmi_timings(ltb, NULL);

	lt8619_read_hdmi_clock_frequency(ltb, NULL);

	lt8619_config_color_space(ltb);

	lt8619_config_lvds_pll(ltb);

	lt8619_lvds_pll_lock_ctl_soft_reset(ltb);

	lt8619_lvds_pll_soft_reset(ltb);

	lt8619_lvds_tx_ctl_soft_reset(ltb);

	lt8619_config_rgb888_phy(ltb, false);

	lt8619_lvds_config(ltb, true, false, false, false, false, false, false);

	/*
	 * When HDMI signal is stable, then we wait LVDS PLL locked.
	 * If it is not locked, LVDS PLL needs to be reset.
	 */
	lt8619_wait_lvds_pll_locked(ltb);

	lt8619_turn_on_lvds(ltb);
}

static void lt8619_bridge_mode_set(struct drm_bridge *bridge,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adj_mode)
{
	struct lt8619_bridge *ltb = drm_bridge_to_ltb(bridge);

	lt8619_config_color_space(ltb);

	lt8619_config_lvds_pll(ltb);

	lt8619_config_rgb888_phy(ltb, false);
}

static int lt8619_bridge_attach(struct drm_bridge *bridge)
{
	struct lt8619_bridge *ltb = drm_bridge_to_ltb(bridge);
	struct drm_connector *connector = &ltb->connector;
	int ret;

	ret = drm_connector_init(bridge->dev,
				 connector,
				 &lt8619_lvds_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR(" drm_connector_init()\n");
		return ret;
	}

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	drm_connector_helper_add(connector, &lt8619_connector_helpers);

	connector->polled = DRM_CONNECTOR_POLL_CONNECT;

	drm_connector_attach_encoder(connector, bridge->encoder);

	DRM_INFO("lt8619 display bridge attached\n");

	return 0;
}

static void lt8619_bridge_detach(struct drm_bridge *bridge)
{
	struct lt8619_bridge *ltb = drm_bridge_to_ltb(bridge);

	backlight_device_unregister(ltb->bklt->device);

	i2c_unregister_device(ltb->client);

	DRM_INFO("lt8619 display bridge detached\n");
}

static const struct drm_bridge_funcs lt8619_bridge_funcs = {
	.attach = lt8619_bridge_attach,
	.detach = lt8619_bridge_detach,
/*	.mode_set = lt8619_bridge_mode_set, */
	.disable = lt8619_bridge_disable,
	.enable = lt8619_bridge_enable,
};

static int loongson_backlight_update(struct backlight_device *bd)
{
	struct loongson_backlight *bklt = bl_get_data(bd);
	unsigned int level = bd->props.brightness;
	unsigned int period_ns;
	unsigned int duty_ns;

	level = clamp(level, bklt->min, bklt->max);

	bklt->level = level;

	period_ns = bklt->pwm_period;

	duty_ns = DIV_ROUND_UP((level * period_ns), bklt->max);

	pwm_config(bklt->pwm, duty_ns, period_ns);

	DRM_DEV_DEBUG_DRIVER(&bklt->device->dev,
			     "level=%u, period=%u, duty_ns=%u\n",
			     level, duty_ns, period_ns);

	return 0;
}

static int loongson_backlight_get_brightness(struct backlight_device *bd)
{
	struct loongson_backlight *bklt = bl_get_data(bd);

	unsigned int duty_ns, period_ns;
	unsigned int level;

	period_ns = bklt->pwm_period;
	duty_ns = pwm_get_duty_cycle(bklt->pwm);

	level = DIV_ROUND_UP((duty_ns * bklt->max), period_ns);
	level = clamp(level, bklt->min, bklt->max);

	return level;
}

static void lt8619_backlight_init(struct loongson_backlight *bklt,
				  unsigned int polarity,
				  unsigned int period)
{
	unsigned int duty_ns;
	unsigned int period_ns;

	period_ns = pwm_get_period(bklt->pwm);
	duty_ns = pwm_get_duty_cycle(bklt->pwm);

	if (period_ns == 0)
		period_ns = period;

	if (duty_ns == 0)
		duty_ns = DIV_ROUND_UP((period_ns * bklt->level), bklt->max);

	DRM_DEV_DEBUG_DRIVER(&bklt->device->dev,
			     "period_ns=%u, duty_ns=%u\n", period_ns, duty_ns);

	bklt->pwm_polarity = polarity;
	bklt->pwm_period = period_ns;

	pwm_set_period(bklt->pwm, period_ns);
	pwm_set_duty_cycle(bklt->pwm, duty_ns);
	pwm_set_polarity(bklt->pwm, bklt->pwm_polarity);
	pwm_enable(bklt->pwm);

	gpio_direction_output(bklt->gpio_vdd, 1);
	gpio_direction_output(bklt->gpio_enable, 1);
}

static const struct backlight_ops bklt_ops = {
	.update_status = loongson_backlight_update,
	.get_brightness = loongson_backlight_get_brightness,
};

static struct loongson_backlight *
lt8619_create_backlight(struct drm_device *ddev,
			unsigned int gpio_vdd,
			unsigned int gpio_enable,
			int pwm_id)
{
	struct loongson_backlight *bklt_obj;
	struct backlight_properties props;
	int ret;

	memset(&props, 0, sizeof(props));

	props.type = BACKLIGHT_RAW;
	props.max_brightness = LOONGSON_BL_MAX_LEVEL;
	props.brightness = 80;

	bklt_obj = devm_kzalloc(ddev->dev, sizeof(*bklt_obj), GFP_KERNEL);
	if (!bklt_obj)
		return NULL;

	bklt_obj->device = backlight_device_register("loongson-backlight",
						     ddev->dev,
						     bklt_obj,
						     &bklt_ops,
						     &props);

	if (IS_ERR(bklt_obj->device)) {
		DRM_ERROR("Failed to register backlight\n");
		bklt_obj->device = NULL;
		return NULL;
	}

	DRM_DEV_INFO(&bklt_obj->device->dev, "pwm backlight registered\n");

	bklt_obj->pwm = pwm_request(pwm_id, "pwm_backlight");
	if (IS_ERR(bklt_obj->pwm)) {
		DRM_ERROR("Failed to get the pwm %u\n", pwm_id);
		bklt_obj->pwm = NULL;
		return NULL;
	}

	bklt_obj->pwm_id = pwm_id;
	bklt_obj->min = LOONGSON_BL_MIN_LEVEL;
	bklt_obj->max = LOONGSON_BL_MAX_LEVEL;
	bklt_obj->level = LOONGSON_BL_DEF_LEVEL;

	ret = gpio_request(gpio_vdd, "GPIO_VDD");
	if (ret) {
		DRM_ERROR("request lcd vdd gpio failed\n");
		return ERR_PTR(ret);
	}

	ret = gpio_request(gpio_enable, "GPIO_ENABLE");
	if (ret < 0) {
		DRM_ERROR("request backlight enable gpio failed!\n");
		return ERR_PTR(ret);
	}

	bklt_obj->gpio_vdd = gpio_vdd;
	bklt_obj->gpio_enable = gpio_enable;

	return bklt_obj;
}

/*
 *  Get edid at initial time, save it for latter usage
 */
static int lt8619_get_edid_once(struct lt8619_bridge *ltb,
				struct drm_device *ddev,
				unsigned int index)
{
	struct loongson_device *ldev = ddev->dev_private;
	enum loongson_edid_method m = get_edid_method(ldev, index);
	u8 *edid_ptr_src;

	if (m == via_vbios) {
		edid_ptr_src = get_vbios_edid(ldev, index);
		if (edid_ptr_src == NULL)
			return -1;

		if (drm_edid_is_valid((struct edid *)edid_ptr_src) == false) {
			DRM_WARN("vbios provided edid is not valid\n");
			goto FALLBACK;
		}

		strscpy(ltb->edid_buf, edid_ptr_src, sizeof(ltb->edid_buf));
		kfree(edid_ptr_src);

		return 0;
	}

FALLBACK:
	return lt8619_read_edid_from_chip(ltb, ltb->edid_buf);
}

static ssize_t version_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;
	unsigned int version;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	lt8619_read_revision(ltb, &version);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", version);
}
static DEVICE_ATTR_RO(version);

static ssize_t on_chip_edid_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;
	unsigned int i;
	unsigned int pos;
	unsigned int dat;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	regmap_write(ltb->regmap, 0x808E, 0x07);
	pos = 0;
	for (i = 0; i < EDID_DATA_SIZE; i++) {
		regmap_write(ltb->regmap, 0x808F, i);
		regmap_read(ltb->regmap, 0x8090, &dat);

		if ((i % 16) == 0) {
			snprintf(&buf[pos], PAGE_SIZE, "\n");
			pos++;
		}

		snprintf(&buf[pos], PAGE_SIZE, "%02x ", dat);
		pos += 3;
	}

	snprintf(&buf[pos], PAGE_SIZE, "\n");
	pos++;

	regmap_write(ltb->regmap, 0x808E, 0x01);

	return pos;
}

static ssize_t on_chip_edid_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;
	int ret;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	ret = lt8619_download_edid_to_chip(ltb, buf, count);

	return ret;
}

static ssize_t simple_edid_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;
	unsigned int i;
	unsigned int pos;
	unsigned int dat;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	pos = 0;
	for (i = 0; i < EDID_DATA_SIZE; i++) {
		if ((i % 16) == 0) {
			snprintf(&buf[pos], PAGE_SIZE, "\n");
			pos++;
		}
		dat = LT8619_SIMPLE_EDID[1][i];
		snprintf(&buf[pos], PAGE_SIZE, "%02x ", dat);
		pos += 3;
	}

	snprintf(&buf[pos], PAGE_SIZE, "\n");
	pos++;

	return pos;
}

static ssize_t simple_edid_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lt8619_bridge *ltb;
	unsigned long row;
	int ret;

	ret = kstrtoul(buf, 0, &row);
	if (ret)
		return ret;

	DRM_DEV_DEBUG_DRIVER(dev, "row = %lu\n", row);

	if (row > ARRAY_SIZE(LT8619_SIMPLE_EDID))
		return ret;

	ltb = (struct lt8619_bridge *)i2c_get_clientdata(client);

	ret = lt8619_download_edid_to_chip(ltb,
					   LT8619_SIMPLE_EDID[row],
					   EDID_DATA_SIZE);
	return ret;
}

static DEVICE_ATTR_RW(on_chip_edid);
static DEVICE_ATTR_RW(simple_edid);

static int lt8619_sysfs_init(struct device *dev)
{
	int ret;

	ret = device_create_file(dev, &dev_attr_version);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_color_space);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_on_chip_edid);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_simple_edid);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_hdmi_clk_freq);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_hdmi_timings);
	if (ret)
		return ret;

	return 0;
}

static struct i2c_board_info lt8619_board_info = {
	.type = LT8619_CHIP_NAME,
	.addr = LT8619_CHIP_ADDR,
};

struct drm_bridge *lt8619_driver_init(struct drm_device *ddev,
				      struct i2c_adapter *adapter,
				      int index)
{
	struct lt8619_bridge *ltb;
	struct regmap *regmap;
	struct drm_bridge *bridge;
	struct i2c_client *client;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("%s: i2c_check_functionality() failed\n",
			  __func__);
		return NULL;
	}

	client = i2c_new_device(adapter, &lt8619_board_info);

	DRM_INFO("I2c client %s(i2c%d-0x%02x) created\n",
		 lt8619_board_info.type,
		 adapter->nr,
		 lt8619_board_info.addr);

	ltb = devm_kzalloc(ddev->dev, sizeof(*ltb), GFP_KERNEL);
	if (!ltb)
		return NULL;

	i2c_set_clientdata(client, ltb);

	regmap = devm_regmap_init_i2c(client, &lt8619_regmap_config);
	if (IS_ERR(regmap)) {
		DRM_ERROR("Failed to regmap: %d\n", ret);
		return NULL;
	}

	bridge = &ltb->bridge;
	bridge->funcs = &lt8619_bridge_funcs;

	ltb->regmap = regmap;
	ltb->client = client;
	ltb->adapter = adapter;

	lt8619_read_revision(ltb, &ltb->chip_id.i);

	lt8619_enable_sys_clk(ltb, true);

	lt8619_lvds_tx_clk_enable(ltb);

	/* read edid only once */
	lt8619_get_edid_once(ltb, ddev, index);

	ltb->bklt = lt8619_create_backlight(ddev,
					    LOONGSON_GPIO_LCD_VDD,
					    LOONGSON_GPIO_LCD_EN,
					    3);

	if (ltb->bklt == NULL)
		return NULL;

	lt8619_backlight_init(ltb->bklt, PWM_POLARITY_INVERSED, 500000);

	lt8619_sysfs_init(&client->dev);

	return bridge;
}

int bridge_phy_lt8619_init(struct bridge_resource *res)
{
	struct loongson_device *ldev = res->ldev;
	unsigned int index = res->display_pipe_index;
	struct loongson_i2c *li2c = &ldev->i2c_bus[index];
	struct drm_encoder *encoder = &res->ls_encoder->base;
	struct drm_bridge *bridge;
	int ret;

	bridge = lt8619_driver_init(ldev->dev, li2c->adapter, index);
	if (!bridge) {
		DRM_ERROR("Failed to create lt8619 bridge\n");
		return -EINVAL;
	}

	ret = drm_bridge_attach(encoder, bridge, NULL);
	if (ret)
		return ret;

	DRM_INFO("Encoder: lt8619 init finish\n");

	return 0;
}
