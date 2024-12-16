// SPDX-License-Identifier: GPL-2.0
/*
 * Authors:
 *           Sui Jingfeng <suijingfeng@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of_device.h>

#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>


#define LT8618_INT_ENABLE                      0x8146
#define LT8618_HOTPLUG_EVENT                   BIT(2)

#define LT8618_REG_START_ADDR                  0x8000
#define LT8618_REG_END_ADDR                    0x8585

#define LT8618_REG_PAGE_CONTROL                0xff

#define LT8618_REG_CHIP_ID                     0x8000
#define LT8618_CHIP_ID_LEN                     0x03

#define LT8618_REG_ENABLE                      0x80EE

#define LT8618_REG_MIPI_RX_SOFT_RESET          0x8011

#define LT8618_REG_HDMI_SOFT_RESET             0x8013

#define LT8618_REG_HPD_SRC_SEL                 0x8258

#define LT8618_REG_HPD_DEBOUNCE_WIDTH          0x8259

#define LT8618_REG_HPD_STATUS                  0x825E

#define LT8618_CEC_BIT                         BIT_MASK(3)
#define LT8618_HPD_BIT                         BIT_MASK(2)
#define LT8618_TX_AC_BIT                       BIT_MASK(1)
#define LT8618_TX_DC_BIT                       BIT_MASK(0)

#define LINK_STATUS_STABLE                     1U
#define LINK_STATUS_UNSTABLE                   0U

#define LT8618_REG_INT_CLR0                    0x8204
#define LT8618_REG_INT_CLR1                    0x8205
#define LT8618_REG_INT_CLR2                    0x8206
#define LT8618_REG_INT_CLR3                    0x8207

#define LT8618_REG_DDC_STATUS                  0x8540

#define DDC_ACCESS_STATUS_ARBITRATION_LOST     BIT_MASK(6)
#define DDC_ACCESS_STATUS_BUS_OCCUPIED         BIT_MASK(5)
#define DDC_ACCESS_STATUS_NO_ACK               BIT_MASK(4)
#define DDC_ACCESS_STATUS_IN_PROGRESS          BIT_MASK(2)
#define DDC_ACCESS_STATUS_DONE                 BIT_MASK(1)

#define LT8618_REG_EDID_OFFSET_ADDR            0x8505

#define LT8618_REG_EDID_NUM_BYTES_TO_READ      0x8506

#define LT8618_REG_EDID_READ_CTL               0x8507

#define LT8618_REG_EDID_STATUS                 0x8582
#define LT8618_REG_EDID_FIFO_DAT               0x8583

#define LT8618_FIFO_BUF_LEN                    32U

#define EDID_FIFO_FULL_BIT                     BIT_MASK(2)

#define EDID_SEG_SIZE                          256

#define LT8618_REG_HDMITX_OUTPUT_MODE          0x82D6

#define PHY_OUTPUT_MODE_HDMI                   BIT_MASK(7)
#define PHY_SYNC_POLARITY_HW                   BIT_MASK(1)
#define PHY_HSYNC_POLARITY_ACTIVE_HIGH         BIT_MASK(2)
#define PHY_VSYNC_POLARITY_ACTIVE_HIGH         BIT_MASK(3)

#define LT8618_REG_AVI_INFOFRAME_PACKET        0x8443

#define AVI_INFO_PACKET_TRANSMISSION_EN        BIT_MASK(3)
#define AUDIO_PACKET_TRANSMISSION_EN           BIT_MASK(1)

#define FS64_CLK_SRC_AD_SCLK                   BIT_MASK(2)

#define D_CLK_INVERT_EN                        BIT_MASK(6)


/******************** video timings ********************/
#define LT_VSYNC_POL_BIT                       BIT_MASK(1)
#define LT_HSYNC_POL_BIT                       BIT_MASK(0)


struct lt8618 {
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	bool power_on;
	bool ac_mode;
	bool dvi_output;
	bool is_cripple;
};

static inline struct lt8618 *bridge_to_lt8618(struct drm_bridge *brip)
{
	return container_of(brip, struct lt8618, bridge);
}

static inline struct lt8618 *connector_to_lt8618(struct drm_connector *con)
{
	return container_of(con, struct lt8618, connector);
}

static void lt8618_hw_reset(struct lt8618 *ltb)
{
	if (!ltb->reset_gpio)
		return;

	gpiod_set_value(ltb->reset_gpio, 1);

	/* The datasheet says treset-min = 100us. Make it 150us to be sure. */
	usleep_range(150, 200);

	gpiod_set_value(ltb->reset_gpio, 0);
}

static enum drm_connector_status
lt8618_connector_detect(struct drm_connector *con, bool force)
{
	struct lt8618 *ltb = connector_to_lt8618(con);
	enum drm_connector_status status = connector_status_unknown;

	if (ltb->is_cripple) {
		struct i2c_adapter *ddc = ltb->i2c->adapter;

		if (ddc) {
			if (drm_probe_ddc(ddc))
				status = connector_status_connected;
			else
				status = connector_status_disconnected;
		}

		status = connector_status_unknown;
	} else {
		unsigned int val;

		regmap_read(ltb->regmap, LT8618_REG_HPD_STATUS, &val);

		if (val & LT8618_HPD_BIT)
			status = connector_status_connected;
		else
			status = connector_status_disconnected;

	}

	return status;
}

static const struct drm_connector_funcs lt8618_connector_funcs = {
	.detect = lt8618_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int lt8618_config_hpd(struct lt8618 *ltb)
{
	int ret;
	unsigned int val;

	/*
	 *  0 0 0 0  1 0 1 0
	 *           ^   ^
	 *           |   `--- Ad_tx_hpd
	 *           |
	 *           `------- HDMI tx detect debounce enable
	 */

	ret = regmap_write(ltb->regmap, LT8618_REG_HPD_SRC_SEL, 0x0a);
	if (ret)
		return ret;

	/* hpd interrupt */
	regmap_read(ltb->regmap, 0x8203, &val);
	val &= ~0xc0;
	regmap_write(ltb->regmap, 0x8203, val);
	regmap_write(ltb->regmap, 0x8207, 0xff); /* clear */
	regmap_write(ltb->regmap, 0x8207, 0x3f);

	return 0;
}


static int lt8618_read_edid_block(void *data,
				  unsigned char *buf,
				  unsigned int block,
				  size_t len)
{
	struct lt8618 *ltb = data;
	unsigned char edid_block[EDID_LENGTH];
	unsigned int base = block * EDID_LENGTH;
	int ret = 0;
	unsigned int num_of_batch;
	unsigned int n;

	/* len = 128, 128/32 = 4, so no need to round */
	num_of_batch = (len + LT8618_FIFO_BUF_LEN - 1) / LT8618_FIFO_BUF_LEN;

	regmap_write(ltb->regmap, 0x8502, 0x0a);

	/* 1:  Enable glitch filtering for SCL and SDA of the I2C master
	 * 1:  Set to enable I2C master to reset FSM to be free of lock state
	 *      due to bus activity time-out.
	 * 0:  Set to enable I2C master not to send STOP condition before
	 *      repeated START condition when continuous access operation
	 *      is performing
	 *
	 * 0:  Set to enable ignoring ACK from I2C slave during access
	 * 1:  One access per command
	 * 0:  Continuous access as long as the command is not changed
	 * 0:  Software-controlled key-loading procedure
	 * 1:  Enable key-loading procedure
	 */
	regmap_write(ltb->regmap, 0x8503, 0xc8);

	regmap_write(ltb->regmap, 0x8504, 0xa0);

	/* HDCP related */
	regmap_write(ltb->regmap, 0x8514, 0x7f);

	for (n = 0; n < num_of_batch; n++) {
		unsigned int status;
		unsigned int offset = n * LT8618_FIFO_BUF_LEN;

		/* update the offset address into the total edid data */
		regmap_write(ltb->regmap,
			     LT8618_REG_EDID_OFFSET_ADDR,
			     base + offset);

		/* number of the edid can be read once a time (32 byte) */
		regmap_write(ltb->regmap,
			     LT8618_REG_EDID_NUM_BYTES_TO_READ,
			     LT8618_FIFO_BUF_LEN);

		regmap_write(ltb->regmap,
			     LT8618_REG_EDID_READ_CTL,
			     0x36); /* Reset I2C master */

		/* 0x34: Enhanced DDC sequential read */
		regmap_write(ltb->regmap,
			     LT8618_REG_EDID_READ_CTL,
			     0x34); /* Sequential byte read, 0x39 ? */

		regmap_write(ltb->regmap,
			     LT8618_REG_EDID_READ_CTL,
			     0x37); /* None, and wait, 0x3f ? */

DDC_RETRY:

		regmap_read(ltb->regmap, LT8618_REG_DDC_STATUS, &status);

		/* Indicates that key-loading procedure is done */
		if (status & DDC_ACCESS_STATUS_DONE) {
			unsigned int i;

			for (i = 0; i < LT8618_FIFO_BUF_LEN; i++) {
				unsigned int dat;

				regmap_read(ltb->regmap,
					    LT8618_REG_EDID_FIFO_DAT,
					    &dat);

				edid_block[offset + i] = dat;
			}
		} else if (status & DDC_ACCESS_STATUS_IN_PROGRESS) {
			DRM_INFO("TX DDC is in using, delay and retry\n");
			goto DDC_RETRY;
		} else if (status & DDC_ACCESS_STATUS_NO_ACK) {
			DRM_ERROR("No acknowledge is received\n");
			ret = -EIO;
			goto E_RAGE_QUIT;
		} else if (status & DDC_ACCESS_STATUS_BUS_OCCUPIED) {
			DRM_ERROR("Bus is occupied during TX DDC\n");
			ret = -EIO;
			goto E_RAGE_QUIT;
		} else {
			DRM_ERROR("Unknown error\n");
			ret = -EIO;
			goto E_RAGE_QUIT;
		}
	}

	memcpy(buf, edid_block, len);

E_RAGE_QUIT:
	/* disable edid reading and send NONE CMD */
	regmap_write(ltb->regmap, LT8618_REG_EDID_READ_CTL, 0x1f);

	return ret;
}

static int lt8618_get_modes(struct drm_connector *connector)
{
	struct lt8618 *ltb = connector_to_lt8618(connector);
	unsigned int count = 0;
	struct edid *edid;

	edid = drm_do_get_edid(connector, lt8618_read_edid_block, ltb);

	if (edid) {
		drm_connector_update_edid_property(connector, edid);

		count = drm_add_edid_modes(connector, edid);

		kfree(edid);

		DRM_DEBUG_KMS("%s: get %u modes\n", __func__, count);
	}

	return count;
}

struct lt8618_mode {
	u16 hdisplay;
	u16 vdisplay;
	u8 vrefresh;
};

static const struct lt8618_mode lt8618_supported_modes[] = {
	{ 1920, 1080, 75 },
	{ 1920, 1080, 60 },
	{ 1680, 1050, 60 },
	{ 1600, 900, 60 },
	{ 1400, 1050, 60 },
	{ 1440, 900, 60 },
	{ 1280, 1024, 60 },
	{ 1280, 800, 60 },
	{ 1280, 720, 60 },
	{ 1152, 864, 60 },
	{ 1024, 768, 60 },
	{ 800, 600, 60 },
	{ 720, 480, 60 },
	{ 640, 480, 60 },
};

const static struct lt8618_mode *
lt8618_find_mode(const struct drm_display_mode *mode)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(lt8618_supported_modes); ++i) {
		if (lt8618_supported_modes[i].hdisplay == mode->hdisplay &&
		    lt8618_supported_modes[i].vdisplay == mode->vdisplay) {
			return &lt8618_supported_modes[i];
		}
	}

	return NULL;
}

static enum drm_mode_status lt8618_mode_valid(struct drm_connector *connector,
					      struct drm_display_mode *mode)
{
	const struct lt8618_mode *ltm = lt8618_find_mode(mode);

	if (!ltm) {
		DRM_DEBUG_KMS("%s is not supported: %dx%d@%dHz\n",
			      mode->name,
			      mode->hdisplay,
			      mode->vdisplay,
			      mode->vrefresh);
		return MODE_BAD;
	}

	return MODE_OK;
}

static const struct drm_connector_helper_funcs lt8618_conn_helper_funcs = {
	.get_modes = lt8618_get_modes,
	.mode_valid = lt8618_mode_valid,
};

static void lt8618_bridge_disable(struct drm_bridge *bridge)
{
	struct lt8618 *ltb = bridge_to_lt8618(bridge);

	regmap_write(ltb->regmap, 0x8130, 0x00);

	/* TODO: power down */
}

static void lt8618_config_frequency_meter_timer(struct lt8618 *ltb)
{
	/* Frequency meter 2 timer cycle for sys_clk
	 * 0x61A8 = 25000, default
	 * 0x77EC = 30700
	 */
	regmap_write(ltb->regmap, 0x821B, 0x77);
	regmap_write(ltb->regmap, 0x821C, 0xEC);
}

static void lt8618_config_analog_front_end(struct lt8618 *ltb)
{
	/* port A rgb: Power down, LDO disabled */
	/* port B rgb mode enable, LDO enabled,  */
	regmap_write(ltb->regmap, 0x8102, 0x66); /* 0x46 ? */
	/* disable mipi rx port A clk to lane0, lane1, lane2, lane3  */
	regmap_write(ltb->regmap, 0x810A, 0x06);
	/* disable mipi rx port B clk to lane0, lane1, lane2, lane3 */
	regmap_write(ltb->regmap, 0x8115, 0x06);

	/* Select HS as DE input, for U3 */
	regmap_write(ltb->regmap, 0x814E, 0xA9);
}

static void lt8618_config_hdmi_phy(struct lt8618 *ltb)
{
	/* HDMI_TX_Phy
	 *
	 * HDMI TX Phy clk channel enable
	 * HDMI TX Phy D0 channel tap0 enable
	 * HDMI TX Phy D0 channel tap1 enable
	 * HDMI TX Phy D1 channel tap0 enable
	 * HDMI TX Phy D2 channel tap0 enable
	 */

	/* DC mode */
	struct reg_sequence reg_cfg[] = {
		{ 0x8130, 0x6a },
		{ 0x8131, 0x44 }, /* HDMI DC mode */
		{ 0x8132, 0x4a },
		{ 0x8133, 0x0b },
		{ 0x8134, 0x00 },
		{ 0x8135, 0x00 },
		{ 0x8136, 0x00 },
		{ 0x8137, 0x44 },
		{ 0x813f, 0x0f },
		{ 0x8140, 0xa0 },
		{ 0x8141, 0xa0 },
		{ 0x8142, 0xa0 },
		{ 0x8143, 0xa0 },
		{ 0x8144, 0x0a },
	};

	/* HDMI AC mode */
	if (ltb->ac_mode)
		reg_cfg[2].def = 0x73;

	regmap_multi_reg_write(ltb->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));

	/* Enable control of AVI packet transmission in every VBLANK period
	 * Enable control of Audio INfoFrame packet transmission
	 * in every VBLANK period
	 */
	regmap_write(ltb->regmap, 0x843D,
			AVI_INFO_PACKET_TRANSMISSION_EN |
			AUDIO_PACKET_TRANSMISSION_EN); /* 0x0a */
}

static int lt8618_config_hdmi_avi_infoframe(struct lt8618 *ltb,
					struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	unsigned char buf[HDMI_INFOFRAME_SIZE(AVI)];
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame, mode, false);
	if (ret < 0) {
		DRM_ERROR("couldn't fill AVI infoframe\n");
		return ret;
	}

	ret = hdmi_avi_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		DRM_ERROR("failed to pack AVI infoframe: %d\n", ret);
		return ret;
	}

	/* Do not send the infoframe header, but keep the CRC field. */
	ret = regmap_bulk_write(ltb->regmap,
				LT8618_REG_AVI_INFOFRAME_PACKET,
				buf + HDMI_INFOFRAME_HEADER_SIZE - 1,
				HDMI_AVI_INFOFRAME_SIZE + 1);
	if (ret < 0) {
		DRM_ERROR("failed to write AVI infoframe to reg: %d\n", ret);
		return ret;
	}

	return 0;
}

static int lt8618_config_hdmi_csc(struct lt8618 *ltb)
{
	return regmap_write(ltb->regmap, 0x82B9, 0x00); /* no CSC */
}

static int lt8618_video_check(struct lt8618 *ltb)
{
	unsigned int vs_pol, hs_pol;
	unsigned int vs_width, hs_width;
	unsigned int v_bp, v_fp, h_bp, h_fp;
	unsigned int h_active, h_total, v_active, v_total;

	unsigned int pix_clk;
	unsigned int reg_val;

	regmap_read(ltb->regmap, 0x8270, &reg_val);

	/* pix_clk is output */
	if (reg_val & LT_HSYNC_POL_BIT)
		hs_pol = 1;

	if (reg_val & LT_VSYNC_POL_BIT)
		vs_pol = 1;

	regmap_read(ltb->regmap, 0x8271, &vs_width);

	regmap_read(ltb->regmap, 0x8272, &reg_val);
	hs_width = reg_val << 8;
	regmap_read(ltb->regmap, 0x8273, &reg_val);
	hs_width |= reg_val;

	/* vertical back porch */
	regmap_read(ltb->regmap, 0x8274, &v_bp);

	/* vertical front porch */
	regmap_read(ltb->regmap, 0x8275, &v_fp);

	/* horizontal back porch */
	regmap_read(ltb->regmap, 0x8276, &reg_val);
	h_bp = reg_val << 8;
	regmap_read(ltb->regmap, 0x8277, &reg_val);
	h_bp |= reg_val;

	/* horizontal front porch */
	regmap_read(ltb->regmap, 0x8278, &reg_val);
	h_fp = reg_val << 8;
	regmap_read(ltb->regmap, 0x8279, &reg_val);
	h_fp |= reg_val;

	/* number of vertical total line */
	regmap_read(ltb->regmap, 0x827A, &reg_val);
	v_total = reg_val << 8;
	regmap_read(ltb->regmap, 0x827B, &reg_val);
	v_total |= reg_val;

	/* number of horizontal total pixel */
	regmap_read(ltb->regmap, 0x827C, &reg_val);
	h_total = reg_val << 8;
	regmap_read(ltb->regmap, 0x827D, &reg_val);
	h_total |= reg_val;

	/* number of vertical active line */
	regmap_read(ltb->regmap, 0x827E, &reg_val);
	v_active = reg_val << 8;
	regmap_read(ltb->regmap, 0x827F, &reg_val);
	v_active |= reg_val;

	/* number of horizontal active line */
	regmap_read(ltb->regmap, 0x8280, &reg_val);
	h_active = reg_val << 8;
	regmap_read(ltb->regmap, 0x8281, &reg_val);
	h_active |= reg_val;


	/* Frequency meter 2: output frequency of fm2_det_clk */
	regmap_read(ltb->regmap, 0x821D, &reg_val);
	pix_clk = (reg_val & 0x0F) << 16;
	regmap_read(ltb->regmap, 0x821E, &reg_val);
	pix_clk |= reg_val << 8;
	regmap_read(ltb->regmap, 0x821F, &reg_val);
	pix_clk |= reg_val;

	DRM_INFO("output frequency: pix_clk=%ukHz\n", pix_clk);

	DRM_INFO("h_active=%u, h_total=%u, h_bp=%u, h_fp=%u, hs_width=%u\n",
		 h_active, h_total, h_bp, h_fp, hs_width);

	DRM_INFO("v_active=%u, v_total=%u, v_bp=%u, v_fp=%u, vs_width=%u\n",
		 v_active, v_total, v_bp, v_fp, vs_width);

	DRM_INFO("vsync polarity: %s\n",
		vs_pol ? "Active High" : "Active Low");

	DRM_INFO("hsync polarity: %s\n",
		vs_pol ? "Active High" : "Active Low");

	return 0;
}

static int lt8618_config_video_input(struct lt8618 *ltb)
{
	const struct reg_sequence video_input_param[] = {
		/* TTL video process control logic clock enable */
		{ 0x800A, 0x80 },
		/* RGB channel order, RGB888 */
		{ 0x8244, 0x00 },
		{ 0x8245, 0x70 },
		/* enable Hsync, Vsync polarity software control */
		{ 0x8247, 0x07 },
		/* Digital d_clk source ad_ttl_d_clk, d_clk inverted if 0x40 */
		{ 0x824F, 0x40 },
		/* Select TTL video data as HDMI video input source */
		{ 0x8250, 0x00 },
	};

	regmap_multi_reg_write(ltb->regmap, video_input_param,
			       ARRAY_SIZE(video_input_param));

	return 0;
}

static int lt8618_soft_reset(struct lt8618 *ltb)
{
	/* Reset the MIPI RX logic, writing to it shouldn't be hurt */
	regmap_write(ltb->regmap, LT8618_REG_MIPI_RX_SOFT_RESET, 0x00);

	/* TTL video process control logic soft reset */
	regmap_write(ltb->regmap, LT8618_REG_HDMI_SOFT_RESET, 0xF1);
	regmap_write(ltb->regmap, LT8618_REG_HDMI_SOFT_RESET, 0xF9);

	return 0;
}

static void lt8618_pre_enable(struct lt8618 *ltb)
{
	regmap_write(ltb->regmap, LT8618_REG_ENABLE, 0x01);

	lt8618_soft_reset(ltb);

	lt8618_config_analog_front_end(ltb);

	lt8618_config_frequency_meter_timer(ltb);

	lt8618_config_video_input(ltb);

	lt8618_config_hdmi_phy(ltb);

	lt8618_config_hpd(ltb);

	lt8618_config_hdmi_csc(ltb);
}

/*
 * This callback should enable the bridge. It is called right after
 * the preceding element in the display pipe is enabled. If the
 * preceding element is a bridge this means it's called after that
 * bridge's @enable function. If the preceding element is a
 * &drm_encoder it's called right after the encoder's
 * &drm_encoder_helper_funcs.enable, &drm_encoder_helper_funcs.commit or
 * &drm_encoder_helper_funcs.dpms hook.
 *
 * The bridge can assume that the display pipe (i.e. clocks and timing
 * signals) feeding it is running when this callback is called. This
 * callback must enable the display link feeding the next bridge in the
 * chain if there is one.
 */
static void lt8618_bridge_enable(struct drm_bridge *bridge)
{
	struct lt8618 *ltb = bridge_to_lt8618(bridge);
	struct device *dev = &ltb->i2c->dev;
	unsigned int output_mode;

	/*
	 *  Set output mode to HDMI
	 *
	 *  Vsync polarity : Active high
	 *  Hsync polarity : Active high
	 *  Output sync polarity setting : software mode ?
	 */

	output_mode = PHY_SYNC_POLARITY_HW |
		      PHY_HSYNC_POLARITY_ACTIVE_HIGH |
		      PHY_VSYNC_POLARITY_ACTIVE_HIGH;

	if (ltb->dvi_output == false)
		output_mode |= PHY_OUTPUT_MODE_HDMI;

	regmap_write(ltb->regmap, LT8618_REG_HDMITX_OUTPUT_MODE, output_mode);

	/*
	 * This function only for informative purpose
	 */

	if (0)
		lt8618_video_check(ltb);

	/* Enable HDMI output */
	regmap_write(ltb->regmap, 0x8130, 0xea);

	dev_dbg(dev, "%s\n", ltb->dvi_output ? "DVI" : "HDMI");
}

static void lt8618_config_from_pclk(struct lt8618 *ltb, unsigned int pclk)
{
	struct device *dev = &ltb->i2c->dev;
	static const unsigned char RGB888_SDR_PLL_TABLE[12][3] = {
		{ 0x00, 0xa8, 0xbb }, /* < 50MHz   */
		{ 0x00, 0x94, 0xaa }, /* 50 ~ 59MHz  */
		{ 0x01, 0xa8, 0xaa }, /* 60 ~ 89MHz  */
		{ 0x02, 0xbc, 0xaa }, /* 90 ~ 99MHz  */
		{ 0x02, 0x9e, 0x99 }, /* 100 ~ 119MHz */
		{ 0x03, 0xa8, 0x99 }, /* 120 - 149MHz  1080p@60Hz */
		{ 0x05, 0x9e, 0x88 }, /* 150 - 179MHz  1080p@75Hz */
		{ 0x05, 0xbc, 0x99 }, /* 180 - 189MHz  */

		{ 0x05, 0x9e, 0x88 }, /* 200 - 209MHz  */
		{ 0x06, 0xa3, 0x88 }, /* 210 - 239MHz  */
		{ 0x07, 0xa8, 0x88 }, /* 240 - 269MHz  */
		{ 0x08, 0xad, 0x88 }, /* >= 270 MHz */
	};
	unsigned int index;

	if (pclk < 50000)
		index = 0;
	else if (pclk < 59999)
		index = 1;
	else if (pclk < 89999)
		index = 2;
	else if (pclk < 99999)
		index = 3;
	else if (pclk < 119999)
		index = 4;
	else if (pclk < 149999)
		index = 5;
	else if (pclk < 179999)
		index = 6;
	else if (pclk < 189999)
		index = 7;
	else if (pclk < 209999)
		index = 8;
	else if (pclk < 239999)
		index = 9;
	else if (pclk < 269999)
		index = 10;
	else
		index = 11;

	regmap_write(ltb->regmap, 0x8125, RGB888_SDR_PLL_TABLE[index][0]);
	regmap_write(ltb->regmap, 0x812C, RGB888_SDR_PLL_TABLE[index][1]);
	regmap_write(ltb->regmap, 0x812D, RGB888_SDR_PLL_TABLE[index][2]);

	dev_dbg(dev, "pclk=%u\n", pclk);
}


static int lt8618_config_hdmi_tx_pll(struct lt8618 *ltb,
				     struct drm_display_mode *mode)
{
	unsigned int pclk = mode->clock;

	const struct reg_sequence pll_cfg[] = {
		/* txpll init */
		{ 0x8123, 0x40 },  /* power up and enable TXPLL vreg block */
		{ 0x8124, 0x64 },
		{ 0x8125, 0x03 },  /* RGB input d_clk, PREDIV=6 */
		{ 0x8126, 0x55 },  /* low pass filter, R = 21K, C = 1.54pF */
		{ 0x8127, 0x06 },  /* jitter cleaning and phase selection */
		{ 0x8128, 0x00 },  /* From RGB input d_clk */
		{ 0x8129, 0x04 },  /* TTL DCLK, TTL input use this */
		{ 0x812A, 0x20 },  /* U3 SDR/DDR fixed phase */
		{ 0x812E, 0x01 },
		{ 0x812F, 0x00 },
		{ 0x814D, 0x00 },  /* Normal polarity, SDR mode,  */
	};

	regmap_multi_reg_write(ltb->regmap, pll_cfg, ARRAY_SIZE(pll_cfg));

	lt8618_config_from_pclk(ltb, pclk);

	/*
	 * The following registers is need to be reconfigured
	 * As long as the resolution or input clock changed
	 */
	regmap_write(ltb->regmap, 0x81DE, 0x00);
	/* TXPLL calibration digital control logic enable */
	/* TXPLL calibration frequency meter enable for TTL */
	regmap_write(ltb->regmap, 0x81DE, 0xc0);

	/* TTL(RGB888) input control logic soft reset */
	regmap_write(ltb->regmap, 0x8016, 0xf1);
	regmap_write(ltb->regmap, 0x8016, 0xf3);

	return 0;
};

static void lt8618_config_display_timing(struct lt8618 *ltb,
				const struct drm_display_mode *mode)
{
	struct device *dev = &ltb->i2c->dev;
	unsigned int h_total;
	unsigned int h_active, h_sync_len, h_front_porch, h_back_porch;
	unsigned int v_total;
	unsigned int v_active, v_sync_len, v_front_porch, v_back_porch;

	h_total = mode->htotal;
	v_total = mode->vtotal;

	h_active = mode->hdisplay;
	h_sync_len = mode->hsync_end - mode->hsync_start;
	h_front_porch = mode->hsync_start - mode->hdisplay;
	h_back_porch = mode->htotal - mode->hsync_end;

	v_active = mode->vdisplay;
	v_sync_len = mode->vsync_end - mode->vsync_start;
	v_front_porch = mode->vsync_start - mode->vdisplay;
	v_back_porch = mode->vtotal - mode->vsync_end;


	/* horizontal total pixel number */
	regmap_write(ltb->regmap, 0x822C, (h_total >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x822D, h_total & 0xFF);

	/* horizontal active pixel */
	regmap_write(ltb->regmap, 0x822E, (h_active >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x822F, h_active & 0xFF);

	/* horizontal front porch */
	regmap_write(ltb->regmap, 0x8230, (h_front_porch >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x8231, h_front_porch & 0xFF);

	/* horizontal back porch */
	regmap_write(ltb->regmap, 0x8232, (h_back_porch >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x8233, h_back_porch & 0xFF);

	/* horizontal sync length */
	regmap_write(ltb->regmap, 0x8234, (h_sync_len >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x8235, h_sync_len & 0xFF);


	/* vertical active pixel */
	regmap_write(ltb->regmap, 0x8236, (v_active >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x8237, v_active & 0xFF);

	/* vertical front porch */
	regmap_write(ltb->regmap, 0x8238, (v_front_porch >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x8239, v_front_porch & 0xFF);

	/* vertical back porch */
	regmap_write(ltb->regmap, 0x823A, (v_back_porch >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x823B, v_back_porch & 0xFF);

	/* vertical sync length */
	regmap_write(ltb->regmap, 0x823C, (v_sync_len >> 8) & 0xFF);
	regmap_write(ltb->regmap, 0x823D, v_sync_len & 0xFF);

	dev_info(dev, "setting mode: %ux%u\n", h_active, v_active);
}


/*
 * The adj_mode parameter is the mode output by the CRTC for the
 * first bridge in the chain. It can be different from the mode
 * parameter that contains the desired mode for the connector at the end
 * of the bridges chain, for instance when the first bridge in the chain
 * performs scaling. The adjusted mode is mostly useful for the first
 * bridge in the chain and is likely irrelevant for the other bridges.
 */
static void lt8618_bridge_mode_set(struct drm_bridge *bridge,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adj_mode)
{
	struct lt8618 *ltb = bridge_to_lt8618(bridge);

	lt8618_config_display_timing(ltb, adj_mode);

	lt8618_config_hdmi_tx_pll(ltb, adj_mode);

	lt8618_config_hdmi_avi_infoframe(ltb, adj_mode);
}

static int lt8618_bridge_attach(struct drm_bridge *bridge)
{
	struct lt8618 *ltb = bridge_to_lt8618(bridge);
	struct drm_device *ddev = bridge->dev;

	int ret;

	drm_connector_helper_add(&ltb->connector,
				 &lt8618_conn_helper_funcs);

	if (!drm_core_check_feature(ddev, DRIVER_ATOMIC)) {
		dev_err(&ltb->i2c->dev,
			"Compatible with atomic updates only\n");
		return -ENOTSUPP;
	}

	ret = drm_connector_init(ddev,
				 &ltb->connector,
				 &lt8618_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);

	if (ret)
		return ret;

	if (ltb->i2c->irq > 0) {
		ltb->connector.polled = DRM_CONNECTOR_POLL_HPD;
		DRM_INFO("%s: irq = %d\n", __func__, ltb->i2c->irq);
	} else
		ltb->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
					DRM_CONNECTOR_POLL_DISCONNECT;

	ret = drm_connector_attach_encoder(&ltb->connector, bridge->encoder);

	return ret;
}

static const struct drm_bridge_funcs lt8618_bridge_funcs = {
	.attach = lt8618_bridge_attach,
	.mode_set = lt8618_bridge_mode_set,
	.disable = lt8618_bridge_disable,
	.enable = lt8618_bridge_enable,
};

static const struct regmap_range lt8618_volatile_ranges[] = {
	regmap_reg_range(LT8618_REG_START_ADDR, LT8618_REG_END_ADDR),
};

static const struct regmap_range_cfg lt8618_ranges[] = {
	{
		.name = "lt8618_register_range",
		.range_min = 0,
		.range_max = 0x85ff,
		.window_start = 0,
		.window_len = 0x100,  /* 256 */
		.selector_reg = LT8618_REG_PAGE_CONTROL,
		.selector_mask = 0xFF,
		.selector_shift = 0,
	},
};

static const struct regmap_config lt8618_regmap_config = {
	.reg_bits = 8,
	.reg_stride = 1,
	.val_bits = 8,
	.max_register = 0x85ff,
	.ranges = lt8618_ranges,
	.num_ranges = ARRAY_SIZE(lt8618_ranges),
	.cache_type = REGCACHE_NONE,
};

static irqreturn_t lt8618_interrupt_handler(int irq, void *data)
{
	struct lt8618 *ltb = data;
	unsigned int status = 0;

	regmap_read(ltb->regmap, LT8618_REG_HPD_STATUS, &status);

	if ((status & LT8618_HOTPLUG_EVENT) && ltb->bridge.dev)
		drm_helper_hpd_irq_event(ltb->bridge.dev);

	return IRQ_HANDLED;
}

static int lt8618_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lt8618 *ltb;
	u8 chipid[3];
	int ret;

	ltb = devm_kzalloc(dev, sizeof(*ltb), GFP_KERNEL);
	if (!ltb)
		return -ENOMEM;

	ltb->i2c = client;
	ltb->regmap = devm_regmap_init_i2c(client, &lt8618_regmap_config);
	if (IS_ERR(ltb->regmap))
		return PTR_ERR(ltb->regmap);

	/* reset */
	ltb->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ltb->reset_gpio)) {
		dev_err(dev, "Failed to retrieve/request reset gpio: %ld\n",
			PTR_ERR(ltb->reset_gpio));
		return PTR_ERR(ltb->reset_gpio);
	}

	ltb->is_cripple = of_property_read_bool(dev->of_node,
						"lontium,cripple");
	dev_info(dev, "usage: is cripple ? %s\n",
			ltb->is_cripple ? "Yes" : "No");

	ltb->ac_mode = of_property_read_bool(dev->of_node, "lontium,ac-mode");

	dev_info(dev, "ac mode ? %s\n",
			ltb->ac_mode ? "Yes" : "No");

	lt8618_hw_reset(ltb);

	/* pre enable */
	lt8618_pre_enable(ltb);

	ret = regmap_bulk_read(ltb->regmap, LT8618_REG_CHIP_ID, &chipid, 3);
	if (ret) {
		dev_err(dev, "regmap_read failed %d\n", ret);
		return ret;
	}

	if ((chipid[0] != 0x17) || (chipid[1] != 0x02)) {
		dev_err(dev, "Invalid chipid: %02x %02x\n",
			chipid[0], chipid[1]);
		return -EINVAL;
	}

	if (chipid[2] == 0xE0)
		dev_info(dev, "LT8618_VER_U0\n");
	else if (chipid[2] == 0xE1)
		dev_info(dev, "LT8618_VER_U1\n");
	else if (chipid[2] == 0xE2)
		dev_info(dev, "LT8618_VER_U2\n");

	/* Clear all pending interrupts */
	regmap_write(ltb->regmap, LT8618_REG_INT_CLR0, 0xFF);
	regmap_write(ltb->regmap, LT8618_REG_INT_CLR1, 0xFF);
	regmap_write(ltb->regmap, LT8618_REG_INT_CLR2, 0xFF);
	regmap_write(ltb->regmap, LT8618_REG_INT_CLR3, 0xFF);

	regmap_write(ltb->regmap, 0x829E, 0x3F);
	regmap_write(ltb->regmap, 0x829F, 0xFF);

	if (client->irq > 0) {
		dev_info(dev, "%s: irq = %d\n", __func__, client->irq);

		regmap_write(ltb->regmap, LT8618_INT_ENABLE, 0xAB);

		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						lt8618_interrupt_handler,
						IRQF_ONESHOT, dev_name(dev),
						ltb);
		if (ret)
			return ret;
	}

	ltb->bridge.funcs = &lt8618_bridge_funcs;
	ltb->bridge.of_node = dev->of_node;

	i2c_set_clientdata(client, ltb);

	drm_bridge_add(&ltb->bridge);

	return 0;
}

static int lt8618_remove(struct i2c_client *client)
{
	struct lt8618 *ltb = i2c_get_clientdata(client);

	disable_irq(client->irq);

	drm_bridge_remove(&ltb->bridge);

	return 0;
}

static struct i2c_device_id lt8618_id[] = {
	{ "lontium,lt8618", 0 },
	{}
};

static const struct of_device_id lt8618_dt_ids[] = {
	{ .compatible = "lontium,lt8618", },
	{ }
};
MODULE_DEVICE_TABLE(of, lt8618_dt_ids);

static struct i2c_driver lt8618xsb_driver = {
	.probe = lt8618_probe,
	.remove = lt8618_remove,
	.id_table = lt8618_id,
	.driver = {
		.name = "lt8618",
		.of_match_table = lt8618_dt_ids,
	},
};
module_i2c_driver(lt8618xsb_driver);

MODULE_AUTHOR("Sui Jingfeng <suijingfeng@loongson.cn>");
MODULE_DESCRIPTION("LT8618SXB RGB to HDMI bridges");
MODULE_LICENSE("GPL");
