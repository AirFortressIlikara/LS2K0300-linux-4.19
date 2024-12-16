/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __BRIDGE_PHY_H__
#define __BRIDGE_PHY_H__

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/hdmi.h>
#include <drm/drm_edid.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <video/videomode.h>
#include "../loongson_drv.h"
#include "display_timing.h"

/* Note: After the vbios1.1 version, try not to used
 * XXX_INTERNAL_DVO and XXX_INTERNAL_HDMI.
 */
/* MISC: 0x00~-0x0f */
#define ENCODER_OBJECT_ID_NONE 0x00
#define ENCODER_OBJECT_ID_INTERNAL_DVO 0x01
#define ENCODER_OBJECT_ID_INTERNAL_HDMI 0x02
/*  VGA: 0x10~-0x1f */
#define ENCODER_OBJECT_ID_VGA_CH7055 0x10
#define ENCODER_OBJECT_ID_VGA_ADV7125 0x11
#define ENCODER_OBJECT_ID_VGA_TRANSPARENT 0x1F
/*  DVI: 0x20~-0x2f */
#define ENCODER_OBJECT_ID_DVI_TFP410 0x20
#define ENCODER_OBJECT_ID_DVI_TRANSPARENT 0x2f
/* HDMI: 0x30~-0x3f */
#define ENCODER_OBJECT_ID_HDMI_IT66121 0x30
#define ENCODER_OBJECT_ID_HDMI_SIL9022 0x31
#define ENCODER_OBJECT_ID_HDMI_LT8618 0x32
#define ENCODER_OBJECT_ID_HDMI_MS7210 0x33
#define ENCODER_OBJECT_ID_HDMI_TRANSPARENT 0x3F
/*  EDP: 0x40~-0x4f */
#define ENCODER_OBJECT_ID_EDP_NCS8805 0x40
#define ENCODER_OBJECT_ID_EDP_LT9721 0x42
#define ENCODER_OBJECT_ID_EDP_LT6711 0x43
#define ENCODER_OBJECT_ID_EDP_TRANSPARENT 0x4F
/*  HDMI to LVDS */
#define ENCODER_OBJECT_ID_HDMI_TO_LVDS_LT8619 0x50
#define ENCODER_OBJECT_ID_MAX 0xFF

#define TIMING_MAX 200U
#define PHY_IT66121_TIMING_MAX 120U
#define PHY_LT8618_TIMING_MAX 146U
#define PHY_NCS8805_TIMING_MAX 32U

#define NAME_SIZE_MAX 50U
#define DDC_FIFO_SIZE_MAX 32U

#define LS7A_GPIO_OFFSET 16U

#define to_dev(phy) (regmap_get_device(phy->phy_regmap))
#define to_bridge_phy(drm_bridge)                                              \
	container_of(drm_bridge, struct bridge_phy, bridge)
#define connector_to_bridge_phy(drm_connector)                                 \
	container_of(drm_connector, struct bridge_phy, connector)

#define __SIMPLE_REG_SEQ_FUNC(NAME, CMD) NAME##_##CMD
#define _SIMPLE_REG_SEQ_FUNC(NAME, CMD) __SIMPLE_REG_SEQ_FUNC(NAME, CMD)
#define SIMPLE_REG_SEQ_FUNC(CMD) _SIMPLE_REG_SEQ_FUNC(CHIP_PREFIX, CMD)

#define __DEFINE_SIMPLE_REG_SEQ_FUNC(NAME, CMD)                                \
	static int NAME##_##CMD(struct bridge_phy *phy)                        \
	{                                                                      \
		regmap_multi_reg_write(phy->phy_regmap, NAME##_##CMD##_seq,    \
				       ARRAY_SIZE(NAME##_##CMD##_seq));        \
		return 0;                                                      \
	}
#define _DEFINE_SIMPLE_REG_SEQ_FUNC(NAME, CMD)                                 \
	__DEFINE_SIMPLE_REG_SEQ_FUNC(NAME, CMD)
#define DEFINE_SIMPLE_REG_SEQ_FUNC(CMD)                                        \
	_DEFINE_SIMPLE_REG_SEQ_FUNC(CHIP_PREFIX, CMD)

struct bridge_phy;
struct bridge_resource;

struct reg_mask_seq {
	unsigned int reg;
	unsigned int mask;
	unsigned int val;
};

#define FEATURE_BIT_MAX 2

enum bridge_phy_feature {
	SUPPORT_HPD = BIT(0),
	SUPPORT_DDC = BIT(1),
	SUPPORT_HDMI_AUX = BIT(2),
};

static const char *const feature_str[] = {
	[SUPPORT_HPD] = "HPD",
	[SUPPORT_DDC] = "DDC",
	[SUPPORT_HDMI_AUX] = "HDMI_AUX",
};

enum hdmi_mode {
	HDMI_MODE_NORMAL = 0,
	HDMI_MODE_DVI,
};

enum input_video_type {
	RGB_WITH_SYNC_DE = 0,
	RGB_WITHOUT_SYNC,
	RGB_WITHOUT_DE,
};

enum input_data_lane_seq {
	DATA_LANE_SEQ_BGR = 0,
	DATA_LANE_SEQ_BRG = 3,
	DATA_LANE_SEQ_GBR,
	DATA_LANE_SEQ_GRB,
	DATA_LANE_SEQ_RBG,
	DATA_LANE_SEQ_RGB,
};

enum latch_clock_type {
	FULL_PERIOD,
	HALF_PERIOD,
};

enum input_signal_sample_type {
	SDR_CLK = 0,
	DDR_CLK,
};

enum color_space_convert {
	CSC_NONE,
	CSC_RGB2YUV,
	CSC_YUV2RGB,
};

enum hpd_status {
	hpd_status_plug_off = 0,
	hpd_status_plug_on = 1,
};

enum int_type {
	interrupt_all = 0,
	interrupt_hpd = 1,
	interrupt_max = 0xff,
};

struct ddc_status {
	struct mutex ddc_bus_mutex;
	bool ddc_bus_idle;
	bool ddc_bus_error;
	bool ddc_fifo_empty;
};

struct bridge_phy_mode_config {
	bool edid_read;
	u8 edid_buf[256];

	union hdmi_infoframe hdmi_frame;
	enum color_space_convert bridge_phy_csc;

	struct {
		bool is_ccir656;
		u32 bus_formats; /* like MEDIA_BUS_FMT_XXX */
		int pclk_delay;
		enum latch_clock_type io_latch_clock;
		enum input_signal_sample_type input_signal_type;
		bool gen_sync;
		struct drm_display_mode *mode;
		struct videomode vmode;
	} input_mode;
	struct {
		u32 color_formats; /* like DRM_COLOR_FORMAT_XXX */
		enum hdmi_mode hdmi_output_mode;
		enum hdmi_colorspace colorspace;
	} output_mode;
};

struct bridge_resource {
	int encoder_obj;
	char chip_name[NAME_SIZE_MAX];
	char vendor_str[NAME_SIZE_MAX];
	unsigned int i2c_bus_num;
	unsigned short i2c_dev_addr;
	unsigned int irq_gpio;
	unsigned int gpio_placement;
	unsigned int reset_gpio;
	unsigned short hotplug;
	unsigned short edid_method;
	unsigned short bl_type;
	int display_pipe_index;
	struct loongson_device *ldev;
	struct loongson_encoder *ls_encoder;
};

struct bridge_phy_cfg_funcs {
	int (*reg_init)(struct bridge_phy *phy);
	int (*hw_reset)(struct bridge_phy *phy);
	int (*sw_enable)(struct bridge_phy *phy);
	int (*sw_reset)(struct bridge_phy *phy);
	int (*suspend)(struct bridge_phy *phy);
	int (*resume)(struct bridge_phy *phy);
	void (*prepare)(struct bridge_phy *phy);
	void (*commit)(struct bridge_phy *phy);
	int (*backlight_ctrl)(struct bridge_phy *phy, int mode);
	int (*video_input_cfg)(struct bridge_phy *phy);
	int (*video_input_check)(struct bridge_phy *phy);
	int (*video_output_cfg)(struct bridge_phy *phy);
	int (*video_output_timing)(struct bridge_phy *phy,
				   const struct drm_display_mode *mode);
	int (*hdmi_output_mode)(struct bridge_phy *phy, enum hdmi_mode mode);
	int (*hdmi_audio)(struct bridge_phy *phy);
	int (*hdmi_csc)(struct bridge_phy *phy);
	int (*hdmi_hdcp_init)(struct bridge_phy *phy);
	int (*afe_high)(struct bridge_phy *phy);
	int (*afe_low)(struct bridge_phy *phy);
	int (*afe_set_tx)(struct bridge_phy *phy, bool enable);
	int (*mode_set_pre)(struct drm_bridge *bridge,
			    const struct drm_display_mode *mode,
			    const struct drm_display_mode *adj_mode);
	int (*mode_set)(struct drm_bridge *bridge,
			const struct drm_display_mode *mode,
			const struct drm_display_mode *adj_mode);
	int (*mode_set_post)(struct drm_bridge *bridge,
			     const struct drm_display_mode *mode,
			     const struct drm_display_mode *adj_mode);
	enum drm_mode_status (*mode_valid)(struct drm_connector *connector,
					   struct drm_display_mode *mode);
};

struct bridge_phy_misc_funcs {
	bool (*chip_id_verify)(struct bridge_phy *phy, char *id);
	int (*debugfs_init)(struct bridge_phy *phy);
	void (*dpms_ctrl)(struct bridge_phy *phy, int mode);
};

struct bridge_phy_hpd_funcs {
	enum hpd_status (*get_hpd_status)(struct bridge_phy *phy);
	int (*int_enable)(struct bridge_phy *phy, enum int_type interrut);
	int (*int_disable)(struct bridge_phy *phy, enum int_type interrut);
	int (*int_clear)(struct bridge_phy *phy, enum int_type interrut);
	irqreturn_t (*irq_handler)(int irq, void *dev);
	irqreturn_t (*isr_thread)(int irq, void *dev);
};

struct bridge_phy_ddc_funcs {
	int (*ddc_fifo_fetch)(struct bridge_phy *phy, u8 *buf, u8 block,
			      size_t len, size_t offset);
	int (*ddc_fifo_abort)(struct bridge_phy *phy);
	int (*ddc_fifo_clear)(struct bridge_phy *phy);
	int (*get_edid_block)(void *data, u8 *buf, unsigned int block,
			      size_t len);
	int (*get_modes)(struct bridge_phy *phy,
			 struct drm_connector *connector);
};

struct bridge_phy_hdmi_aux_funcs {
	int (*set_gcp_avmute)(struct bridge_phy *phy, bool enable,
			      bool blue_screen);
	int (*set_avi_infoframe)(struct bridge_phy *phy,
				 const struct drm_display_mode *mode);
	int (*set_hdcp)(struct bridge_phy *phy);
};

struct bridge_phy_helper {
	const struct regmap_config *regmap_cfg;
	struct bridge_phy_misc_funcs *misc_funcs;
	struct bridge_phy_hpd_funcs *hpd_funcs;
	struct bridge_phy_ddc_funcs *ddc_funcs;
	struct bridge_phy_hdmi_aux_funcs *hdmi_aux_funcs;
};

struct bridge_phy {
	int display_pipe_index;
	struct drm_bridge bridge;
	struct drm_encoder *encoder;
	struct drm_connector connector;
	enum drm_connector_status status;
	const struct drm_display_mode *duplicated_mode;

	struct bridge_resource *res;
	struct loongson_device *ldev;
	struct loongson_i2c *li2c;
	struct i2c_client *i2c_phy;
	struct i2c_client *i2c_edid;
	struct i2c_client *i2c_cec;
	struct regmap *phy_regmap;

	void *priv;
	u8 chip_version;
	u32 feature;
	u32 connector_type;

	u8 sys_status;
	int irq_num;
	atomic_t irq_status;
	enum hpd_status hpd_status;
	struct ddc_status ddc_status;
	u8 fifo_length;
	u8 edid_buf[EDID_LENGTH];
	u8 fifo_buf[DDC_FIFO_SIZE_MAX];

	struct bridge_phy_mode_config mode_config;
	struct bridge_phy_helper *helper;
	const struct bridge_phy_cfg_funcs *cfg_funcs;
	const struct bridge_phy_hpd_funcs *hpd_funcs;
	const struct bridge_phy_ddc_funcs *ddc_funcs;
	const struct bridge_phy_hdmi_aux_funcs *hdmi_aux_funcs;
};

int loongson_bridge_bind(struct loongson_device *ldev, int index);
void bridge_phy_reg_mask_seq(struct bridge_phy *phy,
			     const struct reg_mask_seq *seq, size_t seq_size);
void bridge_phy_reg_update_bits(struct bridge_phy *phy, unsigned int reg,
		unsigned int mask, unsigned int val);
int bridge_phy_reg_dump(struct bridge_phy *phy, size_t start, size_t count);
struct bridge_phy *bridge_phy_alloc(struct bridge_resource *res);
int bridge_phy_register(struct bridge_phy *phy,
			const struct bridge_phy_cfg_funcs *cfg_funcs,
			u32 feature, struct bridge_phy_helper *helper);
void loongson_bridge_resume(struct loongson_device *ldev);
void loongson_bridge_suspend(struct loongson_device *ldev);

#endif /* __BRIDGE_PHY_H__ */
