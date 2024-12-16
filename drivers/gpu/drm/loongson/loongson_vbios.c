/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 * Authors:
 *	Zhu Chen <zhuchen@loongson.cn>
 *	Fang Yaling <fangyaling@loongson.cn>
 *	Zhang Dandan <zhangdandan@loongson.cn>
 *	liyi	<liyi@loongson.cn>
 *	Li Chen Yang  <lichenyang@loongson.cn>
 *	ZhiJie Zhang <zhangzhijie@loongson.cn>
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include "loongson_drv.h"
#include "loongson_legacy_vbios.h"
#include "loongson_vbios.h"

#define VBIOS_SIZE 0x40000
#define VBIOS_DESC_OFFSET 0x6000
#define VBIOS_OFFSET 0x100000
#define LOONGSON_VBIOS_TITLE "Loongson-VBIOS"

static bool is_valid_vbios(void *vbios)
{
	u8 header[16] = {0};
	struct loongson_vbios *ls_header;
	ls_header = (struct loongson_vbios *)vbios;
	memcpy(&header[0], ls_header->title, sizeof(ls_header->title));

	if (0 != memcmp((char *)&header[0],
			LOONGSON_VBIOS_TITLE,
			strlen(LOONGSON_VBIOS_TITLE))) {
		DRM_WARN("vbios signature is invation \n");
		return false;
	}

	return true;
}

static bool read_bios_from_vram(struct loongson_device *ldev)
{
	void *bios;
	u64 vbios_addr = (ldev->mc.vram_end + 1) - VBIOS_OFFSET;
	ldev->vbios = NULL;

	bios = ioremap(vbios_addr, VBIOS_SIZE);
	if (!bios)
		return false;

	ldev->vbios = kmalloc(VBIOS_SIZE, GFP_KERNEL);
	if (!ldev->vbios) {
		return false;
	}

	memcpy(ldev->vbios, bios, VBIOS_SIZE);
	iounmap(bios);
	if (!is_valid_vbios(ldev->vbios)) {
		kfree(ldev->vbios);
		return false;
	}

	DRM_INFO("Get vbios from vram Success \n");
	return true;
}

static bool read_bios_from_sysconf(struct loongson_device *ldev)
{
	ldev->vbios = NULL;
	if (!loongson_sysconf.vgabios_addr)
		return false;

	ldev->vbios = kmalloc(VBIOS_SIZE, GFP_KERNEL);
	if (!ldev->vbios) {
		kfree(ldev->vbios);
		return false;
	}

	memcpy(ldev->vbios, (void *)loongson_sysconf.vgabios_addr, VBIOS_SIZE);
	DRM_INFO("Get vbios from sysconf Success \n");
	return true;
}

#ifdef CONFIG_ACPI
static bool read_bios_from_acpi(struct loongson_device *ldev)
{
	acpi_size tbl_size;
	void *vaddr;
	struct acpi_table_header *hdr;
	struct acpi_viat_table *viat;

	if (!ACPI_SUCCESS(acpi_get_table("VIAT", 1, &hdr)))
		return false;
	tbl_size = hdr->length;
	if (tbl_size != sizeof(struct acpi_viat_table)) {
		DRM_WARN("ACPI viat table present but broken (length error #1)\n");
		return false;
	}
	viat = (struct acpi_viat_table *)hdr;
	ldev->vbios = kmalloc(VBIOS_SIZE, GFP_KERNEL);
	if (!ldev->vbios) {
		kfree(ldev->vbios);
		return false;
	}
	vaddr = phys_to_virt(viat->vbios_addr);
	memcpy(ldev->vbios, vaddr, VBIOS_SIZE);
	DRM_INFO("Get vbios from ACPI success!\n");
	return true;
}
#else
static bool read_bios_from_acpi(struct loongson_device *ldev)
{
	return false;
}
#endif

static  bool get_vbios_data(struct loongson_device *ldev)
{
	if (read_bios_from_vram(ldev))
		goto success;

	if (read_bios_from_acpi(ldev))
		goto success;

	if (read_bios_from_sysconf(ldev))
		goto success;

	DRM_ERROR("Unable to locate a BIOS ROM\n");
	return false;

success:
	return true;
}

static struct loongson_vbios_encoder *
get_encoder_legacy(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios *vbios = (struct loongson_vbios *)ldev->vbios;
	struct loongson_vbios_encoder *encoder = NULL;
	u8 *start;
	u32 offset;

	start = (u8 *)vbios;
	offset = vbios->encoder_offset;
	encoder = (struct loongson_vbios_encoder *)(start + offset);
	if (index == 1) {
		offset = encoder->next;
		encoder = (struct loongson_vbios_encoder *)(start + offset);
	}

	return encoder;
}

static struct loongson_vbios_crtc *get_crtc_legacy(struct loongson_device *ldev,
						   u32 index)
{
	struct loongson_vbios *vbios = ldev->vbios;
	struct loongson_vbios_crtc *crtc = NULL;
	u8 *start;
	u32 offset;

	start = (u8 *)vbios;
	offset = vbios->crtc_offset;
	crtc = (struct loongson_vbios_crtc *)(start + offset);
	if (index == 1) {
		offset = crtc->next;
		crtc = (struct loongson_vbios_crtc *)(start + offset);
	}

	return crtc;
}

static struct loongson_vbios_connector *
get_connector_legacy(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios *vbios = ldev->vbios;
	struct loongson_vbios_connector *connector = NULL;
	u8 *start;
	u32 offset;

	start = (u8 *)vbios;
	offset = vbios->connector_offset;
	connector = (struct loongson_vbios_connector *)(start + offset);
	if (index == 1) {
		offset = connector->next;
		connector = (struct loongson_vbios_connector *)(start + offset);
	}

	return connector;
}

static bool is_legacy_vbios(struct loongson_vbios *vbios)
{
	u32 version = loongson_vbios_version(vbios);

	if (version <= 2)
		return true;
	else
		return false;
}

static void *loongson_vbios_default(struct loongson_device *ldev)
{
	struct loongson_vbios *vbios;
	struct loongson_vbios_crtc *crtc_vbios[2];
	struct loongson_vbios_connector *connector_vbios[2];
	struct loongson_vbios_encoder *encoder_vbios[2];
	unsigned char *vbios_start;

	vbios = kzalloc(VBIOS_SIZE, GFP_KERNEL);
	vbios_start = (unsigned char *)vbios;

	/*Build loongson_vbios struct*/
	vbios->version_major = 0;
	vbios->version_minor = 1;
	vbios->crtc_num = 2;
	vbios->crtc_offset = sizeof(struct loongson_vbios);
	vbios->connector_num = 2;
	vbios->connector_offset = sizeof(struct loongson_vbios) +
				  2 * sizeof(struct loongson_vbios_crtc);
	vbios->encoder_num = 2;
	vbios->encoder_offset = sizeof(struct loongson_vbios) +
				2 * sizeof(struct loongson_vbios_crtc) +
				2 * sizeof(struct loongson_vbios_connector);

	/*Build loongson_vbios_crtc struct*/
	crtc_vbios[0] = (struct loongson_vbios_crtc *)(vbios_start +
						       vbios->crtc_offset);
	crtc_vbios[1] = (struct loongson_vbios_crtc
				 *)(vbios_start + vbios->crtc_offset +
				    sizeof(struct loongson_vbios_crtc));

	crtc_vbios[0]->next = sizeof(struct loongson_vbios) +
			      sizeof(struct loongson_vbios_crtc);
	crtc_vbios[0]->crtc_id = 0;
	crtc_vbios[1]->next = 0;
	crtc_vbios[1]->crtc_id = 1;
	crtc_vbios[1]->encoder_id = 1;

	/*Build loongson_vbios_connector struct*/
	connector_vbios[0] =
		(struct loongson_vbios_connector *)(vbios_start +
						    vbios->connector_offset);
	connector_vbios[1] =
		(struct loongson_vbios_connector
			 *)(vbios_start + vbios->connector_offset +
			    sizeof(struct loongson_vbios_connector));

	connector_vbios[0]->next = vbios->connector_offset +
				   sizeof(struct loongson_vbios_connector);
	connector_vbios[1]->next = 0;

	connector_vbios[0]->hotplug = hpd_disable;
	connector_vbios[1]->hotplug = hpd_disable;

	connector_vbios[0]->i2c_id = 6;
	connector_vbios[1]->i2c_id = 7;

	if (!ldev->gpu_pdev) {
		connector_vbios[0]->edid_method = via_i2c;
		connector_vbios[1]->edid_method = via_i2c;
		connector_vbios[0]->type = connector_dvid;
		connector_vbios[1]->type = connector_vga;
	}

	/*Build loongson_vbios_encoder struct*/
	encoder_vbios[0] =
		(struct loongson_vbios_encoder *)(vbios_start +
						  vbios->encoder_offset);
	encoder_vbios[1] = (struct loongson_vbios_encoder
				    *)(vbios_start + vbios->encoder_offset +
				       sizeof(struct loongson_vbios_encoder));

	encoder_vbios[0]->next =
		vbios->encoder_offset + sizeof(struct loongson_vbios_encoder);
	encoder_vbios[1]->next = 0;

	encoder_vbios[0]->config_type = encoder_bios_config;
	encoder_vbios[1]->config_type = encoder_bios_config;

	encoder_vbios[0]->connector_id = 0;
	encoder_vbios[1]->connector_id = 1;

	if (!ldev->gpu_pdev) {
		encoder_vbios[0]->type = encoder_tmds;
		encoder_vbios[1]->type = encoder_dac;
	}

	return (void *)vbios;
}

static int show_legacy_vbios(struct loongson_device *ldev)
{
	int index;
	struct loongson_vbios *vbios = (struct loongson_vbios *)ldev->vbios;
	struct loongson_vbios_crtc *crtc;
	struct loongson_vbios_connector *connector;
	struct loongson_vbios_encoder *encoder;
	char *config_method;
	char *encoder_methods[] = { "NONE", "OS", "BIOS", "ERR" };

	char *edid_methods[] = { "No EDID", "Reading EDID via built-in I2C",
				 "Use the VBIOS built-in EDID information",
				 "Get EDID via encoder chip" };

	char *detect_methods[] = { "SHOW", "POLL", "HPD", "NONE" };

	for (index = 0; index < vbios->crtc_num; index++) {
		crtc = get_crtc_legacy(ldev, index);
		if (!crtc)
			continue;
		encoder = get_encoder_legacy(ldev, crtc->encoder_id);
		if (!encoder)
			continue;
		connector = get_connector_legacy(ldev, encoder->connector_id);
		if (!connector)
			continue;
		config_method = encoder_methods[encoder->config_type & 0x3];
		DRM_INFO("\tencoder%d(%s) i2c:%d \n", crtc->encoder_id,
			 config_method, encoder->i2c_id);
		DRM_INFO("\tconnector%d:\n", encoder->connector_id);
		DRM_INFO("\t   %s", edid_methods[connector->edid_method & 0x3]);
		DRM_INFO("\t  Detect:%s\n",
			 detect_methods[connector->hotplug & 0x3]);
	}

	return 0;
}

static int loongson_vbios_init_legacy(struct loongson_device *ldev)
{
	struct loongson_vbios *vbios = (struct loongson_vbios *)ldev->vbios;

	if (vbios->version_minor == 1) {
		kfree(vbios);
		vbios = loongson_vbios_default(ldev);
		ldev->vbios = vbios;
	}

	show_legacy_vbios(ldev);

	return 0;
}

static struct crtc_timing *get_crtc_timing_legacy(struct loongson_device *ldev,
						  u32 index)
{
	struct loongson_crtc_config_param *vbios_tables;
	struct loongson_crtc_modeparameter *param;
	struct loongson_vbios_crtc *vbios_crtc;
	struct loongson_timing *tables;
	struct crtc_timing *timing;
	s32 i = 0;
	u32 tables_size = (sizeof(struct loongson_timing) * LS_MAX_RESOLUTIONS);

	vbios_crtc = get_crtc_legacy(ldev, index);

	timing = (struct crtc_timing *)kzalloc(sizeof(struct crtc_timing),
					       GFP_KERNEL);
	if (!timing)
		return NULL;

	tables = kzalloc(tables_size, GFP_KERNEL);
	if (!tables) {
		kfree(timing);
		return NULL;
	}

	timing->tables = tables;

	vbios_tables = vbios_crtc->mode_config_tables;

	while (vbios_tables->resolution.used) {
		param = &vbios_tables->crtc_resol_param;

		tables->htotal = param->htotal;
		tables->hdisplay = param->hdisplay;
		tables->hsync_start = param->hsync_start;
		tables->hsync_width = param->hsync_width;
		tables->hsync_pll = param->hsync_pll;
		tables->vtotal = param->vtotal;
		tables->vdisplay = param->vdisplay;
		tables->vsync_start = param->vsync_start;
		tables->vsync_width = param->vsync_width;
		tables->vsync_pll = param->vsync_pll;
		tables->clock = param->clock;
		tables->hfreq = param->hfreq;
		tables->vfreq = param->vfreq;
		tables->clock_phase = param->clock_phase;

		i++;
		tables++;
		vbios_tables++;
	}
	timing->num = i;

	return timing;
}

static bool parse_vbios_crtc(struct desc_node *this, struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	u32 *val = (u32 *)cmd->res;
	struct vbios_crtc crtc;

	memset(&crtc, 0xff, sizeof(crtc));
	memcpy(&crtc, this->data, min_t(u32, this->desc->size, sizeof(crtc)));

	switch (request) {
	case VBIOS_CRTC_ID:
		*val = crtc.crtc_id;
		break;
	case VBIOS_CRTC_ENCODER_ID:
		*val = crtc.encoder_id;
		break;
	case VBIOS_CRTC_MAX_FREQ:
		*val = crtc.max_freq;
		break;
	case VBIOS_CRTC_MAX_WIDTH:
		*val = crtc.max_width;
		break;
	case VBIOS_CRTC_MAX_HEIGHT:
		*val = crtc.max_height;
		break;
	case VBIOS_CRTC_IS_VB_TIMING:
		*val = crtc.is_vb_timing;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static bool parse_vbios_connector(struct desc_node *this, struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	u32 *val = (u32 *)cmd->res;
	struct vbios_connector connector;

	memset(&connector, 0xff, sizeof(connector));
	memcpy(&connector, this->data,
	       min_t(u32, this->desc->size, sizeof(connector)));

	switch (request) {
	case VBIOS_CONNECTOR_I2C_ID:
		*val = connector.i2c_id;
		break;
	case VBIOS_CONNECTOR_INTERNAL_EDID:
		memcpy((u8 *)(ulong)val, connector.internal_edid,
		       EDID_LENGTH * 2);
		break;
	case VBIOS_CONNECTOR_TYPE:
		*val = connector.type;
		break;
	case VBIOS_CONNECTOR_HOTPLUG:
		*val = connector.hotplug;
		break;
	case VBIOS_CONNECTOR_EDID_METHOD:
		*val = connector.edid_method;
		break;
	case VBIOS_CONNECTOR_IRQ_GPIO:
		*val = connector.irq_gpio;
		break;
	case VBIOS_CONNECTOR_IRQ_PLACEMENT:
		*val = connector.gpio_placement;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static bool parse_vbios_encoder(struct desc_node *this, struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	u32 *val = (u32 *)cmd->res;
	struct vbios_encoder encoder;

	memset(&encoder, 0xff, sizeof(encoder));
	memcpy(&encoder, this->data,
	       min_t(u32, this->desc->size, sizeof(encoder)));

	switch (request) {
	case VBIOS_ENCODER_I2C_ID:
		*val = encoder.i2c_id;
		break;
	case VBIOS_ENCODER_CONNECTOR_ID:
		*val = encoder.connector_id;
		break;
	case VBIOS_ENCODER_TYPE:
		*val = encoder.type;
		break;
	case VBIOS_ENCODER_CONFIG_TYPE:
		*val = encoder.config_type;
		break;
	case VBIOS_ENCODER_CHIP:
		*val = encoder.chip;
		break;
	case VBIOS_ENCODER_CHIP_ADDR:
		*val = encoder.chip_addr;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static bool parse_vbios_cfg_encoder(struct desc_node *this,
				    struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	u32 *val = (u32 *)cmd->res;
	struct cfg_encoder *cfg_encoder;
	struct cfg_encoder *cfg;
	struct vbios_cfg_encoder *vbios_cfg_encoder;
	u32 num, size, i = 0;

	vbios_cfg_encoder = (struct vbios_cfg_encoder *)this->data;
	size = sizeof(struct vbios_cfg_encoder);
	num = this->desc->size / size;

	switch (request) {
	case VBIOS_ENCODER_CONFIG_PARAM:
		cfg_encoder = (struct cfg_encoder *)kzalloc(
			sizeof(struct cfg_encoder) * num, GFP_KERNEL);
		cfg = cfg_encoder;
		for (i = 0; i < num; i++) {
			cfg->reg_num = vbios_cfg_encoder->reg_num;
			cfg->hdisplay = vbios_cfg_encoder->hdisplay;
			cfg->vdisplay = vbios_cfg_encoder->vdisplay;
			memcpy(&cfg->config_regs,
			       &vbios_cfg_encoder->config_regs,
			       sizeof(struct vbios_conf_reg) * 256);

			cfg++;
			vbios_cfg_encoder++;
		}
		cmd->res = (void *)cfg_encoder;
		break;
	case VBIOS_ENCODER_CONFIG_NUM:
		*val = num;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static bool parse_vbios_res_encoder(struct desc_node *this,
				    struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	struct encoder_resources  *encoder_res = NULL;

	cmd->res = (void *)NULL;
	switch (request) {
	case VBIOS_ENCODER_RESOURCES:
		encoder_res = kzalloc(sizeof(struct encoder_resources),
				GFP_KERNEL);
		if (encoder_res)
			memcpy(encoder_res, this->data,
					sizeof(struct encoder_resources));
		else
			return false;
		cmd->res = (void *)encoder_res;
		break;
	default:
		ret = false;
		break;
	}
	return ret;
}

static bool parse_vbios_i2c(struct desc_node *this, struct vbios_cmd *cmd)
{
	bool ret = true;
	int i, num, size;
	struct loongson_i2c *val = (struct loongson_i2c *)cmd->res;
	struct vbios_i2c *vbios_i2c = NULL;
	struct vbios_i2c *i2c;

	size = this->desc->size;
	vbios_i2c = kzalloc(size, GFP_KERNEL);
	if (!vbios_i2c)
		return false;

	memset(vbios_i2c, 0xff, size);
	memcpy(vbios_i2c, this->data, size);
	num = size / sizeof(*vbios_i2c);

	i2c = vbios_i2c;
	for (i = 0; (i < num && i < DC_I2C_BUS_MAX); i++) {
		val->i2c_id = (u32)i2c->id;
		val->use = true;
		val++;
		i2c++;
	}

	kfree(vbios_i2c);
	return ret;
}

static bool parse_vbios_backlight(struct desc_node *this, struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	u32 *val = (u32 *)cmd->res;
	struct vbios_backlight *bl = (struct vbios_backlight *)this->data;

	switch (request) {
	case VBIOS_BACKLIGHT_USED:
		*val = bl->used;
		break;
	case VBIOS_BACKLIGHT_TYPE:
		*val = bl->type;
		break;
	default:
		ret = false;
		break;
	}
	return ret;
}

static bool parse_vbios_pwm(struct desc_node *this, struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	u32 *val = (u32 *)cmd->res;
	struct vbios_pwm *pwm = (struct vbios_pwm *)this->data;

	switch (request) {
	case VBIOS_PWM_ID:
		*val = pwm->pwm;
		break;
	case VBIOS_PWM_PERIOD:
		*val = pwm->peroid;
		break;
	case VBIOS_PWM_POLARITY:
		*val = pwm->polarity;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static bool parse_vbios_gpu(struct desc_node *this, struct vbios_cmd *cmd)
{
	bool ret = true;
	u64 request = (u64)cmd->req;
	u32 *val = (u32 *)cmd->res;
	struct vbios_gpu *gpu = (struct vbios_gpu *)this->data;

	switch (request) {
	case VBIOS_GPU_VRAM_TYPE:
		*val = gpu->type;
		break;
	case VBIOS_GPU_BIT_WIDTH:
		*val = gpu->bit_width;
		break;
	case VBIOS_GPU_VRAM_CAP:
		*val = gpu->cap;
		break;
	case VBIOS_GPU_COUNT_FREQ:
		*val = gpu->count_freq;
		break;
	case VBIOS_GPU_FREQ:
		*val = gpu->freq;
		break;
	case VBIOS_GPU_SHADER_NUM:
		*val = gpu->shaders_num;
		break;
	case VBIOS_GPU_SHADER_FREQ:
		*val = gpu->shaders_freq;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static bool parse_vbios_header(struct desc_node *this, struct vbios_cmd *cmd)
{
	return true;
}

static bool parse_vbios_default(struct desc_node *this, struct vbios_cmd *cmd)
{
	struct vbios_desc *vb_desc;

	vb_desc = this->desc;
	DRM_WARN("Current descriptor[T-%d][V-%d] cannot be interprete.\n",
		 vb_desc->type, vb_desc->ver);
	return false;
}

#define FUNC(t, v, f)                                                          \
	{                                                                      \
		.type = t, .ver = v, .func = f,                                \
	}

static struct desc_func tables[] = {
	FUNC(desc_backlight, ver_v1, parse_vbios_backlight),
	FUNC(desc_pwm, ver_v1, parse_vbios_pwm),
	FUNC(desc_header, ver_v1, parse_vbios_header),
	FUNC(desc_crtc, ver_v1, parse_vbios_crtc),
	FUNC(desc_connector, ver_v1, parse_vbios_connector),
	FUNC(desc_encoder, ver_v1, parse_vbios_encoder),
	FUNC(desc_cfg_encoder, ver_v1, parse_vbios_cfg_encoder),
	FUNC(desc_res_encoder, ver_v1, parse_vbios_res_encoder),
	FUNC(desc_gpu, ver_v1, parse_vbios_gpu),
	FUNC(desc_i2c, ver_v1, parse_vbios_i2c),
};

static inline parse_func *get_parse_func(struct vbios_desc *vb_desc)
{
	int i;
	u32 type = vb_desc->type;
	u32 ver = vb_desc->ver;
	parse_func *func = parse_vbios_default;
	u32 tt_num = ARRAY_SIZE(tables);

	for (i = 0; i < tt_num; i++) {
		if ((tables[i].ver == ver) && (tables[i].type == type)) {
			func = tables[i].func;
			break;
		}
	}

	return func;
}

static inline void free_desc_list(struct loongson_device *ldev)
{
	struct desc_node *node, *tmp;

	list_for_each_entry_safe (node, tmp, &ldev->desc_list, head) {
		list_del(&node->head);
		kfree(node);
	}
}

static inline u32 insert_desc_list(struct loongson_device *ldev,
				   struct vbios_desc *vb_desc)
{
	struct desc_node *node;
	parse_func *func = NULL;

	WARN_ON(!ldev || !vb_desc);
	node = (struct desc_node *)kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	func = get_parse_func(vb_desc);
	node->parse = func;
	node->desc = (void *)vb_desc;
	node->data = ((u8 *)ldev->vbios + vb_desc->offset);
	list_add_tail(&node->head, &ldev->desc_list);

	return 0;
}

static u32 parse_vbios_desc(struct loongson_device *ldev)
{
	u32 ret = 0;
	struct vbios_desc *desc;
	enum desc_type type = 0;
	u8 *vbios = (u8 *)ldev->vbios;

	WARN_ON(!vbios);

	desc = (struct vbios_desc *)(vbios + VBIOS_DESC_OFFSET);
	while (1) {
		type = desc->type;
		if (type == desc_max)
			break;

		ret = insert_desc_list(ldev, desc);
		if (ret)
			DRM_DEBUG_KMS("Parse T-%d V-%d failed[%d]\n", desc->ver,
				      desc->type, ret);

		desc++;
	}

	return ret;
}

static inline struct desc_node *get_desc_node(struct loongson_device *ldev,
					      u16 type, u8 index)
{
	struct desc_node *node, *tmp;
	struct vbios_desc *vb_desc;

	list_for_each_entry_safe (node, tmp, &ldev->desc_list, head) {
		vb_desc = node->desc;
		if (vb_desc->type == type && vb_desc->index == index)
			return node;
	}

	return NULL;
}

static bool vbios_get_data(struct loongson_device *ldev, struct vbios_cmd *cmd)
{
	struct desc_node *node;

	WARN_ON(!cmd);

	node = get_desc_node(ldev, cmd->type, cmd->index);
	if (node && node->parse)
		return node->parse(node, cmd);

	DRM_DEBUG_DRIVER("Failed to get node(%d,%d)\n", cmd->type, cmd->index);
	return false;
}

u32 get_connector_type(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_connector *connector = NULL;
	struct vbios_cmd vbt_cmd;
	u32 type = -1;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		connector = get_connector_legacy(ldev, index);
		type = connector->type;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_connector;
		vbt_cmd.req = (void *)(ulong)VBIOS_CONNECTOR_TYPE;
		vbt_cmd.res = (void *)(ulong)&type;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			type = -1;
		}
	}

	return type;
}

u16 get_connector_i2cid(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_connector *connector = NULL;
	struct vbios_cmd vbt_cmd;
	u16 i2c_id = -1;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		connector = get_connector_legacy(ldev, index);
		i2c_id = connector->i2c_id;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_connector;
		vbt_cmd.req = (void *)(ulong)VBIOS_CONNECTOR_I2C_ID;
		vbt_cmd.res = (void *)(ulong)&i2c_id;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			i2c_id = -1;
		}
	}

	return i2c_id;
}

u32 get_connector_irq_gpio(struct loongson_device *ldev, u32 index)
{
	int ret;
	u32 irq_gpio;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return -1;

	vbt_cmd.index = index;
	vbt_cmd.type = desc_connector;
	vbt_cmd.req = (void *)(ulong)VBIOS_CONNECTOR_IRQ_GPIO;
	vbt_cmd.res = (void *)(ulong)&irq_gpio;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		return -1;

	return irq_gpio;
}

enum gpio_placement get_connector_gpio_placement(struct loongson_device *ldev,
						 u32 index)
{
	int ret;
	enum gpio_placement irq_placement;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return -1;

	vbt_cmd.index = index;
	vbt_cmd.type = desc_connector;
	vbt_cmd.req = (void *)(ulong)VBIOS_CONNECTOR_IRQ_PLACEMENT;
	vbt_cmd.res = (void *)(ulong)&irq_placement;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		return -1;

	return irq_placement;
}

u16 get_hotplug_mode(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_connector *connector = NULL;
	struct vbios_cmd vbt_cmd;
	u16 mode = -1;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		connector = get_connector_legacy(ldev, index);
		mode = connector->hotplug;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_connector;
		vbt_cmd.req = (void *)(ulong)VBIOS_CONNECTOR_HOTPLUG;
		vbt_cmd.res = (void *)(ulong)&mode;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			mode = -1;
		}
	}

	return mode;
}

u16 get_edid_method(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_connector *connector = NULL;
	struct vbios_cmd vbt_cmd;
	u16 method = -1;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		connector = get_connector_legacy(ldev, index);
		method = connector->edid_method;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_connector;
		vbt_cmd.req = (void *)(ulong)VBIOS_CONNECTOR_EDID_METHOD;
		vbt_cmd.res = (void *)(ulong)&method;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			method = -1;
		}
	}

	return method;
}

u8 *get_vbios_edid(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_connector *connector;
	struct vbios_cmd vbt_cmd;
	u8 *edid = NULL;
	bool ret = false;

	edid = kzalloc(sizeof(u8) * EDID_LENGTH * 2, GFP_KERNEL);
	if (!edid)
		return edid;

	if (is_legacy_vbios(ldev->vbios)) {
		connector = get_connector_legacy(ldev, index);
		memcpy(edid, connector->internal_edid, EDID_LENGTH * 2);
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_connector;
		vbt_cmd.req = (void *)(ulong)VBIOS_CONNECTOR_INTERNAL_EDID;
		vbt_cmd.res = (void *)(ulong)edid;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			return NULL;
		}
	}

	return edid;
}

u32 get_vbios_pwm(struct loongson_device *ldev, u32 index, u16 request)
{
	bool ret = false;
	struct loongson_vbios_connector *connector;
	struct vbios_cmd vbt_cmd;
	u32 value = -1;

	if (is_legacy_vbios(ldev->vbios)) {
		connector = get_connector_legacy(ldev, index);
		switch (request) {
		case VBIOS_PWM_ID:
			value = connector->bl_pwm.pwm_id;
			break;
		case VBIOS_PWM_PERIOD:
			value = connector->bl_pwm.period_ns;
			break;
		case VBIOS_PWM_POLARITY:
			value = connector->bl_pwm.polarity;
		}
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_pwm;
		vbt_cmd.req = (void *)(ulong)request;
		vbt_cmd.res = (void *)(ulong)&value;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			/*TODO
			 * add debug mesg
			 * */
			value = 0xffffffff;
		}
	}

	return value;
}

u32 get_crtc_id(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_crtc *crtc;
	struct vbios_cmd vbt_cmd;
	u32 crtc_id = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		crtc = get_crtc_legacy(ldev, index);
		crtc_id = crtc->crtc_id;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_crtc;
		vbt_cmd.req = (void *)(ulong)VBIOS_CRTC_ID;
		vbt_cmd.res = (void *)(ulong)&crtc_id;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			crtc_id = 0;
		}
	}

	return crtc_id;
}

u32 get_crtc_max_freq(struct loongson_device *ldev, u32 index)
{
	struct vbios_cmd vbt_cmd;
	u32 max_freq = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		max_freq = 200000;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_crtc;
		vbt_cmd.req = (void *)(ulong)VBIOS_CRTC_MAX_FREQ;
		vbt_cmd.res = (void *)(ulong)&max_freq;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			max_freq = 0;
		}
	}

	return max_freq;
}

u32 get_crtc_max_width(struct loongson_device *ldev, u32 index)
{
	struct vbios_cmd vbt_cmd;
	u32 max_width = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		max_width = 2048;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_crtc;
		vbt_cmd.req = (void *)(ulong)VBIOS_CRTC_MAX_WIDTH;
		vbt_cmd.res = (void *)(ulong)&max_width;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			max_width = 0;
		}
	}

	return max_width;
}

u32 get_crtc_max_height(struct loongson_device *ldev, u32 index)
{
	struct vbios_cmd vbt_cmd;
	u32 max_height = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		max_height = 2048;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_crtc;
		vbt_cmd.req = (void *)(ulong)VBIOS_CRTC_MAX_HEIGHT;
		vbt_cmd.res = (void *)(ulong)&max_height;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			max_height = 0;
		}
	}

	return max_height;
}

u32 get_crtc_encoder_id(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_crtc *crtc;
	struct vbios_cmd vbt_cmd;
	u32 encoder_id = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		crtc = get_crtc_legacy(ldev, index);
		encoder_id = crtc->encoder_id;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_crtc;
		vbt_cmd.req = (void *)(ulong)VBIOS_CRTC_ENCODER_ID;
		vbt_cmd.res = (void *)(ulong)&encoder_id;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			encoder_id = 0;
		}
	}

	return encoder_id;
}

bool get_crtc_is_vb_timing(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_crtc *crtc;
	struct vbios_cmd vbt_cmd;
	bool vb_timing = false;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		crtc = get_crtc_legacy(ldev, index);
		vb_timing = crtc->use_local_param;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_crtc;
		vbt_cmd.req = (void *)(ulong)VBIOS_CRTC_IS_VB_TIMING;
		vbt_cmd.res = (void *)(ulong)&vb_timing;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			vb_timing = false;
		}
	}

	return vb_timing;
}

struct crtc_timing *get_crtc_timing(struct loongson_device *ldev, u32 index)
{
	if (is_legacy_vbios(ldev->vbios))
		return get_crtc_timing_legacy(ldev, index);

	return NULL;
}

u32 get_encoder_connector_id(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_encoder *encoder = NULL;
	struct vbios_cmd vbt_cmd;
	u32 connector_id = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		encoder = get_encoder_legacy(ldev, index);
		connector_id = encoder->connector_id;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_encoder;
		vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_CONNECTOR_ID;
		vbt_cmd.res = (void *)(ulong)&connector_id;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			connector_id = 0;
		}
	}

	return connector_id;
}

u32 get_encoder_i2c_id(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_encoder *encoder = NULL;
	struct vbios_cmd vbt_cmd;
	u32 i2c_id = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		encoder = get_encoder_legacy(ldev, index);
		i2c_id = encoder->i2c_id;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_encoder;
		vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_I2C_ID;
		vbt_cmd.res = (void *)(ulong)&i2c_id;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			i2c_id = 0;
		}
	}

	return i2c_id;
}

struct cfg_encoder *get_encoder_config(struct loongson_device *ldev, u32 index)
{
	struct cfg_encoder *encoder_config = NULL;
	struct cfg_encoder *encoder_cfg = NULL;
	struct loongson_vbios_encoder *encoder = NULL;
	struct encoder_config_param *vbios_cfg;
	struct vbios_cmd vbt_cmd;
	u32 i, size = 0;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		encoder = get_encoder_legacy(ldev, index);
		vbios_cfg = encoder->encoder_config;
		size = sizeof(struct cfg_encoder);
		encoder_config = kzalloc(size * LS_MAX_RESOLUTIONS, GFP_KERNEL);
		encoder_cfg = encoder_config;
		for (i = 0; i < LS_MAX_RESOLUTIONS; i++) {
			encoder_cfg->hdisplay = vbios_cfg->resolution.hdisplay;
			encoder_cfg->vdisplay = vbios_cfg->resolution.vdisplay;
			encoder_cfg->reg_num = vbios_cfg->encoder_param.reg_num;
			memcpy(&encoder_cfg->config_regs,
			       &vbios_cfg->encoder_param.config_regs,
			       sizeof(struct config_reg) * 256);
			encoder_cfg++;
			vbios_cfg++;
		}
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_cfg_encoder;
		vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_CONFIG_PARAM;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret) {
			encoder_config = (struct cfg_encoder *)vbt_cmd.res;
		}
	}

	return encoder_config;
}

u32 get_encoder_cfg_num(struct loongson_device *ldev, u32 index)
{
	struct vbios_cmd vbt_cmd;
	bool ret = false;
	u32 cfg_num = 0;

	if (is_legacy_vbios(ldev->vbios)) {
		cfg_num = LS_MAX_RESOLUTIONS;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_cfg_encoder;
		vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_CONFIG_NUM;
		vbt_cmd.res = (void *)(ulong)&cfg_num;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			cfg_num = 0;
		}
	}

	return cfg_num;
}

enum encoder_config get_encoder_config_type(struct loongson_device *ldev,
					    u32 index)
{
	struct loongson_vbios_encoder *encoder = NULL;
	struct vbios_cmd vbt_cmd;
	enum encoder_config config_type = encoder_bios_config;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		encoder = get_encoder_legacy(ldev, index);
		config_type = encoder->config_type;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_encoder;
		vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_CONFIG_TYPE;
		vbt_cmd.res = (void *)(ulong)&config_type;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			config_type = encoder_bios_config;
		}
	}

	return config_type;
}

enum encoder_object get_encoder_chip(struct loongson_device *ldev, u32 index)
{
	int ret;
	enum encoder_object chip;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return Unknown;

	vbt_cmd.index = index;
	vbt_cmd.type = desc_encoder;
	vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_CHIP;
	vbt_cmd.res = (void *)(ulong)&chip;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		return Unknown;

	return chip;
}

u8 get_encoder_chip_addr(struct loongson_device *ldev, u32 index)
{
	int ret;
	u8 chip_addr;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return Unknown;

	vbt_cmd.index = index;
	vbt_cmd.type = desc_encoder;
	vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_CHIP_ADDR;
	vbt_cmd.res = (void *)(ulong)&chip_addr;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		return Unknown;

	return chip_addr;
}

struct encoder_resources *get_encoder_resources(struct loongson_device *ldev,
		u32 index)
{
	int ret;
	struct vbios_cmd vbt_cmd;
	struct encoder_resources *res = NULL;

	if (is_legacy_vbios(ldev->vbios))
		return NULL;

	vbt_cmd.index = index;
	vbt_cmd.type = desc_res_encoder;
	vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_RESOURCES;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret)
		res = (struct encoder_resources *)vbt_cmd.res;
	return res;
}

enum encoder_type get_encoder_type(struct loongson_device *ldev, u32 index)
{
	struct loongson_vbios_encoder *encoder;
	struct vbios_cmd vbt_cmd;
	enum encoder_type type = encoder_dac;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		encoder = get_encoder_legacy(ldev, index);
		type = encoder->type;
	} else {
		vbt_cmd.index = index;
		vbt_cmd.type = desc_encoder;
		vbt_cmd.req = (void *)(ulong)VBIOS_ENCODER_TYPE;
		vbt_cmd.res = (void *)(ulong)&type;
		ret = vbios_get_data(ldev, &vbt_cmd);
		if (ret == false) {
			type = encoder_dac;
		}
	}

	return type;
}

bool get_loongson_i2c(struct loongson_device *ldev)
{
	struct vbios_cmd vbt_cmd;
	bool ret = false;

	if (is_legacy_vbios(ldev->vbios)) {
		ldev->i2c_bus[0].i2c_id = 6;
		ldev->i2c_bus[0].use = true;
		ldev->i2c_bus[1].i2c_id = 7;
		ldev->i2c_bus[1].use = true;
		ret = true;
	} else {
		vbt_cmd.index = 0;
		vbt_cmd.type = desc_i2c;
		vbt_cmd.res = (void *)&ldev->i2c_bus;
		ret = vbios_get_data(ldev, &vbt_cmd);
	}

	return ret;
}


bool get_backlight_used(struct loongson_device *ldev, u32 index)
{
	bool ret = false;
	u8 used = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return ret;

	vbt_cmd.index = index;
	vbt_cmd.type = desc_backlight;
	vbt_cmd.req = (void *)(ulong)VBIOS_BACKLIGHT_USED;
	vbt_cmd.res = (void *)(ulong)&used;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		return false;
	return used ? true : false;
}

enum vbios_backlight_type get_backlight_type(struct loongson_device *ldev,
		u32 index)
{
	bool ret = false;
	enum vbios_backlight_type type = bl_unuse;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return type;

	vbt_cmd.index = index;
	vbt_cmd.type = desc_backlight;
	vbt_cmd.req = (void *)(ulong)VBIOS_BACKLIGHT_TYPE;
	vbt_cmd.res = (void *)(ulong)&type;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		type = bl_unuse;
	return type;
}

enum vram_type get_vram_type(struct loongson_device *ldev)
{
	bool ret = false;
	enum vram_type type = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return type;

	vbt_cmd.index = 0;
	vbt_cmd.type = desc_gpu;
	vbt_cmd.req = (void *)(ulong)VBIOS_GPU_VRAM_TYPE;
	vbt_cmd.res = (void *)(ulong)&type;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		type = 0;
	return type;
}

u32  get_bit_width(struct loongson_device *ldev)
{
	bool ret = false;
	u32 bit_width = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return bit_width;

	vbt_cmd.index = 0;
	vbt_cmd.type = desc_gpu;
	vbt_cmd.req = (void *)(ulong)VBIOS_GPU_BIT_WIDTH;
	vbt_cmd.res = (void *)(ulong)&bit_width;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		bit_width = 0;
	return bit_width;
}

u32  get_cap(struct loongson_device *ldev)
{
	bool ret = false;
	u32 cap = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return cap;

	vbt_cmd.index = 0;
	vbt_cmd.type = desc_gpu;
	vbt_cmd.req = (void *)(ulong)VBIOS_GPU_VRAM_CAP;
	vbt_cmd.res = (void *)(ulong)&cap;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		cap = 0;
	return cap;
}

u32  get_count_freq(struct loongson_device *ldev)
{
	bool ret = false;
	u32 count_freq = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return count_freq;

	vbt_cmd.index = 0;
	vbt_cmd.type = desc_gpu;
	vbt_cmd.req = (void *)(ulong)VBIOS_GPU_COUNT_FREQ;
	vbt_cmd.res = (void *)(ulong)&count_freq;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		count_freq = 0;
	return count_freq;
}

u32  get_freq(struct loongson_device *ldev)
{
	bool ret = false;
	u32 freq = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return freq;

	vbt_cmd.index = 0;
	vbt_cmd.type = desc_gpu;
	vbt_cmd.req = (void *)(ulong)VBIOS_GPU_FREQ;
	vbt_cmd.res = (void *)(ulong)&freq;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		freq = 0;
	return freq;
}

u32  get_shaders_num(struct loongson_device *ldev)
{
	bool ret = false;
	u32 shaders_num = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return shaders_num;

	vbt_cmd.index = 0;
	vbt_cmd.type = desc_gpu;
	vbt_cmd.req = (void *)(ulong)VBIOS_GPU_SHADER_NUM;
	vbt_cmd.res = (void *)(ulong)&shaders_num;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		shaders_num = 0;
	return shaders_num;
}

u32  get_shaders_freq(struct loongson_device *ldev)
{
	bool ret = false;
	u32 shaders_freq = 0;
	struct vbios_cmd vbt_cmd;

	if (is_legacy_vbios(ldev->vbios))
		return shaders_freq;

	vbt_cmd.index = 0;
	vbt_cmd.type = desc_gpu;
	vbt_cmd.req = (void *)(ulong)VBIOS_GPU_SHADER_FREQ;
	vbt_cmd.res = (void *)(ulong)&shaders_freq;
	ret = vbios_get_data(ldev, &vbt_cmd);
	if (ret == false)
		shaders_freq = 0;
	return shaders_freq;
}

u8 loongson_vbios_checksum(const u8 *data, int size)
{
	u8 sum = 0;

	while (size--)
		sum += *data++;
	return sum;
}

u32 loongson_vbios_version(struct loongson_vbios *vbios)
{
	u32 minor, major, version;

	minor = vbios->version_minor;
	major = vbios->version_major;
	version = major * 10 + minor;

	return version;
}

bool loongson_vbios_init(struct loongson_device *ldev)
{
	bool status;
	struct loongson_vbios *header;

	switch (ldev->chip) {
	case dc_7a1000:
	case dc_7a2000:
	case dc_2k2000:
		status = get_vbios_data(ldev);
		break;
	case dc_2k0500:
		status = false;
		break;
	}

	if (!status) {
		ldev->vbios = loongson_vbios_default(ldev);
		if (!ldev->vbios)
			return false;
	}

	header = ldev->vbios;
	ldev->num_crtc = header->crtc_num;

	DRM_INFO("Loongson vbios version %d.%d\n", header->version_major,
		 header->version_minor);

	if (is_legacy_vbios(ldev->vbios))
		loongson_vbios_init_legacy(ldev);
	else {
		INIT_LIST_HEAD(&ldev->desc_list);
		parse_vbios_desc(ldev);
	}

	return true;
}

void loongson_vbios_exit(struct loongson_device *ldev)
{
	if (!is_legacy_vbios(ldev->vbios)) {
		free_desc_list(ldev);
	}

	kfree(ldev->vbios);
}
