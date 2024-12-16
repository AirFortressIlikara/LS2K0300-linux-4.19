#ifndef __LOONGSON_DRV_H__
#define __LOONGSON_DRV_H__

#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <video/vga.h>
#include <drm/ttm/ttm_bo_driver.h>
#include <drm/drmP.h>
#include <drm/drm_encoder.h>
#include <drm/drm_gem.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/loongson_drm.h>
#include <loongson-pch.h>
#include "loongson_i2c.h"

#define PCI_DEVICE_ID_GSGPU 0x7A25
#define PCI_DEVICE_ID_GSGPU_DC 0x7a36

#define to_loongson_crtc(x) container_of(x, struct loongson_crtc, base)
#define to_loongson_encoder(x) container_of(x, struct loongson_encoder, base)
#define to_loongson_connector(x)                                               \
	container_of(x, struct loongson_connector, base)
#define to_loongson_framebuffer(x)                                             \
	container_of(x, struct loongson_framebuffer, base)

#define LS_MAX_MODE_INFO 6
#define LOONGSON_MAX_FB_HEIGHT 8192
#define LOONGSON_MAX_FB_WIDTH 8192

#define CUR_WIDTH_SIZE 32
#define CUR_HEIGHT_SIZE 32

#define LOONGSON_BL_MAX_LEVEL 100
#define LOONGSON_BL_MIN_LEVEL 1
#define LOONGSON_BL_DEF_LEVEL 60

#define LOONGSON_GPIO_LCD_EN 62
#define LOONGSON_GPIO_LCD_VDD 63

#define LO_OFF 0
#define HI_OFF 8

#define gem_to_loongson_bo(gobj) container_of((gobj), struct loongson_bo, gem)
#define VRAM_SIZE  (0x2000000)
#define LS_PIX_PLL (0x04b0)

#define CURIOSET_CORLOR (0x4607)
#define CURIOSET_POSITION (0x4608)
#define CURIOLOAD_ARGB (0x4609)
#define CURIOLOAD_IMAGE (0x460A)
#define CURIOHIDE_SHOW (0x460B)
#define FBEDID_GET (0X860C)

#define REG_OFFSET (0x10)
#define LS_FB_CFG_REG (0x1240)
#define LS_FB_HADDR0_REG (0x15a0)
#define LS_FB_LADDR0_REG (0x1260)
#define LS_FB_HADDR1_REG (0x15c0)
#define LS_FB_LADDR1_REG (0x1580)
#define LS_FB_STRI_REG (0x1280)

#define LS_FB_DITCFG_REG (0x1360)
#define LS_FB_DITTAB_LO_REG (0x1380)
#define LS_FB_DITTAB_HI_REG (0x13a0)
#define LS_FB_PANCFG_REG (0x13c0)
#define LS_FB_PANTIM_REG (0x13e0)

#define LS_FB_HDISPLAY_REG (0x1400)
#define LS_FB_HSYNC_REG (0x1420)
#define LS_FB_VDISPLAY_REG (0x1480)
#define LS_FB_VSYNC_REG (0x14a0)

#define LS_FB_GAMINDEX_DVO0_REG (0x14e0)
#define LS_FB_GAMINDEX_DVO1_REG (0x14f0)
#define LS_FB_GAMDATA_DVO0_REG (0x1500)
#define LS_FB_GAMDATA_DVO1_REG (0x1510)

#define LS_FB_CUR_CFG_REG (0x1520)
#define LS_FB_CUR_ADDR_REG (0x1530)
#define LS_FB_CUR_LOC_ADDR_REG (0x1540)
#define LS_FB_CUR_BACK_REG (0x1550)
#define LS_FB_CUR_FORE_REG (0x1560)

#define LS_FB_CUR1_CFG_REG (0x1670)
#define LS_FB_CUR1_ADDR_REG (0x1680)
#define LS_FB_CUR1_LOC_ADDR_REG (0x1690)
#define LS_FB_CUR1_BACK_REG (0x16a0)
#define LS_FB_CUR1_FORE_REG (0x16b0)
#define DC_CURSOR_MODE_32x32 (0 << 2)
#define DC_CURSOR_MODE_64x64 (1 << 2)
#define DC_CURSOR_DISPLAY_MASK    0x1
#define DC_CURSOR_DISPLAY_SHIFT   4
#define CUR_FORMAT_ARGB8888	0x2
#define DC_CURSOR_FORMAT_MASK     0x3
#define DC_CURSOR_FORMAT_SHIFT    0
#define DC_CURSOR_POS_HOT_X_MASK  0x1F
#define DC_CURSOR_POS_HOT_X_SHIFT 16
#define DC_CURSOR_POS_HOT_Y_MASK  0x1F
#define DC_CURSOR_POS_HOT_Y_SHIFT 8

#define LS_FB_INT_REG (0x1570)

/*offset*/
#define LS_FB_CFG_FORMAT32 (1 << 2)
#define LS_FB_CFG_FORMAT16 (3 << 0)
#define LS_FB_CFG_FORMAT15 (1 << 1)
#define LS_FB_CFG_FORMAT12 (1 << 0)
#define LS_FB_CFG_FB_SWITCH (1 << 7)
#define LS_FB_CFG_ENABLE (1 << 8)
#define LS_FB_CFG_SWITCH_PANEL (1 << 9)
#define LS_FB_CFG_GAMMA (1 << 12)
#define LS_FB_CFG_RESET (1 << 20)

#define LS_FB_PANCFG_BASE 0x80001010
#define LS_FB_PANCFG_DE (1 << 0)
#define LS_FB_PANCFG_DEPOL (1 << 1)
#define LS_FB_PANCFG_CLKEN (1 << 8)
#define LS_FB_PANCFG_CLKPOL (1 << 9)

#define LS_FB_HSYNC_POLSE (1 << 30)
#define LS_FB_HSYNC_POL (1 << 31)
#define LS_FB_VSYNC_POLSE (1 << 30)
#define LS_FB_VSYNC_POL (1 << 31)

#define LS_FB_VSYNC1_ENABLE (1 << 16)
#define LS_FB_HSYNC1_ENABLE (1 << 17)
#define LS_FB_VSYNC0_ENABLE (1 << 18)
#define LS_FB_HSYNC0_ENABLE (1 << 19)

#define DC_FB_CFG_DMA_32 (0x3 << 16)
#define DC_FB_CFG_DMA_64 (0x2 << 16)
#define DC_FB_CFG_DMA_128 (0x1 << 16)
#define DC_FB_CFG_DMA_256 (0x0 << 16)

#define DC_HDMI_HOTPLUG_STATUS 0x1ba0
#define DC_VGA_HOTPULG_CFG     0x1bb0
#define VGA_HOTPLUG_ACCESS  BIT(0)
#define VGA_HOTPLUG_EXTRACT  BIT(1)

#define DC_INT_HDMI0_HOTPLUG_EN BIT(29)
#define DC_INT_HDMI1_HOTPLUG_EN BIT(30)
#define DC_INT_VGA_HOTPLUG_EN BIT(31)
#define HDMI0_HOTPLUG_STATUS  BIT(0)
#define HDMI1_HOTPLUG_STATUS  BIT(1)

#define HDMI_INT_HOTPLUG0_CTL   BIT(13)
#define HDMI_INT_HOTPLUG1_CTL   BIT(14)
#define VGA_INT_HOTPLUG_CTL     BIT(15)

#define HDMI_ZONEIDLE_REG       0x1700
#define HDMI_CTRL_REG           0x1720

#define HDMI_AUDIO_BUF_REG       0x1740
#define HDMI_AUDIO_NCFG_REG      0x1760
#define HDMI_AUDIO_CTSCFG_REG    0x1780
#define HDMI_AUDIO_CTSCALCFG_REG 0x17a0
#define HDMI_AUDIO_INFOFRAME_REG 0x17c0
#define HDMI_AUDIO_SAMPLE_REG    0x17e0

#define HDMI_PHY_CTRL_REG        0x1800
#define HDMI_PHY_PLLCFG_REG      0x1820

#define GAMMA_INDEX_REG          0x14e0
#define GAMMA_DATA_REG           0x1500

#define GENERAL_PURPOSE_REG 0x0420

struct loongson_connector;

struct pix_pll {
	unsigned int l2_div;
	unsigned int l1_loopc;
	unsigned int l1_frefc;
};

struct loongson_mc {
	resource_size_t aper_base;
	resource_size_t aper_size;
	u64 mc_mask;

	u64 vram_start;
	u64 vram_end;
	u64 vram_size;

	u32 gtt_start;
	u32 gtt_end;
	u32 gtt_size;
};

struct loongson_backlight {
	int display_pipe_index;
	struct backlight_device *device;
	struct pwm_device *pwm;
	u32 pwm_id;
	u32 pwm_polarity;
	u32 pwm_period;
	u32 gpio_vdd;
	u32 gpio_enable;
	bool present;
	bool hw_enabled;
	unsigned int level, max, min;
	void *driver_private;

	unsigned int (*get_brightness)(struct loongson_backlight *ls_bl);
	void (*set_brightness)(struct loongson_backlight *ls_bl,
			       unsigned int level);
	void (*enable)(struct loongson_backlight *ls_bl);
	void (*disable)(struct loongson_backlight *ls_bl);
	void (*power)(struct loongson_backlight *ls_bl, bool enable);
};

struct loongson_bo {
	struct ttm_buffer_object bo;
	struct ttm_placement placement;
	struct ttm_bo_kmap_obj kmap;
	struct drm_gem_object gem;
	struct ttm_place placements[3];
	int pin_count;
	struct ttm_bo_kmap_obj dma_buf_vmap;
};

struct loongson_timing {
	/* horizontal timing. */
	u32 htotal;
	u32 hdisplay;
	u32 hsync_start;
	u32 hsync_width;
	u32 hsync_pll;

	/* vertical timing. */
	u32 vtotal;
	u32 vdisplay;
	u32 vsync_start;
	u32 vsync_width;
	u32 vsync_pll;

	/* refresh timing. */
	s32 clock;
	u32 hfreq;
	u32 vfreq;

	/* clock phase. this clock phase only applies to panel. */
	u32 clock_phase;
};

struct crtc_timing {
	u32 num;
	struct loongson_timing *tables;
};

struct loongson_framebuffer {
	struct drm_framebuffer base;
	struct drm_gem_object *obj;
};

enum loongson_flip_status {
	LOONGSON_FLIP_NONE,
	LOONGSON_FLIP_PENDING,
	LOONGSON_FLIP_SUBMITTED
};

struct loongson_cursor {
	u32 width, height; /* cursor width and height by User-defined */
	u32 x, y;
	u32 hot_x, hot_y;
	struct loongson_bo *bo;
	u64 paddr;
};

struct loongson_crtc {
	struct drm_crtc base;
	struct loongson_device *ldev;
	struct device_node *pix_node;
	u32 crtc_id;
	u32 crtc_offset;
	s32 width;
	s32 height;
	s32 last_dpms;
	bool enabled;
	struct loongson_flip_work *pflip_works;
	enum loongson_flip_status pflip_status;
	u32 encoder_id;
	u32 max_freq;
	u32 max_width;
	u32 max_height;
	struct loongson_cursor cursor;
	bool is_vb_timing;
	struct crtc_timing *timing;
};

struct loongson_flip_work {
	struct delayed_work flip_work;
	struct loongson_device *ldev;
	int crtc_id;
	u32 target_vblank;
	uint64_t base;
	struct drm_pending_vblank_event *event;
	struct loongson_bo *old_bo;
};

struct loongson_irq {
	bool installed;
	spinlock_t lock;
	unsigned int irq_num;
};

struct config_reg {
	u8 dev_addr;
	u8 reg;
	u8 value;
} __packed;

struct cfg_encoder {
	u32 hdisplay;
	u32 vdisplay;
	u8 reg_num;
	struct config_reg config_regs[256];
};

struct loongson_encoder {
	struct drm_encoder base;
	struct loongson_device *ldev;
	struct loongson_i2c *i2c;
	struct cfg_encoder *encoder_config;
	struct encoder_resources *encoder_res;
	struct drm_display_mode mode;
	int last_dpms;
	u32 cfg_num;
	u32 i2c_id;
	u32 encoder_id;
	u32 connector_id;
	u32 config_type;
	u32 type;
	bool (*mode_set_method)(struct loongson_encoder *ls_encoder,
				struct drm_display_mode *mode);
};

struct loongson_connector {
	struct drm_connector base;
	struct loongson_device *ldev;
	struct mutex hpd_lock;
	u16 id;
	u32 type;
	u16 i2c_id;
	u16 hotplug;
	u16 edid_method;
	u8 *vbios_edid;
	struct loongson_i2c *i2c;
};

struct loongson_mode_info {
	bool connector_is_legacy;
	bool mode_config_initialized;
	struct loongson_device *ldev;
	struct loongson_crtc *crtc;
	struct loongson_encoder *encoder;
	struct loongson_connector *connector;
	struct bridge_phy *bridge_phy;
	struct loongson_backlight *backlight;
};

struct loongson_fbdev {
	struct drm_fb_helper helper;
	struct loongson_framebuffer lfb;
	void *sysram;
	int size;
	struct ttm_bo_kmap_obj mapping;
	int x1, y1, x2, y2; /* dirty rect */
	spinlock_t dirty_lock;
};

enum loongson_chip {
	dc_7a1000,
	dc_7a2000,
	dc_2k0500,
	dc_2k2000
};

struct loongson_config {
	enum drm_mode_status (*mode_filter)(struct drm_crtc *crtc,
				const struct drm_display_mode *mode);

	enum drm_connector_status (*detect_config)(struct drm_connector *connector,
						   unsigned short edid_method);

	void (*hide_cursor)(struct drm_crtc *crtc);
	void (*show_cursor)(struct drm_crtc *crtc, int width);
	void (*cursor_hot_spot)(struct drm_crtc *crtc, int x, int y);
	void (*cursor_local)(struct drm_crtc *crtc, int x, int y);
};

struct loongson_device {
	struct drm_device *dev;
	enum loongson_chip chip;
	u8 chip_revision;

	resource_size_t mmio_base;
	resource_size_t mmio_size;
	void __iomem *mmio;
	void __iomem *io;

	struct loongson_config *dc_config;
	struct loongson_mc mc;
	struct loongson_mode_info mode_info[LS_MAX_MODE_INFO];

	struct loongson_fbdev *lfbdev;
	struct pci_dev *gpu_pdev; /* LS7A gpu device info */
	int num_crtc;
	struct drm_display_mode mode;

	struct loongson_irq irq;
	struct work_struct hotplug_work;
	void *vbios;
	struct list_head desc_list;
	struct loongson_i2c i2c_bus[DC_I2C_BUS_MAX];

	struct {
		struct drm_global_reference mem_global_ref;
		struct ttm_bo_global_ref bo_global_ref;
		struct ttm_bo_device bdev;
	} ttm;
	struct {
		resource_size_t start;
		resource_size_t size;
	} vram;
	unsigned long fb_vram_base;
	bool inited;
	unsigned int vsync0_count;
	unsigned int vsync1_count;
	unsigned int pageflip_count;

	spinlock_t mmio_lock;
	int connector_active0;
	int connector_active1;
	int vga_hpd_status;

	bool enable_gamma;
};

static inline struct loongson_device *
loongson_ttm_ldev(struct ttm_bo_device *bdev)
{
	return container_of(bdev, struct loongson_device, ttm.bdev);
}

/*irq*/
void loongson_hpd_cancel_work(struct loongson_device *ldev);
void loongson_hpd_init_work(struct loongson_device *ldev);
int loongson_irq_enable_vblank(struct drm_device *dev, unsigned int crtc_id);
void loongson_irq_disable_vblank(struct drm_device *dev, unsigned int crtc_id);
u32 loongson_crtc_vblank_count(struct drm_device *dev, unsigned int pipe);
irqreturn_t loongson_irq_handler(int irq, void *arg);
void loongson_irq_preinstall(struct drm_device *dev);
int loongson_irq_postinstall(struct drm_device *dev);
void loongson_irq_uninstall(struct drm_device *dev);
int loongson_irq_init(struct loongson_device *ldev);
int loongson_pageflip_irq(struct loongson_device *ldev, unsigned int crtc_id);
void loongson_flip_work_func(struct work_struct *__work);

int loongson_ttm_init(struct loongson_device *ldev);
void loongson_ttm_fini(struct loongson_device *ldev);
void loongson_ttm_placement(struct loongson_bo *bo, int domain);
int loongson_bo_create(struct drm_device *dev, int size, int align,
		       uint32_t flags, struct loongson_bo **ploongsonbo);

int loongson_bo_create2(struct drm_device *dev, int size, int align,
			uint32_t flags, struct sg_table *sg,
			struct reservation_object *resv,
			struct loongson_bo **pplbo);

unsigned long loongson_bo_size(struct loongson_bo *bo);

int loongson_drm_mmap(struct file *filp, struct vm_area_struct *vma);
int loongson_bo_reserve(struct loongson_bo *bo, bool no_wait);
void loongson_bo_unreserve(struct loongson_bo *bo);

struct loongson_encoder *loongson_encoder_init(struct loongson_device *dev,
					       int index);
enum drm_mode_status
loongson_7a1000_mode_filter(struct drm_crtc *crtc,
				const struct drm_display_mode *mode);
enum drm_mode_status
loongson_7a2000_mode_filter(struct drm_crtc *crtc,
				const struct drm_display_mode *mode);
enum drm_mode_status
loongson_2k2000_mode_filter(struct drm_crtc *crtc,
				const struct drm_display_mode *mode);
struct loongson_crtc *loongson_crtc_init(struct loongson_device *ldev,
					 int index);

enum drm_connector_status
loongson_7a1000_detect_config(struct drm_connector *connector, unsigned short edid_method);
enum drm_connector_status
loongson_7a2000_detect_config(struct drm_connector *connector, unsigned short edid_method);
enum drm_connector_status
loongson_2k2000_detect_config(struct drm_connector *connector,
			      unsigned short edid_method);
struct loongson_connector *loongson_connector_init(struct loongson_device *dev,
						   int index);
void loongson_set_start_address(struct drm_crtc *crtc, u64 addr);

int loongson_bo_push_sysram(struct loongson_bo *bo);
int loongson_bo_pin(struct loongson_bo *bo, u32 pl_flag, u64 *gpu_addr);
int loongson_bo_unpin(struct loongson_bo *bo);
u64 loongson_bo_gpu_offset(struct loongson_bo *bo);
void loongson_bo_unref(struct loongson_bo **bo);
int loongson_gem_create(struct drm_device *dev, u32 size, bool iskernel,
			struct drm_gem_object **obj);
int loongson_framebuffer_init(struct drm_device *dev,
			      struct loongson_framebuffer *lfb,
			      const struct drm_mode_fb_cmd2 *mode_cmd,
			      struct drm_gem_object *obj);

int loongson_fbdev_init(struct loongson_device *ldev);
void loongson_fbdev_fini(struct loongson_device *ldev);
bool loongson_fbdev_lobj_is_fb(struct loongson_device *ldev,
			       struct loongson_bo *lobj);
void loongson_fbdev_restore_mode(struct loongson_device *ldev);
void loongson_fbdev_set_suspend(struct loongson_device *ldev, int state);
void loongson_fb_output_poll_changed(struct loongson_device *ldev);

int loongson_drm_suspend(struct drm_device *dev);
int loongson_drm_resume(struct drm_device *dev);
/* loongson_cursor.c */
void loongson_hide_cursor(struct drm_crtc *crtc);
void loongson_show_cursor(struct drm_crtc *crtc, int width);
void loongson_7a2000_show_cursor(struct drm_crtc *crtc, int width);
void loongson_7a2000_hide_cursor(struct drm_crtc *crtc);
void loongson_7a2000_crtc_cursor_set_local(struct drm_crtc *crtc, int x, int y);
void loongson_7a2000_cursor_set_hot_spot(struct drm_crtc *crtc, int x, int y);
void loongson_cursor_set_hot_spot(struct drm_crtc *crtc, int x, int y);
void loongson_crtc_cursor_set_local(struct drm_crtc *crtc, int x, int y);
int loongson_crtc_cursor_set2(struct drm_crtc *crtc, struct drm_file *file_priv,
			      uint32_t handle, uint32_t width, uint32_t height,
			      int32_t hot_x, int32_t hot_y);
int loongson_crtc_cursor_move(struct drm_crtc *crtc, int x, int y);

void loongson_encoder_resume(struct loongson_device *ldev);
bool loongson_encoder_reset_3a3k(struct loongson_encoder *ls_encoder,
		struct drm_display_mode *mode);
void loongson_connector_resume(struct loongson_device *ldev);

u32 ls_mm_rreg(struct loongson_device *ldev, u32 offset);
void ls_mm_wreg(struct loongson_device *ldev, u32 offset, u32 val);
void ls_mm_wreg_check(struct loongson_device *ldev, u32 offset, u32 val);
u32 ls_mm_rreg_locked(struct loongson_device *ldev, u32 offset);
void ls_mm_wreg_locked(struct loongson_device *ldev, u32 offset, u32 val);

u32 ls_io_rreg(struct loongson_device *ldev, u32 offset);
void ls_io_wreg(struct loongson_device *ldev, u32 offset, u32 val);
u32 ls_io_rreg_locked(struct loongson_device *ldev, u32 offset);
void ls_io_wreg_locked(struct loongson_device *ldev, u32 offset, u32 val);

int loongson_debugfs_init(struct drm_minor *minor);
int loongson_dc_gpio_init(struct loongson_device *ldev);

int loongson_hdmi_init(struct loongson_device *ldev, int index);
void loongson_hotplug_config(struct loongson_device *ldev);
void hdmi_phy_pll_config(struct loongson_device *ldev, int index, int clock);
#endif
