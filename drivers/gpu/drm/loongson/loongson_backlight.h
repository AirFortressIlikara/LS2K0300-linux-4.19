// SPDX-License-Identifier: GPL-2.0+
#ifndef __LOONGSON_BACKLIGHT_H__
#define __LOONGSON_BACKLIGHT_H__

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/pwm.h>

#define BACKLIGHT_DEFAULT_METHOD_CLOSE(ls_bl)\
	do { \
		ls_bl->hw_enabled = false;\
		gpio_set_value(LOONGSON_GPIO_LCD_EN, 0);\
		msleep(10);\
		pwm_disable(ls_bl->pwm); \
	} while (0)

#define BACKLIGHT_DEFAULT_METHOD_FORCE_CLOSE(ls_bl)\
	do { \
		ls_bl->hw_enabled = false;\
		BACKLIGHT_DEFAULT_METHOD_CLOSE(ls_bl);\
		msleep(160);\
		gpio_set_value(LOONGSON_GPIO_LCD_VDD, 0); \
	} while (0)

#define BACKLIGHT_DEFAULT_METHOD_OPEN(ls_bl)\
	do { \
		pwm_enable(ls_bl->pwm);\
		msleep(10);\
		gpio_set_value(LOONGSON_GPIO_LCD_EN, 1);\
		ls_bl->hw_enabled = true; \
	} while (0)

#define BACKLIGHT_DEFAULT_METHOD_FORCE_OPEN(ls_bl)\
	do {\
		gpio_set_value(LOONGSON_GPIO_LCD_VDD, 1);\
		msleep(160);\
		BACKLIGHT_DEFAULT_METHOD_OPEN(ls_bl);\
		ls_bl->hw_enabled = true; \
	} while (0)

int loongson_backlight_register(struct drm_connector *connector);
#endif
