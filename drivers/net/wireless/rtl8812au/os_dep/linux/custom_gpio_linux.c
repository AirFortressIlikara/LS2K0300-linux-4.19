/******************************************************************************
 * Customer code to add GPIO control during WLAN start/stop
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#include "drv_types.h"

#ifdef CONFIG_PLATFORM_SPRD

//gspi func & GPIO define
#include <mach/gpio.h>//0915
#include <mach/board.h>

#if !(defined ANDROID_2X)

#ifdef CONFIG_RTL8188E
#include <mach/regulator.h>
#include <linux/regulator/consumer.h>
#endif // CONFIG_RTL8188E

#ifndef GPIO_WIFI_POWER
#define GPIO_WIFI_POWER -1
#endif // !GPIO_WIFI_POWER

#ifndef GPIO_WIFI_RESET
#define GPIO_WIFI_RESET -1
#endif // !GPIO_WIFI_RESET

#ifndef GPIO_WIFI_PWDN
#define GPIO_WIFI_PWDN -1
#endif // !GPIO_WIFI_RESET
#ifdef CONFIG_GSPI_HCI
extern unsigned int oob_irq;
#endif // CONFIG_GSPI_HCI

#ifdef CONFIG_SDIO_HCI
extern int rtw_mp_mode;
#else // !CONFIG_SDIO_HCI
#endif // !CONFIG_SDIO_HCI

int rtw_wifi_gpio_init(void)
{
#ifdef CONFIG_GSPI_HCI
	if (GPIO_WIFI_IRQ > 0) {
		gpio_request(GPIO_WIFI_IRQ, "oob_irq");
		gpio_direction_input(GPIO_WIFI_IRQ);

		oob_irq = gpio_to_irq(GPIO_WIFI_IRQ);

		DBG_8192C("%s oob_irq:%d\n", __func__, oob_irq);
	}
#endif
	if (GPIO_WIFI_RESET > 0)
		gpio_request(GPIO_WIFI_RESET, "wifi_rst");
	if (GPIO_WIFI_POWER > 0)
		gpio_request(GPIO_WIFI_POWER, "wifi_power");

#ifdef CONFIG_SDIO_HCI
#if (defined(CONFIG_RTL8723A) || defined(CONFIG_RTL8723B)) && (MP_DRIVER == 1)
	if (rtw_mp_mode == 1) {
		DBG_871X("%s GPIO_BT_RESET pin special for mp_test\n", __func__);
		if (GPIO_BT_RESET > 0)
			gpio_request(GPIO_BT_RESET, "bt_rst");
	}
#endif
#endif
	return 0;
}

int rtw_wifi_gpio_deinit(void)
{
#ifdef CONFIG_GSPI_HCI
	if (GPIO_WIFI_IRQ > 0)
		gpio_free(GPIO_WIFI_IRQ);
#endif
	if (GPIO_WIFI_RESET > 0)
		gpio_free(GPIO_WIFI_RESET);
	if (GPIO_WIFI_POWER > 0)
		gpio_free(GPIO_WIFI_POWER);

#ifdef CONFIG_SDIO_HCI
#if (defined(CONFIG_RTL8723A) || defined(CONFIG_RTL8723B)) && (MP_DRIVER == 1)
	if (rtw_mp_mode == 1) {
		DBG_871X("%s GPIO_BT_RESET pin special for mp_test\n", __func__);
		if (GPIO_BT_RESET > 0)
			gpio_free(GPIO_BT_RESET);
	}
#endif
#endif
	return 0;
}

/* Customer function to control hw specific wlan gpios */
void rtw_wifi_gpio_wlan_ctrl(int onoff)
{
	switch (onoff) {
	case WLAN_PWDN_OFF:
		DBG_8192C("%s: call customer specific GPIO(%d) to set wifi power down pin to 0\n",
			  __FUNCTION__, GPIO_WIFI_RESET);

#ifndef CONFIG_DONT_BUS_SCAN
		if (GPIO_WIFI_RESET > 0)
			gpio_direction_output(GPIO_WIFI_RESET, 0);
#endif
		break;

	case WLAN_PWDN_ON:
		DBG_8192C("%s: callc customer specific GPIO(%d) to set wifi power down pin to 1\n",
			  __FUNCTION__, GPIO_WIFI_RESET);

		if (GPIO_WIFI_RESET > 0)
			gpio_direction_output(GPIO_WIFI_RESET, 1);
		break;

	case WLAN_POWER_OFF:
		break;

	case WLAN_POWER_ON:
		break;
#ifdef CONFIG_SDIO_HCI
#if (defined(CONFIG_RTL8723A) || defined(CONFIG_RTL8723B)) && (MP_DRIVER == 1)
	case WLAN_BT_PWDN_OFF:
		if (rtw_mp_mode == 1) {
			DBG_871X("%s: call customer specific GPIO to set wifi power down pin to 0\n",
				 __FUNCTION__);
			if (GPIO_BT_RESET > 0)
				gpio_direction_output(GPIO_BT_RESET, 0);
		}
		break;

	case WLAN_BT_PWDN_ON:
		if (rtw_mp_mode == 1) {
			DBG_871X("%s: callc customer specific GPIO to set wifi power down pin to 1 %x\n",
				 __FUNCTION__, GPIO_BT_RESET);

			if (GPIO_BT_RESET > 0)
				gpio_direction_output(GPIO_BT_RESET, 1);
		}
		break;
#endif
#endif
	}
}

#else //ANDROID_2X

#include <mach/ldo.h>

#ifdef CONFIG_RTL8188E
extern int sprd_3rdparty_gpio_wifi_power;
#endif
extern int sprd_3rdparty_gpio_wifi_pwd;
#if (defined(CONFIG_RTL8723A) || defined(CONFIG_RTL8723B))
extern int sprd_3rdparty_gpio_bt_reset;
#endif

int rtw_wifi_gpio_init(void)
{
#if (defined(CONFIG_RTL8723A) || defined(CONFIG_RTL8723B))
	if (sprd_3rdparty_gpio_bt_reset > 0)
		gpio_direction_output(sprd_3rdparty_gpio_bt_reset, 1);
#endif

	return 0;
}

int rtw_wifi_gpio_deinit(void)
{
	return 0;
}

/* Customer function to control hw specific wlan gpios */
void rtw_wifi_gpio_wlan_ctrl(int onoff)
{
	switch (onoff) {
	case WLAN_PWDN_OFF:
		DBG_8192C("%s: call customer specific GPIO to set wifi power down pin to 0\n",
			  __FUNCTION__);
		if (sprd_3rdparty_gpio_wifi_pwd > 0) {
			gpio_set_value(sprd_3rdparty_gpio_wifi_pwd, 0);
		}

		if (sprd_3rdparty_gpio_wifi_pwd == 60) {
			DBG_8192C("%s: turn off VSIM2 2.8V\n", __func__);
			LDO_TurnOffLDO(LDO_LDO_SIM2);
		}
		break;

	case WLAN_PWDN_ON:
		DBG_8192C("%s: callc customer specific GPIO to set wifi power down pin to 1\n",
			  __FUNCTION__);
		if (sprd_3rdparty_gpio_wifi_pwd == 60) {
			DBG_8192C("%s: turn on VSIM2 2.8V\n", __func__);
			LDO_SetVoltLevel(LDO_LDO_SIM2, LDO_VOLT_LEVEL0);
			LDO_TurnOnLDO(LDO_LDO_SIM2);
		}
		if (sprd_3rdparty_gpio_wifi_pwd > 0) {
			gpio_set_value(sprd_3rdparty_gpio_wifi_pwd, 1);
		}
		break;

	case WLAN_POWER_OFF:
#ifdef CONFIG_RTL8188E
#ifdef CONFIG_WIF1_LDO
		DBG_8192C("%s: turn off VDD-WIFI0 1.2V\n", __FUNCTION__);
		LDO_TurnOffLDO(LDO_LDO_WIF1);
#endif //CONFIG_WIF1_LDO

		DBG_8192C("%s: turn off VDD-WIFI0 3.3V\n", __FUNCTION__);
		LDO_TurnOffLDO(LDO_LDO_WIF0);

		DBG_8192C("%s: call customer specific GPIO(%d) to turn off wifi power\n",
			  __FUNCTION__, sprd_3rdparty_gpio_wifi_power);
		if (sprd_3rdparty_gpio_wifi_power != 65535)
			gpio_set_value(sprd_3rdparty_gpio_wifi_power, 0);
#endif
		break;

	case WLAN_POWER_ON:
#ifdef CONFIG_RTL8188E
		DBG_8192C("%s: call customer specific GPIO(%d) to turn on wifi power\n",
			  __FUNCTION__, sprd_3rdparty_gpio_wifi_power);
		if (sprd_3rdparty_gpio_wifi_power != 65535)
			gpio_set_value(sprd_3rdparty_gpio_wifi_power, 1);

		DBG_8192C("%s: turn on VDD-WIFI0 3.3V\n", __FUNCTION__);
		LDO_TurnOnLDO(LDO_LDO_WIF0);
		LDO_SetVoltLevel(LDO_LDO_WIF0, LDO_VOLT_LEVEL1);

#ifdef CONFIG_WIF1_LDO
		DBG_8192C("%s: turn on VDD-WIFI1 1.2V\n", __func__);
		LDO_TurnOnLDO(LDO_LDO_WIF1);
		LDO_SetVoltLevel(LDO_LDO_WIF1, LDO_VOLT_LEVEL3);
#endif //CONFIG_WIF1_LDO
#endif
		break;

	case WLAN_BT_PWDN_OFF:
		DBG_8192C("%s: call customer specific GPIO to set bt power down pin to 0\n",
			  __FUNCTION__);
#if (defined(CONFIG_RTL8723A) || defined(CONFIG_RTL8723B))
		if (sprd_3rdparty_gpio_bt_reset > 0)
			gpio_set_value(sprd_3rdparty_gpio_bt_reset, 0);
#endif
		break;

	case WLAN_BT_PWDN_ON:
		DBG_8192C("%s: callc customer specific GPIO to set bt power down pin to 1\n",
			  __FUNCTION__);
#if (defined(CONFIG_RTL8723A) || defined(CONFIG_RTL8723B))
		if (sprd_3rdparty_gpio_bt_reset > 0)
			gpio_set_value(sprd_3rdparty_gpio_bt_reset, 1);
#endif
		break;
	}
}
#endif //ANDROID_2X

#else // !CONFIG_PLATFORM_SPRD

int rtw_wifi_gpio_init(void)
{
	return 0;
}

void rtw_wifi_gpio_wlan_ctrl(int onoff)
{
}
#endif //CONFIG_PLATFORM_SPRD
