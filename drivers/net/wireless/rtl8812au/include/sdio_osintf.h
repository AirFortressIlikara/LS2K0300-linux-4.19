/******************************************************************************
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
#ifndef __SDIO_OSINTF_H__
#define __SDIO_OSINTF_H__



u8 sd_hal_bus_init(PADAPTER padapter);
u8 sd_hal_bus_deinit(PADAPTER padapter);
void sd_c2h_hdl(PADAPTER padapter);

#ifdef PLATFORM_OS_CE
extern NDIS_STATUS ce_sd_get_dev_hdl(PADAPTER padapter);
SD_API_STATUS ce_sd_int_callback(SD_DEVICE_HANDLE hDevice,
				 PADAPTER padapter);
extern void sd_setup_irs(PADAPTER padapter);
#endif

#endif

