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
#ifndef __PCI_HAL_H__
#define __PCI_HAL_H__

#ifdef CONFIG_RTL8192C
void rtl8192ce_set_hal_ops(_adapter *padapter);
#endif

#ifdef CONFIG_RTL8192D
void rtl8192de_set_hal_ops(_adapter *padapter);
#endif

#ifdef CONFIG_RTL8188E
void rtl8188ee_set_hal_ops(_adapter *padapter);
#endif

void rtw_set_hal_ops(_adapter *padapter);

#endif //__PCIE_HAL_H__

