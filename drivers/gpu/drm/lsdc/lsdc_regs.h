/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2020 Loongson Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 */

/*
 * Authors:
 *	Sui Jingfeng <suijingfeng@loongson.cn>
 */

#ifndef __LSDC_REGS_H__
#define __LSDC_REGS_H__

#include "lsdc_drv.h"

/*
 * PLL
 */

#define LSDC_PLL_REF_CLK                100000           /* kHz */

/*
 * Those PLL registers is not in dc's bar space,
 * there are relative to LSXX1000_CFG_REG_BASE.
 */

/* LS2K1000 */
#define LS2K1000_DC_PLL_REG             0x04A0

#define LS2K1000_PIX_PLL0_REG           0x04B0
#define LS2K1000_PIX_PLL1_REG           0x04C0

#ifdef CONFIG_LOONGARCH
/* the 2K1000LA chip register base address was changed */
#define LS2K1000_CFG_REG_BASE           0x1fe00000
#else
#define LS2K1000_CFG_REG_BASE           0x1fe10000
#endif

/* LS7A1000 */
#define LS7A1000_DC_PLL_REG             0x0490

#define LS7A1000_PIX_PLL0_REG           0x04B0
#define LS7A1000_PIX_PLL1_REG           0x04C0

#define LS7A1000_CFG_REG_BASE           0x10010000

/* LS2K0500 */
#define LS2K0500_PIX_PLL0_REG           0x0418
#define LS2K0500_PIX_PLL1_REG           0x0420

#define LS2K0500_CFG_REG_BASE           0x1fe10000

/* LS2K0300 */
#define LS2K0300_PIX_PLL0_REG           0x0410
#define LS2K0300_PIX_PLL1_REG           0x0414
#define LS2K0300_CFG_REG_BASE           0x16000000


/*
 *  CRTC CFG
 */
#define CFG_FB_FMT                      GENMASK(2, 0)
#define CFG_FB_SWITCH                   BIT(7)
#define CFG_OUTPUT_ENABLE               BIT(8)
#define CFG_PANEL_SWITCH                BIT(9)
#define CFG_FB_IDX_FLAG                 BIT(11)
#define CFG_GAMMAR_EN_BIT               BIT(12)
#define CFG_RESET_BIT                   BIT(20)


#define CFG_EN_HSYNC                    BIT(30)
#define CFG_INV_HSYNC                   BIT(31)

#define CFG_EN_VSYNC                    BIT(30)
#define CFG_INV_VSYNC                   BIT(31)


/******** CRTC0 & DVO0 ********/

#define LSDC_CRTC0_CFG_REG              0x1240
#define LSDC_CRTC0_FB_ADDR0_REG         0x1260
#define LSDC_CRTC0_FB_ADDR1_REG         0x1580
#define LSDC_CRTC0_FB_HI_ADDR0_REG      0x15A0
#define LSDC_CRTC0_FB_HI_ADDR1_REG      0x15C0

#define LSDC_CRTC0_FB_ORIGIN_REG        0x1300
#define LSDC_CRTC0_STRIDE_REG           0x1280

#define LSDC_CRTC0_GAMMA_INDEX_REG      0x14e0
#define LSDC_CRTC0_GAMMA_DATA_REG       0x1500

#define LSDC_CRTC0_PANCFG_REG           0x13c0
#define LSDC_CRTC0_PANTIM_REG           0x13e0

#define LSDC_CRTC0_HDISPLAY_REG         0x1400
#define LSDC_CRTC0_HSYNC_REG            0x1420
#define LSDC_CRTC0_VDISPLAY_REG         0x1480
#define LSDC_CRTC0_VSYNC_REG            0x14a0

/******** CTRC1 & DVO1(VGA) ********/

#define LSDC_CRTC1_CFG_REG              0x1250
#define LSDC_CRTC1_FB_ADDR0_REG         0x1270
#define LSDC_CRTC1_FB_ADDR1_REG         0x1590
#define LSDC_CRTC1_FB_HI_ADDR0_REG      0x15B0
#define LSDC_CRTC1_FB_HI_ADDR1_REG      0x15D0
#define LSDC_CRTC1_FB_ORIGIN_REG        0x1310
#define LSDC_CRTC1_STRIDE_REG           0x1290

#define LSDC_CRTC1_GAMMA_INDEX_REG      0x14F0
#define LSDC_CRTC1_GAMMA_DATA_REG       0x1510

#define LSDC_CRTC1_PANCFG_REG           0x13d0
#define LSDC_CRTC1_PANTIM_REG           0x13f0

#define LSDC_CRTC1_HDISPLAY_REG         0x1410
#define LSDC_CRTC1_HSYNC_REG            0x1430
#define LSDC_CRTC1_VDISPLAY_REG         0x1490
#define LSDC_CRTC1_VSYNC_REG            0x14b0

/* hardware cusor related regs */

#define CURSOR_ENABLE_MASK              GENMASK(1, 0)
#define CURSOR_FORMAT_DISABLE           0
#define CURSOR_FORMAT_MONOCHROME        1
#define CURSOR_FORMAT_ARGB8888          2
#define CURSOR_SIZE_64X64_BIT           BIT(2)
#define CURSOR_LOCATION_BIT             BIT(4)

#define LSDC_CURSOR0_CFG_REG            0x1520
#define LSDC_CURSOR0_ADDR_LO_REG        0x1530
#define LSDC_CURSOR0_ADDR_HI_REG        0x15e0
#define LSDC_CURSOR0_POSITION_REG       0x1540
#define LSDC_CURSOR0_BG_COLOR_REG       0x1550  /* background color*/
#define LSDC_CURSOR0_FG_COLOR_REG       0x1560  /* foreground color*/

/* LS7A2000 have two hardware cursor */

#define LSDC_CURSOR1_CFG_REG            0x1670
#define LSDC_CURSOR1_ADDR_LO_REG        0x1680
#define LSDC_CURSOR1_ADDR_HI_REG        0x16e0
#define LSDC_CURSOR1_POSITION_REG       0x1690
#define LSDC_CURSOR1_BG_COLOR_REG       0x16A0  /* background color */
#define LSDC_CURSOR1_FG_COLOR_REG       0x16B0  /* foreground color */

/*
 * DC Interrupt Control Register, 32bit, Address Offset: 1570
 *
 * Bits  0:10 inidicate the interrupt type, read only
 * Bits 16:26 control if the specific interrupt corresponding to bit 0~10
 * is enabled or not. Write 1 to enable, write 0 to disable
 *
 * RF: Read Finished
 * IDBU : Internal Data Buffer Underflow
 * IDBFU : Internal Data Buffer Fatal Underflow
 *
 * +----+----+----+----+----+--------+--------+--------+
 * | 31 | 30 | 29 | 28 | 27 |   26   |   25   |   24   |
 * +----+----+----+----+----+--------+--------+--------+
 * |          N/A           | Interrupt Enable Control |
 * +------------------------+--------------------------+
 *
 * +----+----+----+----+----+----+----+----+
 * | 23 | 22 | 21 | 20 | 19 | 18 | 17 | 16 |
 * +----+----+----+----+----+----+----+----+
 * |    Interrupt Enable Control Bits      |
 * +---------------------------------------+
 *
 * +----+----+----+----+------+-----------+-----------+----------+
 * | 15 | 14 | 13 | 12 |  11  |     10    |     9     |     8    |
 * +----+----+----+----+------+-----------+-----------+----------+
 * |           N/A            | FB0 IDBFU | FB1 IDBFU | FB0 IDBU |
 * +--------------------------+-----------+-----------+----------+
 *
 * +----------+--------+--------+-----------+
 * |     7    |  6     |   5    |     4     |
 * +----------+--------+--------+-----------+
 * | FB1 IDBU | FB0 RF | FB1 RF | Cursor RF |
 * +----------+--------+--------+-----------+
 *
 * +------------+------------+------------+------------+
 * |      3     |     2      |     1      |     0      |
 * +------------+------------+------------+------------+
 * | DVO0 HSYNC | DVO0 VSYNC | DVO1 HSYNC | DVO1 VSYNC |
 * +------------+------------+------------+------------+
 *
 */

#define LSDC_INT_REG                           0x1570

#define INT_CRTC0_VS                           BIT(2)
#define INT_CRTC0_HS                           BIT(3)
#define INT_CRTC0_RF                           BIT(6)
#define INT_CRTC0_IDBU                         BIT(8)
#define INT_CRTC0_IDBFU                        BIT(10)

#define INT_CURSOR_RF                          BIT(4)

#define INT_CRTC1_VS                           BIT(0)
#define INT_CRTC1_HS                           BIT(1)
#define INT_CRTC1_RF                           BIT(5)
#define INT_CRTC1_IDBU                         BIT(7)
#define INT_CRTC1_IDBFU                        BIT(9)


#define INT_CRTC0_VS_EN                        BIT(2 + 16)
#define INT_CRTC0_HS_EN                        BIT(3 + 16)
#define INT_CRTC0_RF_EN                        BIT(6 + 16)
#define INT_CRTC0_IDBU_EN                      BIT(8 + 16)
#define INT_CRTC0_IDBFU_EN                     BIT(10 + 16)

#define INT_CURSOR_RF_EN                       BIT(4 + 16)

#define INT_CRTC1_VS_EN                        BIT(0 + 16)
#define INT_CRTC1_HS_EN                        BIT(1 + 16)
#define INT_CRTC1_RF_EN                        BIT(5 + 16)
#define INT_CRTC1_IDBU_EN                      BIT(7 + 16)
#define INT_CRTC1_IDBFU_EN                     BIT(9 + 16)


#define INT_STATUS_MASK                        0x07ff


/*
 * GPIO emulated I2C, LS7A1000 Only
 *
 * DVO : Digital Video Output
 * There are two GPIO emulated i2c in LS7A1000 for reading edid from
 * the monitor, those registers are in the DC control register space.
 *
 * GPIO data register
 *  Address offset: 0x1650
 *   +---------------+-----------+-----------+
 *   | 7 | 6 | 5 | 4 |  3  |  2  |  1  |  0  |
 *   +---------------+-----------+-----------+
 *   |               |    DVO1   |    DVO0   |
 *   +      N/A      +-----------+-----------+
 *   |               | SCL | SDA | SCL | SDA |
 *   +---------------+-----------+-----------+
 */
#define LS7A_DC_GPIO_DAT_REG                   0x1650

/*
 *  GPIO Input/Output direction control register
 *  Address offset: 0x1660
 *  write 1 for Input, 0 for Output.
 */
#define LS7A_DC_GPIO_DIR_REG                   0x1660

static inline u32 lsdc_reg_read32(struct lsdc_device * const ldev,
				  u32 offset)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&ldev->reglock, flags);
	val = readl(ldev->reg_base + offset);
	spin_unlock_irqrestore(&ldev->reglock, flags);

	return val;
}

static inline void lsdc_reg_write32(struct lsdc_device * const ldev,
				    u32 offset, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&ldev->reglock, flags);
	writel(val, ldev->reg_base + offset);
	spin_unlock_irqrestore(&ldev->reglock, flags);
}

/*
 *  7A2000  HDMI Encoder
 */
#define HDMI_EN                 BIT(0)
#define HDMI_PACKET_EN          BIT(1)

#define HDMI0_ZONE_REG          0x1700
#define HDMI1_ZONE_REG          0x1710

#define HDMI0_CTRL_REG          0x1720
#define HDMI1_CTRL_REG          0x1730

#endif
