// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Loongson Corporation
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
 *      Sui Jingfeng <suijingfeng@loongson.cn>
 */
#include <drm/drm_vblank.h>
#include <drm/drm_print.h>

#include "lsdc_drv.h"
#include "lsdc_regs.h"
#include "lsdc_irq.h"

/* function to be called in a threaded interrupt context. */
irqreturn_t lsdc_irq_thread(int irq, void *arg)
{
	struct drm_device *ddev = arg;
	struct lsdc_device *ldev = to_lsdc(ddev);
	u32 val = ldev->irq_status;

	/* trigger the vblank event */
	if (val & INT_CRTC0_VS)
		drm_handle_vblank(ddev, 0);

	if (val & INT_CRTC1_VS)
		drm_handle_vblank(ddev, 1);

	return IRQ_HANDLED;
}

/* Function to be called when the IRQ occurs */
irqreturn_t lsdc_irq_handler(int irq, void *arg)
{
	struct drm_device *ddev = arg;
	struct lsdc_device *ldev = to_lsdc(ddev);

	/* Read & Clear the interrupt status */
	ldev->irq_status = lsdc_reg_read32(ldev, LSDC_INT_REG);
	if ((ldev->irq_status & INT_STATUS_MASK) == 0) {
		DRM_DEBUG("no interrupt occurs\n");
		return IRQ_NONE;
	}

	/* Clear the interrupt status */
	lsdc_reg_write32(ldev, LSDC_INT_REG, ldev->irq_status);

	return IRQ_WAKE_THREAD;
}

/*
 * Function to be called when the IRQ occurs
 */
irqreturn_t ls2k_irq_handler(int irq, void *arg)
{
	struct drm_device *ddev = arg;
	struct lsdc_device *ldev = to_lsdc(ddev);
	u32 val;

	/* Read & Clear the interrupt status */
	val = lsdc_reg_read32(ldev, LSDC_INT_REG);
	if ((val & INT_STATUS_MASK) == 0) {
		DRM_WARN("no interrupt occurs\n");
		return IRQ_NONE;
	}

	ldev->irq_status = val;

	/* Clear pending interrupt status */
	if (val & INT_CRTC0_VS)
		val &= ~INT_CRTC0_VS;

	if (val & INT_CRTC1_VS)
		val &= ~INT_CRTC1_VS;

	lsdc_reg_write32(ldev, LSDC_INT_REG, val);

	return IRQ_WAKE_THREAD;
}
