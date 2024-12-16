// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 Loongson Corporation
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

#include <linux/dma-buf.h>
#include <drm/drm_vma_manager.h>
#include "loongson_drv.h"

struct sg_table *loongson_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct loongson_bo *lbo = gem_to_loongson_bo(obj);
	struct ttm_buffer_object *tbo = &lbo->bo;

	DRM_DEBUG_DRIVER("num pages: %lu\n", tbo->num_pages);

	return drm_prime_pages_to_sg(tbo->ttm->pages, tbo->num_pages);
}

void *loongson_gem_prime_vmap(struct drm_gem_object *obj)
{
	struct loongson_bo *lbo = gem_to_loongson_bo(obj);
	struct ttm_buffer_object *tbo = &lbo->bo;
	struct ttm_bo_kmap_obj *kmap_obj = &lbo->dma_buf_vmap;
	int ret;

	ret = ttm_bo_kmap(tbo, 0, tbo->num_pages, kmap_obj);
	if (ret) {
		DRM_ERROR("ret: %d\n", ret);
		return ERR_PTR(ret);
	}

	DRM_DEBUG_DRIVER("virtual of kmap_obj: %llx\n",
			 (u64)kmap_obj->virtual);

	return kmap_obj->virtual;
}

void loongson_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr)
{
	struct loongson_bo *lbo = gem_to_loongson_bo(obj);

	ttm_bo_kunmap(&lbo->dma_buf_vmap);
}

struct drm_gem_object *
loongson_gem_prime_import_sg_table(struct drm_device *dev,
				   struct dma_buf_attachment *attach,
				   struct sg_table *sg)
{
	struct reservation_object *resv = attach->dmabuf->resv;
	struct loongson_bo *lbo;
	int ret;

	ww_mutex_lock(&resv->lock, NULL);
	ret = loongson_bo_create2(dev,
				  attach->dmabuf->size,
				  PAGE_SIZE,
				  0,
				  sg,
				  resv,
				  &lbo);
	ww_mutex_unlock(&resv->lock);
	if (ret) {
		DRM_ERROR("ret: %d\n", ret);
		return ERR_PTR(ret);
	}

	DRM_DEBUG_DRIVER("attach->dmabuf->size: %lx\n",
			 attach->dmabuf->size);

	return &lbo->gem;
}

int loongson_gem_prime_pin(struct drm_gem_object *obj)
{
	struct loongson_bo *lbo = gem_to_loongson_bo(obj);
	int ret = 0;

	ret = loongson_bo_reserve(lbo, false);
	if (unlikely(ret != 0)) {
		DRM_ERROR("ret: %d\n", ret);
		return ret;
	}

	/* pin buffer into GTT */
	ret = loongson_bo_pin(lbo, TTM_PL_FLAG_SYSTEM, NULL);

	if (ret)
		DRM_ERROR("ret: %d\n", ret);

	loongson_bo_unreserve(lbo);

	return ret;
}

void loongson_gem_prime_unpin(struct drm_gem_object *obj)
{
	struct loongson_bo *lbo = gem_to_loongson_bo(obj);
	int ret = 0;

	ret = loongson_bo_reserve(lbo, false);
	if (unlikely(ret != 0)) {
		DRM_ERROR("ret: %d\n", ret);
		return;
	}

	loongson_bo_unpin(lbo);
	loongson_bo_unreserve(lbo);
}

struct reservation_object *
loongson_gem_prime_res_obj(struct drm_gem_object *obj)
{
	struct loongson_bo *lbo = gem_to_loongson_bo(obj);
	struct ttm_buffer_object *tbo = &lbo->bo;

	return tbo->resv;
}

/**
 * loongson_gem_prime_mmap - &drm_driver.gem_prime_mmap implementation
 * @obj: GEM buffer object
 * @vma: virtual memory area
 *
 * Sets up a userspace mapping of the buffer object's memory in the given
 * virtual memory area.
 *
 * Returns:
 * 0 on success or negative error code.
 */
int loongson_gem_prime_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct loongson_bo *lbo = gem_to_loongson_bo(obj);
	struct ttm_buffer_object *tbo = &lbo->bo;
	struct ttm_bo_device *bdev = tbo->bdev;
	unsigned size = loongson_bo_size(lbo);
	int ret;

	if (!vma->vm_file)
		return -ENODEV;

	/* Check for valid size. */
	if (size < vma->vm_end - vma->vm_start)
		return -EINVAL;

	DRM_DEBUG_DRIVER("vma addr: [%lu, %lu)\n",
			 vma->vm_start, vma->vm_end);

	DRM_DEBUG_DRIVER("bo size: %u\n", size);

	DRM_DEBUG_DRIVER("vm_pgoff: %lu\n", vma->vm_pgoff);

	vma->vm_pgoff += drm_vma_node_offset_addr(&tbo->vma_node) >> PAGE_SHIFT;

	DRM_DEBUG_DRIVER("vm_pgoff: %lu\n", vma->vm_pgoff);

	/* prime mmap does not need to check access, so allow here */
	ret = drm_vma_node_allow(&obj->vma_node, vma->vm_file->private_data);
	if (ret) {
		DRM_ERROR("ret: %d\n", ret);
		return ret;
	}

	ret = ttm_bo_mmap(vma->vm_file, vma, bdev);
	drm_vma_node_revoke(&obj->vma_node, vma->vm_file->private_data);

	return ret;
}
