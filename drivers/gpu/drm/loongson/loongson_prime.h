// SPDX-License-Identifier: GPL-2.0+

#ifndef LOONGSON_PRIME_H_
#define LOONGSON_PRIME_H_

struct sg_table *loongson_gem_prime_get_sg_table(struct drm_gem_object *obj);
void *loongson_gem_prime_vmap(struct drm_gem_object *obj);
void loongson_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr);

struct drm_gem_object *
loongson_gem_prime_import_sg_table(struct drm_device *dev,
				   struct dma_buf_attachment *attach,
				   struct sg_table *sg);

int loongson_gem_prime_pin(struct drm_gem_object *obj);
void loongson_gem_prime_unpin(struct drm_gem_object *obj);

struct reservation_object *
loongson_gem_prime_res_obj(struct drm_gem_object *obj);

int loongson_gem_prime_mmap(struct drm_gem_object *obj,
			    struct vm_area_struct *vma);

#endif
