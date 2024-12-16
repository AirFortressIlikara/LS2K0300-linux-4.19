// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 * Authors:
 *	zhangzhijie <zhangzhijie@loongson.cn>
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kthread.h>
#include <drm/drmP.h>
#include <linux/debugfs.h>
#include "loongson_drv.h"

#ifdef CONFIG_DEBUG_FS
static int loongson_debugfs_get_vbios_dump(struct seq_file *m, void *data);
static int loongson_debugfs_gem_info(struct seq_file *m, void *data);
static int loongson_mm_show(struct seq_file *m, void *data);
;

static const struct drm_info_list loongson_debugfs_list[] = {
	{ "vbios_dump", loongson_debugfs_get_vbios_dump, 0, NULL },
	{ "gem_info", loongson_debugfs_gem_info, 0, NULL },
	{ "mm", loongson_mm_show, 0, NULL },
};

static int loongson_mm_show(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;

	struct drm_printer p = drm_seq_file_printer(m);

	read_lock(&dev->vma_offset_manager->vm_lock);
	drm_mm_print(&dev->vma_offset_manager->vm_addr_space_mm, &p);
	read_unlock(&dev->vma_offset_manager->vm_lock);

	return 0;
}

static int loongson_debugfs_gem_bo_info(int id, void *ptr, void *data)
{
	struct drm_gem_object *gobj = ptr;
	struct loongson_bo *bo = gem_to_loongson_bo(gobj);
	struct seq_file *m = data;

	struct dma_buf_attachment *attachment;
	struct dma_buf *dma_buf;
	unsigned int domain;
	const char *placement;
	unsigned int pin_count;

	domain = bo->bo.mem.mem_type;
	switch (domain) {
	case TTM_PL_VRAM:
		placement = "VRAM";
		break;
	case TTM_PL_TT:
		placement = " GTT";
		break;
	case TTM_PL_SYSTEM:
	default:
		placement = " CPU";
		break;
	}
	seq_printf(m, "\t0x%08x: %12ld byte %s", id, loongson_bo_size(bo),
		   placement);

	pin_count = READ_ONCE(bo->pin_count);
	if (pin_count)
		seq_printf(m, " pin count %d", pin_count);

	dma_buf = READ_ONCE(bo->gem.dma_buf);
	attachment = READ_ONCE(bo->gem.import_attach);

	if (attachment)
		seq_printf(m, " imported from %p", dma_buf);
	else if (dma_buf)
		seq_printf(m, " exported as %p", dma_buf);

	seq_puts(m, "\n");

	return 0;
}

static int loongson_debugfs_gem_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_file *file;
	int r;

	r = mutex_lock_interruptible(&dev->filelist_mutex);
	if (r)
		return r;

	list_for_each_entry (file, &dev->filelist, lhead) {
		struct task_struct *task;

		/*
		 * Although we have a valid reference on file->pid, that does
		 * not guarantee that the task_struct who called get_pid() is
		 * still alive (e.g. get_pid(current) => fork() => exit()).
		 * Therefore, we need to protect this ->comm access using RCU.
		 */
		rcu_read_lock();
		task = pid_task(file->pid, PIDTYPE_PID);
		seq_printf(m, "pid %8d command %s:\n", pid_nr(file->pid),
			   task ? task->comm : "<unknown>");
		rcu_read_unlock();

		spin_lock(&file->table_lock);
		idr_for_each(&file->object_idr, loongson_debugfs_gem_bo_info,
			     m);
		spin_unlock(&file->table_lock);
	}

	mutex_unlock(&dev->filelist_mutex);
	return 0;
}

static int loongson_debugfs_get_vbios_dump(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct loongson_device *ldev = dev->dev_private;

	seq_write(m, ldev->vbios, 256 * 1024);
	seq_puts(m, "\n");
	return 0;
}

int loongson_debugfs_init(struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	int ret;

	ret = drm_debugfs_create_files(loongson_debugfs_list,
				       ARRAY_SIZE(loongson_debugfs_list),
				       minor->debugfs_root, minor);

	if (ret) {
		dev_err(dev->dev, "could not install etnaviv_debugfs_list\n");
		return ret;
	}

	return ret;
}
#endif
