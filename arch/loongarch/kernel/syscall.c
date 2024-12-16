// SPDX-License-Identifier: GPL-2.0+
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/
#include <linux/capability.h>
#include <linux/errno.h>
#include <linux/linkage.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/utsname.h>
#include <linux/unistd.h>
#include <linux/compiler.h>
#include <linux/sched/task_stack.h>
#include <linux/frame.h>

#include <asm/asm.h>
#include <asm/cacheflush.h>
#include <asm/asm-offsets.h>
#include <asm/signal.h>
#include <asm/shmparam.h>
#include <asm/switch_to.h>
#include <asm-generic/syscalls.h>

#undef __SYSCALL
#define __SYSCALL(nr, call)	[nr] = (call),

SYSCALL_DEFINE6(mmap, unsigned long, addr, unsigned long, len,
	unsigned long, prot, unsigned long, flags, unsigned long,
	fd, off_t, offset)
{
	if (offset & ~PAGE_MASK)
		return -EINVAL;
	return ksys_mmap_pgoff(addr, len, prot, flags, fd,
			       offset >> PAGE_SHIFT);
}

SYSCALL_DEFINE6(mmap2, unsigned long, addr, unsigned long, len,
	unsigned long, prot, unsigned long, flags, unsigned long, fd,
	unsigned long, pgoff)
{
	if (pgoff & (~PAGE_MASK >> 12))
		return -EINVAL;

	return ksys_mmap_pgoff(addr, len, prot, flags, fd,
			       pgoff >> (PAGE_SHIFT - 12));
}
#ifdef CONFIG_LS_DUP
void flush_dup(int fd, struct fdtable *fdt)
{
	unsigned int i, j = 0;
	unsigned long *lazy_fds = GET_LAZY_FDS(fdt);
	for (;;) {
		unsigned long set;
		i = j * BITS_PER_LONG;
		if (i >= fdt->max_fds)
			break;
		set = lazy_fds[j++];
		while (set) {
			if (set & 1) {
				lazy_dup_t ret = (long)rcu_dereference_raw(fdt->fd[i]);
				struct lazy_dup_info *fp = (struct lazy_dup_info *)&ret;
				if (fp->src_fd == fd) {
					if (!lazy_info_refill(&ret, i, fdt))
						pr_err("flush_dup: refill NULL!!\n");
				}
			}
			i++;
			set >>= 1;
		}
	}

	__clear_dup_fd(fd, fdt);
}

static unsigned int find_next_fd(struct fdtable *fdt, unsigned int start)
{
	unsigned int maxfd = fdt->max_fds;
	unsigned int maxbit = maxfd / BITS_PER_LONG;
	unsigned int bitbit = start / BITS_PER_LONG;

	bitbit = find_next_zero_bit(fdt->full_fds_bits, maxbit, bitbit) * BITS_PER_LONG;
	if (bitbit > maxfd)
		return maxfd;
	if (bitbit > start)
		start = bitbit;
	return find_next_zero_bit(fdt->open_fds, maxfd, start);
}

static inline void __set_open_fd(unsigned int fd, struct fdtable *fdt)
{
	__set_bit(fd, fdt->open_fds);
	fd /= BITS_PER_LONG;
	if (!~fdt->open_fds[fd])
		__set_bit(fd, fdt->full_fds_bits);
}

extern int expand_files(struct files_struct *files, unsigned int nr);
static int ___alloc_fd(struct files_struct *files, unsigned s_fd)
{
	unsigned int fd;
	int error;
	struct fdtable *fdt;
	spin_lock(&files->file_lock);
repeat:
	fdt = files_fdtable(files);
	if (s_fd >= fdt->max_fds || s_fd < 0 || !(fdt->fd[s_fd])) {
		error = -EBADF;
		goto out;
	}
	fd = 0;
	if (fd < files->next_fd)
		fd = files->next_fd;

	if (fd < fdt->max_fds)
		fd = find_next_fd(fdt, fd);

	/*
	 * N.B. For clone tasks sharing a files structure, this test
	 * will limit the total number of files that can be opened.
	 */
	error = -EMFILE;
	if (fd >= rlimit(RLIMIT_NOFILE))
		goto out;

	error = expand_files(files, fd);
	if (error < 0)
		goto out;
	if (error)
		goto repeat;

	if (files->next_fd >= 0)
		files->next_fd = fd + 1;

	__set_open_fd(fd, fdt);

	__set_lazy_fd(fd, s_fd, fdt);
	if (test_bit(fd, fdt->close_on_exec))
		__clear_bit(fd, fdt->close_on_exec);

	error = fd;
	/* Sanity check */
	if (rcu_access_pointer(fdt->fd[fd]) != NULL) {
		printk(KERN_WARNING "alloc_fd: slot %d not NULL!\n", fd);
		rcu_assign_pointer(fdt->fd[fd], NULL);
	}

out:
	spin_unlock(&files->file_lock);
	return error;
}

static int ls_dup(unsigned int fildes)
{
	int ret = -EBADF;
	lazy_dup_t info;
	lazy_info_init(fildes, &info);
	ret = ___alloc_fd(current->files, fildes);

	if (ret >= 0)
		fd_install(ret, (struct file *)info);
	else if (ret == -EINVAL)
		return ksys_dup(fildes);

	return ret;
}

#endif
void *sys_call_table[__NR_syscalls] = {
	[0 ... __NR_syscalls - 1] = sys_ni_syscall,
#include <asm/unistd.h>
};

void loongarch_syscall_init(void)
{
#ifdef CONFIG_LS_DUP
	sys_call_table[__NR_dup] = ls_dup;
#endif
}
