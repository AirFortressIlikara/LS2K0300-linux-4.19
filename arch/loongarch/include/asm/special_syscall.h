#ifndef __LS_SPECIAL_SYSCALL_H
#define __LS_SPECIAL_SYSCALL_H
#include <linux/posix_types.h>
#include <linux/compiler.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/nospec.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/file.h>

#include <linux/atomic.h>

struct lazy_dup_info {
	unsigned int l_flag;
	unsigned int src_fd;
};

#define lazy_dup_t u64
#define FLAG_LAZY 0x1234abcd
#define REFILL_ACCESS(_x) (((struct lazy_dup_info *)(_x))->l_flag == FLAG_LAZY)
#define GET_DUP_FDS(__f)	(&__f->close_on_exec[__f->max_fds/BITS_PER_LONG])
#define GET_LAZY_FDS(__f)	(&__f->close_on_exec[2*__f->max_fds/BITS_PER_LONG])

#define COPY_LS_BITMAPS(__ofdt, __nfdt, NAME, set, cpy) \
	unsigned long *n_##NAME##_map = GET_##NAME##_FDS(__nfdt);\
	unsigned long *o_##NAME##_map = GET_##NAME##_FDS(__ofdt);\
	memcpy(n_##NAME##_map, o_##NAME##_map, cpy);\
	memset((char *)n_##NAME##_map + cpy, 0, set);

#define COPY_LS_DUP(_ofdt, _nfdt, set, cpy)\
	do {\
		COPY_LS_BITMAPS(_ofdt, _nfdt, DUP, set, cpy)\
		COPY_LS_BITMAPS(_ofdt, _nfdt, LAZY, set, cpy)\
	} while (0)

static inline void __set_lazy_fd(unsigned int lfd, unsigned int dfd, struct fdtable *fdt)
{
	__set_bit(dfd, GET_DUP_FDS(fdt));
	__set_bit(lfd, GET_LAZY_FDS(fdt));
}

static inline bool fd_is_dup(unsigned int fd, struct fdtable *fdt)
{
	return test_bit(fd, GET_DUP_FDS(fdt));
}

static inline bool fd_is_lazy(unsigned int fd, struct fdtable *fdt)
{
	return test_bit(fd, GET_LAZY_FDS(fdt));
}

static inline void __clear_dup_fd(unsigned int fd, struct fdtable *fdt)
{
	__clear_bit(fd, GET_DUP_FDS(fdt));
}

static inline void __clear_lazy_fd(unsigned int fd, struct fdtable *fdt)
{
	unsigned long *lazy_map = GET_LAZY_FDS(fdt);
	__clear_bit(fd, lazy_map);
	if (fdt->max_fds == NR_OPEN_DEFAULT && !lazy_map[0])
		*(lazy_map - 1) = 0;
}

static inline void lazy_info_init(unsigned int fd, lazy_dup_t *p)
{
	struct lazy_dup_info *lp = (struct lazy_dup_info *)p;
	lp->l_flag = FLAG_LAZY;
	lp->src_fd = fd;
}

static inline struct file *lazy_info_refill(lazy_dup_t *p, unsigned int fd, struct fdtable *fdt)
{
	struct lazy_dup_info *lp = (struct lazy_dup_info *)p;
	struct file *file;
	file = fget_raw(lp->src_fd);
	if (file) {
		rcu_assign_pointer(fdt->fd[fd], file);
		__clear_lazy_fd(fd, fdt);
		return file;
	}
	return NULL;
}

#define	ls_get_file(fd, fdt)	\
	do {\
		u64 ___fp = (long)rcu_dereference_raw(fdt->fd[fd]);\
		if (unlikely(REFILL_ACCESS(&___fp))) {\
			struct file *___file; \
			___file = lazy_info_refill(&___fp, fd, fdt);\
			return ___file ? ___file : NULL;\
		} else\
			return (struct file *)___fp;\
	} while (0)

#define ls_dup_check(fd, fdt)\
({	 \
	if (fd_is_dup(fd, fdt))\
		flush_dup(fd, fdt);\
	fd_is_lazy(fd, fdt);\
})

void flush_dup(int fd, struct fdtable *fdt);
#endif
