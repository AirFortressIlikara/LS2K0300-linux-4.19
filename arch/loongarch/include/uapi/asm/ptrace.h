/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/
#ifndef _UAPI_ASM_PTRACE_H
#define _UAPI_ASM_PTRACE_H

#include <linux/types.h>
#ifndef __KERNEL__
#include <stdint.h>
#endif

/* For PTRACE_{POKE,PEEK}USR. 0 - 31 are GPRs, 32 is PC, 33 is BADVADDR. */
#define GPR_BASE	0
#define GPR_NUM		32
#define GPR_END		(GPR_BASE + GPR_NUM - 1)
#define PC		(GPR_END + 1)
#define BADVADDR	(GPR_END + 2)

/*
 * This struct defines the way the registers are stored on the stack during a
 * system call/exception.
 *
 * If you add a register here, also add it to regoffset_table[] in
 * arch/loongarch/kernel/ptrace.c.
 */
struct user_pt_regs {
	unsigned long regs[32];

	unsigned long csr_era;
	unsigned long csr_badv;
	unsigned long reserved[11];
};

struct user_fp_state {
	uint64_t    fpr[32];
	uint64_t    fcc;
	uint32_t    fcsr;
};

struct user_lsx_state {
	/* 32 registers, 128 bits width per register. */
	uint64_t regs[32*2];
};

struct user_lasx_state {
	/* 32 registers, 256 bits width per register. */
	uint64_t regs[32*4];
};

/*
 * This structure definition saves the LBT data structure,
 * the data comes from the task_struct structure, format is as follows:
 * regs[0]: thread.lbt.scr0
 * regs[1]: thread.lbt.scr1
 * regs[2]: thread.lbt.scr2
 * regs[3]: thread.lbt.scr3
 * regs[4]: thread.lbt.eflags
 * regs[5]: thread.fpu.ftop
 */
struct user_lbt_state {
	uint64_t regs[6];
};

/* Read and write watchpoint registers.	 */
#define NUM_WATCH_REGS 16
enum pt_watch_style {
	pt_watch_style_la32,
	pt_watch_style_la64
};
struct la32_watch_regs {
	uint32_t addr;
	uint32_t mask;
	/* irw/irwsta/irwmask I R W bits.
	 * bit 0 -- 1 if W bit is usable.
	 * bit 1 -- 1 if R bit is usable.
	 * bit 2 -- 1 if I bit is usable.
	 */
	uint8_t irw;
	uint8_t irwstat;
	uint8_t irwmask;
} __attribute__((aligned(8)));

struct la64_watch_regs {
	uint64_t addr;
	uint64_t mask;
	uint8_t irw;
	uint8_t irwstat;
	uint8_t irwmask;
} __attribute__((aligned(8)));

struct pt_watch_regs {
	/* NUM_WATCH_REGS in user gdb's struct pt_watch_regs, set by gdb. */
	int16_t max_valid;
	/* The number of valid watch register pairs. */
	int16_t num_valid;
	enum pt_watch_style style;
	union {
		struct la32_watch_regs la32[NUM_WATCH_REGS];
		struct la64_watch_regs la64[NUM_WATCH_REGS];
	};
};

#define PTRACE_GET_WATCH_REGS	0xd0
#define PTRACE_SET_WATCH_REGS	0xd1

/* Watch irw/irwmask/irwstat bit definitions */
#define LA_WATCH_W		(1 << 0)
#define LA_WATCH_R		(1 << 1)
#define LA_WATCH_I		(1 << 2)
#define LA_WATCH_IRW	(LA_WATCH_W | LA_WATCH_R | LA_WATCH_I)

#endif /* _UAPI_ASM_PTRACE_H */
