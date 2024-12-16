/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/
#ifndef _UAPI_ASM_SIGCONTEXT_H
#define _UAPI_ASM_SIGCONTEXT_H

#include <linux/types.h>
#include <linux/posix_types.h>

/* FP context was used */
#define USED_FP			(1 << 0)
/* load/store access flags trigger address error */
#define ADRERR_RD		(1 << 30)
#define ADRERR_WR		(1 << 31)

#define FPU_REG_WIDTH		256

struct sigcontext {
	__u64	sc_pc;
	__u64	sc_regs[32];
	/*
	 * bits allocation of sc_flags:
	 * bit 0 : USED_FP
	 * bit 30: ADRERR_RD
	 * bit 31: ADRERR_WR
	 */
	__u32	sc_flags;
	__u32	sc_fcsr;
	__u32	sc_none;
	__u64	sc_fcc;
	__u64	sc_scr[4];
	union {
		__u32	val32[FPU_REG_WIDTH / 32];
		__u64	val64[FPU_REG_WIDTH / 64];
	} sc_fpregs[32] __attribute__((aligned(32)));
	__u8	sc_reserved[4096] __attribute__((__aligned__(16)));
};

#endif /* _UAPI_ASM_SIGCONTEXT_H */
