/* SPDX-License-Identifier: GPL-2.0 */
#ifndef LOONGSON_CPU_H_
#define LOONGSON_CPU_H_

#include <linux/types.h>
#include <linux/bits.h>

#include <asm/mipsregs.h>
#include <asm/cpu.h>

/*
 * Loongson CPU Option encodings
 */
#define LOONGSON_CPU_CSR	MBIT_ULL(0)	/* CPU has CSR feature */
#define LOONGSON_CPU_LASX	MBIT_ULL(1)	/* CPU supports LoongISA Advanced SIMD eXtentions */
#define LOONGSON_CPU_LAMO	MBIT_ULL(2)	/* CPU supports LoongISA Atomic Memory Operation */
#define LOONGSON_CPU_CAM	MBIT_ULL(3)	/* CPU has CAM feature */
#define LOONGSON_CPU_VZ		MBIT_ULL(4)	/* CPU has VZ feature */
#define LOONGSON_CPU_GFT	MBIT_ULL(5)	/* CPU has Global Fix Timer feature */
#define LOONGSON_CPU_LFT	MBIT_ULL(6)	/* CPU has Local Fix Timer feature */
#define LOONGSON_CPU_CFG	MBIT_ULL(7)	/* CPU has cpu cfg */
#define LOONGSON_CPU_SCALEFREQ	MBIT_ULL(8)	/* CPU support scale cpufreq */

#define LOONGSON_CPU_LBT	MBIT_ULL(9)	/* CPU has lbt */

#define LOONGSON_CPU_MSI128	MBIT_ULL(32)	/* CPU has MSI128 */
#define LOONGSON_CPU_MSI256	MBIT_ULL(33)	/* CPU has MSI256 */
#define LOONGSON_CPU_EXTIOI	MBIT_ULL(34)	/* CPU has EXTIOI */
#define LOONGSON_CPU_CSRIPI	MBIT_ULL(35)	/* CPU has CSR IPI */


#define __loongson_opt(opt)	(cpu_data[0].loongson_options & (opt))

#ifndef loongson_cpu_has_csr
# define loongson_cpu_has_csr	__loongson_opt(LOONGSON_CPU_CSR)
#endif

#if defined(CONFIG_CPU_HAS_LASX) 
# define loongson_cpu_has_lasx	__loongson_opt(LOONGSON_CPU_LASX)
#elif !defined(loongson_cpu_has_lasx)
# define loongson_cpu_has_lasx	0
#endif

#ifndef loongson_cpu_has_lamo
# define loongson_cpu_has_lamo	__loongson_opt(LOONGSON_CPU_LAMO)
#endif

#ifndef loongson_cpu_has_cam
# define loongson_cpu_has_cam	__loongson_opt(LOONGSON_CPU_CAM)
#endif

#ifndef loongson_cpu_has_vz
# define loongson_cpu_has_vz	__loongson_opt(LOONGSON_CPU_VZ)
#endif

#ifndef loongson_cpu_has_gft
# define loongson_cpu_has_gft	__loongson_opt(LOONGSON_CPU_GFT)
#endif

#ifndef loongson_cpu_has_lft
# define loongson_cpu_has_lft	__loongson_opt(LOONGSON_CPU_LFT)
#endif

#ifndef loongson_cpu_has_lbt
# define loongson_cpu_has_lbt __loongson_opt(LOONGSON_CPU_LBT)
#endif

#ifndef loongson_cpu_has_msi128
# define loongson_cpu_has_msi128	__loongson_opt(LOONGSON_CPU_MSI128)
#endif

#ifndef loongson_cpu_has_msi256
# define loongson_cpu_has_msi256	__loongson_opt(LOONGSON_CPU_MSI256)
#endif

#ifndef loongson_cpu_has_extioi
# define loongson_cpu_has_extioi	__loongson_opt(LOONGSON_CPU_EXTIOI)
#endif

#ifndef loongson_cpu_has_csripi
# define loongson_cpu_has_csripi	__loongson_opt(LOONGSON_CPU_CSRIPI)
#endif

#ifndef loongson_cpu_has_cpucfg
# define loongson_cpu_has_cpucfg	__loongson_opt(LOONGSON_CPU_CFG)
#endif

#ifndef loongson_cpu_has_scalefreq
# define loongson_cpu_has_scalefreq	__loongson_opt(LOONGSON_CPU_SCALEFREQ)
#endif

static inline u32 read_cpucfg(u32 reg)
{
	u32 __res;

	__asm__ __volatile__(
		"parse_r __res,%0\n\t"
		"parse_r reg,%1\n\t"
		".insn \n\t"
		".word (0xc8080118 | (reg << 21) | (__res << 11))\n\t"
		:"=r"(__res)
		:"r"(reg)
		:
		);
	return __res;
}

/* Bit Domains for CFG registers */
#define LOONGSON_CFG0		0x0
#define LOONGSON_CFG0_PRID 	GENMASK(31, 0)

#define LOONGSON_CFG1 		0x1
#define LOONGSON_CFG1_FP	BIT(0)
#define LOONGSON_CFG1_FPREV	GENMASK(3, 1)
#define LOONGSON_CFG1_MMI	BIT(4)
#define LOONGSON_CFG1_MSA1	BIT(5)
#define LOONGSON_CFG1_MSA2	BIT(6)
#define LOONGSON_CFG1_CGP	BIT(7)
#define LOONGSON_CFG1_WRP	BIT(8)
#define LOONGSON_CFG1_LSX1	BIT(9)
#define LOONGSON_CFG1_LSX2	BIT(10)
#define LOONGSON_CFG1_LASX	BIT(11)
#define LOONGSON_CFG1_R6FXP	BIT(12)
#define LOONGSON_CFG1_R6CRCP	BIT(13)
#define LOONGSON_CFG1_R6FPP	BIT(14)
#define LOONGSON_CFG1_CNT64	BIT(15)
#define LOONGSON_CFG1_LSLDR0	BIT(16)
#define LOONGSON_CFG1_LSPREF	BIT(17)
#define LOONGSON_CFG1_LSPREFX	BIT(18)
#define LOONGSON_CFG1_LSSYNCI	BIT(19)
#define LOONGSON_CFG1_LSUCA	BIT(20)
#define LOONGSON_CFG1_LLSYNC	BIT(21)
#define LOONGSON_CFG1_TGTSYNC	BIT(22)
#define LOONGSON_CFG1_LLEXC	BIT(23)
#define LOONGSON_CFG1_SCRAND	BIT(24)
#define LOONGSON_CFG1_MUALP	BIT(25)
#define LOONGSON_CFG1_KMUALEN	BIT(26)
#define LOONGSON_CFG1_ITLBT	BIT(27)
#define LOONGSON_CFG1_LSUPERF	BIT(28)
#define LOONGSON_CFG1_SFBP	BIT(29)
#define LOONGSON_CFG1_CDMAP	BIT(30)

#define LOONGSON_CFG2 		0x2
#define LOONGSON_CFG2_LEXT1	BIT(0)
#define LOONGSON_CFG2_LEXT2	BIT(1)
#define LOONGSON_CFG2_LEXT3	BIT(2)
#define LOONGSON_CFG2_LSPW	BIT(3)
#define LOONGSON_CFG2_LBT1	BIT(4)
#define LOONGSON_CFG2_LBT2	BIT(5)
#define LOONGSON_CFG2_LBT3	BIT(6)
#define LOONGSON_CFG2_LBTMMU	BIT(7)
#define LOONGSON_CFG2_LPMP	BIT(8)
#define LOONGSON_CFG2_LPMPREV	GENMASK(11, 9)
#define LOONGSON_CFG2_LAMO	BIT(12)
#define LOONGSON_CFG2_LPIXU	BIT(13)
#define LOONGSON_CFG2_LPIXUN	BIT(14)
#define LOONGSON_CFG2_LVZP	BIT(15)
#define LOONGSON_CFG2_LVZREV	GENMASK(18, 16)
#define LOONGSON_CFG2_LGFTP	BIT(19)
#define LOONGSON_CFG2_LGFTPREV	GENMASK(22, 20)
#define LOONGSON_CFG2_LLFTP	BIT(23)
#define LOONGSON_CFG2_LLFTPREV	GENMASK(24, 26)
#define LOONGSON_CFG2_LCSRP	BIT(27)
#define LOONGSON_CFG2_DISBLKLY	BIT(28)

#define LOONGSON_CFG3		0x3
#define LOONGSON_CFG3_LCAMP	BIT(0)
#define LOONGSON_CFG3_LCAMREV	GENMASK(3, 1)
#define LOONGSON_CFG3_LCAMNUM	GENMASK(11, 4)
#define LOONGSON_CFG3_LCAMKW	GENMASK(19, 12)
#define LOONGSON_CFG3_LCAMVW	GENMASK(27, 20)

#define LOONGSON_CFG4 		0x4
#define LOONGSON_CFG4_CCFREQ	GENMASK(31, 0)

#define LOONGSON_CFG5 		0x5
#define LOONGSON_CFG5_CFM	GENMASK(15, 0)
#define LOONGSON_CFG5_CFD	GENMASK(31, 16)

#define LOONGSON_CFG6 		0x6

#define LOONGSON_CFG7 		0x7
#define LOONGSON_CFG7_GCCAEQRP	BIT(0)
#define LOONGSON_CFG7_UCAWINP	BIT(1)

static inline u32 csr_readl(u32 reg)
{
	u32 __res;

	/* RDCSR reg, val */
	__asm__ __volatile__(
		"parse_r __res,%0\n\t"
		"parse_r reg,%1\n\t"
		".insn \n\t"
		".word (0xc8000118 | (reg << 21) | (__res << 11))\n\t"
		:"=r"(__res)
		:"r"(reg)
		:
		);
	return __res;
}

static inline u64 csr_readq(u32 reg)
{
	u64 __res;

	/* DWRCSR reg, val */
	__asm__ __volatile__(
		"parse_r __res,%0\n\t"
		"parse_r reg,%1\n\t"
		".insn \n\t"
		".word (0xc8020118 | (reg << 21) | (__res << 11))\n\t"
		:"=r"(__res)
		:"r"(reg)
		:
		);
	return __res;
}

static inline void csr_writel(u32 val, u32 reg)
{
	/* WRCSR reg, val */
	__asm__ __volatile__(
		"parse_r reg,%0\n\t"
		"parse_r val,%1\n\t"
		".insn \n\t"
		".word (0xc8010118 | (reg << 21) | (val << 11))\n\t"
		:
		:"r"(reg),"r"(val)
		:
		);
}

static inline void csr_writeq(u64 val, u32 reg)
{
	/* DWRCSR reg, val */
	__asm__ __volatile__(
		"parse_r reg,%0\n\t"
		"parse_r val,%1\n\t"
		".insn \n\t"
		".word (0xc8030118 | (reg << 21) | (val << 11))\n\t"
		:
		:"r"(reg),"r"(val)
		:
		);
}

static inline unsigned long long read_guest_csr(int reg)
{
	unsigned long long __res;

	__asm__ __volatile__(
		"parse_r __res,%0\n\t"
		"parse_r reg,%1\n\t"
		".insn \n\t"
		".word (0xc8040118 | (reg << 21) | (__res << 11))\n\t"
		:"=r"(__res)
		:"r"(reg)
		:
		);
	return __res;
}

static inline void write_guest_csr(int reg,unsigned long long val)
{
	__asm__ __volatile__(
		"parse_r reg,%0\n\t"
		"parse_r val,%1\n\t"
		".insn \n\t"
		".word (0xc8050118 | (reg << 21) | (val << 11))\n\t"
		:
		:"r"(reg),"r"(val)
		:
		);
}

static inline unsigned long long dread_guest_csr(int reg)
{
	unsigned long long __res;

	__asm__ __volatile__(
		"parse_r __res,%0\n\t"
		"parse_r reg,%1\n\t"
		".insn \n\t"
		".word (0xc8060118 | (reg << 21) | (__res << 11))\n\t"
		:"=r"(__res)
		:"r"(reg)
		:
		);
	return __res;
}

static inline void dwrite_guest_csr(int reg,unsigned long long val)
{
	__asm__ __volatile__(
		"parse_r reg,%0\n\t"
		"parse_r val,%1\n\t"
		".insn \n\t"
		".word (0xc8070118 | (reg << 21) | (val << 11))\n\t"
		:
		:"r"(reg),"r"(val)
		:
		);
}

#define LOONGSON_CSR_FEATURES	0x8
#define LOONGSON_CSRF_TEMP	BIT_ULL(0)
#define LOONGSON_CSRF_NODECNT	BIT_ULL(1)
#define LOONGSON_CSRF_MSI	BIT_ULL(2)
#define LOONGSON_CSRF_EXTIOI	BIT_ULL(3)
#define LOONGSON_CSRF_IPI	BIT_ULL(4)
#define LOONGSON_CSRF_FREQ	BIT_ULL(5)
#define LOONGSON_CSRF_FREQSCALE	BIT_ULL(6)
#define LOONGSON_CSRF_DVFSV1	BIT_ULL(7)
#define LOONGSON_CSRF_VM	BIT_ULL(11)

#define LOONGSON_CSR_VENDOR	0x10 /* Vendor name string, should be "Loongson" */
#define LOONGSON_CSR_CPUNAME	0x20 /* Processor name string */
#define LOONGSON_CSR_NODECNT	0x408
#define LOONGSON_CSR_OTHER_FUNC	0x420
#define CSR_OTHER_FUNC_EXT_INT_EN	BIT_ULL(48)
#define LS_CSR_ANYSEND_OTHER_FUNC	0x424
#define CSR_OTHER_FUNC_EXT_IOI	(CSR_OTHER_FUNC_EXT_INT_EN >> 32)
#define LOONGSON_CSR_CPUTEMP	0x428

/* PerCore CSR, only accessable by local cores */
#define LOONGSON_CSR_IPI_STATUS	0x1000
#define LOONGSON_CSR_IPI_EN	0x1004
#define LOONGSON_CSR_IPI_SET	0x1008
#define LOONGSON_CSR_IPI_CLEAR	0x100c
#define LOONGSON_CSR_MAIL_BUF0	0x1020
#define LOONGSON_CSR_MAIL_BUF1	0x1028
#define LOONGSON_CSR_MAIL_BUF2	0x1030
#define LOONGSON_CSR_MAIL_BUF3	0x1038
#define LOONGSON_CSR_IPI_SEND	0x1040

#define CSR_IPI_SEND_IP_SHIFT	0
#define CSR_IPI_SEND_CPU_SHIFT	16
#define CSR_IPI_SEND_BLOCKING	BIT(31)

#define LOONGSON_CSR_MAIL_SEND	0x1048

#define CSR_MAIL_SEND_BLOCKING	BIT_ULL(31)
#define CSR_MAIL_SEND_BOX_SHIFT	2
#define CSR_MAIL_SEND_BOX_LOW(box)	(box << 1)
#define CSR_MAIL_SEND_BOX_HIGH(box)	((box << 1) + 1) 
#define CSR_MAIL_SEND_CPU_SHIFT	16
#define CSR_MAIL_SEND_BUF_SHIFT	32
#define CSR_MAIL_SEND_H32_MASK	0xFFFFFFFF00000000ULL

#define LOONGSON_CSR_ANY_SEND	0x1158

#define CSR_ANY_SEND_BLOCKING	BIT_ULL(31)
#define CSR_ANY_SEND_NODE_SHIFT	18
#define CSR_ANY_SEND_MASK_SHIFT	27
#define CSR_ANY_SEND_BUF_SHIFT	32
#define CSR_ANY_SEND_H32_MASK	0xFFFFFFFF00000000ULL

#define LOONGSON_CSR_EXTIOI_NODEMAP_BASE	0x14a0
#define LOONGSON_CSR_EXTIOI_IPMAP_BASE		0x14c0
#define LOONGSON_CSR_EXTIOI_EN_BASE		0x1600
#define CSR_EXTIOI_VECTOR_NUM			256
#define LOONGSON_CSR_EXTIOI_BOUNCE_BASE		0x1680
#define LOONGSON_CSR_EXTIOI_ISR_BASE		0x1800
#define LOONGSON_CSR_EXTIOI_ROUTE_BASE		0x1c00

static inline u64 drdtime(void)
{
	int rID = 0;
	u64 val = 0;

	__asm__ __volatile__(
		"parse_r rID,%0\n\t"
		"parse_r val,%1\n\t"
		".insn \n\t"
		".word (0xc8090118 | (rID << 21) | (val << 11))\n\t"
		:"=r"(rID),"=r"(val)
		:
		);
	return val;
}

static inline void csr_any_send(unsigned int addr, unsigned int data, unsigned int data_mask, unsigned int node)
{
	uint64_t val = 0;

	val = CSR_ANY_SEND_BLOCKING | addr;
	val |= (node << CSR_ANY_SEND_NODE_SHIFT);
	val |= (data_mask << CSR_ANY_SEND_MASK_SHIFT);
	val |= ((uint64_t)data << CSR_ANY_SEND_BUF_SHIFT);
	csr_writeq(val, LOONGSON_CSR_ANY_SEND);
}

#define GSEX_CODE(x)	((x & 0x7c) >> 2)
#define GSEX_LASXDIS	0x7  /* LASX Disabled exception */

#endif /* !LOONGSON_CPU_H_ */
