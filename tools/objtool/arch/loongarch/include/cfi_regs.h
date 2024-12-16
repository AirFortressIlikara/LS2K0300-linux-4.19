/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _OBJTOOL_CFI_REGS_H
#define _OBJTOOL_CFI_REGS_H

#define CFI_ZERO		0
#define CFI_RA			1
#define CFI_TP			2
#define CFI_SP			3
#define CFI_A0			4
#define CFI_A1			5
#define CFI_A2			6
#define CFI_A3			7
#define CFI_A4			8
#define CFI_A5			9
#define CFI_A6			10
#define CFI_A7			11
#define CFI_T0			12
#define CFI_T1			13
#define CFI_T2			14
#define CFI_T3			15
#define CFI_T4			16
#define CFI_T5			17
#define CFI_T6			18
#define CFI_T7			19
#define CFI_T8			20
/* #define CFI_X			21 */
#define CFI_FP			22
#define CFI_S0			23
#define CFI_S1			24
#define CFI_S2			25
#define CFI_S3			26
#define CFI_S4			27
#define CFI_S5			28
#define CFI_S6			29
#define CFI_S7			30
#define CFI_S8			31
#define CFI_NUM_REGS	32

#define CFI_BP			CFI_FP

#endif /* _OBJTOOL_CFI_REGS_H */
