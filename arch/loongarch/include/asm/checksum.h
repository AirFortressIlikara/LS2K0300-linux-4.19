/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 96, 97, 98, 99, 2001 by Ralf Baechle
 * Copyright (C) 1999 Silicon Graphics, Inc.
 * Copyright (C) 2001 Thiemo Seufer.
 * Copyright (C) 2002 Maciej W. Rozycki
 * Copyright (C) 2014 Imagination Technologies Ltd.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_CHECKSUM_H
#define _ASM_CHECKSUM_H

#ifdef CONFIG_GENERIC_CSUM
#include <asm-generic/checksum.h>
#else
#include <linux/types.h>

/*
 * Fold a partial checksum without adding pseudo headers
 */
static inline __sum16 csum_fold(__wsum csum)
{
	u32 sum = (__force u32)csum;

	sum += (sum << 16);
	csum = (sum < csum);
	sum >>= 16;
	sum += csum;

	return (__force __sum16)~sum;
}
#define csum_fold csum_fold

/*
 * This is a version of ip_compute_csum() optimized for IP headers,
 * which always checksum on 4 octet boundaries.
 *
 * By Jorge Cwik <jorge@laser.satlink.net>, adapted for linux by
 * Arnt Gulbrandsen.
 */
static inline __sum16 ip_fast_csum(const void *iph, unsigned int ihl)
{
	const unsigned int *word = iph;
	const unsigned int *stop = word + ihl;
	unsigned int csum;
	int carry;

	csum = word[0];
	csum += word[1];
	carry = (csum < word[1]);
	csum += carry;

	csum += word[2];
	carry = (csum < word[2]);
	csum += carry;

	csum += word[3];
	carry = (csum < word[3]);
	csum += carry;

	word += 4;
	do {
		csum += *word;
		carry = (csum < *word);
		csum += carry;
		word++;
	} while (word != stop);

	return csum_fold(csum);
}
#define ip_fast_csum ip_fast_csum

static inline __wsum csum_tcpudp_nofold(__be32 saddr, __be32 daddr,
					__u32 len, __u8 proto,
					__wsum sum)
{
#ifdef __clang__
	unsigned long tmp = (__force unsigned long)sum;
#else
	__wsum tmp = sum;
#endif

	__asm__(
#ifdef CONFIG_64BIT
	"	add.d	%0, %0, %2	\n"
	"	add.d	%0, %0, %3	\n"
	"	add.d	%0, %0, %4	\n"
	"	slli.d	$t7, %0, 32	\n"
	"	add.d	%0, %0, $t7	\n"
	"	sltu	$t7, %0, $t7	\n"
	"	srai.d	%0, %0, 32	\n"
	"	add.w	%0, %0, $t7	\n"
#endif
	: "=r" (tmp)
	: "0" ((__force unsigned long)daddr),
	  "r" ((__force unsigned long)saddr),
	  "r" ((proto + len) << 8),
	  "r" ((__force unsigned long)sum)
	: "t7");

	return (__force __wsum)tmp;
}
#define csum_tcpudp_nofold csum_tcpudp_nofold

#define _HAVE_ARCH_IPV6_CSUM
struct in6_addr;
static __inline__ __sum16 csum_ipv6_magic(const struct in6_addr *saddr,
					  const struct in6_addr *daddr,
					  __u32 len, __u8 proto,
					  __wsum sum)
{
	__wsum tmp;

	__asm__(
	"	add.w	%0, %0, %5	# proto (long in network byte order)\n"
	"	sltu	$t7, %0, %5	\n"
	"	add.w	%0, %0, $t7	\n"

	"	add.w	%0, %0, %6	# csum\n"
	"	sltu	$t7, %0, %6	\n"
	"	ld.w	%1, %2, 0	# four words source address\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	ld.w	%1, %2, 4	\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	ld.w	%1, %2, 8	\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	ld.w	%1, %2, 12	\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	ld.w	%1, %3, 0	\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	ld.w	%1, %3, 4	\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	ld.w	%1, %3, 8	\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	ld.w	%1, %3, 12	\n"
	"	add.w	%0, %0, $t7	\n"
	"	add.w	%0, %0, %1	\n"
	"	sltu	$t7, %0, %1	\n"

	"	add.w	%0, %0, $t7	# Add final carry\n"
	: "=&r" (sum), "=&r" (tmp)
	: "r" (saddr), "r" (daddr),
	  "0" (htonl(len)), "r" (htonl(proto)), "r" (sum)
	:"t7");

	return csum_fold(sum);
}

#include <asm-generic/checksum.h>
#endif /* CONFIG_GENERIC_CSUM */

#endif /* _ASM_CHECKSUM_H */
