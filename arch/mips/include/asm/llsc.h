/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Macros for 32/64-bit neutral inline assembler
 */

#ifndef __ASM_LLSC_H
#define __ASM_LLSC_H

#if _MIPS_SZLONG == 32
#define SZLONG_LOG 5
#define SZLONG_MASK 31UL
#define __LL		"ll	"
#define __SC		"sc	"
#define __INS		"ins	"
#define __EXT		"ext	"
#define __AMADD		"amadd.w	"
#define __AMAND		"amand.w	"
#define __AMOR		"amor.w		"
#define __AMXOR		"amxor.w	"
#define __AMAND_SYNC	"amand_sync.w	"
#define __AMOR_SYNC		"amor_sync.w	"
#define __AMXOR_SYNC	"amxor_sync.w	"
#elif _MIPS_SZLONG == 64
#define SZLONG_LOG 6
#define SZLONG_MASK 63UL
#define __LL		"lld	"
#define __SC		"scd	"
#define __INS		"dins	"
#define __EXT		"dext	"
#define __AMADD		"amadd.d	"
#define __AMAND		"amand.d	"
#define __AMOR		"amor.d		"
#define __AMXOR		"amxor.d	"
#define __AMAND_SYNC	"amand_sync.d	"
#define __AMOR_SYNC		"amor_sync.d	"
#define __AMXOR_SYNC	"amxor_sync.d	"
#endif

#endif /* __ASM_LLSC_H  */
