/*
 * Copyright (C) 2018 Loongson Inc.
 * Author:  Bibo Mao, maobibo@loongson.cn
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef _MIPS_RTAS_H
#define _MIPS_RTAS_H
#ifdef __KERNEL__

#include <linux/spinlock.h>
#include <asm/page.h>
/* RTAS tokens */
#define RTAS_TOKEN_BASE      0x2000
#define RTAS_EVENT_REPLY     (RTAS_TOKEN_BASE + 0x0E)
#define RTAS_EVENT_SCAN      (RTAS_TOKEN_BASE + 0x0F)

#define RTAS_EVENT_SCAN_RATE    1
#define RTAS_ERROR_LOG_MAX      2048
#define LOG_NUMBER              64              /* must be a power of two */
#define LOG_NUMBER_MASK         (LOG_NUMBER-1)

/* RTAS event type */
#define RTAS_TYPE_EPOW                  0x40
#define RTAS_TYPE_HOTPLUG               0xE5

/* Platform Resource Reassignment Notification */
#define RTAS_TYPE_PRRN                  0xA0

/* Error types logged.  */
#define ERR_FLAG_ALREADY_LOGGED 0x0
#define ERR_FLAG_BOOT           0x1     /* log was pulled from NVRAM on boot */
#define ERR_TYPE_RTAS_LOG       0x2     /* from rtas event-scan */
#define ERR_TYPE_KERNEL_PANIC   0x4     /* from die()/panic() */
#define ERR_TYPE_KERNEL_PANIC_GZ 0x8    /* ditto, compressed */

#define RTAS_DEBUG KERN_DEBUG "RTAS: "

/* RTAS event classes */
#define RTAS_INTERNAL_ERROR             0x80000000 /* set bit 0 */
#define RTAS_EPOW_WARNING               0x40000000 /* set bit 1 */
#define RTAS_HOTPLUG_EVENTS             0x10000000 /* set bit 3 */
#define RTAS_IO_EVENTS                  0x08000000 /* set bit 4 */
#define RTAS_EVENT_SCAN_ALL_EVENTS      0xffffffff

struct rtas_error_log {
	unsigned char         type;                  /* General event or error*/
	unsigned char         initiator;
	/* XXXXXXXX
	 * XXXX         4: Initiator of event
	 *     XXXX     4: Target of failed operation
	 */
	unsigned char         severity;
	/* XXXXXXXX
	 * XXX          3: Severity level of error
	 *    XX        2: Degree of recovery
	 *      X       1: Extended log present?
	 *       XX     2: Reserved
	 */
	unsigned char   version;                /* Architectural version */
	unsigned int    extended_log_length;    /* length in bytes */
	unsigned char   buffer[1];              /* Start of extended log */
	/* Variable length.      */
};

#define RTAS_SECTION_ID_HOTPLUG              0x4850 /* HP */
#define RTAS_HP_TYPE_CPU                          1
#define RTAS_HP_TYPE_MEMORY                       2
#define RTAS_HP_TYPE_SLOT                         3
#define RTAS_HP_TYPE_PHB                          4
#define RTAS_HP_TYPE_PCI                          5
#define RTAS_HP_ACTION_ADD                        1
#define RTAS_HP_ACTION_REMOVE                     2
#define RTAS_HP_ID_DRC_NAME                       1
#define RTAS_HP_ID_DRC_INDEX                      2
#define RTAS_HP_ID_DRC_COUNT                      3
struct rtas_event_log_hp {
	unsigned short section_id;
	unsigned short section_length;
	unsigned char hotplug_type;
	unsigned char hotplug_action;
	unsigned char hotplug_identifier;
	unsigned char reserved;
	unsigned int index;
	unsigned int count;
};

#define rtas_error_type(x)      ((x)->type)

static inline unsigned char rtas_error_severity(const struct rtas_error_log *elog)
{
	return (elog->severity & 0xE0) >> 5;
}

static inline unsigned char rtas_error_disposition(const struct rtas_error_log *elog)
{
	return (elog->severity & 0x18) >> 3;
}

static inline unsigned char rtas_error_extended(const struct rtas_error_log *elog)
{
	return (elog->severity & 0x04) >> 2;
}

static inline unsigned int rtas_error_extended_log_length(const struct rtas_error_log *elog)
{
	return elog->extended_log_length;
}

#endif
#endif /* _MIPS_RTAS_H */
