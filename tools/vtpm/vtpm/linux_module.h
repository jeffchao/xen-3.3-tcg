/* Software-Based Trusted Platform Module (TPM) Emulator for Linux
 * Copyright (C) 2004 Mario Strasser <mast@gmx.net>,
 * Copyright (C) 2005 INTEL Corp
 *
 * This module is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * $Id: linux_module.h 110 2006-06-14 22:23:17Z hstamer $
 */

#ifndef _LINUX_MODULE_H_
#define _LINUX_MODULE_H_

#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <linux/types.h>

#include <endian.h>
#define __BYTEORDER_HAS_U64__
#ifdef LITTLE_ENDIAN
 #include <linux/byteorder/little_endian.h>
#else
 #include <linux/byteorder/big_endian.h>
#endif

/* module settings */
#define min(A,B) ((A)<(B)?(A):(B))
#ifndef STR
#define STR(s) __STR__(s)
#define __STR__(s) #s
#endif
#include "tpm_version.h"

#define TPM_DEVICE_MINOR  224
#define TPM_DEVICE_NAME   "tpm"
#define TPM_MODULE_NAME   "tpm_emulator"

/* debug and log output functions */
extern int dmi_id; 

#ifdef DEBUG
#define debug(fmt, ...) printf("TPMD[%d]: %s:%d: Debug: " fmt "\n", \
                        dmi_id, __FILE__, __LINE__, ## __VA_ARGS__)
#define debug_nostop(fmt, ...) printf("TPMD[%d]: %s:%d: Debug: " fmt, \
                        dmi_id, __FILE__, __LINE__, ## __VA_ARGS__)
#define debug_more(fmt, ...) printf( fmt, ## __VA_ARGS__ )
#else
#define debug(fmt, ...) 
#define debug_nostop(fmt, ...) 
#define debug_more(fmt, ...)
#endif
#define info(fmt, ...)  printf("TPMD[%d]: %s:%d: Info: " fmt "\n", \
                        dmi_id, __FILE__, __LINE__, ## __VA_ARGS__)
#define error(fmt, ...) printf("TPMD[%d]: %s:%d: Error: " fmt "\n", \
                        dmi_id, __FILE__, __LINE__, ## __VA_ARGS__)
#define alert(fmt, ...) printf("TPMD[%d]: %s:%d: Alert: " fmt "\n", \
                        dmi_id, __FILE__, __LINE__, ## __VA_ARGS__)

/* memory allocation */

static inline void *tpm_malloc(size_t size) 
{
  return malloc(size);  
}

static inline void tpm_free(const void *ptr)
{
  if (ptr != NULL) free( (void *) ptr);
}

/* random numbers */

//FIXME;
void get_random_bytes(void *buf, int nbytes);

static inline void tpm_get_random_bytes(void *buf, int nbytes)
{
  get_random_bytes(buf, nbytes);
}

/* usec since last call */

uint64_t tpm_get_ticks(void);

/* byte order conversions */

#define CPU_TO_BE64(x) __cpu_to_be64(x)
#define CPU_TO_LE64(x) __cpu_to_le64(x)
#define CPU_TO_BE32(x) __cpu_to_be32(x)
#define CPU_TO_LE32(x) __cpu_to_le32(x)
#define CPU_TO_BE16(x) __cpu_to_be16(x)
#define CPU_TO_LE16(x) __cpu_to_le16(x)

#define BE64_TO_CPU(x) __be64_to_cpu(x)
#define LE64_TO_CPU(x) __le64_to_cpu(x)
#define BE32_TO_CPU(x) __be32_to_cpu(x)
#define LE32_TO_CPU(x) __le32_to_cpu(x)
#define BE16_TO_CPU(x) __be16_to_cpu(x)
#define LE16_TO_CPU(x) __le16_to_cpu(x)

#endif /* _LINUX_MODULE_H_ */
