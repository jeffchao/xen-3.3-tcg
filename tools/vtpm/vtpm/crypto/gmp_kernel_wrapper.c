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
 * $Id: gmp_kernel_wrapper.c 1 2004-11-03 17:22:56Z mast $
 */

#include "linux_module.h"
#include "gmp.h"

int __gmp_junk;

/* GNU MP assertion and abort functions */

void __attribute__ ((regparm(0))) __gmp_assert_fail(const char *filename, 
  int linenum, const char *expr) 
{
  error("%s:%d: GNU MP assertion failed: %s\n", 
    filename, linenum, expr);
}

/* overwrite GNU MP random functions (used by mpz/millerrabin.c) */ 

void __attribute__ ((regparm(0))) gmp_randinit(gmp_randstate_t rstate, 
  gmp_randalg_t alg, ...)
{
  /* nothing to do */
}

void __attribute__ ((regparm(0))) gmp_randclear(gmp_randstate_t rstate)
{
  /* nothing to do */
}

#define SIZ(x) ((x)->_mp_size)
#define PTR(x) ((x)->_mp_d)
#define ALLOC(x) ((x)->_mp_alloc)
#define MPN_NORMALIZE(DST, NLIMBS) \
  do {                                                                  \
    while (NLIMBS > 0)                                                  \
      {                                                                 \
        if ((DST)[(NLIMBS) - 1] != 0)                                   \
          break;                                                        \
        NLIMBS--;                                                       \
      }                                                                 \
  } while (0)

void __attribute__ ((regparm(0))) mpz_urandomb(mpz_ptr rop, 
  gmp_randstate_t rstate, unsigned long int nbits)
{
  mp_ptr rp;
  mp_size_t size;

  size = (nbits + GMP_NUMB_BITS - 1) / GMP_NUMB_BITS;
  if (ALLOC (rop) < size) _mpz_realloc (rop, size);
  rp = PTR (rop);
  tpm_get_random_bytes(rp, nbits >> 3);
  MPN_NORMALIZE (rp, size);
  SIZ (rop) = size;
}

/* GNU MP memory management */

void __attribute__ ((regparm(0))) *kernel_allocate(size_t size)
{
  void *ret  = (void*)malloc(size);
  if (!ret) error("GMP: cannot allocate memory (size=%Zu)\n", size);
  return ret;
}

void __attribute__ ((regparm(0))) *kernel_reallocate(void *oldptr, 
  size_t old_size, size_t new_size)
{
  void *ret = (void*)malloc(new_size);
  if (!ret) error("GMP: Cannot reallocate memory "
    "(old_size=%Zu new_size=%Zu)\n", old_size, new_size);
  memcpy(ret, oldptr, old_size);
  free(oldptr);
  return ret;
}

void __attribute__ ((regparm(0))) kernel_free(void *blk_ptr, size_t blk_size)
{
  /* overwrite used memory */
  if (blk_ptr != NULL) { 
    memset(blk_ptr, 0, blk_size);
    free(blk_ptr);
  }
}

void __attribute__ ((regparm(0))) *(*__gmp_allocate_func) 
  __GMP_PROTO ((size_t)) = kernel_allocate;
void __attribute__ ((regparm(0))) *(*__gmp_reallocate_func) 
  __GMP_PROTO ((void *, size_t, size_t)) = kernel_reallocate;
void __attribute__ ((regparm(0))) (*__gmp_free_func) 
  __GMP_PROTO ((void *, size_t)) = kernel_free;

