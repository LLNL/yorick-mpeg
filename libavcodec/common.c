/*
 * Common bit i/o utils
 * Copyright (c) 2000, 2001 Fabrice Bellard.
 * Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * alternative bitstream reader & writer by Michael Niedermayer <michaelni@gmx.at>
 */

/**
 * @file common.c
 * common internal api.
 */

#include "avcodec.h"

const uint8_t ff_sqrt_tab[128]={
  0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,
  11,11,11
};

const uint8_t ff_log2_tab[256]={
  0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
};

void align_put_bits(PutBitContext *s)
{
#ifdef ALT_BITSTREAM_WRITER
    put_bits(s,(  - s->index) & 7,0);
#else
    put_bits(s,s->bit_left & 7,0);
#endif
}

int64_t ff_gcd(int64_t a, int64_t b){
    if(b) return ff_gcd(b, a%b);
    else  return a;
}

#ifndef ALT_BITSTREAM_WRITER
void put_bits(PutBitContext *s, int n, unsigned int value)
{
  unsigned int bit_buf;
  int bit_left;

  assert(n == 32 || value < (1U << n));
    
  bit_buf = s->bit_buf;
  bit_left = s->bit_left;

  /* XXX: optimize */
  if (n < bit_left) {
    bit_buf = (bit_buf<<n) | value;
    bit_left-=n;
  } else {
    bit_buf<<=bit_left;
    bit_buf |= value >> (n - bit_left);
    *(uint32_t *)s->buf_ptr = be2me_32(bit_buf);
    /*printf("bitbuf = %08x\n", bit_buf);*/
    s->buf_ptr+=4;
    bit_left+=32 - n;
    bit_buf = value;
  }

  s->bit_buf = bit_buf;
  s->bit_left = bit_left;
}

#else
void put_bits(PutBitContext *s, int n, unsigned int value)
{
#  ifdef ALIGNED_BITSTREAM_WRITER
  int index= s->index;
  uint32_t *ptr= ((uint32_t *)s->buf)+(index>>5);
    
  value<<= 32-n; 

  ptr[0] |= be2me_32(value>>(index&31));
  ptr[1]  = be2me_32(value<<(32-(index&31)));
  index+= n;
  s->index= index;
#  else /*ALIGNED_BITSTREAM_WRITER*/
  int index= s->index;
  uint32_t *ptr= (uint32_t*)(((uint8_t *)s->buf)+(index>>3));
    
  ptr[0] |= be2me_32(value<<(32-n-(index&7) ));
  ptr[1] = 0;
  index+= n;
  s->index= index;
#  endif /*!ALIGNED_BITSTREAM_WRITER*/
}
#endif
