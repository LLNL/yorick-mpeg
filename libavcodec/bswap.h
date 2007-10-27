/**
 * @file bswap.h
 * byte swap.
 */

#ifndef __BSWAP_H__
#define __BSWAP_H__

#ifdef HAVE_BYTESWAP_H
#include <byteswap.h>
#else

#define bswap_16(x) (((x) & 0x00ff) << 8 | ((x) & 0xff00) >> 8)

/* code from bits/byteswap.h (C) 1997, 1998 Free Software Foundation, Inc.*/
#define bswap_32(x) \
     ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) | \
      (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))

#endif

/* be2me ... BigEndian to MachineEndian*/
/* le2me ... LittleEndian to MachineEndian*/

#ifdef WORDS_BIGENDIAN
#define be2me_16(x) (x)
#define be2me_32(x) (x)
#else
#define be2me_16(x) bswap_16(x)
#define be2me_32(x) bswap_32(x)
#endif

#endif
