/**
 * @file common.h
 * common internal api header.
 */

#ifndef COMMON_H
#define COMMON_H

#if defined(WIN32) && !defined(__MINGW32__) && !defined(__CYGWIN__)
#    define CONFIG_WIN32
#endif

/*#define ALT_BITSTREAM_WRITER*/
/*#define ALIGNED_BITSTREAM_WRITER*/

#ifdef HAVE_AV_CONFIG_H
/* only include the following when compiling package */
#  include "config.h"

#  include <stdlib.h>
#  include <stdio.h>
#  include <string.h>
#  include <ctype.h>
#  include <limits.h>
#  ifndef __BEOS__
#    include <errno.h>
#  else
#    include "berrno.h"
#  endif
#  include <math.h>
#  include <stddef.h>

#endif /* HAVE_AV_CONFIG_H */

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

#ifndef EMULATE_INTTYPES
# include <inttypes.h>
#else
  typedef signed char  int8_t;
  typedef signed short int16_t;
  typedef signed int   int32_t;
  typedef unsigned char  uint8_t;
  typedef unsigned short uint16_t;
  typedef unsigned int   uint32_t;

# ifdef CONFIG_WIN32
    typedef signed __int64   int64_t;
    typedef unsigned __int64 uint64_t;
# else /* other OS */
    typedef signed long long   int64_t;
    typedef unsigned long long uint64_t;
# endif /* other OS */
#endif /* HAVE_INTTYPES_H */

#ifdef EMULATE_FAST_INT
/* note that we don't emulate 64bit ints */
typedef signed char int_fast8_t;
typedef signed int  int_fast16_t;
typedef signed int  int_fast32_t;
typedef unsigned char uint_fast8_t;
typedef unsigned int  uint_fast16_t;
typedef unsigned int  uint_fast32_t;
#endif

#ifndef INT_BIT
#  if INT_MAX != 2147483647
#    define INT_BIT 64
#  else
#    define INT_BIT 32
#  endif
#endif

#ifdef CONFIG_WIN32
/* windows */
#  if !defined(__MINGW32__) && !defined(__CYGWIN__)
#    define int64_t_C(c)     (c ## i64)
#    define uint64_t_C(c)    (c ## i64)
#    ifdef HAVE_AV_CONFIG_H
#      define inline __inline
#    endif
#  else
#    define int64_t_C(c)     (c ## LL)
#    define uint64_t_C(c)    (c ## ULL)
#  endif /* __MINGW32__ */
#  ifdef HAVE_AV_CONFIG_H
#    ifdef _DEBUG
#      define DEBUG
#    endif
#    define snprintf _snprintf
#    define vsnprintf _vsnprintf
#  endif

#elif defined (CONFIG_OS2)
/* OS/2 EMX */
#  ifndef int64_t_C
#    define int64_t_C(c)     (c ## LL)
#    define uint64_t_C(c)    (c ## ULL)
#  endif
#  ifdef HAVE_AV_CONFIG_H
#    include <float.h>
#  endif /* HAVE_AV_CONFIG_H */

#else
/* unix */
#  ifndef int64_t_C
#    if LONG_MAX > 2147483647
#      define int64_t_C(c)     (c ## L)
#      define uint64_t_C(c)    (c ## UL)
#    else
#      define int64_t_C(c)     (c ## LL)
#      define uint64_t_C(c)    (c ## ULL)
#    endif
#  endif
#  ifdef HAVE_AV_CONFIG_H
#    ifdef USE_FASTMEMCPY
#      include "fastmemcpy.h"
#    endif
#  endif /* HAVE_AV_CONFIG_H */

#endif /* !CONFIG_WIN32 && !CONFIG_OS2 */

#ifndef INT64_MAX
#  define INT64_MAX int64_t_C(0x7FFFFFFFFFFFFFFF)
#endif

#ifdef HAVE_AV_CONFIG_H
/* this ifdef extends to bottom of file */

#include "bswap.h"

#ifndef DEBUG
#  define NDEBUG
#endif
#include <assert.h>

/* assume b>0 */
#define ROUNDED_DIV(a,b) (((a)>0 ? (a) + ((b)>>1) : (a) - ((b)>>1))/(b))
#define ABS(a) ((a) >= 0 ? (a) : (-(a)))

#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))

#define NEG_SSR32(a,s) ((( int32_t)(a))>>(32-(s)))
#define NEG_USR32(a,s) (((uint32_t)(a))>>(32-(s)))

/* bit output */

/* buf and buf_end must be present and used by every alternative writer. */
typedef struct PutBitContext {
#ifdef ALT_BITSTREAM_WRITER
  uint8_t *buf, *buf_end;
  int index;
#else
  uint32_t bit_buf;
  int bit_left;
  uint8_t *buf, *buf_ptr, *buf_end;
#endif
} PutBitContext;

/* return the number of bits output */
#ifdef ALT_BITSTREAM_WRITER
#  define put_bits_count(s) ((s)->index)
#else
#  define put_bits_count(s) (((s)->buf_ptr-(s)->buf)*8 + 32 - (s)->bit_left)
#endif

void align_put_bits(PutBitContext *s);

/* used to avoid missaligned exceptions on some archs (alpha, ...) */
#ifdef ARCH_X86
/*#if 1*/
#  define unaligned32(a) (*(uint32_t*)(a))
#else
#  ifdef __GNUC__
static inline uint32_t unaligned32(const void *v) {
  struct Unaligned {
    uint32_t i;
  } __attribute__((packed));
  return ((const struct Unaligned *) v)->i;
}
#  elif defined(__DECC)
#    define unaligned32(a) (*(__unaligned uint32_t*)(a))
#  else
#    define unaligned32(a) (*(uint32_t*)(a))
#  endif
#endif /*!ARCH_X86*/

extern void put_bits(PutBitContext *s, int n, unsigned int value);

#ifdef ALT_BITSTREAM_WRITER
#  define pbBufPtr(s) ((s)->buf + ((s)->index>>3))
#else
#  define pbBufPtr(s) ((s)->buf_ptr)
#endif

/**
 *
 * PutBitContext must be flushed & aligned to a byte boundary before calling this.
 */
#ifdef ALT_BITSTREAM_WRITER
   /* assert((put_bits_count(s)&7)==0); */
#  define skip_put_bytes(s,n) ((s)->index += (n)<<3), \
     FIXME may need some cleaning of the buffer
#else
   /* assert((put_bits_count(s)&7)==0); */
   /* assert(s->bit_left==32); */
#  define skip_put_bytes(s,n) ((s)->buf_ptr += (n))
#endif    

/* misc math functions */
extern const uint8_t ff_log2_tab[256];

#define av_log2_16bit(v) (((v)&0xff00)? 8+ff_log2_tab[(unsigned int)(v)>>8] :\
  ff_log2_tab[v])

#define av_log2(v) (((v)&0xffff0000)? \
  16+av_log2_16bit((unsigned int)(v)>>16) : av_log2_16bit(v))

/* median of 3 */
#define mid_pred(a,b,c) \
  (((a)>(b))? (((c)>(b))? (((c)>(a))?(a):(c)) : (b)) :\
   (((b)>(c))? (((c)>(a))? (c):(a)) : (b)))

#define clip(a,amin,amax) (((int)(a)<(int)(amin))?(int)(amin):\
  (((int)(a)>(int)(amax))?(int)(amax):(int)(a)))

/* math */
extern const uint8_t ff_sqrt_tab[128];

int64_t ff_gcd(int64_t a, int64_t b);

#define COPY3_IF_LT(x,y,a,b,c,d) if((y)<(x)){(x)=(y);(a)=(b);(c)=(d);}

/* avoid usage of various functions */
#define malloc please_use_av_malloc
#define free please_use_av_free
#define realloc please_use_av_realloc
#define time time_is_forbidden_due_to_security_issues
#define rand rand_is_forbidden_due_to_state_trashing
#define srand srand_is_forbidden_due_to_state_trashing
#if !(defined(LIBAVFORMAT_BUILD) || defined(_FRAMEHOOK_H))
#  define printf please_use_av_log
#  define fprintf please_use_av_log
#endif

#define CHECKED_ALLOCZ(p, size)\
{\
  p= av_mallocz(size);\
  if(p==NULL && (size)!=0){\
    perror("malloc");\
    goto fail;\
  }\
}

#endif /* HAVE_AV_CONFIG_H */
#endif /* COMMON_H */
