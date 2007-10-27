/*
 * DSP utils
 * Copyright (c) 2000, 2001, 2002 Fabrice Bellard.
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
 */

/**
 * @file dsputil.h
 * DSP utils.
 */

#ifndef DSPUTIL_H
#define DSPUTIL_H

#include "common.h"
#include "avcodec.h"

typedef short DCTELEM;

extern void ff_jpeg_fdct_islow(DCTELEM *data);

/* encoding scans */
extern const uint8_t ff_alternate_horizontal_scan[64];
extern const uint8_t ff_alternate_vertical_scan[64];
extern const uint8_t ff_zigzag_direct[64];
extern const uint8_t ff_zigzag248_direct[64];

/* pixel operations */
#define MAX_NEG_CROP 1024

/* temporary */
extern uint32_t squareTbl[512];
extern uint8_t cropTbl[256 + 2 * MAX_NEG_CROP];

/* add and put pixel (decoding) */
/* blocksizes for op_pixels_func are 8x4,8x8 16x8 16x16*/
/*h for op_pixels_func is limited to {width/2, width} but never larger than 16 and never smaller then 4*/
typedef void (*op_pixels_func)(uint8_t *block/*align width (8 or 16)*/, const uint8_t *pixels/*align 1*/, int line_size, int h);
typedef void (*tpel_mc_func)(uint8_t *block/*align width (8 or 16)*/, const uint8_t *pixels/*align 1*/, int line_size, int w, int h);
typedef void (*qpel_mc_func)(uint8_t *dst/*align width (8 or 16)*/, uint8_t *src/*align 1*/, int stride);
typedef void (*h264_chroma_mc_func)(uint8_t *dst/*align 8*/, uint8_t *src/*align 1*/, int srcStride, int h, int x, int y);

/* motion estimation */
/* h is limited to {width/2, width, 2*width} but never larger than 16 and never smaller then 2*/
/* allthough currently h<4 is not used as functions with width <8 are not used and neither implemented*/
typedef int (*me_cmp_func)(void /*MpegEncContext*/ *s, uint8_t *blk1/*align width (8 or 16)*/, uint8_t *blk2/*align 1*/, int line_size, int h);


/**
 * DSPContext.
 */
typedef struct DSPContext {
    /* pixel ops : interface with DCT */
    void (*get_pixels)(DCTELEM *block/*align 16*/, const uint8_t *pixels/*align 8*/, int line_size);
    void (*diff_pixels)(DCTELEM *block/*align 16*/, const uint8_t *s1/*align 8*/, const uint8_t *s2/*align 8*/, int stride);
    /**
     * translational global motion compensation.
     */
    void (*gmc1)(uint8_t *dst/*align 8*/, uint8_t *src/*align 1*/, int srcStride, int h, int x16, int y16, int rounder);
    /**
     * global motion compensation.
     */
    void (*gmc )(uint8_t *dst/*align 8*/, uint8_t *src/*align 1*/, int stride, int h, int ox, int oy,
		    int dxx, int dxy, int dyx, int dyy, int shift, int r, int width, int height);
    void (*clear_blocks)(DCTELEM *blocks/*align 16*/);
    int (*pix_sum)(uint8_t * pix, int line_size);
    int (*pix_norm1)(uint8_t * pix, int line_size);

    /**
     * Halfpel motion compensation with rounding (a+b+1)>>1.
     * this is an array[4][4] of motion compensation funcions for 4 
     * horizontal blocksizes (8,16) and the 4 halfpel positions<br>
     * *pixels_tab[ 0->16xH 1->8xH ][ xhalfpel + 2*yhalfpel ]
     * @param block destination where the result is stored
     * @param pixels source
     * @param line_size number of bytes in a horizontal line of block
     * @param h height
     */
    op_pixels_func put_pixels_tab[2][4];

    /**
     * Halfpel motion compensation with rounding (a+b+1)>>1.
     * This is an array[4][4] of motion compensation functions for 4 
     * horizontal blocksizes (8,16) and the 4 halfpel positions<br>
     * *pixels_tab[ 0->16xH 1->8xH ][ xhalfpel + 2*yhalfpel ]
     * @param block destination into which the result is averaged (a+b+1)>>1
     * @param pixels source
     * @param line_size number of bytes in a horizontal line of block
     * @param h height
     */
    op_pixels_func avg_pixels_tab[2][4];

    me_cmp_func pix_abs[2][4];

} DSPContext;

extern int sse16_c(void *s, uint8_t *blk1,uint8_t *blk2, int line_size, int h);
extern int sse8_c(void *s, uint8_t *blk1,uint8_t *blk2, int line_size, int h);
extern int pix_abs16_c(void *s, uint8_t *blk1,uint8_t *blk2, int line_size, int h);
extern int pix_abs8_c(void *s, uint8_t *blk1,uint8_t *blk2, int line_size, int h);

void dsputil_static_init(void);
void dsputil_init(DSPContext* p, AVCodecContext *avctx);

#define	BYTE_VEC32(c)	((c)*0x01010101UL)

#define rnd_avg32(a,b) (((a)|(b)) - ((((a)^(b)) & ~BYTE_VEC32(0x01)) >> 1))

#define __align8

#ifdef __GNUC__

struct unaligned_32 { uint32_t l; } __attribute__((packed));
#define LD32(a) (((const struct unaligned_32 *) (a))->l)

#else

/*#define LD32(a) (*((uint32_t*)(a)))*/
#define LD32(a) unaligned32(a)

#endif

#undef lrintf
#define lrintf(x) ((long)(0.5+(x)))

#endif
