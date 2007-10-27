/*
 * DSP utils
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
 * gmc & q-pel & 32/64 bit based MC by Michael Niedermayer <michaelni@gmx.at>
 */
 
/**
 * @file dsputil.c
 * DSP utils
 */
 
#include "avcodec.h"
#include "dsputil.h"
#include "mpegvideo.h"
#include "simple_idct.h"

uint8_t cropTbl[256 + 2 * MAX_NEG_CROP];
uint32_t squareTbl[512];

const uint8_t ff_zigzag_direct[64] = {
    0,   1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
};

const uint8_t ff_alternate_horizontal_scan[64] = {
    0,  1,   2,  3,  8,  9, 16, 17, 
    10, 11,  4,  5,  6,  7, 15, 14,
    13, 12, 19, 18, 24, 25, 32, 33, 
    26, 27, 20, 21, 22, 23, 28, 29,
    30, 31, 34, 35, 40, 41, 48, 49, 
    42, 43, 36, 37, 38, 39, 44, 45,
    46, 47, 50, 51, 56, 57, 58, 59, 
    52, 53, 54, 55, 60, 61, 62, 63,
};

const uint8_t ff_alternate_vertical_scan[64] = {
    0,  8,  16, 24,  1,  9,  2, 10, 
    17, 25, 32, 40, 48, 56, 57, 49,
    41, 33, 26, 18,  3, 11,  4, 12, 
    19, 27, 34, 42, 50, 58, 35, 43,
    51, 59, 20, 28,  5, 13,  6, 14, 
    21, 29, 36, 44, 52, 60, 37, 45,
    53, 61, 22, 30,  7, 15, 23, 31, 
    38, 46, 54, 62, 39, 47, 55, 63,
};

/* Input permutation for the simple_idct_mmx */
static const uint8_t simple_mmx_permutation[64]={
	0x00, 0x08, 0x04, 0x09, 0x01, 0x0C, 0x05, 0x0D, 
	0x10, 0x18, 0x14, 0x19, 0x11, 0x1C, 0x15, 0x1D, 
	0x20, 0x28, 0x24, 0x29, 0x21, 0x2C, 0x25, 0x2D, 
	0x12, 0x1A, 0x16, 0x1B, 0x13, 0x1E, 0x17, 0x1F, 
	0x02, 0x0A, 0x06, 0x0B, 0x03, 0x0E, 0x07, 0x0F, 
	0x30, 0x38, 0x34, 0x39, 0x31, 0x3C, 0x35, 0x3D, 
	0x22, 0x2A, 0x26, 0x2B, 0x23, 0x2E, 0x27, 0x2F, 
	0x32, 0x3A, 0x36, 0x3B, 0x33, 0x3E, 0x37, 0x3F,
};

static int pix_sum_c(uint8_t * pix, int line_size)
{
    int s, i, j;

    s = 0;
    for (i = 0; i < 16; i++) {
	for (j = 0; j < 16; j += 8) {
	    s += pix[0];
	    s += pix[1];
	    s += pix[2];
	    s += pix[3];
	    s += pix[4];
	    s += pix[5];
	    s += pix[6];
	    s += pix[7];
	    pix += 8;
	}
	pix += line_size - 16;
    }
    return s;
}

static int pix_norm1_c(uint8_t * pix, int line_size)
{
    int s, i, j;
    uint32_t *sq = squareTbl + 256;

    s = 0;
    for (i = 0; i < 16; i++) {
	for (j = 0; j < 16; j += 8) {
#if LONG_MAX > 2147483647
	    register uint64_t x=*(uint64_t*)pix;
	    s += sq[x&0xff];
	    s += sq[(x>>8)&0xff];
	    s += sq[(x>>16)&0xff];
	    s += sq[(x>>24)&0xff];
            s += sq[(x>>32)&0xff];
            s += sq[(x>>40)&0xff];
            s += sq[(x>>48)&0xff];
            s += sq[(x>>56)&0xff];
#else
	    register uint32_t x=*(uint32_t*)pix;
	    s += sq[x&0xff];
	    s += sq[(x>>8)&0xff];
	    s += sq[(x>>16)&0xff];
	    s += sq[(x>>24)&0xff];
            x=*(uint32_t*)(pix+4);
            s += sq[x&0xff];
            s += sq[(x>>8)&0xff];
            s += sq[(x>>16)&0xff];
            s += sq[(x>>24)&0xff];
#endif
	    pix += 8;
	}
	pix += line_size - 16;
    }
    return s;
}

int sse8_c(void *v, uint8_t * pix1, uint8_t * pix2, int line_size, int h)
{
    int s, i;
    uint32_t *sq = squareTbl + 256;

    s = 0;
    for (i = 0; i < h; i++) {
        s += sq[(int)pix1[0] - (int)pix2[0]];
        s += sq[(int)pix1[1] - (int)pix2[1]];
        s += sq[(int)pix1[2] - (int)pix2[2]];
        s += sq[(int)pix1[3] - (int)pix2[3]];
        s += sq[(int)pix1[4] - (int)pix2[4]];
        s += sq[(int)pix1[5] - (int)pix2[5]];
        s += sq[(int)pix1[6] - (int)pix2[6]];
        s += sq[(int)pix1[7] - (int)pix2[7]];
        pix1 += line_size;
        pix2 += line_size;
    }
    return s;
}

int sse16_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;
    uint32_t *sq = squareTbl + 256;

    s = 0;
    for (i = 0; i < h; i++) {
        s += sq[(int)pix1[ 0] - (int)pix2[ 0]];
        s += sq[(int)pix1[ 1] - (int)pix2[ 1]];
        s += sq[(int)pix1[ 2] - (int)pix2[ 2]];
        s += sq[(int)pix1[ 3] - (int)pix2[ 3]];
        s += sq[(int)pix1[ 4] - (int)pix2[ 4]];
        s += sq[(int)pix1[ 5] - (int)pix2[ 5]];
        s += sq[(int)pix1[ 6] - (int)pix2[ 6]];
        s += sq[(int)pix1[ 7] - (int)pix2[ 7]];
        s += sq[(int)pix1[ 8] - (int)pix2[ 8]];
        s += sq[(int)pix1[ 9] - (int)pix2[ 9]];
        s += sq[(int)pix1[10] - (int)pix2[10]];
        s += sq[(int)pix1[11] - (int)pix2[11]];
        s += sq[(int)pix1[12] - (int)pix2[12]];
        s += sq[(int)pix1[13] - (int)pix2[13]];
        s += sq[(int)pix1[14] - (int)pix2[14]];
        s += sq[(int)pix1[15] - (int)pix2[15]];

        pix1 += line_size;
        pix2 += line_size;
    }
    return s;
}

static void get_pixels_c(DCTELEM *block, const uint8_t *pixels, int line_size)
{
    int i;

    /* read the pixels */
    for(i=0;i<8;i++) {
        block[0] = pixels[0];
        block[1] = pixels[1];
        block[2] = pixels[2];
        block[3] = pixels[3];
        block[4] = pixels[4];
        block[5] = pixels[5];
        block[6] = pixels[6];
        block[7] = pixels[7];
        pixels += line_size;
        block += 8;
    }
}

static void diff_pixels_c(DCTELEM *block, const uint8_t *s1,
			  const uint8_t *s2, int stride){
    int i;

    /* read the pixels */
    for(i=0;i<8;i++) {
        block[0] = (int)s1[0] - (int)s2[0];
        block[1] = (int)s1[1] - (int)s2[1];
        block[2] = (int)s1[2] - (int)s2[2];
        block[3] = (int)s1[3] - (int)s2[3];
        block[4] = (int)s1[4] - (int)s2[4];
        block[5] = (int)s1[5] - (int)s2[5];
        block[6] = (int)s1[6] - (int)s2[6];
        block[7] = (int)s1[7] - (int)s2[7];
        s1 += stride;
        s2 += stride;
        block += 8;
    }
}

#define avg2(a,b) ((a+b+1)>>1)
#define avg4(a,b,c,d) ((a+b+c+d+2)>>2)

static void gmc1_c(uint8_t *dst, uint8_t *src, int stride, int h, int x16, int y16, int rounder)
{
    const int A=(16-x16)*(16-y16);
    const int B=(   x16)*(16-y16);
    const int C=(16-x16)*(   y16);
    const int D=(   x16)*(   y16);
    int i;

    for(i=0; i<h; i++)
    {
        dst[0]= (A*src[0] + B*src[1] + C*src[stride+0] + D*src[stride+1] + rounder)>>8;
        dst[1]= (A*src[1] + B*src[2] + C*src[stride+1] + D*src[stride+2] + rounder)>>8;
        dst[2]= (A*src[2] + B*src[3] + C*src[stride+2] + D*src[stride+3] + rounder)>>8;
        dst[3]= (A*src[3] + B*src[4] + C*src[stride+3] + D*src[stride+4] + rounder)>>8;
        dst[4]= (A*src[4] + B*src[5] + C*src[stride+4] + D*src[stride+5] + rounder)>>8;
        dst[5]= (A*src[5] + B*src[6] + C*src[stride+5] + D*src[stride+6] + rounder)>>8;
        dst[6]= (A*src[6] + B*src[7] + C*src[stride+6] + D*src[stride+7] + rounder)>>8;
        dst[7]= (A*src[7] + B*src[8] + C*src[stride+7] + D*src[stride+8] + rounder)>>8;
        dst+= stride;
        src+= stride;
    }
}

static void gmc_c(uint8_t *dst, uint8_t *src, int stride, int h, int ox, int oy, 
                  int dxx, int dxy, int dyx, int dyy, int shift, int r, int width, int height)
{
    int y, vx, vy;
    const int s= 1<<shift;
    
    width--;
    height--;

    for(y=0; y<h; y++){
        int x;

        vx= ox;
        vy= oy;
        for(x=0; x<8; x++){ /*XXX FIXME optimize*/
            int src_x, src_y, frac_x, frac_y, index;

            src_x= vx>>16;
            src_y= vy>>16;
            frac_x= src_x&(s-1);
            frac_y= src_y&(s-1);
            src_x>>=shift;
            src_y>>=shift;
  
            if((unsigned)src_x < width){
                if((unsigned)src_y < height){
                    index= src_x + src_y*stride;
                    dst[y*stride + x]= (  (  src[index         ]*(s-frac_x)
                                           + src[index       +1]*   frac_x )*(s-frac_y)
                                        + (  src[index+stride  ]*(s-frac_x)
                                           + src[index+stride+1]*   frac_x )*   frac_y
                                        + r)>>(shift*2);
                }else{
                    index= src_x + clip(src_y, 0, height)*stride;                    
                    dst[y*stride + x]= ( (  src[index         ]*(s-frac_x) 
                                          + src[index       +1]*   frac_x )*s
                                        + r)>>(shift*2);
                }
            }else{
                if((unsigned)src_y < height){
                    index= clip(src_x, 0, width) + src_y*stride;                    
                    dst[y*stride + x]= (  (  src[index         ]*(s-frac_y) 
                                           + src[index+stride  ]*   frac_y )*s
                                        + r)>>(shift*2);
                }else{
                    index= clip(src_x, 0, width) + clip(src_y, 0, height)*stride;                    
                    dst[y*stride + x]=    src[index         ];
                }
            }
            
            vx+= dxx;
            vy+= dyx;
        }
        ox += dxy;
        oy += dyy;
    }
}

int pix_abs16_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - pix2[0]);
        s += abs(pix1[1] - pix2[1]);
        s += abs(pix1[2] - pix2[2]);
        s += abs(pix1[3] - pix2[3]);
        s += abs(pix1[4] - pix2[4]);
        s += abs(pix1[5] - pix2[5]);
        s += abs(pix1[6] - pix2[6]);
        s += abs(pix1[7] - pix2[7]);
        s += abs(pix1[8] - pix2[8]);
        s += abs(pix1[9] - pix2[9]);
        s += abs(pix1[10] - pix2[10]);
        s += abs(pix1[11] - pix2[11]);
        s += abs(pix1[12] - pix2[12]);
        s += abs(pix1[13] - pix2[13]);
        s += abs(pix1[14] - pix2[14]);
        s += abs(pix1[15] - pix2[15]);
        pix1 += line_size;
        pix2 += line_size;
    }
    return s;
}

static int pix_abs16_x2_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - avg2(pix2[0], pix2[1]));
        s += abs(pix1[1] - avg2(pix2[1], pix2[2]));
        s += abs(pix1[2] - avg2(pix2[2], pix2[3]));
        s += abs(pix1[3] - avg2(pix2[3], pix2[4]));
        s += abs(pix1[4] - avg2(pix2[4], pix2[5]));
        s += abs(pix1[5] - avg2(pix2[5], pix2[6]));
        s += abs(pix1[6] - avg2(pix2[6], pix2[7]));
        s += abs(pix1[7] - avg2(pix2[7], pix2[8]));
        s += abs(pix1[8] - avg2(pix2[8], pix2[9]));
        s += abs(pix1[9] - avg2(pix2[9], pix2[10]));
        s += abs(pix1[10] - avg2(pix2[10], pix2[11]));
        s += abs(pix1[11] - avg2(pix2[11], pix2[12]));
        s += abs(pix1[12] - avg2(pix2[12], pix2[13]));
        s += abs(pix1[13] - avg2(pix2[13], pix2[14]));
        s += abs(pix1[14] - avg2(pix2[14], pix2[15]));
        s += abs(pix1[15] - avg2(pix2[15], pix2[16]));
        pix1 += line_size;
        pix2 += line_size;
    }
    return s;
}

static int pix_abs16_y2_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;
    uint8_t *pix3 = pix2 + line_size;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - avg2(pix2[0], pix3[0]));
        s += abs(pix1[1] - avg2(pix2[1], pix3[1]));
        s += abs(pix1[2] - avg2(pix2[2], pix3[2]));
        s += abs(pix1[3] - avg2(pix2[3], pix3[3]));
        s += abs(pix1[4] - avg2(pix2[4], pix3[4]));
        s += abs(pix1[5] - avg2(pix2[5], pix3[5]));
        s += abs(pix1[6] - avg2(pix2[6], pix3[6]));
        s += abs(pix1[7] - avg2(pix2[7], pix3[7]));
        s += abs(pix1[8] - avg2(pix2[8], pix3[8]));
        s += abs(pix1[9] - avg2(pix2[9], pix3[9]));
        s += abs(pix1[10] - avg2(pix2[10], pix3[10]));
        s += abs(pix1[11] - avg2(pix2[11], pix3[11]));
        s += abs(pix1[12] - avg2(pix2[12], pix3[12]));
        s += abs(pix1[13] - avg2(pix2[13], pix3[13]));
        s += abs(pix1[14] - avg2(pix2[14], pix3[14]));
        s += abs(pix1[15] - avg2(pix2[15], pix3[15]));
        pix1 += line_size;
        pix2 += line_size;
        pix3 += line_size;
    }
    return s;
}

static int pix_abs16_xy2_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;
    uint8_t *pix3 = pix2 + line_size;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - avg4(pix2[0], pix2[1], pix3[0], pix3[1]));
        s += abs(pix1[1] - avg4(pix2[1], pix2[2], pix3[1], pix3[2]));
        s += abs(pix1[2] - avg4(pix2[2], pix2[3], pix3[2], pix3[3]));
        s += abs(pix1[3] - avg4(pix2[3], pix2[4], pix3[3], pix3[4]));
        s += abs(pix1[4] - avg4(pix2[4], pix2[5], pix3[4], pix3[5]));
        s += abs(pix1[5] - avg4(pix2[5], pix2[6], pix3[5], pix3[6]));
        s += abs(pix1[6] - avg4(pix2[6], pix2[7], pix3[6], pix3[7]));
        s += abs(pix1[7] - avg4(pix2[7], pix2[8], pix3[7], pix3[8]));
        s += abs(pix1[8] - avg4(pix2[8], pix2[9], pix3[8], pix3[9]));
        s += abs(pix1[9] - avg4(pix2[9], pix2[10], pix3[9], pix3[10]));
        s += abs(pix1[10] - avg4(pix2[10], pix2[11], pix3[10], pix3[11]));
        s += abs(pix1[11] - avg4(pix2[11], pix2[12], pix3[11], pix3[12]));
        s += abs(pix1[12] - avg4(pix2[12], pix2[13], pix3[12], pix3[13]));
        s += abs(pix1[13] - avg4(pix2[13], pix2[14], pix3[13], pix3[14]));
        s += abs(pix1[14] - avg4(pix2[14], pix2[15], pix3[14], pix3[15]));
        s += abs(pix1[15] - avg4(pix2[15], pix2[16], pix3[15], pix3[16]));
        pix1 += line_size;
        pix2 += line_size;
        pix3 += line_size;
    }
    return s;
}

int pix_abs8_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - pix2[0]);
        s += abs(pix1[1] - pix2[1]);
        s += abs(pix1[2] - pix2[2]);
        s += abs(pix1[3] - pix2[3]);
        s += abs(pix1[4] - pix2[4]);
        s += abs(pix1[5] - pix2[5]);
        s += abs(pix1[6] - pix2[6]);
        s += abs(pix1[7] - pix2[7]);
        pix1 += line_size;
        pix2 += line_size;
    }
    return s;
}

static int pix_abs8_x2_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - avg2(pix2[0], pix2[1]));
        s += abs(pix1[1] - avg2(pix2[1], pix2[2]));
        s += abs(pix1[2] - avg2(pix2[2], pix2[3]));
        s += abs(pix1[3] - avg2(pix2[3], pix2[4]));
        s += abs(pix1[4] - avg2(pix2[4], pix2[5]));
        s += abs(pix1[5] - avg2(pix2[5], pix2[6]));
        s += abs(pix1[6] - avg2(pix2[6], pix2[7]));
        s += abs(pix1[7] - avg2(pix2[7], pix2[8]));
        pix1 += line_size;
        pix2 += line_size;
    }
    return s;
}

static int pix_abs8_y2_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;
    uint8_t *pix3 = pix2 + line_size;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - avg2(pix2[0], pix3[0]));
        s += abs(pix1[1] - avg2(pix2[1], pix3[1]));
        s += abs(pix1[2] - avg2(pix2[2], pix3[2]));
        s += abs(pix1[3] - avg2(pix2[3], pix3[3]));
        s += abs(pix1[4] - avg2(pix2[4], pix3[4]));
        s += abs(pix1[5] - avg2(pix2[5], pix3[5]));
        s += abs(pix1[6] - avg2(pix2[6], pix3[6]));
        s += abs(pix1[7] - avg2(pix2[7], pix3[7]));
        pix1 += line_size;
        pix2 += line_size;
        pix3 += line_size;
    }
    return s;
}

static int pix_abs8_xy2_c(void *v, uint8_t *pix1, uint8_t *pix2, int line_size, int h)
{
    int s, i;
    uint8_t *pix3 = pix2 + line_size;

    s = 0;
    for(i=0;i<h;i++) {
        s += abs(pix1[0] - avg4(pix2[0], pix2[1], pix3[0], pix3[1]));
        s += abs(pix1[1] - avg4(pix2[1], pix2[2], pix3[1], pix3[2]));
        s += abs(pix1[2] - avg4(pix2[2], pix2[3], pix3[2], pix3[3]));
        s += abs(pix1[3] - avg4(pix2[3], pix2[4], pix3[3], pix3[4]));
        s += abs(pix1[4] - avg4(pix2[4], pix2[5], pix3[4], pix3[5]));
        s += abs(pix1[5] - avg4(pix2[5], pix2[6], pix3[5], pix3[6]));
        s += abs(pix1[6] - avg4(pix2[6], pix2[7], pix3[6], pix3[7]));
        s += abs(pix1[7] - avg4(pix2[7], pix2[8], pix3[7], pix3[8]));
        pix1 += line_size;
        pix2 += line_size;
        pix3 += line_size;
    }
    return s;
}

#define CALL_2X_PIXELS(a, b, n)\
static void a(uint8_t *block, const uint8_t *pixels, int line_size, int h){\
    b(block  , pixels  , line_size, h);\
    b(block+n, pixels+n, line_size, h);\
}

#define PIXOP2(OPNAME, OP) \
static void OPNAME ## _pixels8_c(uint8_t *block, const uint8_t *pixels, int line_size, int h){\
    int i;\
    for(i=0; i<h; i++){\
        OP(*((uint32_t*)(block  )), LD32(pixels  ));\
        OP(*((uint32_t*)(block+4)), LD32(pixels+4));\
        pixels+=line_size;\
        block +=line_size;\
    }\
}\
static void OPNAME ## _pixels8_l2(uint8_t *dst, const uint8_t *src1, const uint8_t *src2, int dst_stride, \
                                                int src_stride1, int src_stride2, int h){\
    int i;\
    for(i=0; i<h; i++){\
        uint32_t a,b;\
        a= LD32(&src1[i*src_stride1  ]);\
        b= LD32(&src2[i*src_stride2  ]);\
        OP(*((uint32_t*)&dst[i*dst_stride  ]), rnd_avg32(a, b));\
        a= LD32(&src1[i*src_stride1+4]);\
        b= LD32(&src2[i*src_stride2+4]);\
        OP(*((uint32_t*)&dst[i*dst_stride+4]), rnd_avg32(a, b));\
    }\
}\
\
static void OPNAME ## _pixels16_l2(uint8_t *dst, const uint8_t *src1, const uint8_t *src2, int dst_stride, \
                                                int src_stride1, int src_stride2, int h){\
    OPNAME ## _pixels8_l2(dst  , src1  , src2  , dst_stride, src_stride1, src_stride2, h);\
    OPNAME ## _pixels8_l2(dst+8, src1+8, src2+8, dst_stride, src_stride1, src_stride2, h);\
}\
\
static void OPNAME ## _pixels8_x2_c(uint8_t *block, const uint8_t *pixels, int line_size, int h){\
    OPNAME ## _pixels8_l2(block, pixels, pixels+1, line_size, line_size, line_size, h);\
}\
\
static void OPNAME ## _pixels8_y2_c(uint8_t *block, const uint8_t *pixels, int line_size, int h){\
    OPNAME ## _pixels8_l2(block, pixels, pixels+line_size, line_size, line_size, line_size, h);\
}\
\
static void OPNAME ## _pixels8_xy2_c(uint8_t *block, const uint8_t *pixels, int line_size, int h)\
{\
    int j;\
    for(j=0; j<2; j++){\
        int i;\
        const uint32_t a= LD32(pixels  );\
        const uint32_t b= LD32(pixels+1);\
        uint32_t l0=  (a&0x03030303UL)\
                    + (b&0x03030303UL)\
                    + 0x02020202UL;\
        uint32_t h0= ((a&0xFCFCFCFCUL)>>2)\
                   + ((b&0xFCFCFCFCUL)>>2);\
        uint32_t l1,h1;\
\
        pixels+=line_size;\
        for(i=0; i<h; i+=2){\
            uint32_t a= LD32(pixels  );\
            uint32_t b= LD32(pixels+1);\
            l1=  (a&0x03030303UL)\
               + (b&0x03030303UL);\
            h1= ((a&0xFCFCFCFCUL)>>2)\
              + ((b&0xFCFCFCFCUL)>>2);\
            OP(*((uint32_t*)block), h0+h1+(((l0+l1)>>2)&0x0F0F0F0FUL));\
            pixels+=line_size;\
            block +=line_size;\
            a= LD32(pixels  );\
            b= LD32(pixels+1);\
            l0=  (a&0x03030303UL)\
               + (b&0x03030303UL)\
               + 0x02020202UL;\
            h0= ((a&0xFCFCFCFCUL)>>2)\
              + ((b&0xFCFCFCFCUL)>>2);\
            OP(*((uint32_t*)block), h0+h1+(((l0+l1)>>2)&0x0F0F0F0FUL));\
            pixels+=line_size;\
            block +=line_size;\
        }\
        pixels+=4-line_size*(h+1);\
        block +=4-line_size*h;\
    }\
}\
\
CALL_2X_PIXELS(OPNAME ## _pixels16_c  , OPNAME ## _pixels8_c  , 8)\
CALL_2X_PIXELS(OPNAME ## _pixels16_x2_c , OPNAME ## _pixels8_x2_c , 8)\
CALL_2X_PIXELS(OPNAME ## _pixels16_y2_c , OPNAME ## _pixels8_y2_c , 8)\
CALL_2X_PIXELS(OPNAME ## _pixels16_xy2_c, OPNAME ## _pixels8_xy2_c, 8)\

#define op_avg(a, b) a = rnd_avg32(a, b)
#define op_put(a, b) a = b

PIXOP2(avg, op_avg)
PIXOP2(put, op_put)
#undef op_avg
#undef op_put

/**
 * memset(blocks, 0, sizeof(DCTELEM)*6*64)
 */
static void clear_blocks_c(DCTELEM *blocks)
{
    memset(blocks, 0, sizeof(DCTELEM)*6*64);
}

/* init static data */
void dsputil_static_init(void)
{
    int i;

    for(i=0;i<256;i++) cropTbl[i + MAX_NEG_CROP] = i;
    for(i=0;i<MAX_NEG_CROP;i++) {
        cropTbl[i] = 0;
        cropTbl[i + MAX_NEG_CROP + 256] = 255;
    }
    
    for(i=0;i<512;i++) {
        squareTbl[i] = (i - 256) * (i - 256);
    }
}


void dsputil_init(DSPContext* c, AVCodecContext *avctx)
{
  c->get_pixels = get_pixels_c;
  c->diff_pixels = diff_pixels_c;

  c->gmc1 = gmc1_c;
  c->gmc = gmc_c;
  c->clear_blocks = clear_blocks_c;
  c->pix_sum = pix_sum_c;
  c->pix_norm1 = pix_norm1_c;

  /* TODO [0] 16  [1] 8 */
  c->pix_abs[0][0] = pix_abs16_c;
  c->pix_abs[0][1] = pix_abs16_x2_c;
  c->pix_abs[0][2] = pix_abs16_y2_c;
  c->pix_abs[0][3] = pix_abs16_xy2_c;
  c->pix_abs[1][0] = pix_abs8_c;
  c->pix_abs[1][1] = pix_abs8_x2_c;
  c->pix_abs[1][2] = pix_abs8_y2_c;
  c->pix_abs[1][3] = pix_abs8_xy2_c;

#define dspfunc(PFX, IDX, NUM) \
    c->PFX ## _pixels_tab[IDX][0] = PFX ## _pixels ## NUM ## _c;     \
    c->PFX ## _pixels_tab[IDX][1] = PFX ## _pixels ## NUM ## _x2_c;  \
    c->PFX ## _pixels_tab[IDX][2] = PFX ## _pixels ## NUM ## _y2_c;  \
    c->PFX ## _pixels_tab[IDX][3] = PFX ## _pixels ## NUM ## _xy2_c

    dspfunc(put, 0, 16);
    dspfunc(put, 1, 8);

    dspfunc(avg, 0, 16);
    dspfunc(avg, 1, 8);
#undef dspfunc
}
