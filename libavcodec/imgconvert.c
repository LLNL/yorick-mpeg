/*
 * Misc image convertion routines
 * Copyright (c) 2001, 2002, 2003 Fabrice Bellard.
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
 * @file imgconvert.c
 * Misc image convertion routines.
 */

/* TODO:
 * - write 'ffimg' program to test all the image related stuff
 * - move all api to slice based system
 * - integrate deinterlacing, postprocessing and scaling in the conversion process
 */

#include "avcodec.h"
#include "dsputil.h"

#define FF_COLOR_RGB      0 /* RGB color space */
#define FF_COLOR_GRAY     1 /* gray color space */
#define FF_COLOR_YUV      2 /* YUV color space. 16 <= Y <= 235, 16 <= U, V <= 240 */
#define FF_COLOR_YUV_JPEG 3 /* YUV color space. 0 <= Y <= 255, 0 <= U, V <= 255 */

#define FF_PIXEL_PLANAR   0 /* each channel has one component in AVPicture */
#define FF_PIXEL_PACKED   1 /* only one components containing all the channels */
#define FF_PIXEL_PALETTE  2  /* one components containing indexes for a palette */

typedef struct PixFmtInfo {
    const char *name;
    uint8_t nb_channels;     /* number of channels (including alpha) */
    uint8_t color_type;      /* color type (see FF_COLOR_xxx constants) */
    uint8_t pixel_type;      /* pixel storage type (see FF_PIXEL_xxx constants) */
    uint8_t is_alpha : 1;    /* true if alpha can be specified */
    uint8_t x_chroma_shift;  /* X chroma subsampling factor is 2 ^ shift */
    uint8_t y_chroma_shift;  /* Y chroma subsampling factor is 2 ^ shift */
    uint8_t depth;           /* bit depth of the color components */
} PixFmtInfo;

/* this table gives more information about formats */
static PixFmtInfo pix_fmt_info[PIX_FMT_NB] = {
  /* YUV formats */
  { "yuv420p", 3, FF_COLOR_YUV, FF_PIXEL_PLANAR, 0, 1, 1, 8 },
  { "yuv422", 1, FF_COLOR_YUV, FF_PIXEL_PACKED, 0, 1, 0, 8 },  /* unused */
  /* RGB formats */
  { "rgb24", 3, FF_COLOR_RGB, FF_PIXEL_PACKED, 0, 0, 0, 8 }
};

void avcodec_get_chroma_sub_sample(int pix_fmt, int *h_shift, int *v_shift)
{
    *h_shift = pix_fmt_info[pix_fmt].x_chroma_shift;
    *v_shift = pix_fmt_info[pix_fmt].y_chroma_shift;
}

/* Picture field are filled with 'ptr' addresses. Also return size */
int avpicture_fill(AVPicture *picture, uint8_t *ptr,
		   int pix_fmt, int width, int height)
{
  int size, w2, h2, size2;
  PixFmtInfo *pinfo;
    
  pinfo = &pix_fmt_info[pix_fmt];
  size = width * height;
  switch(pix_fmt) {
  case PIX_FMT_YUV420P:
    w2 = (width + (1 << pinfo->x_chroma_shift) - 1) >> pinfo->x_chroma_shift;
    h2 = (height + (1 << pinfo->y_chroma_shift) - 1) >> pinfo->y_chroma_shift;
    size2 = w2 * h2;
    picture->data[0] = ptr;
    picture->data[1] = picture->data[0] + size;
    picture->data[2] = picture->data[1] + size2;
    picture->linesize[0] = width;
    picture->linesize[1] = w2;
    picture->linesize[2] = w2;
    return size + 2 * size2;
  case PIX_FMT_RGB24:
    picture->data[0] = ptr;
    picture->data[1] = NULL;
    picture->data[2] = NULL;
    picture->linesize[0] = width * 3;
    return size * 3;
  default:
    picture->data[0] = NULL;
    picture->data[1] = NULL;
    picture->data[2] = NULL;
    picture->data[3] = NULL;
    return -1;
  }
}

int avpicture_get_size(int pix_fmt, int width, int height)
{
  AVPicture dummy_pict;
  return avpicture_fill(&dummy_pict, NULL, pix_fmt, width, height);
}

#define SCALEBITS 10
#define ONE_HALF  (1 << (SCALEBITS - 1))
#define FIX(x)	  ((int) ((x) * (1<<SCALEBITS) + 0.5))

#define RGB_TO_Y_CCIR(r, g, b) \
((FIX(0.29900*219.0/255.0) * (r) + FIX(0.58700*219.0/255.0) * (g) + \
  FIX(0.11400*219.0/255.0) * (b) + (ONE_HALF + (16 << SCALEBITS))) >> SCALEBITS)

#define RGB_TO_U_CCIR(r1, g1, b1, shift)\
(((- FIX(0.16874*224.0/255.0) * r1 - FIX(0.33126*224.0/255.0) * g1 +         \
     FIX(0.50000*224.0/255.0) * b1 + (ONE_HALF << shift) - 1) >> (SCALEBITS + shift)) + 128)

#define RGB_TO_V_CCIR(r1, g1, b1, shift)\
(((FIX(0.50000*224.0/255.0) * r1 - FIX(0.41869*224.0/255.0) * g1 -           \
   FIX(0.08131*224.0/255.0) * b1 + (ONE_HALF << shift) - 1) >> (SCALEBITS + shift)) + 128)

#undef RGB_IN
#define RGB_IN(r, g, b, s)\
{\
    r = (s)[0];\
    g = (s)[1];\
    b = (s)[2];\
}

#undef BPP
#define BPP 3

static void 
rgb24_to_yuv420p(AVPicture *dst, const AVPicture *src, int width, int height);

static void 
rgb24_to_yuv420p(AVPicture *dst, const AVPicture *src, int width, int height)
{
  int wrap, wrap3, width2;
  int r, g, b, r1, g1, b1, w;
  uint8_t *lum, *cb, *cr;
  const uint8_t *p;

  lum = dst->data[0];
  cb = dst->data[1];
  cr = dst->data[2];

  width2 = (width + 1) >> 1;
  wrap = dst->linesize[0];
  wrap3 = src->linesize[0];
  p = src->data[0];
  for (; height>=2 ; height-=2) {
    for(w = width ; w>=2 ; w-=2) {
      RGB_IN(r, g, b, p);
      r1 = r;
      g1 = g;
      b1 = b;
      lum[0] = RGB_TO_Y_CCIR(r, g, b);

      RGB_IN(r, g, b, p + BPP);
      r1 += r;
      g1 += g;
      b1 += b;
      lum[1] = RGB_TO_Y_CCIR(r, g, b);
      p += wrap3;
      lum += wrap;

      RGB_IN(r, g, b, p);
      r1 += r;
      g1 += g;
      b1 += b;
      lum[0] = RGB_TO_Y_CCIR(r, g, b);

      RGB_IN(r, g, b, p + BPP);
      r1 += r;
      g1 += g;
      b1 += b;
      lum[1] = RGB_TO_Y_CCIR(r, g, b);

      cb[0] = RGB_TO_U_CCIR(r1, g1, b1, 2);
      cr[0] = RGB_TO_V_CCIR(r1, g1, b1, 2);

      cb++;
      cr++;
      p += -wrap3 + 2 * BPP;
      lum += -wrap + 2;
    }
    if (w) {
      RGB_IN(r, g, b, p);
      r1 = r;
      g1 = g;
      b1 = b;
      lum[0] = RGB_TO_Y_CCIR(r, g, b);
      p += wrap3;
      lum += wrap;
      RGB_IN(r, g, b, p);
      r1 += r;
      g1 += g;
      b1 += b;
      lum[0] = RGB_TO_Y_CCIR(r, g, b);
      cb[0] = RGB_TO_U_CCIR(r1, g1, b1, 1);
      cr[0] = RGB_TO_V_CCIR(r1, g1, b1, 1);
      cb++;
      cr++;
      p += -wrap3 + BPP;
      lum += -wrap + 1;
    }
    p += wrap3 + (wrap3 - width * BPP);
    lum += wrap + (wrap - width);
    cb += dst->linesize[1] - width2;
    cr += dst->linesize[2] - width2;
  }
  /* handle odd height */
  if (height) {
    for(w = width; w >= 2; w -= 2) {
      RGB_IN(r, g, b, p);
      r1 = r;
      g1 = g;
      b1 = b;
      lum[0] = RGB_TO_Y_CCIR(r, g, b);

      RGB_IN(r, g, b, p + BPP);
      r1 += r;
      g1 += g;
      b1 += b;
      lum[1] = RGB_TO_Y_CCIR(r, g, b);
      cb[0] = RGB_TO_U_CCIR(r1, g1, b1, 1);
      cr[0] = RGB_TO_V_CCIR(r1, g1, b1, 1);
      cb++;
      cr++;
      p += 2 * BPP;
      lum += 2;
    }
    if (w) {
      RGB_IN(r, g, b, p);
      lum[0] = RGB_TO_Y_CCIR(r, g, b);
      cb[0] = RGB_TO_U_CCIR(r, g, b, 0);
      cr[0] = RGB_TO_V_CCIR(r, g, b, 0);
    }
  }
}

/* XXX: always use linesize. Return -1 if not supported */
int img_convert(AVPicture *dst, int dst_pix_fmt,
                const AVPicture *src, int src_pix_fmt, 
                int src_width, int src_height)
{
  if (src_pix_fmt!=PIX_FMT_RGB24 || dst_pix_fmt!=PIX_FMT_YUV420P)
    return -1;
  if (src_width <= 0 || src_height <= 0)
    return 0;

  rgb24_to_yuv420p(dst, src, src_width, src_height);
  return 0;
}

#undef FIX
