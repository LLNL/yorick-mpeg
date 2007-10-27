/*
 * MPEG1 codec / MPEG2 decoder
 * Copyright (c) 2000,2001 Fabrice Bellard.
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
 * @file mpeg12.c
 * MPEG1/2 codec
 */
 
/*#define DEBUG*/
#include "avcodec.h"
#include "dsputil.h"
#include "mpegvideo.h"

const int16_t ff_mpeg1_default_intra_matrix[64] = {
	8, 16, 19, 22, 26, 27, 29, 34,
	16, 16, 22, 24, 27, 29, 34, 37,
	19, 22, 26, 27, 29, 34, 34, 38,
	22, 22, 26, 27, 29, 34, 37, 40,
	22, 26, 27, 29, 32, 35, 40, 48,
	26, 27, 29, 32, 35, 40, 48, 58,
	26, 27, 29, 34, 38, 46, 56, 69,
	27, 29, 35, 38, 46, 56, 69, 83
};

const int16_t ff_mpeg1_default_non_intra_matrix[64] = {
    16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16,
};

static const uint16_t vlc_dc_lum_code[12] = {
    0x4, 0x0, 0x1, 0x5, 0x6, 0xe, 0x1e, 0x3e, 0x7e, 0xfe, 0x1fe, 0x1ff,
};
static const unsigned char vlc_dc_lum_bits[12] = {
    3, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 9,
};

const uint16_t vlc_dc_chroma_code[12] = {
    0x0, 0x1, 0x2, 0x6, 0xe, 0x1e, 0x3e, 0x7e, 0xfe, 0x1fe, 0x3fe, 0x3ff,
};
const unsigned char vlc_dc_chroma_bits[12] = {
    2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10,
};

static const uint16_t mpeg1_vlc[113][2] = {
 { 0x3, 2 }, { 0x4, 4 }, { 0x5, 5 }, { 0x6, 7 },
 { 0x26, 8 }, { 0x21, 8 }, { 0xa, 10 }, { 0x1d, 12 },
 { 0x18, 12 }, { 0x13, 12 }, { 0x10, 12 }, { 0x1a, 13 },
 { 0x19, 13 }, { 0x18, 13 }, { 0x17, 13 }, { 0x1f, 14 },
 { 0x1e, 14 }, { 0x1d, 14 }, { 0x1c, 14 }, { 0x1b, 14 },
 { 0x1a, 14 }, { 0x19, 14 }, { 0x18, 14 }, { 0x17, 14 },
 { 0x16, 14 }, { 0x15, 14 }, { 0x14, 14 }, { 0x13, 14 },
 { 0x12, 14 }, { 0x11, 14 }, { 0x10, 14 }, { 0x18, 15 },
 { 0x17, 15 }, { 0x16, 15 }, { 0x15, 15 }, { 0x14, 15 },
 { 0x13, 15 }, { 0x12, 15 }, { 0x11, 15 }, { 0x10, 15 },
 { 0x3, 3 }, { 0x6, 6 }, { 0x25, 8 }, { 0xc, 10 },
 { 0x1b, 12 }, { 0x16, 13 }, { 0x15, 13 }, { 0x1f, 15 },
 { 0x1e, 15 }, { 0x1d, 15 }, { 0x1c, 15 }, { 0x1b, 15 },
 { 0x1a, 15 }, { 0x19, 15 }, { 0x13, 16 }, { 0x12, 16 },
 { 0x11, 16 }, { 0x10, 16 }, { 0x5, 4 }, { 0x4, 7 },
 { 0xb, 10 }, { 0x14, 12 }, { 0x14, 13 }, { 0x7, 5 },
 { 0x24, 8 }, { 0x1c, 12 }, { 0x13, 13 }, { 0x6, 5 },
 { 0xf, 10 }, { 0x12, 12 }, { 0x7, 6 }, { 0x9, 10 },
 { 0x12, 13 }, { 0x5, 6 }, { 0x1e, 12 }, { 0x14, 16 },
 { 0x4, 6 }, { 0x15, 12 }, { 0x7, 7 }, { 0x11, 12 },
 { 0x5, 7 }, { 0x11, 13 }, { 0x27, 8 }, { 0x10, 13 },
 { 0x23, 8 }, { 0x1a, 16 }, { 0x22, 8 }, { 0x19, 16 },
 { 0x20, 8 }, { 0x18, 16 }, { 0xe, 10 }, { 0x17, 16 },
 { 0xd, 10 }, { 0x16, 16 }, { 0x8, 10 }, { 0x15, 16 },
 { 0x1f, 12 }, { 0x1a, 12 }, { 0x19, 12 }, { 0x17, 12 },
 { 0x16, 12 }, { 0x1f, 13 }, { 0x1e, 13 }, { 0x1d, 13 },
 { 0x1c, 13 }, { 0x1b, 13 }, { 0x1f, 16 }, { 0x1e, 16 },
 { 0x1d, 16 }, { 0x1c, 16 }, { 0x1b, 16 },
 { 0x1, 6 }, /* escape */
 { 0x2, 2 }, /* EOB */
};

static const int8_t mpeg1_level[111] = {
  1,  2,  3,  4,  5,  6,  7,  8,
  9, 10, 11, 12, 13, 14, 15, 16,
 17, 18, 19, 20, 21, 22, 23, 24,
 25, 26, 27, 28, 29, 30, 31, 32,
 33, 34, 35, 36, 37, 38, 39, 40,
  1,  2,  3,  4,  5,  6,  7,  8,
  9, 10, 11, 12, 13, 14, 15, 16,
 17, 18,  1,  2,  3,  4,  5,  1,
  2,  3,  4,  1,  2,  3,  1,  2,
  3,  1,  2,  3,  1,  2,  1,  2,
  1,  2,  1,  2,  1,  2,  1,  2,
  1,  2,  1,  2,  1,  2,  1,  2,
  1,  1,  1,  1,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,
};

static const int8_t mpeg1_run[111] = {
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
  1,  1,  1,  1,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,
  1,  1,  2,  2,  2,  2,  2,  3,
  3,  3,  3,  4,  4,  4,  5,  5,
  5,  6,  6,  6,  7,  7,  8,  8,
  9,  9, 10, 10, 11, 11, 12, 12,
 13, 13, 14, 14, 15, 15, 16, 16,
 17, 18, 19, 20, 21, 22, 23, 24,
 25, 26, 27, 28, 29, 30, 31,
};

static RLTable rl_mpeg1 = {
    111,
    111,
    mpeg1_vlc,
    mpeg1_run,
    mpeg1_level,
};

static const uint8_t mbAddrIncrTable[36][2] = {
    {0x1, 1},
    {0x3, 3},
    {0x2, 3},
    {0x3, 4},
    {0x2, 4},
    {0x3, 5},
    {0x2, 5},
    {0x7, 7},
    {0x6, 7},
    {0xb, 8},
    {0xa, 8},
    {0x9, 8},
    {0x8, 8},
    {0x7, 8},
    {0x6, 8},
    {0x17, 10},
    {0x16, 10},
    {0x15, 10},
    {0x14, 10},
    {0x13, 10},
    {0x12, 10},
    {0x23, 11},
    {0x22, 11},
    {0x21, 11},
    {0x20, 11},
    {0x1f, 11},
    {0x1e, 11},
    {0x1d, 11},
    {0x1c, 11},
    {0x1b, 11},
    {0x1a, 11},
    {0x19, 11},
    {0x18, 11},
    {0x8, 11}, /* escape */
    {0xf, 11}, /* stuffing */
    {0x0, 8}, /* end (and 15 more 0 bits should follow) */
};

static const uint8_t mbPatTable[64][2] = {
    {0x1, 9},
    {0xb, 5},
    {0x9, 5},
    {0xd, 6},
    {0xd, 4},
    {0x17, 7},
    {0x13, 7},
    {0x1f, 8},
    {0xc, 4},
    {0x16, 7},
    {0x12, 7},
    {0x1e, 8},
    {0x13, 5},
    {0x1b, 8},
    {0x17, 8},
    {0x13, 8},
    {0xb, 4},
    {0x15, 7},
    {0x11, 7},
    {0x1d, 8},
    {0x11, 5},
    {0x19, 8},
    {0x15, 8},
    {0x11, 8},
    {0xf, 6},
    {0xf, 8},
    {0xd, 8},
    {0x3, 9},
    {0xf, 5},
    {0xb, 8},
    {0x7, 8},
    {0x7, 9},
    {0xa, 4},
    {0x14, 7},
    {0x10, 7},
    {0x1c, 8},
    {0xe, 6},
    {0xe, 8},
    {0xc, 8},
    {0x2, 9},
    {0x10, 5},
    {0x18, 8},
    {0x14, 8},
    {0x10, 8},
    {0xe, 5},
    {0xa, 8},
    {0x6, 8},
    {0x6, 9},
    {0x12, 5},
    {0x1a, 8},
    {0x16, 8},
    {0x12, 8},
    {0xd, 5},
    {0x9, 8},
    {0x5, 8},
    {0x5, 9},
    {0xc, 5},
    {0x8, 8},
    {0x4, 8},
    {0x4, 9},
    {0x7, 3},
    {0xa, 5},
    {0x8, 5},
    {0xc, 6}
};

static const uint8_t mbMotionVectorTable[17][2] = {
{ 0x1, 1 },
{ 0x1, 2 },
{ 0x1, 3 },
{ 0x1, 4 },
{ 0x3, 6 },
{ 0x5, 7 },
{ 0x4, 7 },
{ 0x3, 7 },
{ 0xb, 9 },
{ 0xa, 9 },
{ 0x9, 9 },
{ 0x11, 10 },
{ 0x10, 10 },
{ 0xf, 10 },
{ 0xe, 10 },
{ 0xd, 10 },
{ 0xc, 10 },
};

static const AVRational frame_rate_tab[] = {
    {    0,    0},
    {24000, 1001},
    {   24,    1},
    {   25,    1},
    {30000, 1001},
    {   30,    1},
    {   50,    1},
    {60000, 1001},
    {   60,    1},
  /* Xing's 15fps: (9)*/
    {   15,    1},
  /* libmpeg3's "Unofficial economy rates": (10-13)*/
    {    5,    1},
    {   10,    1},
    {   12,    1},
    {   15,    1},
    {    0,    0},
};

uint8_t ff_mpeg1_dc_scale_table[128]={
/*  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31*/
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
};

static const float mpeg1_aspect[16]={
    0.0000,
    1.0000,
    0.6735,
    0.7031,
    
    0.7615,
    0.8055,
    0.8437,
    0.8935,

    0.9157,
    0.9815,
    1.0255,
    1.0695,

    1.0950,
    1.1575,
    1.2015,
};


/* Start codes. */
#define SEQ_END_CODE		0x000001b7
#define SEQ_START_CODE		0x000001b3
#define GOP_START_CODE		0x000001b8
#define PICTURE_START_CODE	0x00000100
#define SLICE_MIN_START_CODE	0x00000101
#define SLICE_MAX_START_CODE	0x000001af
#define EXT_START_CODE		0x000001b5
#define USER_START_CODE		0x000001b2

static void mpeg1_encode_block(MpegEncContext *s, DCTELEM *block, int n);
static void mpeg1_encode_motion(MpegEncContext *s, int val, int f_or_b_code);    /* RAL: f_code parameter added*/

static uint8_t (*mv_penalty)[MAX_MV*2+1]= NULL;
static uint8_t fcode_tab[MAX_MV*2+1];

static uint32_t uni_mpeg1_ac_vlc_bits[64*64*2];
static uint8_t  uni_mpeg1_ac_vlc_len [64*64*2];

/* simple include everything table for dc, first byte is bits number next 3 are code*/
static uint32_t mpeg1_lum_dc_uni[512];
static uint32_t mpeg1_chr_dc_uni[512];

static uint8_t mpeg1_index_run[2][64];
static int8_t mpeg1_max_level[2][64];

static void init_uni_ac_vlc(RLTable *rl, uint32_t *uni_ac_vlc_bits,
                            uint8_t *uni_ac_vlc_len)
{
  int i;

  for(i=0; i<128; i++){
    int level= i-64;
    int run;
    for(run=0; run<64; run++){
      int len, bits, code;
            
      int alevel= ABS(level);
      int sign= (level>>31)&1;

      if (alevel > rl->max_level[0][run])
        code= 111; /*rl->n*/
      else
        code= rl->index_run[0][run] + alevel - 1;

      if (code < 111 /* rl->n */) {
        /* store the vlc & sign at once */
        len=   mpeg1_vlc[code][1]+1;
        bits= (mpeg1_vlc[code][0]<<1) + sign;
      } else {
        len=  mpeg1_vlc[111/*rl->n*/][1]+6;
        bits= mpeg1_vlc[111/*rl->n*/][0]<<6;

        bits|= run;
        if (alevel < 128) {
          bits<<=8; len+=8;
          bits|= level & 0xff;
        } else {
          bits<<=16; len+=16;
          bits|= level & 0xff;
          if (level < 0) {
            bits|= 0x8001 + level + 255;
          } else {
            bits|= level & 0xffff;
          }
        }
      }

      uni_ac_vlc_bits[UNI_AC_ENC_INDEX(run, i)]= bits;
      uni_ac_vlc_len [UNI_AC_ENC_INDEX(run, i)]= len;
    }
  }
}

static int find_frame_rate_index(MpegEncContext *s)
{
  int i;
  int64_t dmin= INT64_MAX;
  int64_t d;

  for(i=1;i<14;i++) {
    int64_t n0= int64_t_C(1001)/frame_rate_tab[i].den * frame_rate_tab[i].num *
      s->avctx->frame_rate_base;
    int64_t n1= int64_t_C(1001) * s->avctx->frame_rate;

    d = ABS(n0 - n1);
    if(d < dmin){
      dmin=d;
      s->frame_rate_index= i;
    }
  }
  if(dmin)
    return -1;
  else
    return 0;
}

static int encode_init(AVCodecContext *avctx)
{
  MpegEncContext *s = avctx->priv_data;

  if(MPV_encode_init(avctx) < 0)
    return -1;

  if(find_frame_rate_index(s) < 0){
    av_log(avctx, AV_LOG_ERROR, "MPEG1/2 doesnt support %d/%d fps\n", avctx->frame_rate, avctx->frame_rate_base);
    return -1;
  }
    
  return 0;
}

static void put_header(MpegEncContext *s, int header)
{
  align_put_bits(&s->pb);
  put_bits(&s->pb, 16, header>>16);
  put_bits(&s->pb, 16, header&0xFFFF);
}

/* put sequence header if needed */
static void mpeg1_encode_sequence_header(MpegEncContext *s)
{
  unsigned int vbv_buffer_size;
  unsigned int fps, v;
  int i;
  uint64_t time_code;
  float best_aspect_error= 1E10;
  float aspect_ratio= av_q2d(s->avctx->sample_aspect_ratio);
  int constraint_parameter_flag;
        
  if(aspect_ratio==0.0) aspect_ratio= 1.0; /*pixel aspect 1:1 (VGA)*/
        
  if (s->current_picture.key_frame) {
    AVRational framerate= frame_rate_tab[s->frame_rate_index];
    int aspect_ratio_info = 1;

    /* mpeg1 header repeated every gop */
    put_header(s, SEQ_START_CODE);
 
    put_bits(&s->pb, 12, s->width);
    put_bits(&s->pb, 12, s->height);
            
    for(i=1; i<15; i++){
      float error= aspect_ratio;
      error-= 1.0/mpeg1_aspect[i];
             
      error= ABS(error);
                
      if(error < best_aspect_error){
        best_aspect_error= error;
        aspect_ratio_info= i;
      }
    }
            
    put_bits(&s->pb, 4, aspect_ratio_info);
    put_bits(&s->pb, 4, s->frame_rate_index);

    if(s->avctx->rc_max_rate){
      v = (s->avctx->rc_max_rate + 399) / 400;
      if (v > 0x3ffff) v = 0x3ffff;
    }else{
      v= 0x3FFFF;
    }

    if(s->avctx->rc_buffer_size)
      vbv_buffer_size = s->avctx->rc_buffer_size;
    else
      /* VBV calculation: Scaled so that a VCD has the proper VBV size of 40 kilobytes */
      vbv_buffer_size = (( 20 * s->bit_rate) / (1151929 / 2)) * 8 * 1024;
    vbv_buffer_size= (vbv_buffer_size + 16383) / 16384;

    put_bits(&s->pb, 18, v & 0x3FFFF);
    put_bits(&s->pb, 1, 1); /* marker */
    put_bits(&s->pb, 10, vbv_buffer_size & 0x3FF);

    constraint_parameter_flag= 
      s->width <= 768 && s->height <= 576 && 
      s->mb_width * s->mb_height <= 396 &&
      s->mb_width * s->mb_height * framerate.num <= framerate.den*396*25 &&
      framerate.num <= framerate.den*30 &&
      vbv_buffer_size <= 20 &&
      v <= 1856000/400;
                
    put_bits(&s->pb, 1, constraint_parameter_flag);

    ff_write_quant_matrix(&s->pb, s->avctx->intra_matrix);
    ff_write_quant_matrix(&s->pb, s->avctx->inter_matrix);

    put_header(s, GOP_START_CODE);
    put_bits(&s->pb, 1, 0); /* do drop frame */
    /* time code : we must convert from the real frame rate to a
       fake mpeg frame rate in case of low frame rate */
    fps = (framerate.num + framerate.den/2)/ framerate.den;
    time_code = s->current_picture_ptr->coded_picture_number;

    s->gop_picture_number = time_code;
    put_bits(&s->pb, 5, (uint32_t)((time_code / (fps * 3600)) % 24));
    put_bits(&s->pb, 6, (uint32_t)((time_code / (fps * 60)) % 60));
    put_bits(&s->pb, 1, 1);
    put_bits(&s->pb, 6, (uint32_t)((time_code / fps) % 60));
    put_bits(&s->pb, 6, (uint32_t)((time_code % fps)));
    put_bits(&s->pb, 1, 0);
    put_bits(&s->pb, 1, 0); /* broken link */
  }
}

#define encode_mb_skip_run(run) \
  while (run >= 33) {\
    put_bits(&s->pb, 11, 0x008);\
    run -= 33;\
  }\
  put_bits(&s->pb, mbAddrIncrTable[run][1], mbAddrIncrTable[run][0])

void ff_mpeg1_encode_slice_header(MpegEncContext *s)
{
  put_header(s, SLICE_MIN_START_CODE + s->mb_y);
  put_bits(&s->pb, 5, s->qscale); /* quantizer scale */
  put_bits(&s->pb, 1, 0); /* slice extra information */
}

void mpeg1_encode_picture_header(MpegEncContext *s, int picture_number)
{
  mpeg1_encode_sequence_header(s);

  /* mpeg1 picture header */
  put_header(s, PICTURE_START_CODE);
  /* temporal reference */

  /* RAL: s->picture_number instead of s->fake_picture_number*/
  put_bits(&s->pb, 10, (s->picture_number - 
                        s->gop_picture_number) & 0x3ff); 
  put_bits(&s->pb, 3, s->pict_type);

  s->vbv_delay_ptr= s->pb.buf + put_bits_count(&s->pb)/8;
  put_bits(&s->pb, 16, 0xFFFF); /* vbv_delay */
    
  /* RAL: Forward f_code also needed for B frames*/
  if (s->pict_type == P_TYPE || s->pict_type == B_TYPE) {
    put_bits(&s->pb, 1, 0); /* half pel coordinates */
    put_bits(&s->pb, 3, s->f_code); /* forward_f_code */
  }
    
  /* RAL: Backward f_code necessary for B frames*/
  if (s->pict_type == B_TYPE) {
    put_bits(&s->pb, 1, 0); /* half pel coordinates */
    put_bits(&s->pb, 3, s->b_code); /* backward_f_code */
  }

  put_bits(&s->pb, 1, 0); /* extra bit picture */

  s->mb_y=0;
  ff_mpeg1_encode_slice_header(s);
}

#define get_bits_diff(s) (last=(s)->last_bits, ((s)->last_bits=put_bits_count(&(s)->pb)-last))

void mpeg1_encode_mb(MpegEncContext *s, DCTELEM block[6][64],
                     int motion_x, int motion_y)
{
  int i, cbp, last;
  const int mb_x = s->mb_x;
  const int mb_y = s->mb_y;
  const int first_mb= mb_x == s->resync_mb_x && mb_y == s->resync_mb_y;

  /* compute cbp */
  cbp = 0;
  for(i=0;i<6;i++) {
    if (s->block_last_index[i] >= 0)
      cbp |= 1 << (5 - i);
  }
    
  if (cbp == 0 && !first_mb &&
      (mb_x != s->mb_width - 1 || (mb_y != s->mb_height - 1)) && 
      ((s->pict_type == P_TYPE && (motion_x | motion_y) == 0) ||
       (s->pict_type == B_TYPE && s->mv_dir == s->last_mv_dir && (((s->mv_dir & MV_DIR_FORWARD) ? ((s->mv[0][0][0] - s->last_mv[0][0][0])|(s->mv[0][0][1] - s->last_mv[0][0][1])) : 0) |
                                                                  ((s->mv_dir & MV_DIR_BACKWARD) ? ((s->mv[1][0][0] - s->last_mv[1][0][0])|(s->mv[1][0][1] - s->last_mv[1][0][1])) : 0)) == 0))) {
    s->mb_skip_run++;
    s->qscale -= s->dquant;
    s->skip_count++;
    s->misc_bits++;
    s->last_bits++;
    if(s->pict_type == P_TYPE){
      s->last_mv[0][1][0]= s->last_mv[0][0][0]= 
        s->last_mv[0][1][1]= s->last_mv[0][0][1]= 0;
    }
  } else {
    if(first_mb){
      assert(s->mb_skip_run == 0);
      encode_mb_skip_run(s->mb_x);
    }else{
      encode_mb_skip_run(s->mb_skip_run);
    }
        
    if (s->pict_type == I_TYPE) {
      if(s->dquant && cbp){
        put_bits(&s->pb, 2, 1); /* macroblock_type : macroblock_quant = 1 */
        put_bits(&s->pb, 5, s->qscale);
      }else{
        put_bits(&s->pb, 1, 1); /* macroblock_type : macroblock_quant = 0 */
        s->qscale -= s->dquant;
      }
      s->misc_bits+= get_bits_diff(s);
      s->i_count++;
    } else if (s->mb_intra) {
      if(s->dquant && cbp){
        put_bits(&s->pb, 6, 0x01);
        put_bits(&s->pb, 5, s->qscale);
      }else{
        put_bits(&s->pb, 5, 0x03);
        s->qscale -= s->dquant;
      }
      s->misc_bits+= get_bits_diff(s);
      s->i_count++;
      memset(s->last_mv, 0, sizeof(s->last_mv));
    } else if (s->pict_type == P_TYPE) { 
      if (cbp != 0) {
        if ((motion_x|motion_y) == 0) {
          if(s->dquant){
            put_bits(&s->pb, 5, 1); /* macroblock_pattern & quant */
            put_bits(&s->pb, 5, s->qscale);
          }else{
            put_bits(&s->pb, 2, 1); /* macroblock_pattern only */
          }
          s->misc_bits+= get_bits_diff(s);
        } else {
          if(s->dquant){
            put_bits(&s->pb, 5, 2); /* motion + cbp */
            put_bits(&s->pb, 5, s->qscale);
          }else{
            put_bits(&s->pb, 1, 1); /* motion + cbp */
          }
          s->misc_bits+= get_bits_diff(s);
          mpeg1_encode_motion(s, motion_x - s->last_mv[0][0][0], s->f_code);    /* RAL: f_code parameter added*/
          mpeg1_encode_motion(s, motion_y - s->last_mv[0][0][1], s->f_code);    /* RAL: f_code parameter added*/
          s->mv_bits+= get_bits_diff(s);
        }
      } else {
        put_bits(&s->pb, 3, 1); /* motion only */

        s->misc_bits+= get_bits_diff(s);
        mpeg1_encode_motion(s, motion_x - s->last_mv[0][0][0], s->f_code);    /* RAL: f_code parameter added*/
        mpeg1_encode_motion(s, motion_y - s->last_mv[0][0][1], s->f_code);    /* RAL: f_code parameter added*/
        s->qscale -= s->dquant;
        s->mv_bits+= get_bits_diff(s);
      }
      s->last_mv[0][1][0]= s->last_mv[0][0][0]= motion_x;
      s->last_mv[0][1][1]= s->last_mv[0][0][1]= motion_y;

      if(cbp)
        put_bits(&s->pb, mbPatTable[cbp][1], mbPatTable[cbp][0]);
      s->f_count++;
    } else{  
      static const int mb_type_len[4]={0,3,4,2}; /*bak,for,bi*/

      if (cbp){    /* With coded bloc pattern*/
        if (s->dquant) {
          if(s->mv_dir == MV_DIR_FORWARD)
            put_bits(&s->pb, 6, 3);
          else
            put_bits(&s->pb, mb_type_len[s->mv_dir]+3, 2);
          put_bits(&s->pb, 5, s->qscale);
        } else {
          put_bits(&s->pb, mb_type_len[s->mv_dir], 3);
        }
      }else{    /* No coded bloc pattern*/
        put_bits(&s->pb, mb_type_len[s->mv_dir], 2);
        s->qscale -= s->dquant;
      }
      s->misc_bits += get_bits_diff(s);
      if (s->mv_dir&MV_DIR_FORWARD){
        mpeg1_encode_motion(s, s->mv[0][0][0] - s->last_mv[0][0][0], s->f_code); 
        mpeg1_encode_motion(s, s->mv[0][0][1] - s->last_mv[0][0][1], s->f_code); 
        s->last_mv[0][0][0]=s->last_mv[0][1][0]= s->mv[0][0][0];
        s->last_mv[0][0][1]=s->last_mv[0][1][1]= s->mv[0][0][1];
        s->f_count++;
      }
      if (s->mv_dir&MV_DIR_BACKWARD){
        mpeg1_encode_motion(s, s->mv[1][0][0] - s->last_mv[1][0][0], s->b_code); 
        mpeg1_encode_motion(s, s->mv[1][0][1] - s->last_mv[1][0][1], s->b_code); 
        s->last_mv[1][0][0]=s->last_mv[1][1][0]= s->mv[1][0][0];
        s->last_mv[1][0][1]=s->last_mv[1][1][1]= s->mv[1][0][1];
        s->b_count++;
      }

      s->mv_bits += get_bits_diff(s);
      if(cbp)
        put_bits(&s->pb, mbPatTable[cbp][1], mbPatTable[cbp][0]);
    }
    for(i=0;i<6;i++) {
      if (cbp & (1 << (5 - i))) {
        mpeg1_encode_block(s, block[i], i);
      }
    }
    s->mb_skip_run = 0;
    if(s->mb_intra)
      s->i_tex_bits+= get_bits_diff(s);
    else
      s->p_tex_bits+= get_bits_diff(s);
  }
}

/* RAL: Parameter added: f_or_b_code*/
static void mpeg1_encode_motion(MpegEncContext *s, int val, int f_or_b_code)
{
  int code, bit_size, l, bits, range, sign;

  if (val == 0) {
    /* zero vector */
    code = 0;
    put_bits(&s->pb,
             mbMotionVectorTable[0][1], 
             mbMotionVectorTable[0][0]); 
  } else {
    bit_size = f_or_b_code - 1;
    range = 1 << bit_size;
    /* modulo encoding */
    l= INT_BIT - 5 - bit_size;
    val= (val<<l)>>l;

    if (val >= 0) {
      val--;
      code = (val >> bit_size) + 1;
      bits = val & (range - 1);
      sign = 0;
    } else {
      val = -val;
      val--;
      code = (val >> bit_size) + 1;
      bits = val & (range - 1);
      sign = 1;
    }

    assert(code > 0 && code <= 16);

    put_bits(&s->pb,
             mbMotionVectorTable[code][1], 
             mbMotionVectorTable[code][0]); 

    put_bits(&s->pb, 1, sign);
    if (bit_size > 0) {
      put_bits(&s->pb, bit_size, bits);
    }
  }
}

void ff_mpeg1_encode_init(MpegEncContext *s)
{
  static int done=0;

  if(!done){
    int f_code;
    int mv;
    int i;

    done=1;
    init_rl(&rl_mpeg1);

    for(i=0; i<64; i++)
      {
        mpeg1_max_level[0][i]= rl_mpeg1.max_level[0][i];
        mpeg1_index_run[0][i]= rl_mpeg1.index_run[0][i];
      }
        
    init_uni_ac_vlc(&rl_mpeg1, uni_mpeg1_ac_vlc_bits, uni_mpeg1_ac_vlc_len);

    /* build unified dc encoding tables */
    for(i=-255; i<256; i++)
      {
        int adiff, index;
        int bits, code;
        int diff=i;

        adiff = ABS(diff);
        if(diff<0) diff--;
        index = av_log2(2*adiff);

        bits= vlc_dc_lum_bits[index] + index;
        code= (vlc_dc_lum_code[index]<<index) + (diff & ((1 << index) - 1));
        mpeg1_lum_dc_uni[i+255]= bits + (code<<8);
		
        bits= vlc_dc_chroma_bits[index] + index;
        code= (vlc_dc_chroma_code[index]<<index) + (diff & ((1 << index) - 1));
        mpeg1_chr_dc_uni[i+255]= bits + (code<<8);
      }

    mv_penalty= av_mallocz( sizeof(uint8_t)*(MAX_FCODE+1)*(2*MAX_MV+1) );

    for(f_code=1; f_code<=MAX_FCODE; f_code++){
      for(mv=-MAX_MV; mv<=MAX_MV; mv++){
        int len;

        if(mv==0) len= mbMotionVectorTable[0][1];
        else{
          int val, bit_size, range, code;

          bit_size = f_code - 1;
          range = 1 << bit_size;

          val=mv;
          if (val < 0) 
            val = -val;
          val--;
          code = (val >> bit_size) + 1;
          if(code<17){
            len= mbMotionVectorTable[code][1] + 1 + bit_size;
          }else{
            len= mbMotionVectorTable[16][1] + 2 + bit_size;
          }
        }

        mv_penalty[f_code][mv+MAX_MV]= len;
      }
    }
        

    for(f_code=MAX_FCODE; f_code>0; f_code--){
      for(mv=-(8<<f_code); mv<(8<<f_code); mv++){
        fcode_tab[mv+MAX_MV]= f_code;
      }
    }
  }
  s->me.mv_penalty= mv_penalty;
  s->fcode_tab= fcode_tab;
  s->min_qcoeff=-255;
  s->max_qcoeff= 255;
  s->intra_ac_vlc_length=
    s->inter_ac_vlc_length=
    s->intra_ac_vlc_last_length=
    s->inter_ac_vlc_last_length= uni_mpeg1_ac_vlc_len;
}

static void mpeg1_encode_block(MpegEncContext *s, DCTELEM *block, int n)
{
  int alevel, level, last_non_zero, dc, diff, i, j, run, last_index, sign;
  int code, component;

  last_index = s->block_last_index[n];

  /* DC coef */
  if (s->mb_intra) {
    component = (n <= 3 ? 0 : n - 4 + 1);
    dc = block[0]; /* overflow is impossible */
    diff = dc - s->last_dc[component];
    if(((unsigned) (diff+255)) >= 511){
      int index;

      if(diff<0){
        index= av_log2_16bit(-2*diff);
        diff--;
      }else{
        index= av_log2_16bit(2*diff);
      }
      if (component == 0) {
        put_bits(&s->pb, 
                 vlc_dc_lum_bits[index] + index,
                 (vlc_dc_lum_code[index]<<index) + (diff & ((1 << index) - 1)));
      }else{
        put_bits(&s->pb, 
                 vlc_dc_chroma_bits[index] + index,
                 (vlc_dc_chroma_code[index]<<index) + (diff & ((1 << index) - 1)));
      }
    }else{
    if (component == 0) {
      put_bits(&s->pb, 
               mpeg1_lum_dc_uni[diff+255]&0xFF,
               mpeg1_lum_dc_uni[diff+255]>>8);
    } else {
      put_bits(&s->pb, 
               mpeg1_chr_dc_uni[diff+255]&0xFF,
               mpeg1_chr_dc_uni[diff+255]>>8);
    }
  }
    s->last_dc[component] = dc;
    i = 1;

  } else {
    /* encode the first coefficient : needs to be done here because
       it is handled slightly differently */
    level = block[0];
    if (abs(level) == 1) {
      code = ((uint32_t)level >> 31); /* the sign bit */
      put_bits(&s->pb, 2, code | 0x02);
      i = 1;
    } else {
      i = 0;
      last_non_zero = -1;
      goto next_coef;
    }
  }

  /* now quantify & encode AC coefs */
  last_non_zero = i - 1;

  for(;i<=last_index;i++) {
    j = s->intra_scantable.permutated[i];
    level = block[j];
  next_coef:

    if (level != 0) {
      run = i - last_non_zero - 1;
            
      alevel= level;
      sign= alevel>>31;
      alevel= (alevel^sign)-sign;
      sign&=1;

      if (alevel <= mpeg1_max_level[0][run]){
        code= mpeg1_index_run[0][run] + alevel - 1;
        /* store the vlc & sign at once */
        put_bits(&s->pb, mpeg1_vlc[code][1]+1, (mpeg1_vlc[code][0]<<1) + sign);
      } else {
        /* escape seems to be pretty rare <5% so i dont optimize it */
        put_bits(&s->pb, mpeg1_vlc[111/*rl->n*/][1], mpeg1_vlc[111/*rl->n*/][0]);
        /* escape: only clip in this case */
        put_bits(&s->pb, 6, run);
        if (alevel < 128) {
          put_bits(&s->pb, 8, level & 0xff);
        } else {
          if (level < 0) {
            put_bits(&s->pb, 16, 0x8001 + level + 255);
          } else {
            put_bits(&s->pb, 16, level & 0xffff);
          }
        }
      }
      last_non_zero = i;
    }
  }
  /* end of block */
  put_bits(&s->pb, 2, 0x2);
}

AVCodec mpeg1video_encoder = {
  "mpeg1video",
  CODEC_TYPE_VIDEO,
  CODEC_ID_MPEG1VIDEO,
  sizeof(MpegEncContext),
  encode_init,
  MPV_encode_picture,
  MPV_encode_end,
  0,
  CODEC_CAP_DELAY,
  0,
  0,
  frame_rate_tab+1,
  0
};
