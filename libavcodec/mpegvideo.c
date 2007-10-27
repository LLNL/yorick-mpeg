/*
 * The simplest mpeg encoder (well, it was the simplest!)
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
 *
 * 4MV & hq & b-frame encoding stuff by Michael Niedermayer <michaelni@gmx.at>
 */
 
/**
 * @file mpegvideo.c
 * The simplest mpeg encoder (well, it was the simplest!).
 */ 
 
#include "avcodec.h"
#include "dsputil.h"
#include "mpegvideo.h"
#include "simple_idct.h"
#include <limits.h>

static void encode_picture(MpegEncContext *s, int picture_number);

static void dct_unquantize_mpeg1_intra_c(MpegEncContext *s, DCTELEM *block,
                                         int n, int qscale);
static void dct_unquantize_mpeg1_inter_c(MpegEncContext *s, DCTELEM *block,
                                         int n, int qscale);
static int dct_quantize_c(MpegEncContext *s, DCTELEM *block,
                          int n, int qscale, int *overflow);


static const uint8_t ff_default_chroma_qscale_table[32]={
/*  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31*/
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
};


static uint8_t (*default_mv_penalty)[MAX_MV*2+1]=NULL;
static uint8_t default_fcode_tab[MAX_MV*2+1];

static void
convert_matrix(DSPContext *dsp, int (*qmat)[64], uint16_t (*qmat16)[2][64],
               const uint16_t *quant_matrix, int bias, int qmin, int qmax)
{
  int qscale;

  for(qscale=qmin; qscale<=qmax; qscale++){
    int i;
    for(i=0;i<64;i++) {
      /* We can safely suppose that 16 <= quant_matrix[i] <= 255
         So 16           <= qscale * quant_matrix[i]             <= 7905
         so (1<<19) / 16 >= (1<<19) / (qscale * quant_matrix[i]) >= (1<<19) / 7905
         so 32768        >= (1<<19) / (qscale * quant_matrix[i]) >= 67
      */
      qmat[qscale][i] = (int)((uint64_t_C(1) << QMAT_SHIFT) / (qscale * quant_matrix[i]));
      qmat16[qscale][0][i] = (1 << QMAT_SHIFT_MMX) / (qscale * quant_matrix[i]);

      if(qmat16[qscale][0][i]==0 || qmat16[qscale][0][i]==128*256) qmat16[qscale][0][i]=128*256-1;
      qmat16[qscale][1][i]= ROUNDED_DIV(bias<<(16-QUANT_BIAS_SHIFT), qmat16[qscale][0][i]);
    }
  }
}

void
ff_init_scantable(ScanTable *st, const uint8_t *src_scantable)
{
  int i;
  int end;

  st->scantable= src_scantable;

  for(i=0; i<64; i++){
    int j;
    j = src_scantable[i];
    st->permutated[i] = j;
    /*#ifdef ARCH_POWERPC
    st->inverse[j] = i;
    #endif*/
  }
    
  end=-1;
  for(i=0; i<64; i++){
    int j;
    j = st->permutated[i];
    if(j>end) end=j;
    st->raster_end[i]= end;
  }
}

void
ff_write_quant_matrix(PutBitContext *pb, uint16_t *matrix)
{
  int i;
  if(matrix){
    put_bits(pb, 1, 1);
    for(i=0;i<64;i++) {
      put_bits(pb, 8, matrix[ ff_zigzag_direct[i] ]);
    }
  }else
    put_bits(pb, 1, 0);
}

/* init common dct for both encoder and decoder */
int
DCT_common_init(MpegEncContext *s)
{
  ff_init_scantable(&s->inter_scantable  , ff_zigzag_direct);
  ff_init_scantable(&s->intra_scantable  , ff_zigzag_direct);
  ff_init_scantable(&s->intra_h_scantable, ff_alternate_horizontal_scan);
  ff_init_scantable(&s->intra_v_scantable, ff_alternate_vertical_scan);
  return 0;
}

static void
copy_picture(Picture *dst, Picture *src)
{
  *dst = *src;
  dst->type= FF_BUFFER_TYPE_COPY;
}

static void
copy_picture_attributes(MpegEncContext *s, AVFrame *dst, AVFrame *src)
{
  dst->pict_type              = src->pict_type;
  dst->quality                = src->quality;
  dst->coded_picture_number   = src->coded_picture_number;
  dst->display_picture_number = src->display_picture_number;
  dst->pts                    = src->pts;
}

/**
 * allocates a Picture
 * The pixels are allocated/set by calling get_buffer() if shared=0
 */
static int
alloc_picture(MpegEncContext *s, Picture *pic, int shared)
{
  const int big_mb_num= s->mb_stride*(s->mb_height+1) + 1;
    /*the +1 is needed so memset(,,stride*height) doesnt sig11*/
  const int mb_array_size= s->mb_stride*s->mb_height;
  const int b8_array_size= s->b8_stride*s->mb_height*2;
  int i;

  if(shared){
    assert(pic->data[0]);
    assert(pic->type == 0 || pic->type == FF_BUFFER_TYPE_SHARED);
    pic->type= FF_BUFFER_TYPE_SHARED;
  }else{
    int r;

    assert(!pic->data[0]);

    r= s->avctx->get_buffer(s->avctx, (AVFrame*)pic);

    if(r<0 || !pic->age || !pic->type || !pic->data[0]){
      av_log(s->avctx, AV_LOG_ERROR, "get_buffer() failed (%d %d %d %p)\n", r, pic->age, pic->type, pic->data[0]);
      return -1;
    }

    if(s->linesize && (s->linesize != pic->linesize[0] || s->uvlinesize != pic->linesize[1])){
      av_log(s->avctx, AV_LOG_ERROR, "get_buffer() failed (stride changed)\n");
      return -1;
    }

    if(pic->linesize[1] != pic->linesize[2]){
      av_log(s->avctx, AV_LOG_ERROR, "get_buffer() failed (uv stride missmatch)\n");
      return -1;
    }

    s->linesize  = pic->linesize[0];
    s->uvlinesize= pic->linesize[1];
  }
    
  CHECKED_ALLOCZ(pic->mb_var   , mb_array_size * sizeof(int16_t))
  CHECKED_ALLOCZ(pic->mc_mb_var, mb_array_size * sizeof(int16_t))
  CHECKED_ALLOCZ(pic->mb_mean  , mb_array_size * sizeof(int8_t))

  CHECKED_ALLOCZ(pic->mb_type_base , big_mb_num    * sizeof(uint32_t))
  pic->mb_type= pic->mb_type_base + s->mb_stride+1;
  for(i=0; i<2; i++){
    CHECKED_ALLOCZ(pic->motion_val_base[i], 2 * (b8_array_size+2) * sizeof(int16_t))
    pic->motion_val[i]= pic->motion_val_base[i]+2;
    CHECKED_ALLOCZ(pic->ref_index[i], b8_array_size * sizeof(uint8_t))
  }

  /*it might be nicer if the application would keep track of these but it would require a API change*/
  memmove(s->prev_pict_types+1, s->prev_pict_types, PREV_PICT_TYPES_BUFFER_SIZE-1);
  s->prev_pict_types[0]= s->pict_type;
  if(pic->age < PREV_PICT_TYPES_BUFFER_SIZE &&
     s->prev_pict_types[pic->age] == B_TYPE)
    pic->age= INT_MAX;
  /* skiped MBs in b frames are quite rare in mpeg1/2 and its a bit tricky to skip them anyway*/
    
  return 0;
 fail: /*for the CHECKED_ALLOCZ macro*/
  return -1;
}

/**
 * deallocates a picture
 */
static void
free_picture(MpegEncContext *s, Picture *pic)
{
  int i;

  if(pic->data[0] && pic->type!=FF_BUFFER_TYPE_SHARED){
    s->avctx->release_buffer(s->avctx, (AVFrame*)pic);
  }

  av_freep(&pic->mb_var);
  av_freep(&pic->mc_mb_var);
  av_freep(&pic->mb_mean);
  av_freep(&pic->mb_type_base);

  pic->mb_type= NULL;
  for(i=0; i<2; i++){
    av_freep(&pic->motion_val_base[i]);
    av_freep(&pic->ref_index[i]);
  }
    
  if(pic->type == FF_BUFFER_TYPE_SHARED){
    for(i=0; i<4; i++){
      pic->base[i]=
        pic->data[i]= NULL;
    }
    pic->type= 0;        
  }
}

static int init_duplicate_context(MpegEncContext *s, MpegEncContext *base)
{
  /* edge emu needs blocksize + filter length - 1 */
  CHECKED_ALLOCZ(s->allocated_edge_emu_buffer, (s->width+64)*2*17*2);
  /*(width + edge + align)*interlaced*MBsize*tolerance*/
  s->edge_emu_buffer= s->allocated_edge_emu_buffer + (s->width+64)*2*17;

  /*FIXME should be linesize instead of s->width*2 but that isnt known before get_buffer()*/
  CHECKED_ALLOCZ(s->me.scratchpad,  (s->width+64)*4*16*2*sizeof(uint8_t)) 
  s->rd_scratchpad=   s->me.scratchpad;
  s->b_scratchpad=    s->me.scratchpad;

  CHECKED_ALLOCZ(s->me.map      , ME_MAP_SIZE*sizeof(uint32_t))
  CHECKED_ALLOCZ(s->me.score_map, ME_MAP_SIZE*sizeof(uint32_t))

  CHECKED_ALLOCZ(s->blocks, 64*12*2 * sizeof(DCTELEM))
  s->block= s->blocks[0];

  return 0;
 fail:
  return -1; /*free() through MPV_common_end()*/
}

static void free_duplicate_context(MpegEncContext *s){
    if(s==NULL) return;

    av_freep(&s->allocated_edge_emu_buffer); s->edge_emu_buffer= NULL;
    av_freep(&s->me.scratchpad);
    s->rd_scratchpad=   
    s->b_scratchpad= NULL;
    
    av_freep(&s->me.map);
    av_freep(&s->me.score_map);
    av_freep(&s->blocks);
    s->block= NULL;
}

/**
 * sets the given MpegEncContext to common defaults (same for encoding and decoding).
 * the changed fields will not depend upon the prior state of the MpegEncContext.
 */
static void
MPV_common_defaults(MpegEncContext *s)
{
  s->y_dc_scale_table=
    s->c_dc_scale_table= ff_mpeg1_dc_scale_table;
  s->chroma_qscale_table= ff_default_chroma_qscale_table;

  s->coded_picture_number = 0;
  s->picture_number = 0;
  s->input_picture_number = 0;

  s->picture_in_gop_number = 0;

  s->f_code = 1;
  s->b_code = 1;
}

/**
 * sets the given MpegEncContext to defaults for encoding.
 * the changed fields will not depend upon the prior state of the MpegEncContext.
 */
static void MPV_encode_defaults(MpegEncContext *s){
  static int done=0;
    
  MPV_common_defaults(s);
    
  if(!done){
    int i;
    done=1;

    default_mv_penalty= av_mallocz( sizeof(uint8_t)*(MAX_FCODE+1)*(2*MAX_MV+1) );
    memset(default_mv_penalty, 0, sizeof(uint8_t)*(MAX_FCODE+1)*(2*MAX_MV+1));
    memset(default_fcode_tab , 0, sizeof(uint8_t)*(2*MAX_MV+1));

    for(i=-16; i<16; i++){
      default_fcode_tab[i + MAX_MV]= 1;
    }
  }
  s->me.mv_penalty= default_mv_penalty;
  s->fcode_tab= default_fcode_tab;
}

/** 
 * init common structure for both encoder and decoder.
 * this assumes that some variables like width/height are already set
 */
int MPV_common_init(MpegEncContext *s)
{
  int y_size, c_size, yc_size, mb_array_size, mv_table_size, x, y;

  dsputil_init(&s->dsp, s->avctx);
  DCT_common_init(s);

  s->flags= s->avctx->flags;
  s->flags2= s->avctx->flags2;

  s->mb_width  = (s->width  + 15) / 16;
  s->mb_height = (s->height + 15) / 16;
  s->mb_stride = s->mb_width + 1;
  s->b8_stride = s->mb_width*2 + 1;

  mb_array_size= s->mb_height * s->mb_stride;
  mv_table_size= (s->mb_height+2) * s->mb_stride + 1;

  /* set chroma shifts */
  avcodec_get_chroma_sub_sample(s->avctx->pix_fmt,&(s->chroma_x_shift),
                                &(s->chroma_y_shift) );

  /* set default edge pos, will be overriden in decode_header if needed */
  s->h_edge_pos= s->mb_width*16;
  s->v_edge_pos= s->mb_height*16;

  s->mb_num = s->mb_width * s->mb_height;
    
  s->block_wrap[0]=
    s->block_wrap[1]=
    s->block_wrap[2]=
    s->block_wrap[3]= s->b8_stride;
  s->block_wrap[4]=
    s->block_wrap[5]= s->mb_stride;
 
  y_size = s->b8_stride * (2 * s->mb_height + 1);
  c_size = s->mb_stride * (s->mb_height + 1);
  yc_size = y_size + 2 * c_size;

  s->avctx->coded_frame= (AVFrame*)&s->current_picture;

  CHECKED_ALLOCZ(s->mb_index2xy, (s->mb_num+1)*sizeof(int)) /*error ressilience code looks cleaner with this*/
  for(y=0; y<s->mb_height; y++){
    for(x=0; x<s->mb_width; x++){
      s->mb_index2xy[ x + y*s->mb_width ] = x + y*s->mb_stride;
    }
  }
  s->mb_index2xy[ s->mb_height*s->mb_width ] = (s->mb_height-1)*s->mb_stride + s->mb_width; /*FIXME really needed?*/

  /* Allocate MV tables */
  CHECKED_ALLOCZ(s->p_mv_table_base            , mv_table_size * 2 * sizeof(int16_t))
  CHECKED_ALLOCZ(s->b_forw_mv_table_base       , mv_table_size * 2 * sizeof(int16_t))
  CHECKED_ALLOCZ(s->b_back_mv_table_base       , mv_table_size * 2 * sizeof(int16_t))
  CHECKED_ALLOCZ(s->b_bidir_forw_mv_table_base , mv_table_size * 2 * sizeof(int16_t))
  CHECKED_ALLOCZ(s->b_bidir_back_mv_table_base , mv_table_size * 2 * sizeof(int16_t))
  CHECKED_ALLOCZ(s->b_direct_mv_table_base     , mv_table_size * 2 * sizeof(int16_t))
  s->p_mv_table           = s->p_mv_table_base            + s->mb_stride + 1;
  s->b_forw_mv_table      = s->b_forw_mv_table_base       + s->mb_stride + 1;
  s->b_back_mv_table      = s->b_back_mv_table_base       + s->mb_stride + 1;
  s->b_bidir_forw_mv_table= s->b_bidir_forw_mv_table_base + s->mb_stride + 1;
  s->b_bidir_back_mv_table= s->b_bidir_back_mv_table_base + s->mb_stride + 1;
  s->b_direct_mv_table    = s->b_direct_mv_table_base     + s->mb_stride + 1;

  /* Allocate MB type table */
  CHECKED_ALLOCZ(s->mb_type  , mb_array_size * sizeof(uint16_t)) /*needed for encoding*/
        
  CHECKED_ALLOCZ(s->lambda_table, mb_array_size * sizeof(int))
        
  CHECKED_ALLOCZ(s->q_intra_matrix, 64*32 * sizeof(int))
  CHECKED_ALLOCZ(s->q_inter_matrix, 64*32 * sizeof(int))
  CHECKED_ALLOCZ(s->q_intra_matrix16, 64*32*2 * sizeof(uint16_t))
  CHECKED_ALLOCZ(s->q_inter_matrix16, 64*32*2 * sizeof(uint16_t))
  CHECKED_ALLOCZ(s->input_picture, MAX_PICTURE_COUNT * sizeof(Picture*))
  CHECKED_ALLOCZ(s->reordered_input_picture, MAX_PICTURE_COUNT * sizeof(Picture*))

  CHECKED_ALLOCZ(s->picture, MAX_PICTURE_COUNT * sizeof(Picture))

  CHECKED_ALLOCZ(s->prev_pict_types, PREV_PICT_TYPES_BUFFER_SIZE);

  s->parse_context.state= -1;
  s->context_initialized = 1;
  s->thread_context[0]= s;

  if(init_duplicate_context(s->thread_context[0], s) < 0)
    goto fail;
  s->thread_context[0]->start_mb_y= 0;
  s->thread_context[0]->end_mb_y  = s->mb_height;

  return 0;
 fail:
  MPV_common_end(s);
  return -1;
}

/* init common structure for both encoder and decoder */
void MPV_common_end(MpegEncContext *s)
{
  int i, j, k;

  free_duplicate_context(s->thread_context[0]);

  av_freep(&s->parse_context.buffer);
  s->parse_context.buffer_size=0;

  av_freep(&s->mb_type);
  av_freep(&s->p_mv_table_base);
  av_freep(&s->b_forw_mv_table_base);
  av_freep(&s->b_back_mv_table_base);
  av_freep(&s->b_bidir_forw_mv_table_base);
  av_freep(&s->b_bidir_back_mv_table_base);
  av_freep(&s->b_direct_mv_table_base);
  s->p_mv_table= NULL;
  s->b_forw_mv_table= NULL;
  s->b_back_mv_table= NULL;
  s->b_bidir_forw_mv_table= NULL;
  s->b_bidir_back_mv_table= NULL;
  s->b_direct_mv_table= NULL;
  for(i=0; i<2; i++){
    for(j=0; j<2; j++){
      for(k=0; k<2; k++){
        av_freep(&s->b_field_mv_table_base[i][j][k]);
        s->b_field_mv_table[i][j][k]=NULL;
      }
      av_freep(&s->b_field_select_table[i][j]);
      av_freep(&s->p_field_mv_table_base[i][j]);
      s->p_field_mv_table[i][j]=NULL;
    }
    av_freep(&s->p_field_select_table[i]);
  }

  av_freep(&s->prev_pict_types);

  av_freep(&s->mb_index2xy);
  av_freep(&s->lambda_table);
  av_freep(&s->q_intra_matrix);
  av_freep(&s->q_inter_matrix);
  av_freep(&s->q_intra_matrix16);
  av_freep(&s->q_inter_matrix16);
  av_freep(&s->input_picture);
  av_freep(&s->reordered_input_picture);

  if(s->picture){
    for(i=0; i<MAX_PICTURE_COUNT; i++){
      free_picture(s, &s->picture[i]);
    }
  }
  av_freep(&s->picture);
  s->context_initialized = 0;
  s->last_picture_ptr=
    s->next_picture_ptr=
    s->current_picture_ptr= NULL;

  for(i=0; i<3; i++)
    av_freep(&s->visualization_buffer[i]);
}

/* init video encoder */
int MPV_encode_init(AVCodecContext *avctx)
{
  MpegEncContext *s = avctx->priv_data;
  int i, dummy;
  int chroma_h_shift, chroma_v_shift;
    
  MPV_encode_defaults(s);

  avctx->pix_fmt = PIX_FMT_YUV420P; /* FIXME*/

  s->bit_rate = avctx->bit_rate;
  s->width = avctx->width;
  s->height = avctx->height;
  if(avctx->gop_size > 600){
    av_log(avctx, AV_LOG_ERROR, "Warning keyframe interval too large! reducing it ...\n");
    avctx->gop_size=600;
  }
  s->gop_size = avctx->gop_size;
  s->avctx = avctx;
  s->flags= avctx->flags;
  s->flags2= avctx->flags2;
  s->max_b_frames= avctx->max_b_frames;
  s->codec_id= avctx->codec->id;

  s->intra_dc_precision= avctx->intra_dc_precision;

  if (s->gop_size <= 1) {
    s->intra_only = 1;
    s->gop_size = 12;
  } else {
    s->intra_only = 0;
  }

  if(avctx->rc_max_rate && !avctx->rc_buffer_size){
    av_log(avctx, AV_LOG_ERROR, "a vbv buffer size is needed, for encoding with a maximum bitrate\n");
    return -1;
  }    

  if(avctx->rc_min_rate && avctx->rc_max_rate != avctx->rc_min_rate){
    av_log(avctx, AV_LOG_INFO, "Warning min_rate > 0 but min_rate != max_rate isnt recommanded!\n");
  }
    
  if(avctx->rc_min_rate && avctx->rc_min_rate > avctx->bit_rate){
    av_log(avctx, AV_LOG_INFO, "bitrate below min bitrate\n");
    return -1;
  }
    
  if(avctx->rc_max_rate && avctx->rc_max_rate < avctx->bit_rate){
    av_log(avctx, AV_LOG_INFO, "bitrate above max bitrate\n");
    return -1;
  }
        
  if(s->avctx->rc_max_rate && s->avctx->rc_min_rate == s->avctx->rc_max_rate 
     && int64_t_C(90000) * (avctx->rc_buffer_size-1) >
     s->avctx->rc_max_rate*int64_t_C(0xFFFF)){
    av_log(avctx, AV_LOG_INFO, "Warning vbv_delay will be set to 0xFFFF (=VBR) as the specified vbv buffer is too large for the given bitrate!\n");
  }

  i= ff_gcd(avctx->frame_rate, avctx->frame_rate_base);
  if(i > 1){
    av_log(avctx, AV_LOG_INFO, "removing common factors from framerate\n");
    avctx->frame_rate /= i;
    avctx->frame_rate_base /= i;
    /*        return -1;*/
  }
    
  {
    s->intra_quant_bias= 3<<(QUANT_BIAS_SHIFT-3); /*(a + x*3/8)/x*/
    s->inter_quant_bias= 0;
  }
    
  if(avctx->intra_quant_bias != FF_DEFAULT_QUANT_BIAS)
    s->intra_quant_bias= avctx->intra_quant_bias;
  if(avctx->inter_quant_bias != FF_DEFAULT_QUANT_BIAS)
    s->inter_quant_bias= avctx->inter_quant_bias;
        
  avcodec_get_chroma_sub_sample(avctx->pix_fmt, &chroma_h_shift, &chroma_v_shift);

  av_reduce(&s->time_increment_resolution, &dummy, s->avctx->frame_rate, s->avctx->frame_rate_base, (1<<16)-1);

  s->out_format = FMT_MPEG1;

  avctx->delay= (s->max_b_frames + 1);

  /* init */
  if (MPV_common_init(s) < 0)
    return -1;

  ff_mpeg1_encode_init(s);

  /* init q matrix */
  for(i=0;i<64;i++) {
    s->intra_matrix[i] = ff_mpeg1_default_intra_matrix[i];
    s->inter_matrix[i] = ff_mpeg1_default_non_intra_matrix[i];
    if(s->avctx->intra_matrix)
      s->intra_matrix[i] = s->avctx->intra_matrix[i];
    if(s->avctx->inter_matrix)
      s->inter_matrix[i] = s->avctx->inter_matrix[i];
  }

  /* precompute matrix */
  /* for mjpeg, we do include qscale in the matrix */
  {
    convert_matrix(&s->dsp, s->q_intra_matrix, s->q_intra_matrix16, 
                   s->intra_matrix, s->intra_quant_bias, 1, 31);
    convert_matrix(&s->dsp, s->q_inter_matrix, s->q_inter_matrix16, 
                   s->inter_matrix, s->inter_quant_bias, 1, 31);
  }

  if(ff_rate_control_init(s) < 0)
    return -1;
    
  return 0;
}

int MPV_encode_end(AVCodecContext *avctx)
{
  MpegEncContext *s = avctx->priv_data;

  ff_rate_control_uninit(s);

  MPV_common_end(s);

  return 0;
}

void init_rl(RLTable *rl)
{
  int8_t max_level[MAX_RUN+1], max_run[MAX_LEVEL+1];
  uint8_t index_run[MAX_RUN+1];
  int last, run, level, start, end, i;

  /* compute max_level[], max_run[] and index_run[] */
  for(last=0;last<2;last++) {
    if (last == 0) {
      start = 0;
      end = rl->last;
    } else {
      start = rl->last;
      end = rl->n;
    }

    memset(max_level, 0, MAX_RUN + 1);
    memset(max_run, 0, MAX_LEVEL + 1);
    memset(index_run, rl->n, MAX_RUN + 1);
    for(i=start;i<end;i++) {
      run = rl->table_run[i];
      level = rl->table_level[i];
      if (index_run[run] == rl->n)
        index_run[run] = i;
      if (level > max_level[run])
        max_level[run] = level;
      if (run > max_run[level])
        max_run[level] = run;
    }
    rl->max_level[last] = av_malloc(MAX_RUN + 1);
    memcpy(rl->max_level[last], max_level, MAX_RUN + 1);
    rl->max_run[last] = av_malloc(MAX_LEVEL + 1);
    memcpy(rl->max_run[last], max_run, MAX_LEVEL + 1);
    rl->index_run[last] = av_malloc(MAX_RUN + 1);
    memcpy(rl->index_run[last], index_run, MAX_RUN + 1);
  }
}

int ff_find_unused_picture(MpegEncContext *s, int shared){
  int i;
    
  if(shared){
    for(i=0; i<MAX_PICTURE_COUNT; i++){
      if(s->picture[i].data[0]==NULL && s->picture[i].type==0) return i;
    }
  }else{
    for(i=0; i<MAX_PICTURE_COUNT; i++){
      if(s->picture[i].data[0]==NULL && s->picture[i].type!=0) return i; /*FIXME*/
    }
    for(i=0; i<MAX_PICTURE_COUNT; i++){
      if(s->picture[i].data[0]==NULL) return i;
    }
  }

  assert(0);
  return -1;
}

/**
 * generic function for encode/decode called after coding/decoding the header and before a frame is coded/decoded
 */
int MPV_frame_start(MpegEncContext *s, AVCodecContext *avctx)
{
  assert(s->last_picture_ptr==NULL);

  /* mark&release old frames */
  if (s->pict_type != B_TYPE && s->last_picture_ptr && s->last_picture_ptr != s->next_picture_ptr && s->last_picture_ptr->data[0]) {
    avctx->release_buffer(avctx, (AVFrame*)s->last_picture_ptr);
  }
 alloc:

  s->current_picture_ptr->pict_type= s->pict_type;
  s->current_picture_ptr->key_frame= s->pict_type == I_TYPE;

  copy_picture(&s->current_picture, s->current_picture_ptr);
  
  {
    if (s->pict_type != B_TYPE) {
      s->last_picture_ptr= s->next_picture_ptr;
      if(!s->dropable)
        s->next_picture_ptr= s->current_picture_ptr;
    }
    /*    av_log(s->avctx, AV_LOG_DEBUG, "L%p N%p C%p L%p N%p C%p type:%d drop:%d\n", s->last_picture_ptr, s->next_picture_ptr,s->current_picture_ptr,
          s->last_picture_ptr    ? s->last_picture_ptr->data[0] : NULL, 
          s->next_picture_ptr    ? s->next_picture_ptr->data[0] : NULL, 
          s->current_picture_ptr ? s->current_picture_ptr->data[0] : NULL,
          s->pict_type, s->dropable);*/
    
    if(s->last_picture_ptr) copy_picture(&s->last_picture, s->last_picture_ptr);
    if(s->next_picture_ptr) copy_picture(&s->next_picture, s->next_picture_ptr);
    
    if(s->pict_type != I_TYPE && (s->last_picture_ptr==NULL || s->last_picture_ptr->data[0]==NULL)){
      av_log(avctx, AV_LOG_ERROR, "warning: first frame is no keyframe\n");
      assert(s->pict_type != B_TYPE); /*these should have been dropped if we dont have a reference*/
      goto alloc;
    }

    assert(s->pict_type == I_TYPE || (s->last_picture_ptr && s->last_picture_ptr->data[0]));
  }

  return 0;
}

/* generic function for encode/decode called after a frame has been coded/decoded */
void MPV_frame_end(MpegEncContext *s)
{
  int i;
  /* draw edge for correct motion prediction if outside */

  s->last_pict_type    = s->pict_type;
  if(s->pict_type!=B_TYPE){
    s->last_non_b_pict_type= s->pict_type;
  }

  /* release non refernce frames */
  for(i=0; i<MAX_PICTURE_COUNT; i++){
    if(s->picture[i].data[0] && !s->picture[i].reference /*&& s->picture[i].type!=FF_BUFFER_TYPE_SHARED*/){
      s->avctx->release_buffer(s->avctx, (AVFrame*)&s->picture[i]);
    }
  }
}

static int get_sae(uint8_t *src, int ref, int stride)
{
  int x,y;
  int acc=0;
    
  for(y=0; y<16; y++){
    for(x=0; x<16; x++){
      acc+= ABS(src[x+y*stride] - ref);
    }
  }
    
  return acc;
}

static int get_intra_count(MpegEncContext *s, uint8_t *src, uint8_t *ref,
                           int stride)
{
  int x, y, w, h;
  int acc=0;
    
  w= s->width &~15;
  h= s->height&~15;
    
  for(y=0; y<h; y+=16){
    for(x=0; x<w; x+=16){
      int offset= x + y*stride;
      int sad = pix_abs16_c(NULL, src + offset, ref + offset, stride, 16);
      int mean= (s->dsp.pix_sum(src + offset, stride) + 128)>>8;
      int sae = get_sae(src + offset, mean, stride);
            
      acc+= sae + 500 < sad;
    }
  }
  return acc;
}


static int load_input_picture(MpegEncContext *s, AVFrame *pic_arg)
{
  AVFrame *pic=NULL;
  int i;
  const int encoding_delay= s->max_b_frames;
  int direct=1;
    
  if(pic_arg){
    if(encoding_delay) direct=0;
    if(pic_arg->linesize[0] != s->linesize) direct=0;
    if(pic_arg->linesize[1] != s->uvlinesize) direct=0;
    if(pic_arg->linesize[2] != s->uvlinesize) direct=0;

    if(direct){
      i= ff_find_unused_picture(s, 1);

      pic= (AVFrame*)&s->picture[i];
      pic->reference= 3;

      for(i=0; i<4; i++){
        pic->data[i]= pic_arg->data[i];
        pic->linesize[i]= pic_arg->linesize[i];
      }
      alloc_picture(s, (Picture*)pic, 1);
    }else{
      int offset= 16;
      i= ff_find_unused_picture(s, 0);

      pic= (AVFrame*)&s->picture[i];
      pic->reference= 3;

      alloc_picture(s, (Picture*)pic, 0);

      if(   pic->data[0] + offset == pic_arg->data[0] 
            && pic->data[1] + offset == pic_arg->data[1]
            && pic->data[2] + offset == pic_arg->data[2]){
        /* empty*/
      }else{
        int h_chroma_shift, v_chroma_shift;
        avcodec_get_chroma_sub_sample(s->avctx->pix_fmt, &h_chroma_shift, &v_chroma_shift);

        for(i=0; i<3; i++){
          int src_stride= pic_arg->linesize[i];
          int dst_stride= i ? s->uvlinesize : s->linesize;
          int h_shift= i ? h_chroma_shift : 0;
          int v_shift= i ? v_chroma_shift : 0;
          int w= s->width >>h_shift;
          int h= s->height>>v_shift;
          uint8_t *src= pic_arg->data[i];
          uint8_t *dst= pic->data[i] + offset;
            
          if(src_stride==dst_stride)
            memcpy(dst, src, src_stride*h);
          else{
            while(h--){
              memcpy(dst, src, w);
              dst += dst_stride;
              src += src_stride;
            }
          }
        }
      }
    }
    copy_picture_attributes(s, pic, pic_arg);

    pic->display_picture_number= s->input_picture_number++;
    if(pic->pts != AV_NOPTS_VALUE){ 
      s->user_specified_pts= pic->pts;
    }else{
      if(s->user_specified_pts){
        pic->pts= s->user_specified_pts + AV_TIME_BASE*(int64_t)s->avctx->frame_rate_base / s->avctx->frame_rate;
        av_log(s->avctx, AV_LOG_INFO, "Warning: AVFrame.pts=? trying to guess (%Ld)\n", pic->pts);
      }else{
        pic->pts= av_rescale(pic->display_picture_number*(int64_t)s->avctx->frame_rate_base, AV_TIME_BASE, s->avctx->frame_rate);
      }
    }
  }

  /* shift buffer entries */
  for(i=1; i<MAX_PICTURE_COUNT /*s->encoding_delay+1*/; i++)
    s->input_picture[i-1]= s->input_picture[i];

  s->input_picture[encoding_delay]= (Picture*)pic;

  return 0;
}

static void select_input_picture(MpegEncContext *s)
{
  int i;

  for(i=1; i<MAX_PICTURE_COUNT; i++)
    s->reordered_input_picture[i-1]= s->reordered_input_picture[i];
  s->reordered_input_picture[MAX_PICTURE_COUNT-1]= NULL;

  /* set next picture types & ordering */
  if(s->reordered_input_picture[0]==NULL && s->input_picture[0]){
    if(/*s->picture_in_gop_number >= s->gop_size ||*/ s->next_picture_ptr==NULL || s->intra_only){
      s->reordered_input_picture[0]= s->input_picture[0];
      s->reordered_input_picture[0]->pict_type= I_TYPE;
      s->reordered_input_picture[0]->coded_picture_number= s->coded_picture_number++;
    }else{
      int b_frames;

      if(s->input_picture[0]->pict_type){
        /* user selected pict_type */
        for(b_frames=0; b_frames<s->max_b_frames+1; b_frames++){
          if(s->input_picture[b_frames]->pict_type!=B_TYPE) break;
        }
            
        if(b_frames > s->max_b_frames){
          av_log(s->avctx, AV_LOG_ERROR, "warning, too many bframes in a row\n");
          b_frames = s->max_b_frames;
        }
      }else if(s->avctx->b_frame_strategy==0){
        b_frames= s->max_b_frames;
        while(b_frames && !s->input_picture[b_frames]) b_frames--;
      }else if(s->avctx->b_frame_strategy==1){
        for(i=1; i<s->max_b_frames+1; i++){
          if(s->input_picture[i] && s->input_picture[i]->b_frame_score==0){
            s->input_picture[i]->b_frame_score= 
              get_intra_count(s, s->input_picture[i  ]->data[0], 
                              s->input_picture[i-1]->data[0], s->linesize) + 1;
          }
        }
        for(i=0; i<s->max_b_frames; i++){
          if(s->input_picture[i]==NULL || s->input_picture[i]->b_frame_score - 1 > s->mb_num/40) break;
        }
                                
        b_frames= FFMAX(0, i-1);
                
        /* reset scores */
        for(i=0; i<b_frames+1; i++){
          s->input_picture[i]->b_frame_score=0;
        }
      }else{
        av_log(s->avctx, AV_LOG_ERROR, "illegal b frame strategy\n");
        b_frames=0;
      }

      if(s->picture_in_gop_number + b_frames >= s->gop_size){
        s->input_picture[b_frames]->pict_type= I_TYPE;
      }

      s->reordered_input_picture[0]= s->input_picture[b_frames];
      if(s->reordered_input_picture[0]->pict_type != I_TYPE)
        s->reordered_input_picture[0]->pict_type= P_TYPE;
      s->reordered_input_picture[0]->coded_picture_number= s->coded_picture_number++;
      for(i=0; i<b_frames; i++){
        s->reordered_input_picture[i+1]= s->input_picture[i];
        s->reordered_input_picture[i+1]->pict_type= B_TYPE;
        s->reordered_input_picture[i+1]->coded_picture_number= s->coded_picture_number++;
      }
    }
  }
    
  if(s->reordered_input_picture[0]){
    s->reordered_input_picture[0]->reference= s->reordered_input_picture[0]->pict_type!=B_TYPE ? 3 : 0;

    copy_picture(&s->new_picture, s->reordered_input_picture[0]);

    if(s->reordered_input_picture[0]->type == FF_BUFFER_TYPE_SHARED){
      /* input is a shared pix, so we cant modifiy it -> alloc a new one & ensure that the shared one is reuseable*/
        
      int i= ff_find_unused_picture(s, 0);
      Picture *pic= &s->picture[i];

      /* mark us unused / free shared pic */
      for(i=0; i<4; i++)
        s->reordered_input_picture[0]->data[i]= NULL;
      s->reordered_input_picture[0]->type= 0;
            
      pic->reference              = s->reordered_input_picture[0]->reference;
            
      alloc_picture(s, pic, 0);

      copy_picture_attributes(s, (AVFrame*)pic, (AVFrame*)s->reordered_input_picture[0]);

      s->current_picture_ptr= pic;
    }else{
      /* input is not a shared pix -> reuse buffer for current_pix*/

      assert(   s->reordered_input_picture[0]->type==FF_BUFFER_TYPE_USER 
                || s->reordered_input_picture[0]->type==FF_BUFFER_TYPE_INTERNAL);
            
      s->current_picture_ptr= s->reordered_input_picture[0];
      for(i=0; i<4; i++){
        s->new_picture.data[i]+=16;
      }
    }
    copy_picture(&s->current_picture, s->current_picture_ptr);
    
    s->picture_number= s->new_picture.display_picture_number;

  }else{
    memset(&s->new_picture, 0, sizeof(Picture));
  }
}

#ifdef ALT_BITSTREAM_WRITER
#  define init_put_bits(s, buffer, buffer_size)\
  (s)->buf = (buffer);\
  (s)->buf_end = (s)->buf + (buffer_size);\
  (s)->index=0;\
  ((uint32_t*)((s)->buf))[0]=0
#else
#  define init_put_bits(s, buffer, buffer_size)\
  (s)->buf = (buffer);\
  (s)->buf_end = (s)->buf + (buffer_size);\
  (s)->buf_ptr = (s)->buf;\
  (s)->bit_left=32;\
  (s)->bit_buf=0
#endif

/* pad the end of the output stream with zeros */
#ifdef ALT_BITSTREAM_WRITER
#  define flush_put_bits(s) align_put_bits(s)
#else
#  define flush_put_bits(s)\
    (s)->bit_buf<<= (s)->bit_left;\
    while ((s)->bit_left < 32) {\
        /* XXX: should test end of buffer */\
        *(s)->buf_ptr++=(s)->bit_buf >> 24;\
        (s)->bit_buf<<=8;\
        (s)->bit_left+=8;\
    }\
    (s)->bit_left=32;\
    (s)->bit_buf=0
#endif

int MPV_encode_picture(AVCodecContext *avctx,
                       unsigned char *buf, int buf_size, void *data)
{
  MpegEncContext *s = avctx->priv_data;
  AVFrame *pic_arg = data;
  int stuffing_count;

  int start_y= s->thread_context[0]->start_mb_y;
  int   end_y= s->thread_context[0]->  end_mb_y;
  int h= s->mb_height;
  uint8_t *start= buf + buf_size*start_y/h;
  uint8_t *end  = buf + buf_size*  end_y/h;

  init_put_bits(&s->thread_context[0]->pb, start, end - start);

  s->picture_in_gop_number++;

  load_input_picture(s, pic_arg);
  select_input_picture(s);

  /* output? */
  if(s->new_picture.data[0]){
    s->pict_type= s->new_picture.pict_type;

    MPV_frame_start(s, avctx);

    encode_picture(s, s->picture_number);

    avctx->real_pict_num  = s->picture_number;
    avctx->header_bits = s->header_bits;
    avctx->mv_bits     = s->mv_bits;
    avctx->misc_bits   = s->misc_bits;
    avctx->i_tex_bits  = s->i_tex_bits;
    avctx->p_tex_bits  = s->p_tex_bits;
    avctx->i_count     = s->i_count;
    avctx->p_count     = s->mb_num - s->i_count - s->skip_count; /*FIXME f/b_count in avctx*/
    avctx->skip_count  = s->skip_count;

    MPV_frame_end(s);

    flush_put_bits(&s->pb);
    s->frame_bits  = put_bits_count(&s->pb);

    stuffing_count= ff_vbv_update(s, s->frame_bits);
    if(stuffing_count){
      while(stuffing_count--){
        put_bits(&s->pb, 8, 0);
      }
      flush_put_bits(&s->pb);
      s->frame_bits  = put_bits_count(&s->pb);
    }

    /* update mpeg1/2 vbv_delay for CBR */    
    if(s->avctx->rc_max_rate && s->avctx->rc_min_rate == s->avctx->rc_max_rate
       && int64_t_C(90000)*(avctx->rc_buffer_size-1) <=
       s->avctx->rc_max_rate*int64_t_C(0xFFFF)){
      int vbv_delay;

      vbv_delay= lrintf(90000 * s->rc_context.buffer_index / s->avctx->rc_max_rate);
      assert(vbv_delay < 0xFFFF);

      s->vbv_delay_ptr[0] &= 0xF8;
      s->vbv_delay_ptr[0] |= vbv_delay>>13;
      s->vbv_delay_ptr[1]  = vbv_delay>>5;
      s->vbv_delay_ptr[2] &= 0x07;
      s->vbv_delay_ptr[2] |= vbv_delay<<3;
    }
    s->total_bits += s->frame_bits;
    avctx->frame_bits  = s->frame_bits;
  }else{
    assert((pbBufPtr(&s->pb) == s->pb.buf));
    s->frame_bits=0;
  }
  assert((s->frame_bits&7)==0);

  return s->frame_bits/8;
}

/**
 * Copies a rectangular area of samples to a temporary buffer and replicates the boarder samples.
 * @param buf destination buffer
 * @param src source buffer
 * @param linesize number of bytes between 2 vertically adjacent samples in both the source and destination buffers
 * @param block_w width of block
 * @param block_h height of block
 * @param src_x x coordinate of the top left sample of the block in the source buffer
 * @param src_y y coordinate of the top left sample of the block in the source buffer
 * @param w width of the source buffer
 * @param h height of the source buffer
 */
void
ff_emulated_edge_mc(uint8_t *buf, uint8_t *src, int linesize, int block_w,
                    int block_h, int src_x, int src_y, int w, int h)
{
  int x, y;
  int start_y, start_x, end_y, end_x;

  if(src_y>= h){
    src+= (h-1-src_y)*linesize;
    src_y=h-1;
  }else if(src_y<=-block_h){
    src+= (1-block_h-src_y)*linesize;
    src_y=1-block_h;
  }
  if(src_x>= w){
    src+= (w-1-src_x);
    src_x=w-1;
  }else if(src_x<=-block_w){
    src+= (1-block_w-src_x);
    src_x=1-block_w;
  }

  start_y= FFMAX(0, -src_y);
  start_x= FFMAX(0, -src_x);
  end_y= FFMIN(block_h, h-src_y);
  end_x= FFMIN(block_w, w-src_x);

  /* copy existing part*/
  for(y=start_y; y<end_y; y++){
    for(x=start_x; x<end_x; x++){
      buf[x + y*linesize]= src[x + y*linesize];
    }
  }

  /*top*/
  for(y=0; y<start_y; y++){
    for(x=start_x; x<end_x; x++){
      buf[x + y*linesize]= buf[x + start_y*linesize];
    }
  }

  /*bottom*/
  for(y=end_y; y<block_h; y++){
    for(x=start_x; x<end_x; x++){
      buf[x + y*linesize]= buf[x + (end_y-1)*linesize];
    }
  }
                                    
  for(y=0; y<block_h; y++){
    /*left*/
    for(x=0; x<start_x; x++){
      buf[x + y*linesize]= buf[start_x + y*linesize];
    }
       
    /*right*/
    for(x=end_x; x<block_w; x++){
      buf[x + y*linesize]= buf[end_x - 1 + y*linesize];
    }
  }
}

/* apply one mpeg motion vector to the three components */
static void
mpeg_motion(MpegEncContext *s, uint8_t *dest_y, uint8_t *dest_cb,
            uint8_t *dest_cr, uint8_t **ref_picture,
            op_pixels_func (*pix_op)[4], int motion_x, int motion_y)
{
  uint8_t *ptr_y, *ptr_cb, *ptr_cr;
  int dxy, uvdxy, mx, my, src_x, src_y, uvsrc_x, uvsrc_y,
    v_edge_pos, uvlinesize, linesize;

  v_edge_pos = s->v_edge_pos;
  linesize   = s->current_picture.linesize[0];
  uvlinesize = s->current_picture.linesize[1];

  dxy = ((motion_y & 1) << 1) | (motion_x & 1);
  src_x = s->mb_x* 16               + (motion_x >> 1);
  src_y =(s->mb_y<<4) + (motion_y >> 1);

  if(s->chroma_y_shift){
    mx = motion_x / 2;
    my = motion_y / 2;
    uvdxy = ((my & 1) << 1) | (mx & 1);
    uvsrc_x = s->mb_x* 8               + (mx >> 1);
    uvsrc_y = (s->mb_y<<3) + (my >> 1);
  } else {
    if(s->chroma_x_shift){
      /*Chroma422*/
      mx = motion_x / 2;
      uvdxy = ((motion_y & 1) << 1) | (mx & 1);
      uvsrc_x = s->mb_x* 8           + (mx >> 1);
      uvsrc_y = src_y;
    } else {
      /*Chroma444*/
      uvdxy = dxy;
      uvsrc_x = src_x;
      uvsrc_y = src_y;
    }
  }

  ptr_y  = ref_picture[0] + src_y * linesize + src_x;
  ptr_cb = ref_picture[1] + uvsrc_y * uvlinesize + uvsrc_x;
  ptr_cr = ref_picture[2] + uvsrc_y * uvlinesize + uvsrc_x;

  if(   (unsigned)src_x > s->h_edge_pos - (motion_x&1) - 16
        || (unsigned)src_y >    v_edge_pos - (motion_y&1) - 16){
    av_log(s->avctx,AV_LOG_DEBUG,"MPEG motion vector out of boundary\n");
    return;
  }

  pix_op[0][dxy](dest_y, ptr_y, linesize, 16);

  pix_op[s->chroma_x_shift][uvdxy](dest_cb, ptr_cb, uvlinesize,
                                   16 >> s->chroma_y_shift);
  pix_op[s->chroma_x_shift][uvdxy](dest_cr, ptr_cr, uvlinesize,
                                   16 >> s->chroma_y_shift);
}

/* put block[] to dest[] */
#define put_dct(i, dest, line_size, qscale) \
  dct_unquantize_mpeg1_intra_c(s, block[i], (i), (qscale));\
  simple_idct_put((dest), (line_size), block[i])

#define add_dequant_dct(i, dest, line_size, qscale) \
  if (s->block_last_index[i] >= 0) {\
    dct_unquantize_mpeg1_inter_c(s, block[i], (i), (qscale));\
    simple_idct_add((dest), (line_size), block[i]);\
  }

/* generic function called after a macroblock has been parsed by the
   decoder or after it has been encoded by the encoder.

   Important variables used:
   s->mb_intra : true if intra macroblock
   s->mv_dir   : motion vector direction
   s->mv       : motion vector
 */
void MPV_decode_mb(MpegEncContext *s, DCTELEM block[12][64])
{
  /* update DC predictors for P macroblocks */
  if (!s->mb_intra) {
    s->last_dc[0] = s->last_dc[1] =
      s->last_dc[2] = 128 << s->intra_dc_precision;
  }

  if (!(s->intra_only || s->pict_type==B_TYPE)) { /*FIXME precalc*/
    uint8_t *dest_y, *dest_cb, *dest_cr;
    int dct_linesize, dct_offset;
    const int linesize= s->current_picture.linesize[0]; /*not s->linesize as this woulnd be wrong for field pics*/
    const int uvlinesize= s->current_picture.linesize[1];

    dct_linesize = linesize;
    dct_offset = linesize*8;

    dest_y=  s->dest[0];
    dest_cb= s->dest[1];
    dest_cr= s->dest[2];
    if (!s->mb_intra) {
      /* motion handling */
      /* add dct residue */
      add_dequant_dct(0, dest_y, dct_linesize, s->qscale);
      add_dequant_dct(1, dest_y + 8, dct_linesize, s->qscale);
      add_dequant_dct(2, dest_y + dct_offset, dct_linesize, s->qscale);
      add_dequant_dct(3, dest_y + dct_offset + 8, dct_linesize, s->qscale);
      add_dequant_dct(4, dest_cb, uvlinesize, s->chroma_qscale);
      add_dequant_dct(5, dest_cr, uvlinesize, s->chroma_qscale);
    } else {
      /* dct only in intra block */
      put_dct(0, dest_y, dct_linesize, s->qscale);
      put_dct(1, dest_y + 8, dct_linesize, s->qscale);
      put_dct(2, dest_y + dct_offset, dct_linesize, s->qscale);
      put_dct(3, dest_y + dct_offset + 8, dct_linesize, s->qscale);
      put_dct(4, dest_cb, uvlinesize, s->chroma_qscale);
      put_dct(5, dest_cr, uvlinesize, s->chroma_qscale);
    }
  }
}

void ff_init_block_index(MpegEncContext *s){ /*FIXME maybe rename*/
  const int linesize= s->current_picture.linesize[0]; /*not s->linesize as this woulnd be wrong for field pics*/
  const int uvlinesize= s->current_picture.linesize[1];
        
  s->block_index[0]= s->b8_stride*(s->mb_y*2    ) - 2 + s->mb_x*2;
  s->block_index[1]= s->b8_stride*(s->mb_y*2    ) - 1 + s->mb_x*2;
  s->block_index[2]= s->b8_stride*(s->mb_y*2 + 1) - 2 + s->mb_x*2;
  s->block_index[3]= s->b8_stride*(s->mb_y*2 + 1) - 1 + s->mb_x*2;
  s->block_index[4]= s->mb_stride*(s->mb_y + 1)                + s->b8_stride*s->mb_height*2 + s->mb_x - 1;
  s->block_index[5]= s->mb_stride*(s->mb_y + s->mb_height + 2) + s->b8_stride*s->mb_height*2 + s->mb_x - 1;
  /*block_index is not used by mpeg2, so it is not affected by chroma_format*/

  s->dest[0] = s->current_picture.data[0] + (s->mb_x - 1)*16;
  s->dest[1] = s->current_picture.data[1] + (s->mb_x - 1)*(16 >> s->chroma_x_shift);
  s->dest[2] = s->current_picture.data[2] + (s->mb_x - 1)*(16 >> s->chroma_x_shift);
    
  s->dest[0] += s->mb_y *   linesize * 16;
  s->dest[1] += s->mb_y * uvlinesize * (16 >> s->chroma_y_shift);
  s->dest[2] += s->mb_y * uvlinesize * (16 >> s->chroma_y_shift);
}

static void
encode_mb(MpegEncContext *s, int motion_x, int motion_y)
{
  const int mb_x= s->mb_x;
  const int mb_y= s->mb_y;
  int i;
  int skip_dct[6];
  int dct_offset   = s->linesize*8; /*default for progressive frames*/
  uint8_t *ptr_y, *ptr_cb, *ptr_cr;
  int wrap_y, wrap_c;
    
  for(i=0; i<6; i++) skip_dct[i]=0;

  wrap_y = s->linesize;
  wrap_c = s->uvlinesize;
  ptr_y = s->new_picture.data[0] + (mb_y * 16 * wrap_y) + mb_x * 16;
  ptr_cb = s->new_picture.data[1] + (mb_y * 8 * wrap_c) + mb_x * 8;
  ptr_cr = s->new_picture.data[2] + (mb_y * 8 * wrap_c) + mb_x * 8;

  if(mb_x*16+16 > s->width || mb_y*16+16 > s->height){
    ff_emulated_edge_mc(s->edge_emu_buffer            , ptr_y , wrap_y,16,16,mb_x*16,mb_y*16, s->width   , s->height);
    ptr_y= s->edge_emu_buffer;
    ff_emulated_edge_mc(s->edge_emu_buffer+18*wrap_y  , ptr_cb, wrap_c, 8, 8, mb_x*8, mb_y*8, s->width>>1, s->height>>1);
    ptr_cb= s->edge_emu_buffer+18*wrap_y;
    ff_emulated_edge_mc(s->edge_emu_buffer+18*wrap_y+9, ptr_cr, wrap_c, 8, 8, mb_x*8, mb_y*8, s->width>>1, s->height>>1);
    ptr_cr= s->edge_emu_buffer+18*wrap_y+9;
  }

  if (s->mb_intra) {
    s->dsp.get_pixels(s->block[0], ptr_y                 , wrap_y);
    s->dsp.get_pixels(s->block[1], ptr_y              + 8, wrap_y);
    s->dsp.get_pixels(s->block[2], ptr_y + dct_offset    , wrap_y);
    s->dsp.get_pixels(s->block[3], ptr_y + dct_offset + 8, wrap_y);

    s->dsp.get_pixels(s->block[4], ptr_cb, wrap_c);
    s->dsp.get_pixels(s->block[5], ptr_cr, wrap_c);

  }else{
    op_pixels_func (*op_pix)[4];
    uint8_t *dest_y, *dest_cb, *dest_cr;

    dest_y  = s->dest[0];
    dest_cb = s->dest[1];
    dest_cr = s->dest[2];

    op_pix = s->dsp.put_pixels_tab;

    if (s->mv_dir & MV_DIR_FORWARD) {
      mpeg_motion(s, dest_y, dest_cb, dest_cr, s->last_picture.data, op_pix,
                  s->mv[0][0][0], s->mv[0][0][1]);
      op_pix = s->dsp.avg_pixels_tab;
    }
    if (s->mv_dir & MV_DIR_BACKWARD) {
      mpeg_motion(s, dest_y, dest_cb, dest_cr, s->next_picture.data, op_pix,
                  s->mv[1][0][0], s->mv[1][0][1]);
    }

    s->dsp.diff_pixels(s->block[0], ptr_y                 , dest_y                 , wrap_y);
    s->dsp.diff_pixels(s->block[1], ptr_y              + 8, dest_y              + 8, wrap_y);
    s->dsp.diff_pixels(s->block[2], ptr_y + dct_offset    , dest_y + dct_offset    , wrap_y);
    s->dsp.diff_pixels(s->block[3], ptr_y + dct_offset + 8, dest_y + dct_offset + 8, wrap_y);
        
    {
      s->dsp.diff_pixels(s->block[4], ptr_cb, dest_cb, wrap_c);
      s->dsp.diff_pixels(s->block[5], ptr_cr, dest_cr, wrap_c);
    }
    /* pre quantization */         
    if(s->current_picture.mc_mb_var[s->mb_stride*mb_y+ mb_x] <
       2*s->qscale*s->qscale){
      /*FIXME optimize*/
      if(pix_abs8_c(NULL, ptr_y               , dest_y               , wrap_y, 8) < 20*s->qscale) skip_dct[0]= 1;
      if(pix_abs8_c(NULL, ptr_y            + 8, dest_y            + 8, wrap_y, 8) < 20*s->qscale) skip_dct[1]= 1;
      if(pix_abs8_c(NULL, ptr_y +dct_offset   , dest_y +dct_offset   , wrap_y, 8) < 20*s->qscale) skip_dct[2]= 1;
      if(pix_abs8_c(NULL, ptr_y +dct_offset+ 8, dest_y +dct_offset+ 8, wrap_y, 8) < 20*s->qscale) skip_dct[3]= 1;
      if(pix_abs8_c(NULL, ptr_cb              , dest_cb              , wrap_c, 8) < 20*s->qscale) skip_dct[4]= 1;
      if(pix_abs8_c(NULL, ptr_cr              , dest_cr              , wrap_c, 8) < 20*s->qscale) skip_dct[5]= 1;
    }
  }

  /* DCT & quantize */
  assert(s->qscale==8);
  for(i=0;i<6;i++) {
    if(!skip_dct[i]){
      int overflow;
      s->block_last_index[i] = dct_quantize_c(s, s->block[i], i, s->qscale,
                                              &overflow);
      /* FIXME we could decide to change to quantizer instead of clipping*/
      /* JS: I don't think that would be a good idea it could lower quality instead*/
      /*     of improve it. Just INTRADC clipping deserves changes in quantizer*/
      if (overflow) {
        int icc;
        DCTELEM *block = s->block[i];
        int last_index = s->block_last_index[i];
        const int maxlevel= s->max_qcoeff;
        const int minlevel= s->min_qcoeff;
        overflow=0;
    
        if(s->mb_intra){
          icc=1; /*skip clipping of intra dc*/
        }else
          icc=0;
    
        for(;icc<=last_index; icc++){
          const int j= s->intra_scantable.permutated[icc];
          int level = block[j];
       
          if     (level>maxlevel){
            level=maxlevel;
            overflow++;
          }else if(level<minlevel){
            level=minlevel;
            overflow++;
          }
        
          block[j]= level;
        }
    
        if(overflow)
          av_log(s->avctx, AV_LOG_INFO, "warning, cliping %d dct coefficents to %d..%d\n", overflow, minlevel, maxlevel);
      }
    }else
      s->block_last_index[i]= -1;
  }

  /* huffman encode */
  mpeg1_encode_mb(s, s->block, motion_x, motion_y);
}

void ff_mpeg_flush(AVCodecContext *avctx)
{
  int i;
  MpegEncContext *s = avctx->priv_data;
    
  if(s==NULL || s->picture==NULL) 
    return;
    
  for(i=0; i<MAX_PICTURE_COUNT; i++){
    if(s->picture[i].data[0] && (   s->picture[i].type == FF_BUFFER_TYPE_INTERNAL
                                    || s->picture[i].type == FF_BUFFER_TYPE_USER))
      avctx->release_buffer(avctx, (AVFrame*)&s->picture[i]);
  }
  s->current_picture_ptr = s->last_picture_ptr = s->next_picture_ptr = NULL;

  s->parse_context.state= -1;
  s->parse_context.frame_start_found= 0;
  s->parse_context.overread= 0;
  s->parse_context.overread_index= 0;
  s->parse_context.index= 0;
  s->parse_context.last_index= 0;
}

void ff_copy_bits(PutBitContext *pb, uint8_t *src, int length)
{
  const uint16_t *srcw= (uint16_t*)src;
  int words= length>>4;
  int bits= length&15;
  int i;

  if(length==0) return;
    
  if(words < 16){
    for(i=0; i<words; i++) put_bits(pb, 16, be2me_16(srcw[i]));
  }else if(put_bits_count(pb)&7){
    for(i=0; i<words; i++) put_bits(pb, 16, be2me_16(srcw[i]));
  }else{
    for(i=0; put_bits_count(pb)&31; i++)
      put_bits(pb, 8, src[i]);
    flush_put_bits(pb);
    memcpy(pbBufPtr(pb), src+i, 2*words-i);
    skip_put_bytes(pb, 2*words-i);
  }

  put_bits(pb, bits, be2me_16(srcw[words])>>(16-bits));
}

static int
sse(MpegEncContext *s, uint8_t *src1, uint8_t *src2, int w, int h, int stride)
{
    uint32_t *sq = squareTbl + 256;
    int acc=0;
    int x,y;
    
    if(w==16 && h==16) 
        return sse16_c(NULL, src1, src2, stride, 16);
    else if(w==8 && h==8)
        return sse8_c(NULL, src1, src2, stride, 8);
    
    for(y=0; y<h; y++){
        for(x=0; x<w; x++){
            acc+= sq[src1[x + y*stride] - src2[x + y*stride]];
        } 
    }
    
    assert(acc>=0);
    
    return acc;
}

static int estimate_motion_thread(AVCodecContext *c, void *arg)
{
  MpegEncContext *s= arg;

  s->first_slice_line=1;
  for(s->mb_y= s->start_mb_y; s->mb_y < s->end_mb_y; s->mb_y++) {
    s->mb_x=0; /*for block init below*/
    ff_init_block_index(s);
    for(s->mb_x=0; s->mb_x < s->mb_width; s->mb_x++) {
      s->block_index[0]+=2;
      s->block_index[1]+=2;
      s->block_index[2]+=2;
      s->block_index[3]+=2;
            
      /* compute motion vector & mb_type and store in context */
      if(s->pict_type==B_TYPE)
        ff_estimate_b_frame_motion(s, s->mb_x, s->mb_y);
      else
        ff_estimate_p_frame_motion(s, s->mb_x, s->mb_y);
    }
    s->first_slice_line=0;
  }
  return 0;
}

static int mb_var_thread(AVCodecContext *c, void *arg)
{
  MpegEncContext *s= arg;
  int mb_x, mb_y;

  for(mb_y=s->start_mb_y; mb_y < s->end_mb_y; mb_y++) {
    for(mb_x=0; mb_x < s->mb_width; mb_x++) {
      int xx = mb_x * 16;
      int yy = mb_y * 16;
      uint8_t *pix = s->new_picture.data[0] + (yy * s->linesize) + xx;
      int varc;
      int sum = s->dsp.pix_sum(pix, s->linesize);
    
      varc = (s->dsp.pix_norm1(pix, s->linesize) - (((unsigned)(sum*sum))>>8) + 500 + 128)>>8;

      s->current_picture.mb_var [s->mb_stride * mb_y + mb_x] = varc;
      s->current_picture.mb_mean[s->mb_stride * mb_y + mb_x] = (sum+128)>>8;
      s->me.mb_var_sum_temp    += varc;
    }
  }
  return 0;
}

/**
 * set qscale and update qscale dependant variables.
 */
void ff_set_qscale(MpegEncContext * s, int qscale)
{
  if (qscale < 1)
    qscale = 1;
  else if (qscale > 31)
    qscale = 31;

  s->qscale = qscale;
  s->chroma_qscale= s->chroma_qscale_table[qscale];

  s->y_dc_scale= s->y_dc_scale_table[ qscale ];
  s->c_dc_scale= s->c_dc_scale_table[ s->chroma_qscale ];
}

static int encode_thread(AVCodecContext *c, void *arg)
{
  MpegEncContext *s= arg;
  int mb_x, mb_y;
  int i;
  uint8_t bit_buf[2][3000];
  uint8_t bit_buf2[2][3000];
  uint8_t bit_buf_tex[2][3000];
  PutBitContext pb[2], pb2[2], tex_pb[2];

  for(i=0; i<2; i++){
    init_put_bits(&pb    [i], bit_buf    [i], 3000);
    init_put_bits(&pb2   [i], bit_buf2   [i], 3000);
    init_put_bits(&tex_pb[i], bit_buf_tex[i], 3000);
  }

  s->last_bits= put_bits_count(&s->pb);
  s->mv_bits=0;
  s->misc_bits=0;
  s->i_tex_bits=0;
  s->p_tex_bits=0;
  s->i_count=0;
  s->f_count=0;
  s->b_count=0;
  s->skip_count=0;

  for(i=0; i<3; i++){
    /* init last dc values */
    /* note: quant matrix value (8) is implied here */
    s->last_dc[i] = 128 << s->intra_dc_precision;
  }
  s->mb_skip_run = 0;
  memset(s->last_mv, 0, sizeof(s->last_mv));
     
  s->last_mv_dir = 0;

  s->resync_mb_x=0;
  s->resync_mb_y=0; 
  s->first_slice_line = 1;

  for(mb_y= s->start_mb_y; mb_y < s->end_mb_y; mb_y++) {

    s->mb_x=0;
    s->mb_y= mb_y;

    ff_set_qscale(s, s->qscale);
    ff_init_block_index(s);

    for(mb_x=0; mb_x < s->mb_width; mb_x++) {
      const int xy= mb_y*s->mb_stride + mb_x;
      int mb_type= s->mb_type[xy];

      s->mb_x = mb_x;

      s->block_index[0]+=2;
      s->block_index[1]+=2;
      s->block_index[2]+=2;
      s->block_index[3]+=2;
      s->block_index[4]++;
      s->block_index[5]++;
      s->dest[0]+= 16;
      s->dest[1]+= 8;
      s->dest[2]+= 8;

      /* write gob / video packet header  */

      if(  (s->resync_mb_x   == s->mb_x)
           && s->resync_mb_y+1 == s->mb_y){
        s->first_slice_line=0; 
      }

      s->dquant=0; /*only for QP_RD*/

      {
        int motion_x, motion_y;
        /* only one MB-Type possible*/
                
        switch(mb_type){
        case CANDIDATE_MB_TYPE_INTRA:
          s->mv_dir = 0;
          s->mb_intra= 1;
          motion_x= s->mv[0][0][0] = 0;
          motion_y= s->mv[0][0][1] = 0;
          break;
        case CANDIDATE_MB_TYPE_INTER:
          s->mv_dir = MV_DIR_FORWARD;
          s->mb_intra= 0;
          motion_x= s->mv[0][0][0] = s->p_mv_table[xy][0];
          motion_y= s->mv[0][0][1] = s->p_mv_table[xy][1];
          break;
        case CANDIDATE_MB_TYPE_BIDIR:
          s->mv_dir = MV_DIR_FORWARD | MV_DIR_BACKWARD;
          s->mb_intra= 0;
          motion_x=0;
          motion_y=0;
          s->mv[0][0][0] = s->b_bidir_forw_mv_table[xy][0];
          s->mv[0][0][1] = s->b_bidir_forw_mv_table[xy][1];
          s->mv[1][0][0] = s->b_bidir_back_mv_table[xy][0];
          s->mv[1][0][1] = s->b_bidir_back_mv_table[xy][1];
          break;
        case CANDIDATE_MB_TYPE_BACKWARD:
          s->mv_dir = MV_DIR_BACKWARD;
          s->mb_intra= 0;
          motion_x= s->mv[1][0][0] = s->b_back_mv_table[xy][0];
          motion_y= s->mv[1][0][1] = s->b_back_mv_table[xy][1];
          break;
        case CANDIDATE_MB_TYPE_FORWARD:
          s->mv_dir = MV_DIR_FORWARD;
          s->mb_intra= 0;
          motion_x= s->mv[0][0][0] = s->b_forw_mv_table[xy][0];
          motion_y= s->mv[0][0][1] = s->b_forw_mv_table[xy][1];
          /*                    printf(" %d %d ", motion_x, motion_y);*/
          break;
        default:
          motion_x=motion_y=0; /*gcc warning fix*/
          av_log(s->avctx, AV_LOG_ERROR, "illegal MB type\n");
        }

        encode_mb(s, motion_x, motion_y);

        /* RAL: Update last macrobloc type*/
        s->last_mv_dir = s->mv_dir;

        MPV_decode_mb(s, s->block);
      }

      /* clean the MV table in IPS frames for direct mode in B frames */
      if(s->mb_intra /* && I,P_TYPE */){
        s->p_mv_table[xy][0]=0;
        s->p_mv_table[xy][1]=0;
      }

    }
  }

  align_put_bits(&s->pb);
  flush_put_bits(&s->pb);

  return 0;
}

/* must be called before writing the header */
void ff_set_mpeg4_time(MpegEncContext * s, int picture_number)
{
  assert(s->current_picture_ptr->pts != AV_NOPTS_VALUE);
  s->time= (s->current_picture_ptr->pts*s->time_increment_resolution +
            AV_TIME_BASE/2)/AV_TIME_BASE;

  if(s->pict_type==B_TYPE){
    s->pb_time= s->pp_time - (s->last_non_b_time - s->time);
  }else{
    s->pp_time= s->time - s->last_non_b_time;
    s->last_non_b_time= s->time;
  }
}

static void encode_picture(MpegEncContext *s, int picture_number)
{
  int i;
  int bits;

  s->picture_number = picture_number;
    
  /* Reset the average MB variance */
  s->me.mb_var_sum_temp    =
    s->me.mc_mb_var_sum_temp = 0;

  ff_set_mpeg4_time(s, s->picture_number);  /*FIXME rename and use has_b_frames or similar*/
        
  s->me.scene_change_score=0;

  s->mb_intra=0; /*for the rate distoration & bit compare functions*/

  ff_init_me(s);

  /* Estimate motion for every MB */
  if(s->pict_type != I_TYPE){
    estimate_motion_thread(s->avctx, s->thread_context[0]);
  }else /* if(s->pict_type == I_TYPE) */{
    /* I-Frame */
    for(i=0; i<s->mb_stride*s->mb_height; i++)
      s->mb_type[i]= CANDIDATE_MB_TYPE_INTRA;

    /* finding spatial complexity for I-frame rate control */
    mb_var_thread(s->avctx, s->thread_context[0]);
  }

  s->current_picture.mc_mb_var_sum= s->current_picture_ptr->mc_mb_var_sum= s->me.mc_mb_var_sum_temp;
  s->current_picture.   mb_var_sum= s->current_picture_ptr->   mb_var_sum= s->me.   mb_var_sum_temp;

  if(s->me.scene_change_score > 0 && s->pict_type == P_TYPE){
    s->pict_type= I_TYPE;
    for(i=0; i<s->mb_stride*s->mb_height; i++)
      s->mb_type[i]= CANDIDATE_MB_TYPE_INTRA;
    /*printf("Scene change detected, encoding as I Frame %d %d\n", s->current_picture.mb_var_sum, s->current_picture.mc_mb_var_sum);*/
  }

  if(s->pict_type==P_TYPE) {
    s->f_code= ff_get_best_fcode(s, s->p_mv_table, CANDIDATE_MB_TYPE_INTER);
    ff_fix_long_mvs(s, s->p_mv_table, s->f_code, CANDIDATE_MB_TYPE_INTER, 0);
  }

  if(s->pict_type==B_TYPE) {
    int a, b;

    a = ff_get_best_fcode(s, s->b_forw_mv_table, CANDIDATE_MB_TYPE_FORWARD);
    b = ff_get_best_fcode(s, s->b_bidir_forw_mv_table, CANDIDATE_MB_TYPE_BIDIR);
    s->f_code = FFMAX(a, b);

    a = ff_get_best_fcode(s, s->b_back_mv_table, CANDIDATE_MB_TYPE_BACKWARD);
    b = ff_get_best_fcode(s, s->b_bidir_back_mv_table, CANDIDATE_MB_TYPE_BIDIR);
    s->b_code = FFMAX(a, b);

    ff_fix_long_mvs(s, s->b_forw_mv_table, s->f_code, CANDIDATE_MB_TYPE_FORWARD, 1);
    ff_fix_long_mvs(s, s->b_back_mv_table, s->b_code, CANDIDATE_MB_TYPE_BACKWARD, 1);
    ff_fix_long_mvs(s, s->b_bidir_forw_mv_table, s->f_code, CANDIDATE_MB_TYPE_BIDIR, 1);
    ff_fix_long_mvs(s, s->b_bidir_back_mv_table, s->b_code, CANDIDATE_MB_TYPE_BIDIR, 1);

  }

  s->current_picture.quality = ff_rate_estimate_qscale(s); /*FIXME pic_ptr*/
  s->lambda= s->current_picture.quality;

  s->qscale= (s->lambda*139 + FF_LAMBDA_SCALE*64) >> (FF_LAMBDA_SHIFT + 7);
  s->qscale= clip(s->qscale, s->avctx->qmin, s->avctx->qmax);
  s->lambda2= (s->lambda*s->lambda + FF_LAMBDA_SCALE/2) >> FF_LAMBDA_SHIFT;

  if(s->qscale < 3 && s->max_qcoeff<=128 && s->pict_type==I_TYPE) 
    s->qscale= 3; /*reduce cliping problems*/

  /*FIXME var duplication*/
  s->current_picture.key_frame= s->pict_type == I_TYPE; /*FIXME pic_ptr*/
  s->current_picture.pict_type= s->pict_type;

  if(s->current_picture.key_frame)
    s->picture_in_gop_number=0;

  s->last_bits= put_bits_count(&s->pb);
  mpeg1_encode_picture_header(s, picture_number);

  bits= put_bits_count(&s->pb);
  s->header_bits= bits - s->last_bits;

  encode_thread(s->avctx, s->thread_context[0]);

}

static int dct_quantize_c(MpegEncContext *s, DCTELEM *block, int n,
                          int qscale, int *overflow)
{
  int i, j, level, last_non_zero, q, start_i;
  const int *qmat;
  const uint8_t *scantable= s->intra_scantable.scantable;
  int bias;
  int max=0;
  unsigned int threshold1, threshold2;

  ff_jpeg_fdct_islow(block);

  if (s->mb_intra) {
    if (n < 4)
      q = s->y_dc_scale;
    else
      q = s->c_dc_scale;
    q = q << 3;

    /* note: block[0] is assumed to be positive */
    block[0] = (block[0] + (q >> 1)) / q;
    start_i = 1;
    last_non_zero = 0;
    qmat = s->q_intra_matrix[qscale];
    bias= s->intra_quant_bias<<(QMAT_SHIFT - QUANT_BIAS_SHIFT);
  } else {
    start_i = 0;
    last_non_zero = -1;
    qmat = s->q_inter_matrix[qscale];
    bias= s->inter_quant_bias<<(QMAT_SHIFT - QUANT_BIAS_SHIFT);
  }
  threshold1= (1<<QMAT_SHIFT) - bias - 1;
  threshold2= (threshold1<<1);
  for(i=63;i>=start_i;i--) {
    j = scantable[i];
    level = block[j] * qmat[j];

    if(((unsigned)(level+threshold1))>threshold2){
      last_non_zero = i;
      break;
    }else{
      block[j]=0;
    }
  }
  for(i=start_i; i<=last_non_zero; i++) {
    j = scantable[i];
    level = block[j] * qmat[j];

    if(((unsigned)(level+threshold1))>threshold2){
      if(level>0){
        level= (bias + level)>>QMAT_SHIFT;
        block[j]= level;
      }else{
        level= (bias - level)>>QMAT_SHIFT;
        block[j]= -level;
      }
      max |=level;
    }else{
      block[j]=0;
    }
  }
  *overflow= s->max_qcoeff < max; /*overflow might have happend*/

  return last_non_zero;
}

static void dct_unquantize_mpeg1_intra_c(MpegEncContext *s, DCTELEM *block,
                                         int n, int qscale)
{
  int i, level, nCoeffs;
  const uint16_t *quant_matrix;

  nCoeffs= s->block_last_index[n];

  if (n < 4) 
    block[0] = block[0] * s->y_dc_scale;
  else
    block[0] = block[0] * s->c_dc_scale;
  /* XXX: only mpeg1 */
  quant_matrix = s->intra_matrix;
  for(i=1;i<=nCoeffs;i++) {
    int j= s->intra_scantable.permutated[i];
    level = block[j];
    if (level) {
      if (level < 0) {
        level = -level;
        level = (int)(level * qscale * quant_matrix[j]) >> 3;
        level = (level - 1) | 1;
        level = -level;
      } else {
        level = (int)(level * qscale * quant_matrix[j]) >> 3;
        level = (level - 1) | 1;
      }
      block[j] = level;
    }
  }
}

static void dct_unquantize_mpeg1_inter_c(MpegEncContext *s, DCTELEM *block,
                                         int n, int qscale)
{
  int i, level, nCoeffs;
  const uint16_t *quant_matrix;

  nCoeffs= s->block_last_index[n];

  quant_matrix = s->inter_matrix;
  for(i=0; i<=nCoeffs; i++) {
    int j= s->intra_scantable.permutated[i];
    level = block[j];
    if (level) {
      if (level < 0) {
        level = -level;
        level = (((level << 1) + 1) * qscale *
                 ((int) (quant_matrix[j]))) >> 4;
        level = (level - 1) | 1;
        level = -level;
      } else {
        level = (((level << 1) + 1) * qscale *
                 ((int) (quant_matrix[j]))) >> 4;
        level = (level - 1) | 1;
      }
      block[j] = level;
    }
  }
}
