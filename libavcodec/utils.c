/*
 * utils for libavcodec
 * Copyright (c) 2001 Fabrice Bellard.
 * Copyright (c) 2003 Michel Bardiaux for the av_log API
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
 * @file utils.c
 * utils.
 */
 
#include "avcodec.h"
#include "dsputil.h"
#include "mpegvideo.h"
#include "integer.h"
#include <stdarg.h>
#include <limits.h>

static void avcodec_default_free_buffers(AVCodecContext *s);

void *av_mallocz(unsigned int size)
{
    void *ptr;
    
    ptr = av_malloc(size);
    if (!ptr)
        return NULL;
    memset(ptr, 0, size);
    return ptr;
}

/**
 * realloc which does nothing if the block is large enough
 */
void *av_fast_realloc(void *ptr, unsigned int *size, unsigned int min_size)
{
    if(min_size < *size) 
        return ptr;
    
    *size= 17*min_size/16 + 32;

    return av_realloc(ptr, *size);
}

/**
 * Frees memory and sets the pointer to NULL.
 * @param arg pointer to the pointer which should be freed
 */
void av_freep(void *arg)
{
    void **ptr= (void**)arg;
    av_free(*ptr);
    *ptr = NULL;
}

/* encoder management */
AVCodec *first_avcodec;

void register_avcodec(AVCodec *format)
{
    AVCodec **p;
    p = &first_avcodec;
    while (*p != NULL) p = &(*p)->next;
    *p = format;
    format->next = NULL;
}

typedef struct InternalBuffer{
    int last_pic_num;
    uint8_t *base[4];
    uint8_t *data[4];
    int linesize[4];
}InternalBuffer;

#define INTERNAL_BUFFER_SIZE 32

#define ALIGN(x, a) (((x)+(a)-1)&~((a)-1))

void avcodec_align_dimensions(AVCodecContext *s, int *width, int *height){
    int w_align= 1;    
    int h_align= 1;    
    
    switch(s->pix_fmt){
    case PIX_FMT_YUV420P:
        w_align= 16; /*FIXME check for non mpeg style codecs and use less alignment*/
        h_align= 16;
        break;
    default:
        w_align= 1;
        h_align= 1;
        break;
    }

    *width = ALIGN(*width , w_align);
    *height= ALIGN(*height, h_align);
}

int avcodec_default_get_buffer(AVCodecContext *s, AVFrame *pic){
    int i;
    int w= s->width;
    int h= s->height;
    InternalBuffer *buf;
    int *picture_number;

    assert(pic->data[0]==NULL);
    assert(INTERNAL_BUFFER_SIZE > s->internal_buffer_count);

    if(s->internal_buffer==NULL){
        s->internal_buffer= av_mallocz(INTERNAL_BUFFER_SIZE*sizeof(InternalBuffer));
    }

    buf= &((InternalBuffer*)s->internal_buffer)[s->internal_buffer_count];
    picture_number= &(((InternalBuffer*)s->internal_buffer)[INTERNAL_BUFFER_SIZE-1]).last_pic_num; /*FIXME ugly hack*/
    (*picture_number)++;
    
    if(buf->base[0]){
        pic->age= *picture_number - buf->last_pic_num;
        buf->last_pic_num= *picture_number;
    }else{
        int h_chroma_shift, v_chroma_shift;
        int s_align, pixel_size;
        
        avcodec_get_chroma_sub_sample(s->pix_fmt, &h_chroma_shift, &v_chroma_shift);
        
        switch(s->pix_fmt){
        case PIX_FMT_YUV422:
            pixel_size=2;
            break;
        case PIX_FMT_RGB24:
            pixel_size=3;
            break;
        default:
            pixel_size=1;
        }

        avcodec_align_dimensions(s, &w, &h);
        /*#if defined(ARCH_POWERPC)*/
#if 0
        s_align= 16;
#else
        s_align= 8;
#endif
            
        w+= EDGE_WIDTH*2;
        h+= EDGE_WIDTH*2;

        buf->last_pic_num= -256*256*256*64;

        for(i=0; i<3; i++){
            const int h_shift= i==0 ? 0 : h_chroma_shift;
            const int v_shift= i==0 ? 0 : v_chroma_shift;

            /*FIXME next ensures that linesize= 2^x uvlinesize, thats needed because some MC code assumes it*/
            buf->linesize[i]= ALIGN(pixel_size*w>>h_shift, s_align<<(h_chroma_shift-h_shift)); 

            buf->base[i]= av_mallocz((buf->linesize[i]*h>>v_shift)+16); /*FIXME 16*/
            if(buf->base[i]==NULL) return -1;
            memset(buf->base[i], 128, buf->linesize[i]*h>>v_shift);
        
            buf->data[i] = buf->base[i] + ALIGN((buf->linesize[i]*EDGE_WIDTH>>v_shift) + (EDGE_WIDTH>>h_shift), s_align);
        }
        pic->age= 256*256*256*64;
    }
    pic->type= FF_BUFFER_TYPE_INTERNAL;

    for(i=0; i<4; i++){
        pic->base[i]= buf->base[i];
        pic->data[i]= buf->data[i];
        pic->linesize[i]= buf->linesize[i];
    }
    s->internal_buffer_count++;

    return 0;
}

void avcodec_default_release_buffer(AVCodecContext *s, AVFrame *pic){
    int i;
    InternalBuffer *buf, *last, temp;

    assert(pic->type==FF_BUFFER_TYPE_INTERNAL);
    assert(s->internal_buffer_count);

    buf = NULL; /* avoids warning */
    for(i=0; i<s->internal_buffer_count; i++){ /*just 3-5 checks so is not worth to optimize*/
        buf= &((InternalBuffer*)s->internal_buffer)[i];
        if(buf->data[0] == pic->data[0])
            break;
    }
    assert(i < s->internal_buffer_count);
    s->internal_buffer_count--;
    last = &((InternalBuffer*)s->internal_buffer)[s->internal_buffer_count];

    temp= *buf;
    *buf= *last;
    *last= temp;

    for(i=0; i<3; i++){
        pic->data[i]=NULL;
/*        pic->base[i]=NULL;*/
    }
}

static const char* context_to_name(void* ptr)
{
  AVCodecContext *avc= ptr;

  if(avc && avc->codec && avc->codec->name)
    return avc->codec->name; 
  else
    return "NULL";
}

static AVClass av_codec_context_class = { "AVCodecContext", context_to_name };

void avcodec_get_context_defaults(AVCodecContext *s)
{
  memset(s, 0, sizeof(AVCodecContext));

  s->av_class= &av_codec_context_class;
  s->bit_rate= 800*1000;
  s->bit_rate_tolerance= s->bit_rate*10;
  s->qmin= 2;
  s->qmax= 31;

  s->max_qdiff= 3;
  s->b_quant_factor=1.25;
  s->b_quant_offset=1.25;
  s->i_quant_factor=-0.8;
  s->i_quant_offset=0.0;

  s->frame_rate_base= 1;
  s->frame_rate = 25;
  s->gop_size= 50;

  s->get_buffer= avcodec_default_get_buffer;
  s->release_buffer= avcodec_default_release_buffer;

  s->lmin= FF_QP2LAMBDA * s->qmin;
  s->lmax= FF_QP2LAMBDA * s->qmax;
  s->sample_aspect_ratio.num= 0;
  s->sample_aspect_ratio.den= 1;

  s->intra_quant_bias= FF_DEFAULT_QUANT_BIAS;
  s->inter_quant_bias= FF_DEFAULT_QUANT_BIAS;
}

/**
 * allocates a AVCodecContext and set it to defaults.
 * this can be deallocated by simply calling free() 
 */
AVCodecContext *avcodec_alloc_context(void)
{
  AVCodecContext *avctx= av_malloc(sizeof(AVCodecContext));
  if(avctx==NULL) return NULL;
  avcodec_get_context_defaults(avctx);
  return avctx;
}

void avcodec_get_frame_defaults(AVFrame *pic)
{
  memset(pic, 0, sizeof(AVFrame));
  pic->pts= AV_NOPTS_VALUE;
}

/**
 * allocates a AVPFrame and set it to defaults.
 * this can be deallocated by simply calling free() 
 */
AVFrame *avcodec_alloc_frame(void)
{
  AVFrame *pic= av_malloc(sizeof(AVFrame));
  if(pic==NULL) return NULL;
  avcodec_get_frame_defaults(pic);
  return pic;
}

int avcodec_open(AVCodecContext *avctx, AVCodec *codec)
{
  int ret;

  if(avctx->codec)
    return -1;

  avctx->codec = codec;
  avctx->codec_id = codec->id;
  avctx->frame_number = 0;
  if (codec->priv_data_size > 0) {
    avctx->priv_data = av_mallocz(codec->priv_data_size);
    if (!avctx->priv_data) 
      return -ENOMEM;
  } else {
    avctx->priv_data = NULL;
  }
  ret = avctx->codec->init(avctx);
  if (ret < 0) {
    av_freep(&avctx->priv_data);
    return ret;
  }
  return 0;
}

int avcodec_encode_video(AVCodecContext *avctx, uint8_t *buf, int buf_size, 
                         const AVFrame *pict)
{
  if((avctx->codec->capabilities & CODEC_CAP_DELAY) || pict){
    int ret = avctx->codec->encode(avctx, buf, buf_size, (void *)pict);
    avctx->frame_number++;
    return ret;
  }else
    return 0;
}

int avcodec_close(AVCodecContext *avctx)
{
  if (avctx->codec->close)
    avctx->codec->close(avctx);
  avcodec_default_free_buffers(avctx);
  av_freep(&avctx->priv_data);
  avctx->codec = NULL;
  return 0;
}

AVCodec *avcodec_find_encoder(enum CodecID id)
{
  AVCodec *p;
  p = first_avcodec;
  while (p) {
    if (p->encode != NULL && p->id == id)
      return p;
    p = p->next;
  }
  return NULL;
}

unsigned avcodec_version( void )
{
  return LIBAVCODEC_VERSION_INT;
}

/* must be called before any other functions */
void avcodec_init(void)
{
  static int inited = 0;

  if (inited != 0)
    return;
  inited = 1;

  dsputil_static_init();
}

static void avcodec_default_free_buffers(AVCodecContext *s)
{
  int i, j;

  if(s->internal_buffer==NULL) return;
    
  for(i=0; i<INTERNAL_BUFFER_SIZE; i++){
    InternalBuffer *buf= &((InternalBuffer*)s->internal_buffer)[i];
    for(j=0; j<4; j++){
      av_freep(&buf->base[j]);
      buf->data[j]= NULL;
    }
  }
  av_freep(&s->internal_buffer);
    
  s->internal_buffer_count=0;
}

int av_reduce(int *dst_nom, int *dst_den, int64_t nom, int64_t den, int64_t max){
  int exact=1, sign=0;
  int64_t gcd;

  assert(den != 0);

  if(den < 0)
    return av_reduce(dst_nom, dst_den, -nom, -den, max);
    
  sign= nom < 0;
  nom= ABS(nom);
    
  gcd = ff_gcd(nom, den);
  nom /= gcd;
  den /= gcd;
    
  if(nom > max || den > max){
    AVRational a0={0,1}, a1={1,0};
    exact=0;

    for(;;){
      int64_t x= nom / den;
      int64_t a2n= x*a1.num + a0.num;
      int64_t a2d= x*a1.den + a0.den;

      if(a2n > max || a2d > max) break;

      nom %= den;
        
      a0= a1;
      a1.num= a2n;
      a1.den= a2d;
      if(nom==0) break;
      x= nom; nom=den; den=x;
    }
    nom= a1.num;
    den= a1.den;
  }
    
  assert(ff_gcd(nom, den) == 1);
    
  *dst_nom = sign ? -nom : nom;
  *dst_den = den;
    
  return exact;
}

int64_t av_rescale(int64_t a, int64_t b, int64_t c)
{
  AVInteger ai, ci;
  assert(c > 0);
  assert(b >=0);
    
  if(a<0) return -av_rescale(-a, b, c);
    
  if(b<=INT_MAX && c<=INT_MAX){
    if(a<=INT_MAX)
      return (a * b + c/2)/c;
    else
      return a/c*b + (a%c*b + c/2)/c;
  }
    
  ai= av_mul_i(av_int2i(a), av_int2i(b));
  ci= av_int2i(c);
  ai= av_add_i(ai, av_shr_i(ci,1));
    
  return av_i2int(av_div_i(ai, ci));
}

/* av_log API */

static int av_log_level = AV_LOG_DEBUG;

static void av_log_default_callback(void* ptr, int level, const char* fmt, va_list vl)
{
  static int print_prefix=1;
  AVClass* avc= ptr ? *(AVClass**)ptr : NULL;
  if(level>av_log_level)
    return;
#undef fprintf
  if(print_prefix && avc) {
    fprintf(stderr, "[%s @ %p]", avc->item_name(ptr), avc);
  }
#define fprintf please_use_av_log
        
  print_prefix= strstr(fmt, "\n") != NULL;
        
  vfprintf(stderr, fmt, vl);
}

static void (*av_log_callback)(void*, int, const char*, va_list) = av_log_default_callback;

void av_log(void* avcl, int level, const char *fmt, ...)
{
  va_list vl;
  va_start(vl, fmt);
  av_vlog(avcl, level, fmt, vl);
  va_end(vl);
}

void av_vlog(void* avcl, int level, const char *fmt, va_list vl)
{
  av_log_callback(avcl, level, fmt, vl);
}

int av_log_get_level(void)
{
  return av_log_level;
}

void av_log_set_level(int level)
{
  av_log_level = level;
}

void av_log_set_callback(void (*callback)(void*, int, const char*, va_list))
{
  av_log_callback = callback;
}
