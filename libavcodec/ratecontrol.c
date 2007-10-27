/*
 * Rate control for video encoders
 *
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
 * @file ratecontrol.c
 * Rate control for video encoders.
 */ 

#include "avcodec.h"
#include "dsputil.h"
#include "mpegvideo.h"
#include <math.h>

#undef NDEBUG /* allways check asserts, the speed effect is far too small to disable them*/
#include <assert.h>

#ifndef M_E
#define M_E 2.718281828
#endif

static double get_qscale(MpegEncContext *s, RateControlEntry *rce,
                         double rate_factor, int frame_num);

int ff_rate_control_init(MpegEncContext *s)
{
  RateControlContext *rcc= &s->rc_context;
  int i;

  for(i=0; i<5; i++){
    rcc->pred[i].coeff= FF_QP2LAMBDA * 7.0;
    rcc->pred[i].count= 1.0;
    
    rcc->pred[i].decay= 0.4;
    rcc->i_cplx_sum [i]=
      rcc->p_cplx_sum [i]=
      rcc->mv_bits_sum[i]=
      rcc->qscale_sum [i]=
      rcc->frame_count[i]= 1; /* 1 is better cuz of 1/0 and such*/
    rcc->last_qscale_for[i]=FF_QP2LAMBDA * 5;
  }
  rcc->buffer_index= s->avctx->rc_initial_buffer_occupancy;

  rcc->short_term_qsum=0.001;
  rcc->short_term_qcount=0.001;
    
  rcc->pass1_rc_eq_output_sum= 0.001;
  rcc->pass1_wanted_bits=0.001;
        
  /* init stuff with the user specified complexity */
  if(s->avctx->rc_initial_cplx){
    for(i=0; i<60*30; i++){
      double bits= s->avctx->rc_initial_cplx * (i/10000.0 + 1.0)*s->mb_num;
      RateControlEntry rce;
      double q;
                
      if     (i%((s->gop_size+3)/4)==0) rce.pict_type= I_TYPE;
      else if(i%(s->max_b_frames+1))    rce.pict_type= B_TYPE;
      else                              rce.pict_type= P_TYPE;

      rce.new_pict_type= rce.pict_type;
      rce.mc_mb_var_sum= bits*s->mb_num/100000;
      rce.mb_var_sum   = s->mb_num;
      rce.qscale   = FF_QP2LAMBDA * 2;
      rce.f_code   = 2;
      rce.b_code   = 1;
      rce.misc_bits= 1;

      if(s->pict_type== I_TYPE){
        rce.i_count   = s->mb_num;
        rce.i_tex_bits= bits;
        rce.p_tex_bits= 0;
        rce.mv_bits= 0;
      }else{
        rce.i_count   = 0; /*FIXME we do know this approx*/
        rce.i_tex_bits= 0;
        rce.p_tex_bits= bits*0.9;
        rce.mv_bits= bits*0.1;
      }
      rcc->i_cplx_sum [rce.pict_type] += rce.i_tex_bits*rce.qscale;
      rcc->p_cplx_sum [rce.pict_type] += rce.p_tex_bits*rce.qscale;
      rcc->mv_bits_sum[rce.pict_type] += rce.mv_bits;
      rcc->frame_count[rce.pict_type] ++;

      bits= rce.i_tex_bits + rce.p_tex_bits;

      q= get_qscale(s, &rce,
                    rcc->pass1_wanted_bits/rcc->pass1_rc_eq_output_sum, i);
      rcc->pass1_wanted_bits+=
        s->bit_rate/(s->avctx->frame_rate / (double)s->avctx->frame_rate_base);
    }
  }

  return 0;
}

void ff_rate_control_uninit(MpegEncContext *s)
{
    RateControlContext *rcc= &s->rc_context;

    av_freep(&rcc->entry);
}

#define qp2bits(qp) \
  (rce->qscale * (double)(rce->i_tex_bits + rce->p_tex_bits+1)/ (qp))

#define bits2qp(bits) \
  (rce->qscale * (double)(rce->i_tex_bits + rce->p_tex_bits+1)/ (bits))

int ff_vbv_update(MpegEncContext *s, int frame_size){
    RateControlContext *rcc= &s->rc_context;
    const double fps= (double)s->avctx->frame_rate / (double)s->avctx->frame_rate_base;
    const int buffer_size= s->avctx->rc_buffer_size;
    const double min_rate= s->avctx->rc_min_rate/fps;
    const double max_rate= s->avctx->rc_max_rate/fps;
    
/*printf("%d %f %d %f %f\n", buffer_size, rcc->buffer_index, frame_size, min_rate, max_rate);*/
    if(buffer_size){
        int left;

        rcc->buffer_index-= frame_size;
        if(rcc->buffer_index < 0){
            av_log(s->avctx, AV_LOG_ERROR, "rc buffer underflow\n");
            rcc->buffer_index= 0;
        }

        left= buffer_size - rcc->buffer_index - 1;
        rcc->buffer_index += clip(left, min_rate, max_rate);

        if(rcc->buffer_index > buffer_size){
            int stuffing= ceil((rcc->buffer_index - buffer_size)/8);

            rcc->buffer_index -= 8*stuffing;

            return stuffing;
        }
    }
    return 0;
}

/**
 * modifies the bitrate curve from pass1 for one frame
 */
static double get_qscale(MpegEncContext *s, RateControlEntry *rce,
                         double rate_factor, int frame_num)
{
    RateControlContext *rcc= &s->rc_context;
    double q, bits;
    const int pict_type= rce->new_pict_type;

    /* this is rc_eq="tex^qComp" with qComp=0.5 */
    bits = sqrt((rce->i_tex_bits + rce->p_tex_bits)*(double)rce->qscale);

    rcc->pass1_rc_eq_output_sum+= bits;
    bits*=rate_factor;
    if(bits<0.0) bits=0.0;
    bits+= 1.0; /*avoid 1/0 issues*/

    q= bits2qp(bits);
    
    /* I/B difference */
    if     (pict_type==I_TYPE && s->avctx->i_quant_factor<0.0)
        q= -q*s->avctx->i_quant_factor + s->avctx->i_quant_offset;
    else if(pict_type==B_TYPE && s->avctx->b_quant_factor<0.0)
        q= -q*s->avctx->b_quant_factor + s->avctx->b_quant_offset;
        
    return q;
}

static double get_diff_limited_q(MpegEncContext *s, RateControlEntry *rce, double q){
    RateControlContext *rcc= &s->rc_context;
    AVCodecContext *a= s->avctx;
    const int pict_type= rce->new_pict_type;
    const double last_p_q    = rcc->last_qscale_for[P_TYPE];
    const double last_non_b_q= rcc->last_qscale_for[rcc->last_non_b_pict_type];
    
    if     (pict_type==I_TYPE && (a->i_quant_factor>0.0 || rcc->last_non_b_pict_type==P_TYPE))
        q= last_p_q    *ABS(a->i_quant_factor) + a->i_quant_offset;
    else if(pict_type==B_TYPE && a->b_quant_factor>0.0)
        q= last_non_b_q*    a->b_quant_factor  + a->b_quant_offset;

    /* last qscale / qdiff stuff */
    if(rcc->last_non_b_pict_type==pict_type || pict_type!=I_TYPE){
        double last_q= rcc->last_qscale_for[pict_type];
        const int maxdiff= FF_QP2LAMBDA * a->max_qdiff;

        if     (q > last_q + maxdiff) q= last_q + maxdiff;
        else if(q < last_q - maxdiff) q= last_q - maxdiff;
    }

    rcc->last_qscale_for[pict_type]= q; /*Note we cant do that after blurring*/
    
    if(pict_type!=B_TYPE)
        rcc->last_non_b_pict_type= pict_type;

    return q;
}

/**
 * gets the qmin & qmax for pict_type
 */
static void get_qminmax(int *qmin_ret, int *qmax_ret, MpegEncContext *s, int pict_type){
    int qmin= s->avctx->lmin;                                                       
    int qmax= s->avctx->lmax;
    
    assert(qmin <= qmax);

    if(pict_type==B_TYPE){
        qmin= (int)(qmin*ABS(s->avctx->b_quant_factor)+s->avctx->b_quant_offset + 0.5);
        qmax= (int)(qmax*ABS(s->avctx->b_quant_factor)+s->avctx->b_quant_offset + 0.5);
    }else if(pict_type==I_TYPE){
        qmin= (int)(qmin*ABS(s->avctx->i_quant_factor)+s->avctx->i_quant_offset + 0.5);
        qmax= (int)(qmax*ABS(s->avctx->i_quant_factor)+s->avctx->i_quant_offset + 0.5);
    }

    qmin= clip(qmin, 1, FF_LAMBDA_MAX);
    qmax= clip(qmax, 1, FF_LAMBDA_MAX);

    if(qmax<qmin) qmax= qmin;
    
    *qmin_ret= qmin;
    *qmax_ret= qmax;
}

static double modify_qscale(MpegEncContext *s, RateControlEntry *rce, double q, int frame_num){
    RateControlContext *rcc= &s->rc_context;
    int qmin, qmax;
    double bits;
    const int pict_type= rce->new_pict_type;
    const double buffer_size= s->avctx->rc_buffer_size;
    const double fps= (double)s->avctx->frame_rate / (double)s->avctx->frame_rate_base;
    const double min_rate= s->avctx->rc_min_rate / fps;
    const double max_rate= s->avctx->rc_max_rate / fps;
    
    get_qminmax(&qmin, &qmax, s, pict_type);

    bits= qp2bits(q);
/*printf("q:%f\n", q);*/
    /* buffer overflow/underflow protection */
    if(buffer_size){
        double expected_size= rcc->buffer_index;
        double q_limit;

        if(min_rate){
            double d= 2*(buffer_size - expected_size)/buffer_size;
            if(d>1.0) d=1.0;
            else if(d<0.0001) d=0.0001;
            q*= pow(d, 1.0/s->avctx->rc_buffer_aggressivity);

            q_limit= bits2qp(FFMAX((min_rate - buffer_size + rcc->buffer_index)*3, 1));
            if (q > q_limit) q= q_limit;
        }

        if(max_rate){
            double d= 2*expected_size/buffer_size;
            if(d>1.0) d=1.0;
            else if(d<0.0001) d=0.0001;
            q/= pow(d, 1.0/s->avctx->rc_buffer_aggressivity);

            q_limit= bits2qp(FFMAX(rcc->buffer_index/3, 1));
            if(q < q_limit) q= q_limit;
        }
    }
    if     (q<qmin) q=qmin;
    else if(q>qmax) q=qmax;
    
    return q;
}

/*----------------------------------*/
/* 1 Pass Code*/

static double predict_size(Predictor *p, double q, double var)
{
     return p->coeff*var / (q*p->count);
}

static void update_predictor(Predictor *p, double q, double var, double size)
{
    double new_coeff= size*q / (var + 1);
    if(var<10) return;

    p->count*= p->decay;
    p->coeff*= p->decay;
    p->count++;
    p->coeff+= new_coeff;
}

float ff_rate_estimate_qscale(MpegEncContext *s)
{
  float q;
  int qmin, qmax;
  float br_compensation;
  double diff;
  double fps;
  int picture_number= s->picture_number;
  int64_t wanted_bits;
  RateControlContext *rcc= &s->rc_context;
  AVCodecContext *a= s->avctx;
  RateControlEntry local_rce, *rce;
  double bits;
  double rate_factor;
  int var;
  const int pict_type= s->pict_type;
  Picture * const pic= &s->current_picture;

  get_qminmax(&qmin, &qmax, s, pict_type);

  fps= (double)s->avctx->frame_rate / (double)s->avctx->frame_rate_base;
  /*printf("input_pic_num:%d pic_num:%d frame_rate:%d\n", s->input_picture_number, s->picture_number, s->frame_rate);*/
  /* update predictors */
  if(picture_number>2){
    const int last_var= s->last_pict_type == I_TYPE ? rcc->last_mb_var_sum : rcc->last_mc_mb_var_sum;
    update_predictor(&rcc->pred[s->last_pict_type], rcc->last_qscale, sqrt(last_var), s->frame_bits);
  }

  rce= &local_rce;
  wanted_bits= (uint64_t)(s->bit_rate*(double)picture_number/fps);

  diff= s->total_bits - wanted_bits;
  br_compensation= (a->bit_rate_tolerance - diff)/a->bit_rate_tolerance;
  if(br_compensation<=0.0) br_compensation=0.001;

  var= pict_type == I_TYPE ? pic->mb_var_sum : pic->mc_mb_var_sum;

  rce->pict_type= 
    rce->new_pict_type= pict_type;
  rce->mc_mb_var_sum= pic->mc_mb_var_sum;
  rce->mb_var_sum   = pic->   mb_var_sum;
  rce->qscale   = FF_QP2LAMBDA * 2;
  rce->f_code   = s->f_code;
  rce->b_code   = s->b_code;
  rce->misc_bits= 1;

  bits= predict_size(&rcc->pred[pict_type], rce->qscale, sqrt(var));
  if(pict_type== I_TYPE){
    rce->i_count   = s->mb_num;
    rce->i_tex_bits= bits;
    rce->p_tex_bits= 0;
    rce->mv_bits= 0;
  }else{
    rce->i_count   = 0; /*FIXME we do know this approx*/
    rce->i_tex_bits= 0;
    rce->p_tex_bits= bits*0.9;
            
    rce->mv_bits= bits*0.1;
  }
  rcc->i_cplx_sum [pict_type] += rce->i_tex_bits*rce->qscale;
  rcc->p_cplx_sum [pict_type] += rce->p_tex_bits*rce->qscale;
  rcc->mv_bits_sum[pict_type] += rce->mv_bits;
  rcc->frame_count[pict_type] ++;

  bits= rce->i_tex_bits + rce->p_tex_bits;
  rate_factor= rcc->pass1_wanted_bits/rcc->pass1_rc_eq_output_sum *
    br_compensation;

  q= get_qscale(s, rce, rate_factor, picture_number);

  assert(q>0.0);
  q= get_diff_limited_q(s, rce, q);
  assert(q>0.0);

  q= modify_qscale(s, rce, q, picture_number);

  rcc->pass1_wanted_bits+= s->bit_rate/fps;

  assert(q>0.0);

  if     (q<qmin) q=qmin; 
  else if(q>qmax) q=qmax;

  q= (int)(q + 0.5);
    
  rcc->last_qscale= q;
  rcc->last_mc_mb_var_sum= pic->mc_mb_var_sum;
  rcc->last_mb_var_sum= pic->mb_var_sum;

  return q;
}
