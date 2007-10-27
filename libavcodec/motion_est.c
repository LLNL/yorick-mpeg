/*
 * Motion estimation 
 * Copyright (c) 2000,2001 Fabrice Bellard.
 * Copyright (c) 2002-2004 Michael Niedermayer
 * 
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
 * new Motion Estimation (X1/EPZS) by Michael Niedermayer <michaelni@gmx.at>
 */
 
/**
 * @file motion_est.c
 * Motion estimation.
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include "avcodec.h"
#include "dsputil.h"
#include "mpegvideo.h"

#undef NDEBUG
#include <assert.h>

#define SQ(a) ((a)*(a))

#define P_LEFT P[1]
#define P_TOP P[2]
#define P_TOPRIGHT P[3]
#define P_MEDIAN P[4]
#define P_MV1 P[9]

#define init_ref(init_ref2)\
  {\
    int i, offset[3];\
    offset[0] = (mb_y<<4)*c->stride + (mb_x<<4);\
    offset[1] = offset[2] = (mb_y<<3)*c->uvstride + (mb_x<<3);\
    for(i=0; i<3; i++){\
      c->src[0][i]= s->new_picture.data[i] + offset[i];\
      c->ref[0][i]= s->last_picture.data[i] + offset[i];\
      init_ref2\
    }\
  }
#define init_ref2 c->ref[2][i]= s->next_picture.data[i] + offset[i];

#define cmp(x,y) pix_abs16_c(s, c->src[0][0], c->ref[ref_index][0] + (x) + (y)*c->stride, c->stride, 16)

#define CHECK_MV(x,y)\
{\
  const int key= ((y)<<ME_MAP_MV_BITS) + (x) + map_generation;\
  const int index= (((y)<<ME_MAP_SHIFT) + (x))&(ME_MAP_SIZE-1);\
  if(map[index]!=key){\
    d= cmp(x, y);\
    map[index]= key;\
    score_map[index]= d;\
    d += (mv_penalty[((x)<<shift)-pred_x] + mv_penalty[((y)<<shift)-pred_y])*penalty_factor;\
    COPY3_IF_LT(dmin, d, best[0], x, best[1], y)\
  }\
}

#define CHECK_CLIPED_MV(ax,ay)\
{\
  const int x= ax;\
  const int y= ay;\
  const int x2= FFMAX(xmin, FFMIN(x, xmax));\
  const int y2= FFMAX(ymin, FFMIN(y, ymax));\
  CHECK_MV(x2, y2)\
}

#define CHECK_MV_DIR(x,y,new_dir)\
{\
  const int key= ((y)<<ME_MAP_MV_BITS) + (x) + map_generation;\
  const int index= (((y)<<ME_MAP_SHIFT) + (x))&(ME_MAP_SIZE-1);\
  if(map[index]!=key){\
    d= cmp(x, y);\
    map[index]= key;\
    score_map[index]= d;\
    d += (mv_penalty[((x)<<shift)-pred_x] + mv_penalty[((y)<<shift)-pred_y])*penalty_factor;\
    if(d<dmin){\
      best[0]=x;\
      best[1]=y;\
      dmin=d;\
      next_dir= new_dir;\
    }\
  }\
}

static int
epzs_motion_search(MpegEncContext * s, int *mx_ptr, int *my_ptr,
                   int P[10][2], int ref_index, int16_t (*last_mv)[2], 
                   int ref_mv_scale)
{
  MotionEstContext * const c= &s->me;
  int best[2]={0, 0};
  int d, dmin;
  int map_generation;
  const int penalty_factor= c->penalty_factor;
  const int ref_mv_stride= s->mb_stride; /*pass as arg  FIXME*/
  const int ref_mv_xy= s->mb_x + s->mb_y*ref_mv_stride; /*add to last_mv beforepassing FIXME*/

  uint32_t * const score_map= c->score_map;
  const int xmin= c->xmin;
  const int ymin= c->ymin;
  const int xmax= c->xmax;
  const int ymax= c->ymax;
  uint8_t *mv_penalty= c->current_mv_penalty;
  const int pred_x= c->pred_x;
  const int pred_y= c->pred_y;

  uint32_t *map= c->map;
  const int shift= 1;

  c->map_generation+= 1<<(ME_MAP_MV_BITS*2);
  if(c->map_generation==0){
    c->map_generation= 1<<(ME_MAP_MV_BITS*2);
    memset(c->map, 0, sizeof(uint32_t)*ME_MAP_SIZE);
  }
  map_generation= c->map_generation;

  dmin= cmp(0, 0);
  map[0]= map_generation;
  score_map[0]= dmin;

  /* first line */
  if (s->first_slice_line) {
    CHECK_MV(P_LEFT[0]>>shift, P_LEFT[1]>>shift)
    CHECK_CLIPED_MV((last_mv[ref_mv_xy][0]*ref_mv_scale + (1<<15))>>16, 
                    (last_mv[ref_mv_xy][1]*ref_mv_scale + (1<<15))>>16)
  }else{
    if(dmin<256 && ( P_LEFT[0]    |P_LEFT[1]
                     |P_TOP[0]     |P_TOP[1]
                     |P_TOPRIGHT[0]|P_TOPRIGHT[1])==0){
      *mx_ptr= 0;
      *my_ptr= 0;
      c->skip=1;
      return dmin;
    }
    CHECK_MV(P_MEDIAN[0]>>shift, P_MEDIAN[1]>>shift)
    if(dmin>256*2){
      CHECK_CLIPED_MV((last_mv[ref_mv_xy][0]*ref_mv_scale + (1<<15))>>16, 
                      (last_mv[ref_mv_xy][1]*ref_mv_scale + (1<<15))>>16)
      CHECK_MV(P_LEFT[0]    >>shift, P_LEFT[1]    >>shift)
      CHECK_MV(P_TOP[0]     >>shift, P_TOP[1]     >>shift)
      CHECK_MV(P_TOPRIGHT[0]>>shift, P_TOPRIGHT[1]>>shift)
    }
  }
  if(dmin>256*4){
    CHECK_CLIPED_MV((last_mv[ref_mv_xy+1][0]*ref_mv_scale + (1<<15))>>16, 
                    (last_mv[ref_mv_xy+1][1]*ref_mv_scale + (1<<15))>>16)
    if(s->mb_y+1<s->end_mb_y)  /*FIXME replace at least with last_slice_line*/
      CHECK_CLIPED_MV((last_mv[ref_mv_xy+ref_mv_stride][0]*ref_mv_scale + (1<<15))>>16, 
                      (last_mv[ref_mv_xy+ref_mv_stride][1]*ref_mv_scale + (1<<15))>>16)
  }

  /* dmin= diamond_search(s, best, dmin, ref_index, penalty_factor); */
  {
    int next_dir=-1;

    { /* ensure that the best point is in the MAP as h/qpel refinement needs it */
      const int key= (best[1]<<ME_MAP_MV_BITS) + best[0] + map_generation;
      const int index= ((best[1]<<ME_MAP_SHIFT) + best[0])&(ME_MAP_SIZE-1);
      if(map[index]!=key){ /*this will be executed only very rarey*/
        score_map[index]= cmp(best[0], best[1]);
        map[index]= key;
      }
    }

    for(;;){
      int d;
      const int dir= next_dir;
      const int x= best[0];
      const int y= best[1];
      next_dir=-1;

      if(dir!=2 && x>xmin) CHECK_MV_DIR(x-1, y  , 0)
      if(dir!=3 && y>ymin) CHECK_MV_DIR(x  , y-1, 1)
      if(dir!=0 && x<xmax) CHECK_MV_DIR(x+1, y  , 2)
      if(dir!=1 && y<ymax) CHECK_MV_DIR(x  , y+1, 3)

      if(next_dir==-1) break;
    }
  }

  *mx_ptr= best[0];
  *my_ptr= best[1];    

  return dmin;
}

#define get_penalty_factor(s) ((s)->lambda>>FF_LAMBDA_SHIFT)

void ff_init_me(MpegEncContext *s)
{
  MotionEstContext * const c= &s->me;
  c->avctx= s->avctx;

  c->flags    = 0;

  if(s->linesize){
    c->stride  = s->linesize; 
    c->uvstride= s->uvlinesize;
  }else{
    c->stride  = 16*s->mb_width + 32;
    c->uvstride=  8*s->mb_width + 16;
  }

  c->temp= c->scratchpad;
}


#define Z_THRESHOLD 256

#define CHECK_SAD_HALF_MV(suffix, x, y) \
{\
  d= s->dsp.pix_abs[0][(x?1:0)+(y?2:0)](NULL, pix, ptr+((x)>>1), stride, 16);\
  d += (mv_penalty[pen_x + x] + mv_penalty[pen_y + y])*penalty_factor;\
  COPY3_IF_LT(dminh, d, dx, x, dy, y)\
}

static int sad_hpel_motion_search(MpegEncContext * s,
				  int *mx_ptr, int *my_ptr, int dmin,
                                  int ref_index)
{
  MotionEstContext * const c= &s->me;
  const int penalty_factor= c->sub_penalty_factor;
  int mx, my, dminh;
  uint8_t *pix, *ptr;
  int stride= c->stride;

  uint32_t * const score_map= c->score_map;
  const int xmin= c->xmin;
  const int ymin= c->ymin;
  const int xmax= c->xmax;
  const int ymax= c->ymax;
  uint8_t *mv_penalty= c->current_mv_penalty;
  const int pred_x= c->pred_x;
  const int pred_y= c->pred_y;

  if(c->skip){
    *mx_ptr = 0;
    *my_ptr = 0;
    return dmin;
  }

  pix = c->src[0][0];

  mx = *mx_ptr;
  my = *my_ptr;
  ptr = c->ref[ref_index][0] + (my * stride) + mx;

  dminh = dmin;

  if (mx > xmin && mx < xmax && 
      my > ymin && my < ymax) {
    int dx=0, dy=0;
    int d, pen_x, pen_y; 
    const int index= (my<<ME_MAP_SHIFT) + mx;
    const int t= score_map[(index-(1<<ME_MAP_SHIFT))&(ME_MAP_SIZE-1)];
    const int l= score_map[(index- 1               )&(ME_MAP_SIZE-1)];
    const int r= score_map[(index+ 1               )&(ME_MAP_SIZE-1)];
    const int b= score_map[(index+(1<<ME_MAP_SHIFT))&(ME_MAP_SIZE-1)];
    mx<<=1;
    my<<=1;

        
    pen_x= pred_x + mx;
    pen_y= pred_y + my;

    ptr-= stride;
    if(t<=b){
      CHECK_SAD_HALF_MV(y2 , 0, -1)
      if(l<=r){
        CHECK_SAD_HALF_MV(xy2, -1, -1)
        if(t+r<=b+l){
          CHECK_SAD_HALF_MV(xy2, +1, -1)
          ptr+= stride;
        }else{
          ptr+= stride;
          CHECK_SAD_HALF_MV(xy2, -1, +1)
        }
        CHECK_SAD_HALF_MV(x2 , -1,  0)
      }else{
        CHECK_SAD_HALF_MV(xy2, +1, -1)
        if(t+l<=b+r){
          CHECK_SAD_HALF_MV(xy2, -1, -1)
          ptr+= stride;
        }else{
          ptr+= stride;
          CHECK_SAD_HALF_MV(xy2, +1, +1)
        }
        CHECK_SAD_HALF_MV(x2 , +1,  0)
      }
    }else{
      if(l<=r){
        if(t+l<=b+r){
          CHECK_SAD_HALF_MV(xy2, -1, -1)
          ptr+= stride;
        }else{
          ptr+= stride;
          CHECK_SAD_HALF_MV(xy2, +1, +1)
        }
        CHECK_SAD_HALF_MV(x2 , -1,  0)
        CHECK_SAD_HALF_MV(xy2, -1, +1)
      }else{
        if(t+r<=b+l){
          CHECK_SAD_HALF_MV(xy2, +1, -1)
          ptr+= stride;
        }else{
          ptr+= stride;
          CHECK_SAD_HALF_MV(xy2, -1, +1)
        }
        CHECK_SAD_HALF_MV(x2 , +1,  0)
        CHECK_SAD_HALF_MV(xy2, +1, +1)
      }
      CHECK_SAD_HALF_MV(y2 ,  0, +1)
    }
    mx+=dx;
    my+=dy;

  }else{
    mx<<=1;
    my<<=1;
  }

  *mx_ptr = mx;
  *my_ptr = my;
  return dminh;
}

static void set_p_mv_tables(MpegEncContext * s, int mx, int my, int mv4)
{
  const int xy= s->mb_x + s->mb_y*s->mb_stride;
    
  s->p_mv_table[xy][0] = mx;
  s->p_mv_table[xy][1] = my;

  /* has allready been set to the 4 MV if 4MV is done */
  if(mv4){
    int mot_xy= s->block_index[0];

    s->current_picture.motion_val[0][mot_xy  ][0]= mx;
    s->current_picture.motion_val[0][mot_xy  ][1]= my;
    s->current_picture.motion_val[0][mot_xy+1][0]= mx;
    s->current_picture.motion_val[0][mot_xy+1][1]= my;

    mot_xy += s->b8_stride;
    s->current_picture.motion_val[0][mot_xy  ][0]= mx;
    s->current_picture.motion_val[0][mot_xy  ][1]= my;
    s->current_picture.motion_val[0][mot_xy+1][0]= mx;
    s->current_picture.motion_val[0][mot_xy+1][1]= my;
  }
}

static int ff_sqrt(int a)
{
  int ret=0, s, ret_sq=0;
  if(a<128) return ff_sqrt_tab[a];
  for(s=15 ; s>=0 ; s--){
    int b= ret_sq + (1<<(s<<1)) + (ret<<(s+1));
    if(b<=a){
      ret_sq=b;
      ret+= 1<<s;
    }
  }
  return ret;
}

/**
 * get fullpel ME search limits.
 */
#define get_limits \
  c->xmin = - 16*mb_x;\
  c->ymin = - 16*mb_y;\
  c->xmax = - 16*mb_x + s->mb_width *16 - 16;\
  c->ymax = - 16*mb_y + s->mb_height*16 - 16

void ff_estimate_p_frame_motion(MpegEncContext * s, int mb_x, int mb_y)
{
  MotionEstContext * const c= &s->me;
  uint8_t *pix, *ppix;
  int sum, varc, vard, mx, my, dmin;
  int P[10][2];
  const int shift= 1;
  int mb_type=0;
  Picture * const pic= &s->current_picture;
    
  init_ref();

  assert(s->linesize == c->stride);
  assert(s->uvlinesize == c->uvstride);

  c->penalty_factor    = get_penalty_factor(s);
  c->sub_penalty_factor= get_penalty_factor(s);
  c->mb_penalty_factor = get_penalty_factor(s);
  c->current_mv_penalty= c->mv_penalty[s->f_code] + MAX_MV;

  get_limits;
  c->skip=0;

  /* intra / predictive decision */
  pix = c->src[0][0];
  sum = s->dsp.pix_sum(pix, s->linesize);
  varc = (s->dsp.pix_norm1(pix, s->linesize) - (((unsigned)(sum*sum))>>8) + 500 + 128)>>8;

  pic->mb_mean[s->mb_stride * mb_y + mb_x] = (sum+128)>>8;
  pic->mb_var [s->mb_stride * mb_y + mb_x] = varc;
  c->mb_var_sum_temp += varc;

  {
    const int mot_stride = s->b8_stride;
    const int mot_xy = s->block_index[0];

    P_LEFT[0]       = s->current_picture.motion_val[0][mot_xy - 1][0];
    P_LEFT[1]       = s->current_picture.motion_val[0][mot_xy - 1][1];

    if(P_LEFT[0] > (c->xmax<<shift)) P_LEFT[0] = (c->xmax<<shift);

    if(!s->first_slice_line) {
      P_TOP[0]      = s->current_picture.motion_val[0][mot_xy - mot_stride    ][0];
      P_TOP[1]      = s->current_picture.motion_val[0][mot_xy - mot_stride    ][1];
      P_TOPRIGHT[0] = s->current_picture.motion_val[0][mot_xy - mot_stride + 2][0];
      P_TOPRIGHT[1] = s->current_picture.motion_val[0][mot_xy - mot_stride + 2][1];
      if(P_TOP[1]      > (c->ymax<<shift)) P_TOP[1]     = (c->ymax<<shift);
      if(P_TOPRIGHT[0] < (c->xmin<<shift)) P_TOPRIGHT[0]= (c->xmin<<shift);
      if(P_TOPRIGHT[1] > (c->ymax<<shift)) P_TOPRIGHT[1]= (c->ymax<<shift);
        
      P_MEDIAN[0]= mid_pred(P_LEFT[0], P_TOP[0], P_TOPRIGHT[0]);
      P_MEDIAN[1]= mid_pred(P_LEFT[1], P_TOP[1], P_TOPRIGHT[1]);

      c->pred_x= P_LEFT[0];
      c->pred_y= P_LEFT[1];
    }else{
      c->pred_x= P_LEFT[0];
      c->pred_y= P_LEFT[1];
    }

  }
  dmin = epzs_motion_search(s, &mx, &my, P, 0, s->p_mv_table, (1<<16)>>shift);       

  /* At this point (mx,my) are full-pell and the relative displacement */
  ppix = c->ref[0][0] + (my * s->linesize) + mx;
        
  vard = (sse16_c(NULL, pix, ppix, s->linesize, 16) + 128)>>8;

  pic->mc_mb_var[s->mb_stride * mb_y + mb_x] = vard;
  c->mc_mb_var_sum_temp += vard;

  if(mb_type){
    if (vard <= 64 || vard < varc)
      c->scene_change_score+= ff_sqrt(vard) - ff_sqrt(varc);
    else
      c->scene_change_score+= s->qscale;

    if(mb_type == CANDIDATE_MB_TYPE_INTER){
      sad_hpel_motion_search(s, &mx, &my, dmin, 0);
      set_p_mv_tables(s, mx, my, 1);
    }else{
      mx <<=shift;
      my <<=shift;
    }

  }else{
    int intra_score, i;
    mb_type= CANDIDATE_MB_TYPE_INTER;

    dmin= sad_hpel_motion_search(s, &mx, &my, dmin, 0);

    set_p_mv_tables(s, mx, my, 1);

    /* get intra luma score */
    {
      int mean= (sum+128)>>8;
      mean*= 0x01010101;
            
      for(i=0; i<16; i++){
        *(uint32_t*)(&c->scratchpad[i*s->linesize+ 0]) = mean;
        *(uint32_t*)(&c->scratchpad[i*s->linesize+ 4]) = mean;
        *(uint32_t*)(&c->scratchpad[i*s->linesize+ 8]) = mean;
        *(uint32_t*)(&c->scratchpad[i*s->linesize+12]) = mean;
      }

      intra_score= pix_abs16_c(s, c->scratchpad, pix, s->linesize, 16);
    }

    intra_score += c->mb_penalty_factor*16;
        
    if(intra_score < dmin){
      mb_type= CANDIDATE_MB_TYPE_INTRA;
      s->current_picture.mb_type[mb_y*s->mb_stride + mb_x]= CANDIDATE_MB_TYPE_INTRA; /*FIXME cleanup*/
    }else
      s->current_picture.mb_type[mb_y*s->mb_stride + mb_x]= 0;
        
    if (vard <= 64 || vard < varc) { /*FIXME*/
      c->scene_change_score+= ff_sqrt(vard) - ff_sqrt(varc);
    }else{
      c->scene_change_score+= s->qscale;
    }
  }

  s->mb_type[mb_y*s->mb_stride + mb_x]= mb_type;
}

static int
ff_estimate_motion_b(MpegEncContext * s, int mb_x, int mb_y,
                     int16_t (*mv_table)[2], int ref_index, int f_code)
{
  MotionEstContext * const c= &s->me;
  int mx, my, dmin;
  int P[10][2];
  const int shift= 1;
  const int mot_stride = s->mb_stride;
  const int mot_xy = mb_y*mot_stride + mb_x;
  uint8_t * const mv_penalty= c->mv_penalty[f_code] + MAX_MV;
  int mv_scale;

  c->penalty_factor    = get_penalty_factor(s);
  c->sub_penalty_factor= get_penalty_factor(s);
  c->mb_penalty_factor = get_penalty_factor(s);
  c->current_mv_penalty= mv_penalty;

  get_limits;

  P_LEFT[0]        = mv_table[mot_xy - 1][0];
  P_LEFT[1]        = mv_table[mot_xy - 1][1];

  if(P_LEFT[0]       > (c->xmax<<shift)) P_LEFT[0]       = (c->xmax<<shift);

  /* special case for first line */
  if (!s->first_slice_line) {
    P_TOP[0] = mv_table[mot_xy - mot_stride             ][0];
    P_TOP[1] = mv_table[mot_xy - mot_stride             ][1];
    P_TOPRIGHT[0] = mv_table[mot_xy - mot_stride + 1         ][0];
    P_TOPRIGHT[1] = mv_table[mot_xy - mot_stride + 1         ][1];
    if(P_TOP[1] > (c->ymax<<shift)) P_TOP[1]= (c->ymax<<shift);
    if(P_TOPRIGHT[0] < (c->xmin<<shift)) P_TOPRIGHT[0]= (c->xmin<<shift);
    if(P_TOPRIGHT[1] > (c->ymax<<shift)) P_TOPRIGHT[1]= (c->ymax<<shift);
        
    P_MEDIAN[0]= mid_pred(P_LEFT[0], P_TOP[0], P_TOPRIGHT[0]);
    P_MEDIAN[1]= mid_pred(P_LEFT[1], P_TOP[1], P_TOPRIGHT[1]);
  }
  c->pred_x= P_LEFT[0];
  c->pred_y= P_LEFT[1];

  if(mv_table == s->b_forw_mv_table){
    mv_scale= (s->pb_time<<16) / (s->pp_time<<shift);
  }else{
    mv_scale= ((s->pb_time - s->pp_time)<<16) / (s->pp_time<<shift);
  }

  dmin= epzs_motion_search(s, &mx, &my, P, ref_index, s->p_mv_table, mv_scale);
  dmin= sad_hpel_motion_search(s, &mx, &my, dmin, ref_index);

  mv_table[mot_xy][0]= mx;
  mv_table[mot_xy][1]= my;

  return dmin;
}

/* refine the bidir vectors in hq mode and return the score in both lq & hq mode*/
static int bidir_refine(MpegEncContext * s, int mb_x, int mb_y)
{
  const int mot_stride = s->mb_stride;
  const int xy = mb_y *mot_stride + mb_x;
  int fbmin;
  int pred_fx= s->b_bidir_forw_mv_table[xy-1][0];
  int pred_fy= s->b_bidir_forw_mv_table[xy-1][1];
  int pred_bx= s->b_bidir_back_mv_table[xy-1][0];
  int pred_by= s->b_bidir_back_mv_table[xy-1][1];
  int motion_fx= s->b_bidir_forw_mv_table[xy][0]= s->b_forw_mv_table[xy][0];
  int motion_fy= s->b_bidir_forw_mv_table[xy][1]= s->b_forw_mv_table[xy][1];
  int motion_bx= s->b_bidir_back_mv_table[xy][0]= s->b_back_mv_table[xy][0];
  int motion_by= s->b_bidir_back_mv_table[xy][1]= s->b_back_mv_table[xy][1];

  /*FIXME do refinement and add flag*/
    
  {
    /*FIXME optimize?*/
    /*FIXME better f_code prediction (max mv & distance)*/
    /*FIXME pointers*/
    MotionEstContext * const c= &s->me;
    uint8_t * const mv_penalty= c->mv_penalty[s->f_code] + MAX_MV; /* f_code of the prev frame*/
    int stride= c->stride;
    uint8_t *dest_y = c->scratchpad;
    uint8_t *ptr;
    int dxy;
    int src_x, src_y;
    uint8_t **src_data= c->src[0];
    uint8_t **ref_data= c->ref[0];
    uint8_t **ref2_data= c->ref[2];

    dxy = ((motion_fy & 1) << 1) | (motion_fx & 1);
    src_x = motion_fx >> 1;
    src_y = motion_fy >> 1;

    ptr = ref_data[0] + (src_y * stride) + src_x;
    s->dsp.put_pixels_tab[0][dxy](dest_y    , ptr    , stride, 16);

    dxy = ((motion_by & 1) << 1) | (motion_bx & 1);
    src_x = motion_bx >> 1;
    src_y = motion_by >> 1;

    ptr = ref2_data[0] + (src_y * stride) + src_x;
    s->dsp.avg_pixels_tab[0][dxy](dest_y    , ptr    , stride, 16);

    fbmin = (mv_penalty[motion_fx-pred_fx] + mv_penalty[motion_fy-pred_fy])*
      c->mb_penalty_factor
      + (mv_penalty[motion_bx-pred_bx] + mv_penalty[motion_by-pred_by])*
      c->mb_penalty_factor
      + pix_abs16_c(s, src_data[0], dest_y, stride, 16); /*FIXME new_pic*/
  }

  return fbmin;
}

void ff_estimate_b_frame_motion(MpegEncContext * s, int mb_x, int mb_y)
{
  MotionEstContext * const c= &s->me;
  const int penalty_factor= c->mb_penalty_factor;
  int fmin, bmin, dmin, fbmin, bimin, fimin;
  int type=0;
  init_ref(init_ref2);

  get_limits;

  c->skip=0;

  dmin= INT_MAX;
  /*FIXME penalty stuff for non mpeg4*/
  c->skip=0;
  fmin= ff_estimate_motion_b(s, mb_x, mb_y, s->b_forw_mv_table, 0, s->f_code) + 3*penalty_factor;
    
  c->skip=0;
  bmin= ff_estimate_motion_b(s, mb_x, mb_y, s->b_back_mv_table, 2, s->b_code) + 2*penalty_factor;

  c->skip=0;
  fbmin= bidir_refine(s, mb_x, mb_y) + penalty_factor;

  fimin= bimin= INT_MAX;

  {
    int score= fmin;
    type = CANDIDATE_MB_TYPE_FORWARD;
        
    if (dmin <= score){
      score = dmin;
      type = CANDIDATE_MB_TYPE_DIRECT;
    }
    if(bmin<score){
      score=bmin;
      type= CANDIDATE_MB_TYPE_BACKWARD; 
    }
    if(fbmin<score){
      score=fbmin;
      type= CANDIDATE_MB_TYPE_BIDIR;
    }

    score= ((unsigned)(score*score + 128*256))>>16;
    c->mc_mb_var_sum_temp += score;
    s->current_picture.mc_mb_var[mb_y*s->mb_stride + mb_x] = score; /*FIXME use SSE*/
  }

  s->mb_type[mb_y*s->mb_stride + mb_x]= type;
}

/* find best f_code for ME which do unlimited searches */
int ff_get_best_fcode(MpegEncContext * s, int16_t (*mv_table)[2], int type)
{
  int score[8];
  int i, y;
  uint8_t * fcode_tab= s->fcode_tab;
  int best_fcode=-1;
  int best_score=-10000000;

  for(i=0; i<8; i++) score[i]= s->mb_num*(8-i);

  for(y=0; y<s->mb_height; y++){
    int x;
    int xy= y*s->mb_stride;
    for(x=0; x<s->mb_width; x++){
      if(s->mb_type[xy] & type){
        int fcode= FFMAX(fcode_tab[mv_table[xy][0] + MAX_MV],
                         fcode_tab[mv_table[xy][1] + MAX_MV]);
        int j;
                    
        for(j=0; j<fcode && j<8; j++){
          if(s->pict_type==B_TYPE || s->current_picture.mc_mb_var[xy] < s->current_picture.mb_var[xy])
            score[j]-= 170;
        }
      }
      xy++;
    }
  }
        
  for(i=1; i<8; i++){
    if(score[i] > best_score){
      best_score= score[i];
      best_fcode= i;
    }
  }

  return best_fcode;
}

/**
 *
 * @param truncate 1 for truncation, 0 for using intra
 */
void ff_fix_long_mvs(MpegEncContext * s, int16_t (*mv_table)[2],
                     int f_code, int type, int truncate)
{
  int y, h_range, v_range;

  /* RAL: 8 in MPEG-1, 16 in MPEG-4*/
  int range = (8 << f_code);

  h_range= range;
  v_range= range;

  /* clip / convert to intra 16x16 type MVs */
  for(y=0; y<s->mb_height; y++){
    int x;
    int xy= y*s->mb_stride;
    for(x=0; x<s->mb_width; x++){
      if (s->mb_type[xy] & type){    /* RAL: "type" test added...*/
        if(   mv_table[xy][0] >=h_range || mv_table[xy][0] <-h_range
              || mv_table[xy][1] >=v_range || mv_table[xy][1] <-v_range){

          if(truncate){
            if     (mv_table[xy][0] > h_range-1) mv_table[xy][0]=  h_range-1;
            else if(mv_table[xy][0] < -h_range ) mv_table[xy][0]= -h_range;
            if     (mv_table[xy][1] > v_range-1) mv_table[xy][1]=  v_range-1;
            else if(mv_table[xy][1] < -v_range ) mv_table[xy][1]= -v_range;
          }else{
            s->mb_type[xy] &= ~type;
            s->mb_type[xy] |= CANDIDATE_MB_TYPE_INTRA;
            mv_table[xy][0]= mv_table[xy][1]= 0;
          }
        }
      }
      xy++;
    }
  }
}
