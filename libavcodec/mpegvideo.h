/*
 * Generic DCT based hybrid video encoder
 * Copyright (c) 2000, 2001, 2002 Fabrice Bellard.
 * Copyright (c) 2002-2004 Michael Niedermayer
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
 * @file mpegvideo.h
 * mpegvideo header.
 */
 
#ifndef AVCODEC_MPEGVIDEO_H
#define AVCODEC_MPEGVIDEO_H

#include "dsputil.h"

#define FRAME_SKIPED 100 /*/< return value for header parsers if frame is not coded*/

enum OutputFormat {
  FMT_MPEG1
};

#define EDGE_WIDTH 16

#define MPEG_BUF_SIZE (16 * 1024)

#define QMAT_SHIFT_MMX 16
#define QMAT_SHIFT 22

#define MAX_FCODE 7
#define MAX_MV 2048

#define MAX_PICTURE_COUNT 15

#define ME_MAP_SIZE 64
#define ME_MAP_SHIFT 3
#define ME_MAP_MV_BITS 11

/* run length table */
#define MAX_RUN    64
#define MAX_LEVEL  64

#define I_TYPE FF_I_TYPE  /*/< Intra*/
#define P_TYPE FF_P_TYPE  /*/< Predicted*/
#define B_TYPE FF_B_TYPE  /*/< Bi-dir predicted*/

typedef struct Predictor{
    double coeff;
    double count;
    double decay;
} Predictor;

typedef struct RateControlEntry{
    int pict_type;
    float qscale;
    int mv_bits;
    int i_tex_bits;
    int p_tex_bits;
    int misc_bits;
    uint64_t expected_bits;
    int new_pict_type;
    float new_qscale;
    int mc_mb_var_sum;
    int mb_var_sum;
    int i_count;
    int f_code;
    int b_code;
}RateControlEntry;

/**
 * rate control context.
 */
typedef struct RateControlContext{
    FILE *stats_file;
    int num_entries;              /*/< number of RateControlEntries */
    RateControlEntry *entry;
    double buffer_index;          /*/< amount of bits in the video/audio buffer */
    Predictor pred[5];
    double short_term_qsum;       /*/< sum of recent qscales */
    double short_term_qcount;     /*/< count of recent qscales */
    double pass1_rc_eq_output_sum;/*/< sum of the output of the rc equation, this is used for normalization  */
    double pass1_wanted_bits;     /*/< bits which should have been outputed by the pass1 code (including complexity init) */
    double last_qscale;
    double last_qscale_for[5];    /*/< last qscale for a specific pict type, used for max_diff & ipb factor stuff */
    int last_mc_mb_var_sum;
    int last_mb_var_sum;
    uint64_t i_cplx_sum[5];
    uint64_t p_cplx_sum[5];
    uint64_t mv_bits_sum[5];
    uint64_t qscale_sum[5];
    int frame_count[5];
    int last_non_b_pict_type;
}RateControlContext;

/**
 * Scantable.
 */
typedef struct ScanTable{
    const uint8_t *scantable;
    uint8_t permutated[64];
    uint8_t raster_end[64];
  /*#ifdef ARCH_POWERPC*/
		/** Used by dct_quantise_alitvec to find last-non-zero */
  /* uint8_t __align8 inverse[64]; */
  /*#endif*/
} ScanTable;

/**
 * Picture.
 */
typedef struct Picture{
    FF_COMMON_FRAME

    /**
     * halfpel luma planes.
     */
    uint8_t *interpolated[3];
    int16_t (*motion_val_base[2])[2];
    uint32_t *mb_type_base;
#define MB_TYPE_INTRA MB_TYPE_INTRA4x4 /*default mb_type if theres just one type*/

    int mb_var_sum;             /*/< sum of MB variance for current frame */
    int mc_mb_var_sum;          /*/< motion compensated MB variance for current frame */
    uint16_t *mb_var;           /*/< Table for MB variances */
    uint16_t *mc_mb_var;        /*/< Table for motion compensated MB variances */
    uint8_t *mb_mean;           /*/< Table for MB luminance */

    int b_frame_score;          /* */
} Picture;

typedef struct ParseContext{
    uint8_t *buffer;
    int index;
    int last_index;
    int buffer_size;
    uint32_t state;             /*/< contains the last few bytes in MSB order*/
    int frame_start_found;
    int overread;               /*/< the number of bytes which where irreversibly read from the next frame*/
    int overread_index;         /*/< the index into ParseContext.buffer of the overreaded bytes*/
} ParseContext;

struct MpegEncContext;

/**
 * Motion estimation context.
 */
typedef struct MotionEstContext{
  AVCodecContext *avctx;
  int skip;                      /* set if ME is skiped for the current MB */
  uint8_t *scratchpad;           /* data area for the me algo, so that the ME doesnt need to malloc/free */
  uint8_t *best_mb;
  uint8_t *temp;
  uint32_t *map;                 /* map to avoid duplicate evaluations */
  uint32_t *score_map;           /* map to store the scores */
  int map_generation;  

  int penalty_factor;
  int sub_penalty_factor;
  int mb_penalty_factor;
  int flags;

  int xmin;
  int xmax;
  int ymin;
  int ymax;
  int pred_x;
  int pred_y;
  uint8_t *src[4][4];
  uint8_t *ref[4][4];
  int stride;
  int uvstride;
  /* temp variables for picture complexity calculation */
  int mc_mb_var_sum_temp;
  int mb_var_sum_temp;
  int scene_change_score;

  uint8_t (*mv_penalty)[MAX_MV*2+1];  /* bits needed to encode a MV */
  uint8_t *current_mv_penalty;
}MotionEstContext;

/**
 * MpegEncContext.
 */
typedef struct MpegEncContext {
    struct AVCodecContext *avctx;
    /* the following parameters must be initialized before encoding */
    int width, height;/*/< picture size. must be a multiple of 16 */
    int gop_size;
    int intra_only;   /*/< if true, only intra pictures are generated */
    int bit_rate;     /*/< wanted bit rate */
    enum OutputFormat out_format; /*/< output format */

    int codec_id;     /* see CODEC_ID_xxx */

    int flags;        /*/< AVCodecContext.flags (HQ, MV4, ...) */
    int flags2;       /*/< AVCodecContext.flags2*/
    int max_b_frames; /*/< max number of b-frames for encoding */

    /* the following fields are managed internally by the encoder */

    /** bit output */
    PutBitContext pb;

    /* sequence parameters */
    int context_initialized;
    int input_picture_number;  /*/< used to set pic->display_picture_number, shouldnt be used for/by anything else*/
    int coded_picture_number;  /*/< used to set pic->coded_picture_number, shouldnt be used for/by anything else*/
    int picture_number;       /*FIXME remove, unclear definition*/
    int picture_in_gop_number; /*/< 0-> first pic in gop, ... */
    int b_frames_since_non_b;  /*/< used for encoding, relative to not yet reordered input */
    int64_t user_specified_pts;/*/< last non zero pts from AVFrame which was passed into avcodec_encode_video()*/
    int mb_width, mb_height;   /*/< number of MBs horizontally & vertically */
    int mb_stride;             /*/< mb_width+1 used for some arrays to allow simple addressng of left & top MBs withoutt sig11*/
    int b8_stride;             /*/< 2*mb_width+1 used for some 8x8 block arrays to allow simple addressng*/

    int h_edge_pos, v_edge_pos;/*/< horizontal / vertical position of the right/bottom edge (pixel replicateion)*/
    int mb_num;                /*/< number of MBs of a picture */
    int linesize;              /*/< line size, in bytes, may be different from width */
    int uvlinesize;            /*/< line size, for chroma in bytes, may be different from width */
    Picture *picture;          /*/< main picture buffer */
    Picture **input_picture;   /*/< next pictures on display order for encoding*/
    Picture **reordered_input_picture; /*/< pointer to the next pictures in codedorder for encoding*/
    
    int start_mb_y;            /*/< start mb_y of this thread (so current thread should process start_mb_y <= row < end_mb_y)*/
    int end_mb_y;              /*/< end   mb_y of this thread (so current thread should process start_mb_y <= row < end_mb_y)*/
    struct MpegEncContext *thread_context[1];

    /** 
     * copy of the previous picture structure.
     * note, linesize & data, might not match the previous picture (for field pictures)
     */
    Picture last_picture;       

    /** 
     * copy of the next picture structure.
     * note, linesize & data, might not match the next picture (for field pictures)
     */
    Picture next_picture;

    /** 
     * copy of the source picture structure for encoding.
     * note, linesize & data, might not match the source picture (for field pictures)
     */
    Picture new_picture;

    /** 
     * copy of the current picture structure.
     * note, linesize & data, might not match the current picture (for field pictures)
     */
    Picture current_picture;    /*/< buffer to store the decompressed current picture */

    Picture *last_picture_ptr;     /*/< pointer to the previous picture.*/
    Picture *next_picture_ptr;     /*/< pointer to the next picture (for bidir pred) */
    Picture *current_picture_ptr;  /*/< pointer to the current picture*/
    uint8_t *visualization_buffer[3]; /*< temporary buffer vor MV visualization*/
    int last_dc[3];                /*/< last DC values for MPEG1 */

    int y_dc_scale, c_dc_scale;
    uint8_t *y_dc_scale_table;     /*/< qscale -> y_dc_scale table */
    uint8_t *c_dc_scale_table;     /*/< qscale -> c_dc_scale table */
    const uint8_t *chroma_qscale_table;  /*/< qscale -> chroma_qscale (h263)*/

    uint8_t *prev_pict_types;     /*/< previous picture types in bitstream order, used for mb skip */
#define PREV_PICT_TYPES_BUFFER_SIZE 256

    uint8_t *allocated_edge_emu_buffer;
    uint8_t *edge_emu_buffer;     /*/< points into the middle of allocated_edge_emu_buffer*/
    uint8_t *rd_scratchpad;       /*/< scartchpad for rate distortion mb decission*/
    uint8_t *b_scratchpad;        /*/< scratchpad used for writing into write only buffers*/

    int qscale;                 /*/< QP */
    int chroma_qscale;          /*/< chroma QP */
    int lambda;                 /*/< lagrange multipler used in rate distortion*/
    int lambda2;                /*/< (lambda*lambda) >> FF_LAMBDA_SHIFT */
    int *lambda_table;

    int dquant;                 /*/< qscale difference to prev qscale  */
    int pict_type;              /*/< I_TYPE, P_TYPE, B_TYPE, ... */
    int last_pict_type; /*FIXME removes*/
    int last_non_b_pict_type;   /*/< used for mpeg4 gmc b-frames & ratecontrol */
    int dropable;
    int frame_rate_index;
    int frame_rate_ext_n;       /*/< MPEG-2 specific framerate modificators (numerator)*/
    int frame_rate_ext_d;       /*/< MPEG-2 specific framerate modificators (denominator)*/

    int decode;                 /*/< if 0 then decoding will be skiped (for encoding b frames for example)*/

    DSPContext dsp;             /*/< pointers for accelerated dsp fucntions */
    int f_code;                 /*/< forward MV resolution */
    int b_code;                 /*/< backward MV resolution for B Frames (mpeg4) */
    int16_t (*p_mv_table_base)[2];
    int16_t (*b_forw_mv_table_base)[2];
    int16_t (*b_back_mv_table_base)[2];
    int16_t (*b_bidir_forw_mv_table_base)[2]; 
    int16_t (*b_bidir_back_mv_table_base)[2]; 
    int16_t (*b_direct_mv_table_base)[2];
    int16_t (*p_field_mv_table_base[2][2])[2];
    int16_t (*b_field_mv_table_base[2][2][2])[2];
    int16_t (*p_mv_table)[2];            /*/< MV table (1MV per MB) p-frame encoding */
    int16_t (*b_forw_mv_table)[2];       /*/< MV table (1MV per MB) forward mode b-frame encoding */
    int16_t (*b_back_mv_table)[2];       /*/< MV table (1MV per MB) backward mode b-frame encoding */
    int16_t (*b_bidir_forw_mv_table)[2]; /*/< MV table (1MV per MB) bidir mode b-frame encoding */
    int16_t (*b_bidir_back_mv_table)[2]; /*/< MV table (1MV per MB) bidir mode b-frame encoding */
    int16_t (*b_direct_mv_table)[2];     /*/< MV table (1MV per MB) direct mode b-frame encoding */
    int16_t (*p_field_mv_table[2][2])[2];   /*/< MV table (2MV per MB) interlaced p-frame encoding*/
    int16_t (*b_field_mv_table[2][2][2])[2];/*/< MV table (4MV per MB) interlaced b-frame encoding*/
    uint8_t (*p_field_select_table[2]);
    uint8_t (*b_field_select_table[2][2]);

    int mv_dir;
#define MV_DIR_BACKWARD  1
#define MV_DIR_FORWARD   2
#define MV_DIRECT        4 /*/< bidirectional mode where the difference equals the MV of the last P/S/I-Frame (mpeg4)*/

    /**motion vectors for a macroblock 
       first coordinate : 0 = forward 1 = backward
       second "         : depend on type
       third  "         : 0 = x, 1 = y
    */
    int mv[2][4][2];
    int field_select[2][2];
    int last_mv[2][2][2];             /*/< last MV, used for MV prediction in MPEG1 & B-frame MPEG4 */
    uint8_t *fcode_tab;               /*/< smallest fcode needed for each MV */

    MotionEstContext me;

    /* macroblock layer */
    int mb_x, mb_y;
    int mb_skip_run;
    int mb_intra;
    uint16_t *mb_type;           /*/< Table for candidate MB types for encoding*/
#define CANDIDATE_MB_TYPE_INTRA    0x01
#define CANDIDATE_MB_TYPE_INTER    0x02
#define CANDIDATE_MB_TYPE_INTER4V  0x04
#define CANDIDATE_MB_TYPE_SKIPED   0x08

#define CANDIDATE_MB_TYPE_DIRECT   0x10
#define CANDIDATE_MB_TYPE_FORWARD  0x20
#define CANDIDATE_MB_TYPE_BACKWARD 0x40
#define CANDIDATE_MB_TYPE_BIDIR    0x80

#define CANDIDATE_MB_TYPE_INTER_I    0x100
#define CANDIDATE_MB_TYPE_FORWARD_I  0x200
#define CANDIDATE_MB_TYPE_BACKWARD_I 0x400
#define CANDIDATE_MB_TYPE_BIDIR_I    0x800

    int block_index[6]; /*/< index to current MB in block based arrays with edges*/
    int block_wrap[6];
    uint8_t *dest[3];
    
    int *mb_index2xy;        /*/< mb_index -> mb_x + mb_y*mb_stride*/

    /** matrix transmitted in the bitstream */
    uint16_t intra_matrix[64];
    uint16_t chroma_intra_matrix[64];
    uint16_t inter_matrix[64];
    uint16_t chroma_inter_matrix[64];
#define QUANT_BIAS_SHIFT 8
    int intra_quant_bias;    /*/< bias for the quantizer */
    int inter_quant_bias;    /*/< bias for the quantizer */
    int min_qcoeff;          /*/< minimum encodable coefficient */
    int max_qcoeff;          /*/< maximum encodable coefficient */
    int ac_esc_length;       /*/< num of bits needed to encode the longest esc */
    uint8_t *intra_ac_vlc_length;
    uint8_t *intra_ac_vlc_last_length;
    uint8_t *inter_ac_vlc_length;
    uint8_t *inter_ac_vlc_last_length;
    uint8_t *luma_dc_vlc_length;
    uint8_t *chroma_dc_vlc_length;
#define UNI_AC_ENC_INDEX(run,level) ((run)*128 + (level))

    int coded_score[6];

    /** precomputed matrix (combine qscale and DCT renorm) */
    int (*q_intra_matrix)[64];
    int (*q_inter_matrix)[64];
    /** identical to the above but for MMX & these are not permutated, second 64 entries are bias*/
    uint16_t (*q_intra_matrix16)[2][64];
    uint16_t (*q_inter_matrix16)[2][64];
    int block_last_index[12];  /*/< last non zero coefficient in block*/
    /* scantables */
    ScanTable __align8 intra_scantable;
    ScanTable intra_h_scantable;
    ScanTable intra_v_scantable;
    ScanTable inter_scantable; /*/< if inter == intra then intra should be used to reduce tha cache usage*/

    void *opaque;              /*/< private data for the user*/

    /* bit rate control */
    int64_t wanted_bits;
    int64_t total_bits;
    int frame_bits;                /*/< bits used for the current frame */
    RateControlContext rc_context; /*/< contains stuff only accessed in ratecontrol.c*/

    /* statistics, used for 2-pass encoding */
    int mv_bits;
    int header_bits;
    int i_tex_bits;
    int p_tex_bits;
    int i_count;
    int f_count;
    int b_count;
    int skip_count;
    int misc_bits; /*/< cbp, mb_type*/
    int last_bits; /*/< temp var used for calculating the above vars*/

    int resync_mb_x;                 /*/< x position of last resync marker */
    int resync_mb_y;                 /*/< y position of last resync marker */

    int mb_num_left;                 /*/< number of MBs left in this video packet (for partitioned Slices only)*/
    int next_p_frame_damaged;        /*/< set if the next p frame is damaged, to avoid showing trashed b frames */

    ParseContext parse_context;

    /* mpeg4 specific */
    int time_increment_resolution;
    int64_t time;       /* time of current frame  */
    uint16_t pp_time;   /* time distance between the last 2 p,s,i frames */
    uint16_t pb_time;   /* time distance between the last b and p,s,i frame */
    int64_t last_non_b_time;

    PutBitContext tex_pb;            /*/< used for data partitioned VOPs */
    PutBitContext pb2;               /*/< used for data partitioned VOPs */

    int t_frame;                       /*/< time distance of first I -> B, used for interlaced b frames */

    /* lavc specific stuff, used to workaround bugs in libavcodec */
    int ffmpeg_version;
    int lavc_build;

    /* MSMPEG4 specific */
    int first_slice_line;  /*/< used in mpeg4 too to handle resync markers */

    int inter_intra_pred;
    int mspel;

    /* Mpeg1 specific */
    int gop_picture_number;  /*/< index of the first picture of a GOP based on fake_pic_num & mpeg1 specific */
    int last_mv_dir;         /*/< last mv_dir, used for b frame encoding */
    int broken_link;         /*/< no_output_of_prior_pics_flag*/
    uint8_t *vbv_delay_ptr;  /*/< pointer to vbv_delay in the bitstream */

    int intra_dc_precision;

    int chroma_x_shift;/*depend on pix_format, that depend on chroma_format*/
    int chroma_y_shift;

    int first_slice;

    DCTELEM (*block)[64]; /*/< points to one of the following blocks */
    DCTELEM (*blocks)[6][64]; /* for HQ mode we need to keep the best block*/
} MpegEncContext;


int DCT_common_init(MpegEncContext *s);
int MPV_common_init(MpegEncContext *s);
void MPV_common_end(MpegEncContext *s);
void MPV_decode_mb(MpegEncContext *s, DCTELEM block[12][64]);
int MPV_frame_start(MpegEncContext *s, AVCodecContext *avctx);
void MPV_frame_end(MpegEncContext *s);
int MPV_encode_init(AVCodecContext *avctx);
int MPV_encode_end(AVCodecContext *avctx);
int MPV_encode_picture(AVCodecContext *avctx, unsigned char *buf, int buf_size, void *data);

void ff_copy_bits(PutBitContext *pb, uint8_t *src, int length);
void ff_init_scantable(ScanTable *st, const uint8_t *src_scantable);
void ff_draw_horiz_band(MpegEncContext *s, int y, int h);
void ff_emulated_edge_mc(uint8_t *buf, uint8_t *src, int linesize, int block_w, int block_h, 
                                    int src_x, int src_y, int w, int h);
#define END_NOT_FOUND -100
int ff_combine_frame(ParseContext *pc, int next, uint8_t **buf, int *buf_size);

void ff_mpeg_flush(AVCodecContext *avctx);

void ff_write_quant_matrix(PutBitContext *pb, uint16_t *matrix);
int ff_find_unused_picture(MpegEncContext *s, int shared);

void ff_er_frame_start(MpegEncContext *s);
void ff_er_frame_end(MpegEncContext *s);
void ff_er_add_slice(MpegEncContext *s, int startx, int starty, int endx, int endy, int status);

void ff_init_block_index(MpegEncContext *s);

/* motion_est.c */
void ff_estimate_p_frame_motion(MpegEncContext * s,
                             int mb_x, int mb_y);
void ff_estimate_b_frame_motion(MpegEncContext * s,
                             int mb_x, int mb_y);
int ff_get_best_fcode(MpegEncContext * s, int16_t (*mv_table)[2], int type);
void ff_fix_long_mvs(MpegEncContext * s, int16_t (*mv_table)[2],
                     int f_code, int type, int truncate);
void ff_init_me(MpegEncContext *s);


/* mpeg12.c */
extern const int16_t ff_mpeg1_default_intra_matrix[64];
extern const int16_t ff_mpeg1_default_non_intra_matrix[64];
extern uint8_t ff_mpeg1_dc_scale_table[128];

void mpeg1_encode_picture_header(MpegEncContext *s, int picture_number);
void mpeg1_encode_mb(MpegEncContext *s,
                     DCTELEM block[6][64],
                     int motion_x, int motion_y);
void ff_mpeg1_encode_init(MpegEncContext *s);
void ff_mpeg1_encode_slice_header(MpegEncContext *s);
int ff_mpeg1_find_frame_end(ParseContext *pc, const uint8_t *buf,int buf_size);

/** RLTable. */
typedef struct RLTable {
  int n;                         /*/< number of entries of table_vlc minus 1 */
  int last;                      /*/< number of values for last = 0 */
  const uint16_t (*table_vlc)[2];
  const int8_t *table_run;
  const int8_t *table_level;
  uint8_t *index_run[2];         /*/< encoding only */
  int8_t *max_level[2];          /*/< encoding & decoding */
  int8_t *max_run[2];            /*/< encoding & decoding */
} RLTable;

void init_rl(RLTable *rl);

void ff_set_mpeg4_time(MpegEncContext * s, int picture_number);
void ff_set_qscale(MpegEncContext * s, int qscale);

/* rate control */
int ff_rate_control_init(MpegEncContext *s);
float ff_rate_estimate_qscale(MpegEncContext *s);
void ff_write_pass1_stats(MpegEncContext *s);
void ff_rate_control_uninit(MpegEncContext *s);
double ff_eval(char *s, double *const_value, const char **const_name,
               double (**func1)(void *, double), const char **func1_name,
               double (**func2)(void *, double, double), char **func2_name,
               void *opaque);
int ff_vbv_update(MpegEncContext *s, int frame_size);


#endif /* AVCODEC_MPEGVIDEO_H */
