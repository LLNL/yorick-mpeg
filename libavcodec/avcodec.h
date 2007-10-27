#ifndef AVCODEC_H
#define AVCODEC_H

/**
 * @file avcodec.h
 * external api header.
 */


#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include <sys/types.h> /* size_t */

#define FFMPEG_VERSION_INT     0x000409
#define FFMPEG_VERSION         "0.4.9-pre1"

#define LIBAVCODEC_VERSION_INT FFMPEG_VERSION_INT
#define LIBAVCODEC_VERSION     FFMPEG_VERSION

#define AV_NOPTS_VALUE int64_t_C(0x8000000000000000)
#define AV_TIME_BASE 1000000

typedef struct AVRational{
    int num; 
    int den;
} AVRational;

#define av_q2d(a) ((double)(a).num/(double)(a).den)

enum CodecID {
  CODEC_ID_NONE, 
  CODEC_ID_MPEG1VIDEO
};

enum CodecType {
  CODEC_TYPE_UNKNOWN = -1,
  CODEC_TYPE_VIDEO
};

/**
 * Pixel format. Notes: 
 *
 * PIX_FMT_RGBA32 is handled in an endian-specific manner. A RGBA
 * color is put together as:
 *  (A << 24) | (R << 16) | (G << 8) | B
 * This is stored as BGRA on little endian CPU architectures and ARGB on
 * big endian CPUs.
 *
 * When the pixel format is palettized RGB (PIX_FMT_PAL8), the palettized
 * image data is stored in AVFrame.data[0]. The palette is transported in
 * AVFrame.data[1] and, is 1024 bytes long (256 4-byte entries) and is
 * formatted the same as in PIX_FMT_RGBA32 described above (i.e., it is
 * also endian-specific). Note also that the individual RGB palette
 * components stored in AVFrame.data[1] should be in the range 0..255.
 * This is important as many custom PAL8 video codecs that were designed
 * to run on the IBM VGA graphics adapter use 6-bit palette components.
 */
enum PixelFormat {
  PIX_FMT_YUV420P,
  PIX_FMT_YUV422,   /* unused, but keeps numbering same as ffmpeg */
  PIX_FMT_RGB24,
  PIX_FMT_NB
};

/* motion estimation type, EPZS by default */
enum Motion_Est_ID {
  ME_ZERO = 1,
  ME_FULL,
  ME_LOG,
  ME_PHODS,
  ME_EPZS,
  ME_X1
};

#define FF_MAX_B_FRAMES 8

/**
 * Codec uses get_buffer() for allocating buffers.
 * direct rendering method 1
 */
/** codec has a non zero delay and needs to be feeded with NULL at the end to get the delayed data */
#define CODEC_CAP_DELAY           0x0020

/* the following defines might change, so dont expect compatibility if u use them */
#define MB_TYPE_INTRA4x4   0x0001

#define FF_COMMON_FRAME \
    /**\
     * pointer to the picture planes.\
     * this might be different from the first allocated byte\
     * - encoding: \
     * - decoding: \
     */\
    uint8_t *data[4];\
    int linesize[4];\
    /**\
     * pointer to the first allocated byte of the picture. can be used in get_buffer/release_buffer\
     * this isnt used by lavc unless the default get/release_buffer() is used\
     * - encoding: \
     * - decoding: \
     */\
    uint8_t *base[4];\
    /**\
     * 1 -> keyframe, 0-> not\
     * - encoding: set by lavc\
     * - decoding: set by lavc\
     */\
    int key_frame;\
\
    /**\
     * picture type of the frame, see ?_TYPE below.\
     * - encoding: set by lavc for coded_picture (and set by user for input)\
     * - decoding: set by lavc\
     */\
    int pict_type;\
\
    /**\
     * presentation timestamp in AV_TIME_BASE (=micro seconds currently) (time when frame should be shown to user)\
     * if AV_NOPTS_VALUE then the frame_rate will be used as reference\
     * - encoding: MUST be set by user\
     * - decoding: set by lavc\
     */\
    int64_t pts;\
\
    /**\
     * picture number in bitstream order.\
     * - encoding: set by\
     * - decoding: set by lavc\
     */\
    int coded_picture_number;\
    /**\
     * picture number in display order.\
     * - encoding: set by\
     * - decoding: set by lavc\
     */\
    int display_picture_number;\
\
    /**\
     * quality (between 1 (good) and FF_LAMBDA_MAX (bad)) \
     * - encoding: set by lavc for coded_picture (and set by user for input)\
     * - decoding: set by lavc\
     */\
    int quality; \
\
    /**\
     * buffer age (1->was last buffer and dint change, 2->..., ...).\
     * set to INT_MAX if the buffer has not been used yet \
     * - encoding: unused\
     * - decoding: MUST be set by get_buffer()\
     */\
    int age;\
\
    /**\
     * is this picture used as reference\
     * - encoding: unused\
     * - decoding: set by lavc (before get_buffer() call))\
     */\
    int reference;\
\
    /**\
     * Motion vector table\
     * - encoding: set by user\
     * - decoding: set by lavc\
     */\
    int16_t (*motion_val[2])[2];\
\
    /**\
     * Macroblock type table\
     * mb_type_base + mb_width + 2\
     * - encoding: set by user\
     * - decoding: set by lavc\
     */\
    uint32_t *mb_type;\
\
    /**\
     * type of the buffer (to keep track of who has to dealloc data[*])\
     * - encoding: set by the one who allocs it\
     * - decoding: set by the one who allocs it\
     * Note: user allocated (direct rendering) & internal buffers can not coexist currently\
     */\
    int type;\
\
    /**\
     * \
     */\
    int qscale_type;\
\
    /**\
     * Motion referece frame index\
     * - encoding: set by user\
     * - decoding: set by lavc\
     */\
    int8_t *ref_index[2];

#define FF_BUFFER_TYPE_INTERNAL 1
#define FF_BUFFER_TYPE_USER     2 /*/< Direct rendering buffers (image is (de)allocated by user)*/
#define FF_BUFFER_TYPE_SHARED   4 /*/< buffer from somewher else, dont dealloc image (data/base), all other tables are not shared*/
#define FF_BUFFER_TYPE_COPY     8 /*/< just a (modified) copy of some other buffer, dont dealloc anything*/


#define FF_I_TYPE 1 /* Intra*/
#define FF_P_TYPE 2 /* Predicted*/
#define FF_B_TYPE 3 /* Bi-dir predicted*/

/**
 * Audio Video Frame.
 */
typedef struct AVFrame {
    FF_COMMON_FRAME
} AVFrame;

#define DEFAULT_FRAME_RATE_BASE 1001000

/**
 * Used by av_log
 */
typedef struct AVCLASS AVClass;
struct AVCLASS {
  const char* class_name;
  const char* (*item_name)(void*);
  /* actually passing a pointer to an AVCodecContext
     or AVFormatContext, which begin with an AVClass.
     Needed because av_log is in libavcodec and has no visibility
     of AVIn/OutputFormat */
};

/**
 * main external api structure.
 */
typedef struct AVCodecContext {
  /**
   * Info on struct for av_log
   * - set by avcodec_alloc_context
   */
  AVClass *av_class;
  /**
   * the average bitrate.
   * - encoding: set by user. unused for constant quantizer encoding
   * - decoding: set by lavc. 0 or some bitrate if this info is available in the stream 
   */
  int bit_rate;

  /**
   * number of bits the bitstream is allowed to diverge from the reference.
   *           the reference can be CBR (for CBR pass1) or VBR (for pass2)
   * - encoding: set by user. unused for constant quantizer encoding
   * - decoding: unused
   */
  int bit_rate_tolerance; 

  /**
   * CODEC_FLAG_*.
   * - encoding: set by user.
   * - decoding: set by user.
   */
  int flags;

  /* video only */
  /**
   * frames per sec multiplied by frame_rate_base.
   * for variable fps this is the precission, so if the timestamps 
   * can be specified in msec precssion then this is 1000*frame_rate_base
   * - encoding: MUST be set by user
   * - decoding: set by lavc. 0 or the frame_rate if available
   */
  int frame_rate;
    
  /**
   * width / height.
   * - encoding: MUST be set by user. 
   * - decoding: set by user if known, codec should override / dynamically change if needed
   */
  int width, height;

#define FF_ASPECT_EXTENDED 15

  /**
   * the number of pictures in a group of pitures, or 0 for intra_only.
   * - encoding: set by user.
   * - decoding: unused
   */
  int gop_size;

  /**
   * pixel format, see PIX_FMT_xxx.
   * - encoding: FIXME: used by ffmpeg to decide whether an pix_fmt
   *                    conversion is in order. This only works for
   *                    codecs with one supported pix_fmt, we should
   *                    do something for a generic case as well.
   * - decoding: set by lavc.
   */
  enum PixelFormat pix_fmt;

  /* the following data should not be initialized */
  int frame_size;     /*/< in samples, initialized when calling 'init' */
  int frame_number;   /*/< audio or video frame number */
  int real_pict_num;  /*/< returns the real picture number of previous encoded frame */

  /**
   * number of frames the decoded output will be delayed relative to 
   * the encoded input.
   * - encoding: set by lavc.
   * - decoding: unused
   */
  int delay;

  /**
   * minimum quantizer.
   * - encoding: set by user.
   * - decoding: unused
   */
  int qmin;

  /**
   * maximum quantizer.
   * - encoding: set by user.
   * - decoding: unused
   */
  int qmax;

  /**
   * maximum quantizer difference etween frames.
   * - encoding: set by user.
   * - decoding: unused
   */
  int max_qdiff;

  /**
   * maximum number of b frames between non b frames.
   * note: the output will be delayed by max_b_frames+1 relative to the input
   * - encoding: set by user.
   * - decoding: unused
   */
  int max_b_frames;

  /**
   * qscale factor between ip and b frames.
   * - encoding: set by user.
   * - decoding: unused
   */
  float b_quant_factor;

  /** obsolete FIXME remove */
  int b_frame_strategy;

  struct AVCodec *codec;
    
  void *priv_data;

  /* statistics, used for 2-pass encoding */
  int mv_bits;
  int header_bits;
  int i_tex_bits;
  int p_tex_bits;
  int i_count;
  int p_count;
  int skip_count;
  int misc_bits;

  /**
   * number of bits used for the previously encoded frame.
   * - encoding: set by lavc
   * - decoding: unused
   */
  int frame_bits;

  char codec_name[32];
  enum CodecType codec_type; /* see CODEC_TYPE_xxx */
  enum CodecID codec_id; /* see CODEC_ID_xxx */

  /**
   * qscale offset between ip and b frames.
   * if > 0 then the last p frame quantizer will be used (q= lastp_q*factor+offset)
   * if < 0 then normal ratecontrol will be done (q= -normal_q*factor+offset)
   * - encoding: set by user.
   * - decoding: unused
   */
  float b_quant_offset;

  /**
   * called at the beginning of each frame to get a buffer for it.
   * if pic.reference is set then the frame will be read later by lavc
   * avcodec_align_dimensions() should be used to find the required width and
   * height, as they normally need to be rounded up to the next multiple of 16
   * - encoding: unused
   * - decoding: set by lavc, user can override
   */
  int (*get_buffer)(struct AVCodecContext *c, AVFrame *pic);
    
  /**
   * called to release buffers which where allocated with get_buffer.
   * a released buffer can be reused in get_buffer()
   * pic.data[*] must be set to NULL
   * - encoding: unused
   * - decoding: set by lavc, user can override
   */
  void (*release_buffer)(struct AVCodecContext *c, AVFrame *pic);

  /**
   * maximum bitrate.
   * - encoding: set by user.
   * - decoding: unused
   */
  int rc_max_rate;
    
  /**
   * minimum bitrate.
   * - encoding: set by user.
   * - decoding: unused
   */
  int rc_min_rate;
    
  /**
   * decoder bitstream buffer size.
   * - encoding: set by user.
   * - decoding: unused
   */
  int rc_buffer_size;
  float rc_buffer_aggressivity;

  /**
   * qscale factor between p and i frames.
   * if > 0 then the last p frame quantizer will be used (q= lastp_q*factor+offset)
   * if < 0 then normal ratecontrol will be done (q= -normal_q*factor+offset)
   * - encoding: set by user.
   * - decoding: unused
   */
  float i_quant_factor;

  /**
   * qscale offset between p and i frames.
   * - encoding: set by user.
   * - decoding: unused
   */
  float i_quant_offset;
    
  /**
   * initial complexity for pass1 ratecontrol.
   * - encoding: set by user.
   * - decoding: unused
   */
  float rc_initial_cplx;

  /**
   * dct algorithm, see FF_DCT_* below.
   * - encoding: set by user
   * - decoding: unused
   */
  int dct_algo;
#define FF_DCT_AUTO    0
#define FF_DCT_FASTINT 1
#define FF_DCT_INT     2
#define FF_DCT_MMX     3
#define FF_DCT_MLIB    4
#define FF_DCT_ALTIVEC 5
#define FF_DCT_FAAN    6

  /**
   * slice count.
   * - encoding: set by lavc
   * - decoding: set by user (or 0)
   */
  int slice_count;
  /**
   * slice offsets in the frame in bytes.
   * - encoding: set/allocated by lavc
   * - decoding: set/allocated by user (or NULL)
   */
  int *slice_offset;

  /**
   * bits per sample/pixel from the demuxer (needed for huffyuv).
   * - encoding: set by lavc
   * - decoding: set by user
   */
  int bits_per_sample;

  /**
   * sample aspect ratio (0 if unknown).
   * numerator and denominator must be relative prime and smaller then 256 for some video standards
   * - encoding: set by user.
   * - decoding: set by lavc.
   */
  AVRational sample_aspect_ratio;

  /**
   * the picture in the bitstream.
   * - encoding: set by lavc
   * - decoding: set by lavc
   */
  AVFrame *coded_frame;

#define FF_CMP_SAD  0
#define FF_CMP_VSAD 8

  /**
   * frame_rate_base.
   * for variable fps this is 1
   * - encoding: set by user.
   * - decoding: set by lavc.
   * @todo move this after frame_rate
   */

  int frame_rate_base;
  /**
   * intra quantizer bias.
   * - encoding: set by user.
   * - decoding: unused
   */
  int intra_quant_bias;
#define FF_DEFAULT_QUANT_BIAS 999999

  /**
   * inter quantizer bias.
   * - encoding: set by user.
   * - decoding: unused
   */
  int inter_quant_bias;

  /**
   * internal_buffer count. 
   * Dont touch, used by lavc default_get_buffer()
   */
  int internal_buffer_count;

  /**
   * internal_buffers. 
   * Dont touch, used by lavc default_get_buffer()
   */
  void *internal_buffer;

#define FF_LAMBDA_SHIFT 7
#define FF_LAMBDA_SCALE (1<<FF_LAMBDA_SHIFT)
#define FF_QP2LAMBDA 118 /*/< factor to convert from H.263 QP to lambda*/
#define FF_LAMBDA_MAX (256*128-1)

  /**
   * custom intra quantization matrix
   * - encoding: set by user, can be NULL
   * - decoding: set by lavc
   */
  uint16_t *intra_matrix;

  /**
   * custom inter quantization matrix
   * - encoding: set by user, can be NULL
   * - decoding: set by lavc
   */
  uint16_t *inter_matrix;

  /**
   * minimum lagrange multipler
   * - encoding: set by user.
   * - decoding: unused
   */
  int lmin;

  /**
   * maximum lagrange multipler
   * - encoding: set by user.
   * - decoding: unused
   */
  int lmax;

  /**
   * noise reduction strength
   * - encoding: set by user.
   * - decoding: unused
   */
  int noise_reduction;

  /**
   * number of bits which should be loaded into the rc buffer before decoding starts
   * - encoding: set by user.
   * - decoding: unused
   */
  int rc_initial_buffer_occupancy;

  /**
   * CODEC_FLAG2_*.
   * - encoding: set by user.
   * - decoding: set by user.
   */
  int flags2;

  /**
   * simulates errors in the bitstream to test error concealment.
   * - encoding: set by user.
   * - decoding: unused.
   */
  int error_rate;

  /**
   * precision of the intra dc coefficient - 8.
   * - encoding: set by user
   * - decoding: unused
   */
  int intra_dc_precision;
} AVCodecContext;



/**
 * AVCodec.
 */
typedef struct AVCodec {
  const char *name;
  enum CodecType type;
  int id;
  int priv_data_size;
  int (*init)(AVCodecContext *);
  int (*encode)(AVCodecContext *, uint8_t *buf, int buf_size, void *data);
  int (*close)(AVCodecContext *);
  int (*decode)(AVCodecContext *, void *outdata, int *outdata_size,
                uint8_t *buf, int buf_size);
  int capabilities;
  struct AVCodec *next;
  void (*flush)(AVCodecContext *);
  const AVRational *supported_framerates;
  const enum PixelFormat *pix_fmts;
} AVCodec;

/**
 * four components are given, that's all.
 * the last component is alpha
 */
typedef struct AVPicture {
    uint8_t *data[4];
    int linesize[4];       /*/< number of bytes per line*/
} AVPicture;

extern AVCodec mpeg1video_encoder;

/**
 * Allocate memory for a picture.  Call avpicture_free to free it.
 *
 * @param picture the picture to be filled in.
 * @param pix_fmt the format of the picture.
 * @param width the width of the picture.
 * @param height the height of the picture.
 * @return 0 if successful, -1 if not.
 */
int avpicture_alloc(AVPicture *picture, int pix_fmt, int width, int height);

/* Free a picture previously allocated by avpicture_alloc. */
void avpicture_free(AVPicture *picture);

int avpicture_fill(AVPicture *picture, uint8_t *ptr,
                   int pix_fmt, int width, int height);
int avpicture_layout(const AVPicture* src, int pix_fmt, int width, int height,
                     unsigned char *dest, int dest_size);
int avpicture_get_size(int pix_fmt, int width, int height);
void avcodec_get_chroma_sub_sample(int pix_fmt, int *h_shift, int *v_shift);
enum PixelFormat avcodec_get_pix_fmt(const char* name);

/* convert among pixel formats */
int img_convert(AVPicture *dst, int dst_pix_fmt,
                const AVPicture *src, int pix_fmt, 
                int width, int height);

/* external high level API */

extern AVCodec *first_avcodec;

/* returns LIBAVCODEC_VERSION_INT constant */
unsigned avcodec_version(void);
/* returns LIBAVCODEC_BUILD constant */
unsigned avcodec_build(void);
void avcodec_init(void);

void register_avcodec(AVCodec *format);
AVCodec *avcodec_find_encoder(enum CodecID id);
void avcodec_string(char *buf, int buf_size, AVCodecContext *enc, int encode);

void avcodec_get_context_defaults(AVCodecContext *s);
AVCodecContext *avcodec_alloc_context(void);
void avcodec_get_frame_defaults(AVFrame *pic);
AVFrame *avcodec_alloc_frame(void);

int avcodec_default_get_buffer(AVCodecContext *s, AVFrame *pic);
void avcodec_default_release_buffer(AVCodecContext *s, AVFrame *pic);

void avcodec_align_dimensions(AVCodecContext *s, int *width, int *height);

/**
 * opens / inits the AVCodecContext.
 * not thread save!
 */
int avcodec_open(AVCodecContext *avctx, AVCodec *codec);

int avcodec_encode_video(AVCodecContext *avctx, uint8_t *buf, int buf_size, 
                         const AVFrame *pict);

int avcodec_close(AVCodecContext *avctx);

/**
 * reduce a fraction.
 * this is usefull for framerate calculations
 * @param max the maximum allowed for dst_nom & dst_den
 * @return 1 if exact, 0 otherwise
 */
int av_reduce(int *dst_nom, int *dst_den, int64_t nom, int64_t den, int64_t max);

/**
 * rescale a 64bit integer.
 * a simple a*b/c isnt possible as it can overflow
 */
int64_t av_rescale(int64_t a, int64_t b, int64_t c);


/* memory */
void *av_malloc(unsigned int size);
void *av_mallocz(unsigned int size);
void *av_realloc(void *ptr, unsigned int size);
void av_free(void *ptr);

void av_freep(void *ptr);
void *av_fast_realloc(void *ptr, unsigned int *size, unsigned int min_size);

/* av_log API */

#include <stdarg.h>

#define AV_LOG_QUIET -1
#define AV_LOG_ERROR 0
#define AV_LOG_INFO 1
#define AV_LOG_DEBUG 2

#ifdef __GNUC__
extern void av_log(void*, int level, const char *fmt, ...) __attribute__ ((__format__ (__printf__, 3, 4)));
#else
extern void av_log(void*, int level, const char *fmt, ...);
#endif

extern void av_vlog(void*, int level, const char *fmt, va_list);
extern int av_log_get_level(void);
extern void av_log_set_level(int);
extern void av_log_set_callback(void (*)(void*, int, const char*, va_list));

/* endian macros */
#if !defined(BE_16) || !defined(BE_32) || !defined(LE_16) || !defined(LE_32)
#define BE_16(x)  ((((uint8_t*)(x))[0] << 8) | ((uint8_t*)(x))[1])
#define BE_32(x)  ((((uint8_t*)(x))[0] << 24) | \
                   (((uint8_t*)(x))[1] << 16) | \
                   (((uint8_t*)(x))[2] << 8) | \
                    ((uint8_t*)(x))[3])
#define LE_16(x)  ((((uint8_t*)(x))[1] << 8) | ((uint8_t*)(x))[0])
#define LE_32(x)  ((((uint8_t*)(x))[3] << 24) | \
                   (((uint8_t*)(x))[2] << 16) | \
                   (((uint8_t*)(x))[1] << 8) | \
                    ((uint8_t*)(x))[0])
#endif

#ifdef __cplusplus
}
#endif

#endif /* AVCODEC_H */
