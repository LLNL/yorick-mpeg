#
# This bears essentially no resemblance to the original,
# which bore the following notice:
# libavcodec Makefile
# (c) 2000-2003 Fabrice Bellard
#

MAKE=make
CC=gcc
AR=ar
RANLIB=ranlib
STRIP=strip
OPTFLAGS=-O3 -g -ansi -pedantic -Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
LIBPREF=lib
LIBSUF=.a

# following is necessary on Linux systems to avoid SELinux problems
# if you link this to a shared library (like yompeg.so)
# see  http://people.redhat.com/drepper/textrelocs.html
PICFLAG=-fPIC

# NOTE: -I.. is needed to include config.h
CFLAGS=$(OPTFLAGS) $(PICFLAG) -DHAVE_AV_CONFIG_H -I.. -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_GNU_SOURCE

OBJS= common.o utils.o mem.o dsputil.o mpegvideo.o mpeg12.o imgconvert.o \
      ratecontrol.o motion_est.o integer.o simple_idct.o jfdctint.o

ASM_OBJS=

LIB= $(LIBPREF)avcodec$(LIBSUF)

all: $(LIB)

$(LIB): $(OBJS)
	rm -f $@
	$(AR) rc $@ $(OBJS)
	$(RANLIB) $@

# common.h->../config.h bswap.h
# avcodec.h->common.h
# dsputil.h->common.h avcodec.h
# mpegvideo.h->dsputil.h
AVCODEC_H=avcodec.h common.h bswap.h ../config.h
common.o: $(AVCODEC_H)
utils.o: integer.h mpegvideo.h dsputil.h $(AVCODEC_H)
mem.o: $(AVCODEC_H)
dsputil.o: simple_idct.h dsputil.h $(AVCODEC_H)
mpegvideo.o: mpegvideo.h simple_idct.h dsputil.h $(AVCODEC_H)
imgconvert.o: dsputil.h $(AVCODEC_H)
mpeg12.o: mpegvideo.h dsputil.h $(AVCODEC_H)
ratecontrol.o: mpegvideo.h dsputil.h $(AVCODEC_H)
motion_est.o: mpegvideo.h dsputil.h $(AVCODEC_H)
integer.o: integer.h common.h bswap.h ../config.h
simple_idct.o: simple_idct.h dsputil.h $(AVCODEC_H)
jfdctint.o: dsputil.h $(AVCODEC_H)

clean:
	rm -f *.o *.d *~ .depend $(LIB) *.so

distclean: clean
	rm -f Makefile.bak .depend
