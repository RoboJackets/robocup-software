MAINTAINERCLEANFILES = Makefile.in
AM_CPPFLAGS = -I $(top_srcdir)

A = grab_gray_image grab_partial_image grab_color_image \
	grab_color_image2 helloworld ladybug \
	basler_sff_info basler_sff_extended_data
B = dc1394_reset_bus

if USE_LINUX
B += dc1394_vloopback
if XVOK
A += dc1394_multiview
endif
endif

noinst_PROGRAMS = $(A)
bin_PROGRAMS = $(B)

LDADD =
if USE_LINUX
LDADD += -lraw1394
endif
if USE_MACOSX
AM_LDFLAGS = -framework CoreFoundation -framework Carbon
LDADD += -lIOKit
endif

helloworld_SOURCES = helloworld.c
helloworld_LDADD = $(LDADD) ../dc1394/libdc1394.la

ladybug_SOURCES = ladybug.c
ladybug_LDADD = $(LDADD) ../dc1394/libdc1394.la

grab_gray_image_SOURCES = grab_gray_image.c
grab_gray_image_LDADD = $(LDADD) ../dc1394/libdc1394.la

grab_color_image_SOURCES = grab_color_image.c
grab_color_image_LDADD = $(LDADD) ../dc1394/libdc1394.la

grab_color_image2_SOURCES = grab_color_image2.c
grab_color_image2_LDADD = $(LDADD) ../dc1394/libdc1394.la

grab_partial_image_SOURCES = grab_partial_image.c
grab_partial_image_LDADD = $(LDADD) ../dc1394/libdc1394.la

dc1394_vloopback_SOURCES = dc1394_vloopback.c affine.c affine.h
dc1394_vloopback_LDADD = $(LDADD) -lm ../dc1394/libdc1394.la

dc1394_reset_bus_SOURCES = dc1394_reset_bus.c
dc1394_reset_bus_LDADD = $(LDADD) ../dc1394/libdc1394.la

basler_sff_info_SOURCES = basler_sff_info.c
basler_sff_info_LDADD = $(LDADD) ../dc1394/libdc1394.la

basler_sff_extended_data_SOURCES = basler_sff_extended_data.c
basler_sff_extended_data_LDADD = $(LDADD) ../dc1394/libdc1394.la

if XVOK
dc1394_multiview_CFLAGS = $(X_CFLAGS) $(XV_CFLAGS)
dc1394_multiview_SOURCES = dc1394_multiview.c
dc1394_multiview_LDADD = $(LDADD) ../dc1394/libdc1394.la $(X_LIBS) $(X_PRE_LIBS) $(XV_LIBS) -lX11 $(X_EXTRA_LIBS)
endif
