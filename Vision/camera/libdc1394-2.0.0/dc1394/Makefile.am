MAINTAINERCLEANFILES = Makefile.in
lib_LTLIBRARIES = libdc1394.la

SUBDIRS = $(dc1394platform) vendor
DIST_SUBDIRS = linux macosx vendor juju msw
AM_CFLAGS = $(platform_CFLAGS) -I$(top_srcdir)

libdc1394_la_LDFLAGS = $(platform_LDFLAGS) -version-info @lt_current@:@lt_revision@:@lt_age@

libdc1394_la_SOURCES =  \
	control.c 	\
	control.h	\
	enumeration.c   \
	platform.h      \
	capture.c       \
	offsets.h	\
	format7.c       \
	register.c      \
	register.h      \
	utils.c         \
	utils.h         \
	internal.c      \
	internal.h      \
	conversions.c   \
	conversions.h   \
	bayer.c         \
	log.c		\
	log.h		\
	iso.c 		\
	iso.h

libdc1394_la_LIBADD = \
	$(dc1394platform)/libdc1394-$(dc1394platform).la \
	vendor/libdc1394-vendor.la

# headers to be installed
pkginclude_HEADERS =  	\
	dc1394.h	\
	types.h		\
	camera.h	\
	control.h     	\
	capture.h	\
	video.h		\
	format7.h	\
	utils.h       	\
	conversions.h 	\
	register.h    	\
	log.h	      	\
	iso.h
