noinst_LTLIBRARIES = libdc1394-linux.la

pkglinuxincludedir = $(pkgincludedir)/linux

AM_CFLAGS =-I$(top_srcdir) -I$(top_srcdir)/dc1394
libdc1394_linux_la_SOURCES =  \
	control.c \
	capture.c \
	capture.h \
	kernel-video1394.h \
	linux.h 

# headers to be installed
pkglinuxinclude_HEADERS = \
	capture.h

