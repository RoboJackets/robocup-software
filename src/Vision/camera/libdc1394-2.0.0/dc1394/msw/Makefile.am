noinst_LTLIBRARIES = libdc1394-msw.la

pkglinuxincludedir = $(pkgincludedir)/linux

AM_CFLAGS = -DDC1394_DLL_EXPORTS -I$(top_srcdir) -I$(srcdir)/.. -I$(WINDDK)/inc -I$(WINDDK)/src/wdm/1394/inc -I$(WINDDK)/src/wdm/1394
libdc1394_msw_la_SOURCES =  \
	control-msw.c \
	msw1394.c \
	msw1394.h \
	msw.h

# headers to be installed
#pkglinuxinclude_HEADERS = \
#	capture.h

