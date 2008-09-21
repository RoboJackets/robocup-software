# process this file with automake to create a Makefile.in

SUBDIRS = dc1394 examples
MAINTAINERCLEANFILES = Makefile.in aclocal.m4 configure config.h.in \
	stamp-h.in
EXTRA_DIST = libdc1394-2.pc.in
# Doxygen part
MOSTLYCLEANFILES = $(DX_CLEANFILES)
include $(top_srcdir)/aminclude.am
# pkg-config part
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libdc1394-2.pc

if USE_LINUX
install-exec-hook:
	@if [ ! -c /dev/raw1394 ]; then                                 \
		echo;                                                   \
		echo "***********************************************"; \
		echo "Please install libraw1394 before installing    "; \
		echo "the 1394-Based Digital Camera Control Library  "; \
		echo "***********************************************"; \
		echo;                                                   \
	fi
endif
