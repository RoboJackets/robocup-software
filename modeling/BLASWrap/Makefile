## Makefile for Blaswrap

## Edit thise variables to configure installation path
# root to install to for `make install'
PREFIX := /usr/local
# if you use stow, root of your stow package directory
STOWBASE := /usr/local/stow
# if you use stow, a name for this stow package

## Mabe you care about these
CC := g++
CXXFLAGS := -g -I./include -fPIC
OCXXFLAGS := -O3 -march=nocona -msse3 -ftree-vectorize -fargument-noalias-global -ftree-vectorizer-verbose=3 -I./include


## You probably don't care aboute these
VERSION := 0.20090422
PROJECT := blaswrap
PROJVER := $(PROJECT)-$(VERSION)

STOWDIR := $(PROJECT)-$(VERSION)
STOWPREFIX := $(STOWBASE)/$(STOWDIR)

DISTPATH := $(HOME)/prism/tarballs
DOXPATH := $(HOME)/prism/public_html/dox




HEADERS := $(addprefix include/blaswrap/,blaswrap_base.h blaswrap_source.h def.h undef.h paste.h)

DISTFILES := blaswrap.cpp  blaswrap_source.cpp  blaswrap_test.cpp  Doxyfile  Makefile  NEWS
DISTHEAD := $(wildcard include/blaswrap/*.h) 

.PHONY: doc default clean stow dist


default: blaswrap.o libblaswrap.so blaswrap_test 

blaswrap_test: blaswrap_test.o blaswrap.o
	$(CC) -lblas -llapack -o $@ blaswrap_test.o blaswrap.o

blaswrap_memtest: blaswrap_memtest.cpp blaswrap.o
	$(CC) -lblas -llapack -o $@ blaswrap_memtest.cpp blaswrap.o

libblaswrap.so: blaswrap.o
	gcc -shared -Wl,-soname,$@ -o $@ $<

blaswrap_test.o: blaswrap_test.cpp blaswrap_source.cpp  include/blaswrap/blaswrap.h $(HEADERS)
	$(CC) $(CXXFLAGS) -c -o $@ $<

blaswrap.o: blaswrap.cpp blaswrap_source.cpp  include/blaswrap/blaswrap.h $(HEADERS)
	$(CC) $(CXXFLAGS) -c -o $@ $<

blaswrap.s: blaswrap.cpp blaswrap_source.cpp  include/blaswrap/blaswrap.h $(HEADERS)
	$(CC) $(OCXXFLAGS) -S   -o $@ $<

blaswrap_doc.h: $(HEADERS)
	cpp -C $< $@

clean:
	rm -fv blaswrap_doc.h *.o *.so blaswrap.s blaswrap_test

distclean: clean
	rm -rf doc

doc: blaswrap_doc.h
	doxygen

docul: doc
	cp -Tr doc/html $(DOXPATH)/$(PROJECT)

dist: $(DISTFILES)
	rm -rf dist
	mkdir -p dist/$(PROJVER)
	mkdir -p dist/$(PROJVER)/include/blaswrap
	cp -rv $(DISTFILES) dist/$(PROJVER)
	cp -rv $(DISTHEAD) dist/$(PROJVER)/include/blaswrap
	cd  dist &&               \
	tar --lzma -cvf $(DISTPATH)/$(PROJVER).tar.lzma $(PROJVER)

stow: libblaswrap.so
	mkdir -p $(STOWPREFIX)/include/blaswrap
	mkdir -p $(STOWPREFIX)/lib/
	install --mode 755 libblaswrap.so $(STOWPREFIX)/lib
	install --mode 644 include/blaswrap/*.h $(STOWPREFIX)/include/blaswrap
	cd $(STOWBASE) && stow $(STOWDIR)


install: libblaswrap.so
	mkdir -p $(PREFIX)/include/blaswrap
	mkdir -p $(PREFIX)/lib/
	install --mode 755 libblaswrap.so $(PREFIX)/lib
	install --mode 644 include/blaswrap/*.h $(PREFIX)/include/blaswrap
