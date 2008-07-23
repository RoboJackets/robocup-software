#
# /* LICENSE:  =========================================================================
#     RoboCup F180 Referee Box Source Code Release
#   -------------------------------------------------------------------------
#     Copyright (C) 2003 RoboCup Federation
#   -------------------------------------------------------------------------
#     This software is distributed under the GNU General Public License,
#     version 2.  If you do not have a copy of this licence, visit
#     www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
#     Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
#     in the hope that it will be useful, but WITHOUT ANY WARRANTY,
#     including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#   ------------------------------------------------------------------------- 
# 
#  */
#
CC=g++

CFLAGS := -g `pkg-config gtkmm-2.4 --cflags`
CFLAGS += -Wall -O2
LIBS += -lpthread `pkg-config gtkmm-2.4 --libs`
LDFLAGS=

install  := install -D --owner 0 --group 0
usr/bin  := $(DESTDIR)/usr/bin/
etc      := $(DESTDIR)/etc/
usr/share/applications      := $(DESTDIR)/usr/share/applications/

BINFILES := $(usr/bin)sslrefbox
ETCFILES := $(etc)/sslrefbox/referee.conf
DESKTOPFILES := $(usr/share/applications)/sslrefbox.desktop

SRCS= refereemm.cc gamecontrol.cc gameinfo.cc serial.cc udp_broadcast.cc
OBJS= $(SRCS:.cc=.o) \
#DEPENDS := $(SRCS:%.cc=.%.dep)

# all:: sslrefbox
%.o: %.cc
	$(CC) -c $(CFLAGS) $(DEFS) -o $@ $<

#include $(DEPENDS)


.%.dep: %.cc
	@echo "Generating dependencies for: " $<
	@$(CC) -M $(CFLAGS) $< > $@.tmp
	@echo -n "$@ " >$@
	@cat $@.tmp >>$@
	@rm $@.tmp

all::sslrefbox

clean::
	rm -f *.o
	rm -f sslrefbox

sslrefbox: $(OBJS)
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $^ $(LIBS)

$(BINFILES) : all
		$(install) --mode a=rx $(notdir $@) $@

$(ETCFILES) : referee.conf
		$(install) --mode a=r $(notdir $@) $@

$(DESKTOPFILES) : sslrefbox.desktop
		$(install) --mode a=r $(notdir $@) $@

install: $(BINFILES) $(ETCFILES) $(DESKTOPFILES) $(DESKTOPFILES)
