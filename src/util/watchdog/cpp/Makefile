CC=gcc
CXX=g++
CXXFLAGS:=-O2 -g3 -Wall $(shell pkg-config --cflags libxml-2.0)
LDFLAGS:=-lpthread -lstatgrab $(shell pkg-config --libs libxml-2.0)

TARGET=watchdog
SRCS=watchdog.cpp process.cpp stats.cpp comm.cpp errno_error.cpp log.cpp config.cpp
OBJS:=$(addsuffix .o, $(basename $(SRCS)))

.PHONY: all clean run install

all:		$(TARGET)

clean:
		rm -f $(TARGET) $(OBJS) depend

depend:		$(SRCS)
		$(CXX) $(CXXFLAGS) -MM $(SRCS) > depend

run:		$(TARGET)
		./$(TARGET)

install:	$(TARGET) ../www/html/log
		cp -f $(TARGET) $(UTILS_DIR)/bin/

../www/html/log:
		ln -fs /tmp/watchdog $@

$(TARGET):	$(OBJS)
		$(CXX) $(LDFLAGS) -o $(TARGET) $(OBJS)

include depend

