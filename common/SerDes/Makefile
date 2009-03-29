OBJS=t.o
CPPFLAGS=-O2 -Wall -g3 -I. -I../Network
LDFLAGS=-lpthread

.PHONY: all clean

all:	t

clean:
	rm -f test.hpp t $(OBJS)

t:	$(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $@

%.hpp:	%.xml gen_cpp
	./gen_cpp $< > $@

t.o: test.hpp Serialization.hpp
