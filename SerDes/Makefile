OBJS=t.o Sender.o Receiver.o
CPPFLAGS=-O2 -Wall -g3 -I.
LDFLAGS=-lpthread

.PHONY: all clean

all:	t

clean:
	rm -f test.hpp t $(OBJS)

t:	$(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $@

%.hpp:	%.xml gen_cpp Serialization.hpp
	./gen_cpp $< > $@

t.o: test.hpp Sender.hpp Receiver.hpp
