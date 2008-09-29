CPPFLAGS=-O2 -Wall -g3 -I.

.PHONY: all clean

all:	t

clean:
	rm -f test.hpp t

t:	t.cpp test.hpp
	$(CXX) $(CPPFLAGS) t.cpp -o $@

%.hpp:	%.xml gen_cpp Serialization.hpp
	./gen_cpp $< > $@
