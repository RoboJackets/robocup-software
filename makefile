
all: all-make

all-make:
	mkdir -p build
	cd build; cmake .. && make

all-ninja:
	mkdir -p build
	cd build; cmake -GNinja .. && ninja

clean:
	rm -r build
