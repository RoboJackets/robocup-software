
all: all-make

all-make:
	mkdir -p build
	cd build; cmake .. && make

all-ninja:
	mkdir -p build
	cd build; cmake -GNinja .. && ninja

test:
	mkdir -p build;
	cd build; cmake .. && make test

clean:
	rm -r build
