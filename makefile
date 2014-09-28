
all: all-make

all-make:
	mkdir -p build
	cd build; cmake .. && make

all-ninja:
	mkdir -p build
	cd build; cmake -GNinja .. && ninja

tests:
	mkdir -p build;
	cd build; cmake .. && make
	run/run_tests

clean:
	rm -r build
