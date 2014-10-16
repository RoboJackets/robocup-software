
all: all-make

all-make:
	mkdir -p build
	cd build; cmake .. -Wno-dev && make

all-ninja:
	mkdir -p build
	cd build; cmake -GNinja .. && ninja

tests: all-make
	run/run_tests && cd soccer/gameplay; ./run_tests.sh

clean:
	rm -rf build
