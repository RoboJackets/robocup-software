
all:
	mkdir -p build
	cd build; cmake -GNinja .. && ninja

clean:
	rm -r build
