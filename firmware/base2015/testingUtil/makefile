all: main

main: main.cpp base2015.h
	export LD_LIBRARY_PATH=/usr/local/lib
	g++ -std=c++11 main.cpp -o main -I /usr/include/hidapi -lhidapi-hidraw

run: main
	./main
