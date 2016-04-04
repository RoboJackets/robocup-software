#!/bin/bash

main: main.c base2015.h
	export LD_LIBRARY_PATH=/usr/local/lib
	gcc -std=c11 main.c -o main -I /usr/include/hidapi -lhidapi-hidraw

run: main
	./main
