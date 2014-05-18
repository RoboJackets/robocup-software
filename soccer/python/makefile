
# This makefile just compiles the module extension
# using setup.py and checks to make sure importing it works.
# Other than that, this makefile is irrelevant to what
# we're trying to do with python for highlevel soccer stuff

all:
	python3 setup.py build_ext

run: all
	cd build/lib*; python3 -c 'import robocup'

clean:
	rm -rf build
