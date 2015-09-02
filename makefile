MAKE_FLAGS=--no-print-directory
all:
	mkdir -p build
	cd build; cmake .. -Wno-dev && make $(MAKE_FLAGS)

run: all
	cd run; ./soccer
run-sim: all
	-pkill -f './simulator --headless'
	cd run; ./simulator --headless &
	cd run; ./soccer -sim
run-sim2play: all
	-pkill -f './simulator --headless'
	cd run; ./simulator --headless &
	cd run; ./soccer -sim -y & ./soccer -sim -b

# Run both C++ and python unit tests
tests: test-cpp test-python
test-cpp: test-soccer test-firmware
test-soccer:
	mkdir -p build
	cd build && cmake --target test-soccer .. && make $(MAKE_FLAGS) test-soccer && cd .. && run/test-soccer
test-firmware:
	mkdir -p build
	cd build && cmake --target test-firmware .. && make $(MAKE_FLAGS) test-firmware && cd .. && run/test-firmware
test-python: all
	cd soccer/gameplay && ./run_tests.sh
pylint:
	cd soccer && pylint -E gameplay

clean:
	cd build && make $(MAKE_FLAGS) clean || true
	rm -rf build

# Robot firmware (both 2008/2011)
robot:
	cd firmware; scons robot
robot-prog:
	cd firmware; scons robot; sudo scons robot-prog
robot-ota:
	cd firmware; scons robot; scons robot-ota
robot-prog-samba:
	cd firmware; scons robot; sudo scons robot-prog-samba

# robot 2015 firmware
robot2015:
	mkdir -p build && cd build && cmake --target robot2015 .. && make $(MAKE_FLAGS) robot2015

robot2015-prog:
	mkdir -p build && cd build && cmake --target robot2015-prog .. && make $(MAKE_FLAGS) robot2015-prog

# Base station 2015 firmware
base2015:
	mkdir -p build && cd build && cmake --target base2015 .. && make $(MAKE_FLAGS) base2015

base2015-prog:
	mkdir -p build && cd build && cmake --target base2015-prog .. && make $(MAKE_FLAGS) base2015-prog

# Robot FPGA
fpga2011:
	cd firmware; scons fpga2011
# program the fpga over the usb spi interface
fpga2011-spi:
	cd firmware; scons fpga2011; sudo scons fpga2011-spi
# program the fpga over the jtag interface
fpga2011-jtag:
	cd firmware; scons fpga2011; sudo scons fpga2011-jtag

# USB Radio Base Station
base2011:
	cd firmware; scons base2011
base2011-prog:
	cd firmware; scons base2011; sudo scons base2011-prog


static-analysis:
	mkdir -p build/static-analysis
	cd build/static-analysis; scan-build cmake ../.. -Wno-dev -DSTATIC_ANALYSIS=ON && scan-build -o output make $(MAKE_FLAGS)
modernize:
	# Runs CMake with a sepcial flag telling it to output the compilation
	# database, which lists the files to be compiled and the flags passed to
	# the compiler for each one. Then runs clang-modernize, using the
	# compilation database as input, on all c/c++ files in the repo.
	mkdir -p build
	cd build; cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -Wno-dev && make $(MAKE_FLAGS)
	# You can pass specific flags to clang-modernize if you want it to only run some types of
	# transformations, rather than all transformations that it's capable of.
	# See `clang-modernize --help` for more info.
	clang-modernize -p build -include=common,logging,simulator,soccer

STYLE_EXCLUDE_DIRS=build \
	external \
	firmware/robot/cpu/at91sam7s256 \
	firmware/robot/cpu/at91sam7s321 \
	firmware/robot/cpu/at91sam7s64 \
	firmware/robot/cpu/usb \
	firmware/robot/cpu/invensense \
	firmware/robot2015 \
	firmware/common2015
# automatically format code according to our style config defined in .clang-format
pretty:
	stylize --diffbase=master --clang_style=file --yapf_style=file --exclude_dirs $(STYLE_EXCLUDE_DIRS)
# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	stylize --diffbase=master --clang_style=file --yapf_style=file --exclude_dirs $(STYLE_EXCLUDE_DIRS) --check
