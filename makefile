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

# general target for calling the hardware tests
robot2015-test:
	mkdir -p build && cd build && cmake -DHW_TEST_UNIT:STRING=$(HW_TEST_UNIT) --target robot2015-test .. && make $(MAKE_FLAGS) robot2015-test
robot2015-test-prog:
	mkdir -p build && cd build && cmake -DHW_TEST_UNIT:STRING=$(HW_TEST_UNIT) --target robot2015-test-prog .. && make $(MAKE_FLAGS) robot2015-test-prog

# I2C bus hardware test
robot2015-i2c: robot2015-i2c-set robot2015-test
robot2015-i2c-prog: robot2015-i2c-set robot2015-test-prog
robot2015-i2c-set:
HW_TEST_UNIT := i2c

# IO expander hardware test
robot2015-io-expander: robot2015-io-expander-set robot2015-test
robot2015-io-expander-prog: robot2015-io-expander-set robot2015-test-prog
robot2015-io-expander-set:
HW_TEST_UNIT := io-expander

# fpga hardware test
#robot2015-fpga: fpga2015 robot2015-fpga-set robot2015-test
#robot2015-fpga-prog: fpga2015-prog robot2015-fpga-set robot2015-test-prog
robot2015-fpga: robot2015-fpga-set robot2015-test
robot2015-fpga-prog: robot2015-fpga-set robot2015-test-prog
robot2015-fpga-set:
HW_TEST_UNIT := fpga

# piezo buzzer hardware test
robot2015-piezo: robot2015-piezo-set robot2015-test
robot2015-piezo-prog: robot2015-piezo-set robot2015-test-prog
robot2015-piezo-set:
HW_TEST_UNIT := piezo

# kicker 2015 firmware
kicker2015:
	mkdir -p build && cd build && cmake --target kicker2015 .. && make $(MAKE_FLAGS) kicker2015
kicker2015-prog:
	mkdir -p build && cd build && cmake --target kicker2015-prog .. && make $(MAKE_FLAGS) kicker2015-prog

# fpga 2015 synthesis
fpga2015:
	mkdir -p build && cd build && cmake --target fpga2015 .. && make $(MAKE_FLAGS) fpga2015
fpga2015-prog:
	mkdir -p build && cd build && cmake --target fpga2015 .. && make $(MAKE_FLAGS) fpga2015-prog

# Build all of the 2015 firmware for a robot, and/or move all of the binaries over to the mbed
firmware2015: robot2015 kicker2015 fpga2015 
firmware2015-prog: robot2015-prog kicker2015-prog fpga2015-prog
	
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
