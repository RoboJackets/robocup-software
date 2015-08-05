MAKE_FLAGS=--no-print-directory
all:
	mkdir -p build
	cd build; cmake .. -Wno-dev && make $(MAKE_FLAGS)

static-analysis:
	mkdir -p build/static-analysis
	cd build/static-analysis; scan-build cmake ../.. -Wno-dev -DSTATIC_ANALYSIS=ON && scan-build -o output make $(MAKE_FLAGS)

run: all
	cd run; ./soccer
run-sim: all
	cd run; ./simulator --headless &
	cd run; ./soccer -sim

# Run both C++ and python unit tests
tests: test-cpp test-python
test-cpp:
	cd build && cmake --target test-cpp .. && make $(MAKE_FLAGS) test-cpp && cd .. && run/test-cpp
test-python: all
	cd soccer/gameplay && ./run_tests.sh
pylint:
	cd soccer && pylint -E gameplay

clean:
	@if [ -d "./build" ]; then						\
		cd ./build && make $(MAKE_FLAGS) clean; 	\
		cd .. &&  rm -rf ./build;					\
	fi
	@if [ -d "./firmware/build" ]; then				\
		rm -rf ./firmware/build/robot; 				\
		rm -rf ./firmware/build/build;				\
	fi
	@echo "Cleaned"
	

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
	@mkdir -p build && cd build && cmake --target robot2015 -DTOC=FALSE .. && make $(MAKE_FLAGS) robot2015

robot2015-prog:
	@mkdir -p build && cd build && cmake --target robot2015-prog .. && make $(MAKE_FLAGS) robot2015-prog

robot2015-log:
	@mkdir -p build && cd build && cmake --target robot2015 -DTOC=TRUE .. && make $(MAKE_FLAGS) robot2015

# kicker 2015 firmware
kicker2015:
	@mkdir -p build && cd build && cmake --target kicker2015 .. && make $(MAKE_FLAGS) kicker2015
kicker2015-prog:
	@mkdir -p build && cd build && cmake --target kicker2015-prog .. && make $(MAKE_FLAGS) kicker2015-prog
	
# Base station 2015 firmware
base2015:
	@mkdir -p build && cd build && cmake --target base2015 .. && make $(MAKE_FLAGS) base2015

base2015-prog:
	@mkdir -p build && cd build && cmake --target base2015-prog .. && make $(MAKE_FLAGS) base2015-prog

# FPGA 2015
fpga2015:
	@mkdir -p build && cd build && cmake --target fpga2015 .. && make $(MAKE_FLAGS) fpga2015
	# && make $(MAKE_FLAGS) fpga2015

firmware2015: robot2015 kicker2015 fpga2015 base2015

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
