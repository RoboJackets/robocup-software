
all:
	mkdir -p build
	cd build; cmake .. -Wno-dev && make

run: all
	cd run; ./soccer
run-sim: all
	cd run; ./simulator &
	cd run; ./soccer -sim

# Run both C++ and python unit tests
tests: test-cpp test-python
test-cpp: all
	cd build && make test-cpp && cd .. && run/test-cpp
test-python: all
	cd soccer/gameplay && ./run_tests.sh

clean:
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
robot2015: all
	cd build && make robot2015


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
