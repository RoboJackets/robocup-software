MAKE_FLAGS = --no-print-directory
TESTS = *
FIRMWR_TESTS = -i2c -io-expander -fpga -piezo -neopixel

# build a specified target with CMake and Ninja
# usage: $(call cmake_build_target, target, extraCmakeFlags)
define cmake_build_target
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev --target $1 $2 .. && ninja $1
endef

# Similar to the above build target command, but for firmware.  This is used
# because CMake can only handle one toolchain at a time, so we build the MBED-
# targeted code separately.
define cmake_build_target_fw
	mkdir -p build/firmware
	cd build && cmake -Wno-dev --target $1 $2 .. && make $1 $(MAKE_FLAGS)
endef

all:
	$(call cmake_build_target, all)

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

	$(call cmake_build_target, test-soccer)
	run/test-soccer --gtest_filter=$(TESTS)
test-firmware:
	$(call cmake_build_target, test-firmware)
	run/test-firmware --gtest_filter=$(TESTS)
test-python: all
	cd soccer/gameplay && ./run_tests.sh
pylint:
	cd soccer && pylint -E gameplay

COV_BUILD_DIR=build/coverage
coverage:
	mkdir -p ${COV_BUILD_DIR}
	cd ${COV_BUILD_DIR} && cmake -GNinja -Wno-dev --target test-soccer test-firmware  \
		-D CMAKE_CXX_FLAGS="--coverage" ../../ && ninja test-soccer test-firmware
	run/test-soccer		# Kind of hacky, but w/e
	run/test-firmware
	-coveralls -b ${COV_BUILD_DIR} -r . \
		-e ${COV_BUILD_DIR}/tmp/ -e ${COV_BUILD_DIR}/src/ \
		-e ${COV_BUILD_DIR}/simulator/ -e ${COV_BUILD_DIR}/firmware/ \
		-E '(^.*((moc_)|(automoc)|(ui_)|([Tt]est)).*$$)|(^.*((include)|(mbed)|(googletest)|(gtest)|(protobuf)|(qt5)).*$$)' \
		--gcov-options '\-lp'

behavior-diagrams: all
	cd soccer/gameplay && python3 generate_fsm_diagrams.py
	@echo -e "\n=> Open up 'soccer/gameplay/diagrams' to view behavior state machine diagrams"

clean:
	cd build && ninja clean || true
	rm -rf build

# the alias names that point to the current set of firmware targets
robot: robot2015
robot-prog: robot2015-prog
fpga: fpga2015
fpga-prog: fpga2015-prog
radio: radio2015
radio-prog: radio2015-prog
kicker: kicker2015
kicker-prog: kicker2015-prog
firmware: firmware2015

# robot 2015 firmware
robot2015:
	$(call cmake_build_target_fw, robot2015)
robot2015-prog:
	$(call cmake_build_target_fw, robot2015-prog)

# robot2015-test-<test_unit>{-prog}
# defines the targets described at the line above - test units defined in FIRMWR_TESTS
$(FIRMWR_TESTS:-%=robot2015-test-%):
	$(call cmake_build_target_fw, robot2015-test, -DHW_TEST_UNIT:STRING=$(@F:robot2015-test-%=%))
$(FIRMWR_TESTS:-%=robot2015-test-%-prog):
	$(call cmake_build_target_fw, robot2015-test-prog, -DHW_TEST_UNIT:STRING=$(@F:robot2015-test-%-prog=%))

GDB_PORT ?= 3333
.INTERMEDIATE: build/robot2015-gdb.pid
build/robot2015-gdb.pid:
# this will cache sudo use without a password in the environment
# so we won't enter the gdb server and skip past the password prompt.
	@sudo echo "starting pyocd-gdbserver, logging to build/robot2015-gdb.log"
# now we can refresh the sudo timeout and start up the gdb server
	sudo -v && { sudo pyocd-gdbserver --allow-remote --port $(GDB_PORT) --reset-break \
	--target lpc1768 -S -G > build/robot2015-gdb.log 2>&1 & sudo echo $$! > $@; }

GDB_NO_CONN ?= 0
robot2015-gdb: robot2015 build/robot2015-gdb.pid
# use pyocd-gdbserver, and explicitly pass it the type of target we want to connect with,
# making sure that we enable semihosting and use gdb syscalls for the file io
	@trap 'sudo pkill -9 -P `cat build/robot2015-gdb.pid`; exit' TERM INT EXIT && \
	if [ $(GDB_NO_CONN) -eq 0 ]; then \
		arm-none-eabi-gdb build/firmware/firmware/robot2015/src-ctrl/robot2015_elf \
		  -ex "target remote localhost:$(GDB_PORT)" \
		  -ex "load" \
		  -ex "tbreak main" \
		  -ex "continue"; \
	else \
		while true; do sleep 10; done; \
	fi

# kicker 2015 firmware
kicker2015:
	$(call cmake_build_target_fw, kicker2015)
kicker2015-prog:
	$(call cmake_build_target_fw, kicker2015-prog)

# fpga 2015 synthesis
fpga2015:
	$(call cmake_build_target_fw, fpga2015)
fpga2015-prog:
	$(call cmake_build_target_fw, fpga2015-prog)
	
# Base station 2015 firmware
radio2015:
	$(call cmake_build_target_fw, base2015)
radio2015-prog:
	$(call cmake_build_target_fw, base2015-prog)

# Build all of the 2015 firmware for a robot, and/or move all of the binaries over to the mbed
firmware2015: robot2015 kicker2015 fpga2015 radio2015
firmware2015-prog: robot2015-prog kicker2015-prog fpga2015-prog

# Robot firmware (both 2008/2011)
robot2011:
	cd firmware && scons robot
robot2011-prog: robot
	cd firmware && sudo scons robot-prog
robot2011-prog-samba: robot
	cd firmware && sudo scons robot-prog-samba
robot2011-prog-ota: robot
	cd firmware && scons robot-ota

# Robot FPGA
fpga2011:
	cd firmware && scons fpga2011
# program the fpga over the usb spi interface
fpga2011-prog-spi: fpga2011
	cd firmware && sudo scons fpga2011-spi
# program the fpga over the jtag interface
fpga2011-prog-jtag: fpga2011
	cd firmware && sudo scons fpga2011-jtag
# USB Radio Base Station
radio2011:
	cd firmware && scons base2011
radio2011-prog: base2011
	cd firmware && sudo scons base2011-prog


static-analysis:
	mkdir -p build/static-analysis
	cd build/static-analysis; scan-build cmake ../.. -Wno-dev -DSTATIC_ANALYSIS=ON && scan-build -o output make $(MAKE_FLAGS)
modernize:
	# Runs CMake with a sepcial flag telling it to output the compilation
	# database, which lists the files to be compiled and the flags passed to
	# the compiler for each one. Then runs clang-modernize, using the
	# compilation database as input, on all c/c++ files in the repo.
	mkdir -p build/modernize
	cd build/modernize; cmake ../.. -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && make $(MAKE_FLAGS)
	# You can pass specific flags to clang-modernize if you want it to only run some types of
	# transformations, rather than all transformations that it's capable of.
	# See `clang-modernize --help` for more info.
	clang-modernize -p build/modernize -include=common,logging,simulator,soccer

apidocs:
	doxygen doc/Doxyfile
	cp doc/doxygen.css api_docs/html/
	@echo "\n=> Open up 'api_docs/html/index.html' in a browser to view a local copy of the documentation"

STYLIZE_DIFFBASE ?= master
STYLE_EXCLUDE_DIRS=build \
	external \
	firmware/robot/cpu/at91sam7s256 \
	firmware/robot/cpu/at91sam7s321 \
	firmware/robot/cpu/at91sam7s64 \
	firmware/robot/cpu/usb \
	firmware/robot/cpu/invensense
# automatically format code according to our style config defined in .clang-format
pretty:
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS)
# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl $${LINK_PREFIX:-file://}clean.patch)\n\n"
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS) --check --output_patch_file="$${CIRCLE_ARTIFACTS:-.}/clean.patch"
