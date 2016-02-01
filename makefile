MAKE_FLAGS=--no-print-directory

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
	cd build/firmware && cmake -Wno-dev --target $1 $2 ../.. && make $1 $(MAKE_FLAGS)
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
	run/test-soccer
test-firmware:
	$(call cmake_build_target, test-firmware)
	run/test-firmware
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
	$(call cmake_build_target_fw, robot2015)

robot2015-prog:
	$(call cmake_build_target_fw, robot2015-prog)

# Base station 2015 firmware
base2015:
	$(call cmake_build_target_fw, base2015)

base2015-prog:
	$(call cmake_build_target_fw, base2015-prog)

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
	@stylize --diffbase=master --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS)
# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl $${LINK_PREFIX:-./}clean.patch)\n\n"
	@stylize --diffbase=master --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS) --check --output_patch_file="$${CIRCLE_ARTIFACTS:-.}/clean.patch"
