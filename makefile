SHELL=/bin/bash -o pipefail
MAKE_FLAGS = --no-print-directory
TESTS = *
FIRMWR_TESTS = -i2c -io-expander -fpga -piezo -neopixel -attiny -led -radio-sender -radio-receiver

# circleci has 2 cores, but advertises 32, which causes OOMs
ifeq ($(CIRCLECI), true)
	NINJA_FLAGS=-j2
endif

# Tell CMake to create compile_commands.json for debug builds for clang-tidy
DEBUG_FLAGS=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
CMAKE_FLAGS="-DCMAKE_INSTALL_PREFIX=$(shell pwd)/install"

# build a specified target with CMake and Ninja
# usage: $(call cmake_build_target, target, extraCmakeFlags)
define cmake_build_target
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=Debug $(DEBUG_FLAGS) $(CMAKE_FLAGS) --target $1 $2 .. && ninja $(NINJA_FLAGS) $1 install
endef

define cmake_build_target_release
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=Release $(CMAKE_FLAGS) --target $1 $2 .. && ninja $(NINJA_FLAGS) $1 install
endef

define cmake_build_target_perf
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=RelWithDebInfo $(CMAKE_FLAGS) --target $1 $2 .. && ninja $(NINJA_FLAGS) $1 install
endef

all:
	$(call cmake_build_target, all)

all_including_tests:
	$(call cmake_build_target, all)
	$(call cmake_build_target, test-soccer)

all-release:
	$(call cmake_build_target_release, all)

all-perf:
	$(call cmake_build_target_perf, all)
perf: all-perf

run: all
	./run/soccer
run-comp:
	./runcomp.sh
r:	run
rs: run-sim
run-sim: all backend-headless-simulator-soccer
run-headmore: all backend-simulator-soccer

run-sim2play: all
	-pkill -f './grSim'
	-(cd run && ./grSim) &
	@echo '!!![WARNING]!!! Multiple soccer instances will not work unless your grSim is broadcasting over the proper IP.'
	@echo 'Please set your grSim broadcast IP to "224.5.23.2:10020", or you will experience issues.'
	./run/soccer -sim -b & sleep 2 && ./run/soccer -sim -y -defend plus
	-pkill -f './grSim'

run-release: all-release
	./run/soccer
run-sim-release: all-release backend-headless-simulator-soccer
rsr: run-sim-release
rrs: rsr
rr: run-release
view:
	./run/soccer -vlog $(file)

# backend targets to launch soccer with grSim in headless
backend-headless-simulator-soccer:
	-pkill -f './grSim'
	-(cd run && ./grSim --headless) &
	./run/soccer -sim -pbk testing.pbk
# Kill grSim once we unblock
	-pkill -f './grSim'

# backend targets to launch soccer with a grSim window
backend-simulator-soccer:
	-pkill -f './grSim'
	-(cd run && ./grSim) &
	./run/soccer -sim -pbk testing.pbk
# Kill grSim once we unblock
	-pkill -f './grSim'


debug: all
ifeq ($(shell uname), Linux)
	gdb ./run/soccer
else
	lldb ./run/soccer.app
endif

debug-sim: all
	-pkill -f './grSim'
	-(cd run && ./grSim) &
ifeq ($(shell uname), Linux)
	gdb --args ./run/soccer -sim
else
	lldb -- ./run/soccer.app -sim
endif

# Run both C++ and python unit tests
tests: test-cpp test-python
test-cpp: test-soccer
test-soccer:
	$(call cmake_build_target, test-soccer)
	./install/lib/rj_robocup/test-soccer --gtest_filter=$(TESTS)
test-soccer-nobuild:
	./install/lib/rj_robocup/test-soccer --gtest_filter=$(TESTS)

test-python: all
	cd soccer/src/gameplay && source /opt/foxy/setup.sh && ./run_tests.sh
test-python-nobuild:
	cd soccer/src/gameplay && source /opt/foxy/setup.sh && ./run_tests.sh
pylint:
	pylint -j8 --reports=n soccer/src/gameplay
mypy:
	mypy soccer/src/gameplay

COV_BUILD_DIR=build/coverage
coverage:
	mkdir -p ${COV_BUILD_DIR}
	cd ${COV_BUILD_DIR} && cmake -GNinja -Wno-dev --target test-soccer \
		-D CMAKE_CXX_FLAGS="--coverage" ../../ && ninja $(NINJA_FLAGS) test-soccer
	run/test-soccer		# Kind of hacky, but w/e
	-coveralls -b ${COV_BUILD_DIR} -r . \
		-e ${COV_BUILD_DIR}/tmp/ -e ${COV_BUILD_DIR}/src/ \
		-E '(^.*((moc_)|(automoc)|(ui_)|([Tt]est)).*$$)|(^.*((include)|(mbed)|(googletest)|(gtest)|(protobuf)|(qt5)).*$$)' \
		--gcov-options '\-lp'

behavior-diagrams: all
	cd soccer/gameplay && python3 generate_fsm_diagrams.py
	@echo -e "\n=> Open up 'soccer/gameplay/diagrams' to view behavior state machine diagrams"

clean:
	cd build && ninja clean || true
	rm -rf build

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
	clang-modernize -p build/modernize -include=common,logging,soccer

apidocs:
	doxygen doc/Doxyfile
	cp doc/doxygen.css api_docs/html/
	@echo "\n=> Open up 'api_docs/html/index.html' in a browser to view a local copy of the documentation"

# Find the most recent common ancestor. This prevents new commits on staging from registering as diffs.
DIFFBRANCH ?= staging
DIFFBASE ?= $(shell git merge-base $(DIFFBRANCH) HEAD)

# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl -L $${LINK_PREFIX:-file://}clean.patch)\n\n"
	@stylize.v1 --git_diffbase=$(DIFFBASE) --patch_output "$${CIRCLE_ARTIFACTS:-.}/clean.patch"

CLANG_FORMAT_BINARY=clang-format-10
CLANG_TIDY_BINARY=clang-tidy-10
COMPILE_COMMANDS_DIR=build

# circleci has 2 cores, but advertises 32
ifeq ($(CIRCLECI), true)
	CORES=2
else
	CORES=$(shell nproc)
endif

pretty-lines:
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-format-diff.py -binary $(CLANG_FORMAT_BINARY) -i -p1
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/yapf-diff.py -style .style.yapf -i -p1

tidy-lines:
ifeq ("$(wildcard $(COMPILE_COMMANDS_DIR)/compile_commands.json)","")
	@printf "$(COMPILE_COMMANDS_DIR)/compile_commands.json file is missing! Run 'make all' to generate the compile db for clang-tidy."
	exit 1
endif
	@printf "Running clang-tidy-diff...\n"
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-tidy-diff.py -clang-tidy-binary $(CLANG_TIDY_BINARY) -p1 -path $(COMPILE_COMMANDS_DIR) -j$(CORES) -ignore ".*Test.cpp"

checkstyle-lines:
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-format-diff.py -binary $(CLANG_FORMAT_BINARY) -p1 | tee /tmp/checkstyle.patch
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/yapf-diff.py -style .style.yapf -p1 | tee -a /tmp/checkstyle.patch
	@bash -c '[[ ! "$$(cat /tmp/checkstyle.patch)" ]] || (echo "****************************** Checkstyle errors *******************************" && exit 1)'

checktidy-lines:
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-tidy-diff.py -clang-tidy-binary $(CLANG_TIDY_BINARY) -p1 -path $(COMPILE_COMMANDS_DIR) -j$(CORES) -ignore ".*(Test|test).cpp" > /tmp/checktidy.patch
