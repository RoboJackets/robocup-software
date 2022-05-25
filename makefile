SHELL=/bin/bash -o pipefail
MAKE_FLAGS = --no-print-directory
TESTS = *
FIRMWR_TESTS = -i2c -io-expander -fpga -piezo -neopixel -attiny -led -radio-sender -radio-receiver
export CMAKE_PREFIX_PATH=/opt/ros/foxy

# circleci has 2 cores, but advertises 32, which causes OOMs
ifeq ($(CIRCLECI), true)
	NINJA_FLAGS=-j2
endif

# Tell CMake to create compile_commands.json for debug builds for clang-tidy
DEBUG_FLAGS=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
CMAKE_FLAGS=-DCMAKE_INSTALL_PREFIX="$(shell pwd)/install" -DNO_WALL=ON -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
# FOR MACHINES WITHOUT CLANG:
# 1) try installing w/ ./util/ubuntu-setup
# 2) change clang/clang++ to gcc/g++ above

# build a specified target with CMake and Ninja
# usage: $(call cmake_build_target, target, extraCmakeFlags)
define cmake_build_target
	mkdir -p build-debug
 	cd build-debug && cmake -GNinja -Wno-dev -DNO_WALL=ON -DCMAKE_BUILD_TYPE=Debug $(DEBUG_FLAGS) $(CMAKE_FLAGS) --target -DBUILD_TESTS=ON .. && ninja $(NINJA_FLAGS) $1 install
endef

define cmake_build_target_release
	mkdir -p build-release
 	cd build-release && cmake -GNinja -Wno-dev -DNO_WALL=ON -DCMAKE_BUILD_TYPE=Release $(CMAKE_FLAGS) --target -DBUILD_TESTS=ON .. && ninja $(NINJA_FLAGS) $1 install
endef

define cmake_build_target_perf
	mkdir -p build-release-debug
 	cd build-release-debug && cmake -GNinja -Wno-dev -DNO_WALL=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo $(CMAKE_FLAGS) --target -DBUILD_TESTS=ON .. && ninja $(NINJA_FLAGS) $1 install
endef

all-perf:
	$(call cmake_build_target_perf, all)
# perf (or "RelWithDebInfo"): almost as fast as release, some debug symbols
perf: all-perf

# used in GH Actions build-and-test
all:
	$(call cmake_build_target, all)
# debug: slow executable, but many debug symbols (e.g. for GDB)
debug: all

# NOT used in build-and-test
all_including_tests:
	$(call cmake_build_target, all)
	$(call cmake_build_target, test-soccer)

all-release:
	$(call cmake_build_target_release, all)
# release: fast executable, no debug symbols
release: all-release

# run once build-release-debug/ exists from a previous build
again:
	(cd build-release-debug/ && ninja install)

run-soccer:
	ros2 launch rj_robocup soccer.launch.py

run-sim:
	ros2 launch rj_robocup sim.launch.py

run-sim-external:
	ros2 launch rj_robocup sim.launch.py use_internal_ref:=False

run-sim-ex: run-sim-external

# actually, config must be changed manually
run-real-sim:
	ros2 launch rj_robocup sim.launch.py config_yaml:=real.yaml use_internal_ref:=False use_sim_radio:=False

run-real:
	ros2 launch rj_robocup soccer.launch.py config_yaml:=real.yaml use_internal_ref:=False use_sim_radio:=False

run-sim2play:
	ros2 launch rj_robocup sim2play.launch.py

run-sim2: run-sim2play

# Run both C++ and python unit tests
tests: test-cpp test-python
test-cpp: test-soccer
test-soccer:
	$(call cmake_build_target, test-soccer)
	./install/lib/rj_robocup/test-soccer --gtest_filter=$(TESTS)
test-soccer-nobuild:
	./install/lib/rj_robocup/test-soccer --gtest_filter=$(TESTS)

test-python: perf
	python3 -m pytest --cov rj_gameplay --cov stp rj_gameplay --cov-report xml

# could be useful in mypy GH Actions but unused
pylint:
	pylint -j0 rj_gameplay/rj_gameplay rj_gameplay/stp

# could be useful in mypy GH Actions but unused
mypy:
	mypy --ignore-missing-imports rj_gameplay/rj_gameplay rj_gameplay/stp

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

clean:
	((rm build-debug -rf); (rm build-release -rf); (rm build-release-debug -rf)) || true
	git clean -f -X -d cmake-*
	rm -rf install/bin install/lib install/share install/include

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

# build docs
docs:
	(cd docs/ && make html)
# open docs
open-docs:
	xdg-open docs/build/html/index.html

# build docs (old)
apidocs:
	doxygen doc/Doxyfile
	cp doc/doxygen.css api_docs/html/
	@echo "\n=> Open up 'api_docs/html/index.html' in a browser to view a local copy of the documentation"

# Find the most recent common ancestor. This prevents new commits on staging from registering as diffs.
DIFFBRANCH ?= ros2
DIFFBASE ?= $(shell git merge-base $(DIFFBRANCH) HEAD)

# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl -L $${LINK_PREFIX:-file://}clean.patch)\n\n"
	@stylize.v1 --git_diffbase=$(DIFFBASE) --patch_output "$${CIRCLE_ARTIFACTS:-.}/clean.patch"

CLANG_FORMAT_BINARY=clang-format-10
CLANG_TIDY_BINARY=clang-tidy-10
COMPILE_COMMANDS_DIR=build-debug

# circleci has 2 cores, but advertises 32
ifeq ($(CIRCLECI), true)
	CORES=2
else
	CORES=$(shell nproc)
endif

# Restyles all C++ (Clang formatter) excluding files in the external and build folders. For Python, run black rj_gameplay.
pretty-lines:
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-format-diff.py -binary $(CLANG_FORMAT_BINARY) -i -p1

tidy-lines:
ifeq ("$(wildcard $(COMPILE_COMMANDS_DIR)/compile_commands.json)","")
	@printf "$(COMPILE_COMMANDS_DIR)/compile_commands.json file is missing! Run 'make all' to generate the compile db for clang-tidy."
	exit 1
endif
	@echo "Removing GCC precompiled headers from compile_commands.json so that clang-tidy will work"
	@sed -i 's/-include [^ ]*cmake_pch\.hxx//' $(COMPILE_COMMANDS_DIR)/compile_commands.json
	@printf "Running clang-tidy-diff...\n"
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-tidy-diff.py -clang-tidy-binary $(CLANG_TIDY_BINARY) -p1 -path $(COMPILE_COMMANDS_DIR) -j$(CORES) -ignore ".*(Test|test).cpp" -quiet

checkstyle-lines:
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-format-diff.py -binary $(CLANG_FORMAT_BINARY) -p1 | tee /tmp/checkstyle.patch
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/yapf-diff.py -style .style.yapf -p1 | tee -a /tmp/checkstyle.patch
	@bash -c '[[ ! "$$(cat /tmp/checkstyle.patch)" ]] || (echo "****************************** Checkstyle errors *******************************" && exit 1)'

# used in GH Actions - build-and-test
checktidy-lines:
	@echo "Removing GCC precompiled headers from compile_commands.json so that clang-tidy will work"
	@sed -i 's/-include [^ ]*cmake_pch\.hxx//' $(COMPILE_COMMANDS_DIR)/compile_commands.json
	@git diff -U0 --no-color $(DIFFBASE) | python3 util/clang-tidy-diff.py -clang-tidy-binary $(CLANG_TIDY_BINARY) -p1 -path $(COMPILE_COMMANDS_DIR) -j$(CORES)
