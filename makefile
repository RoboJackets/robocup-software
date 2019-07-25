MAKE_FLAGS = --no-print-directory
TESTS = *
FIRMWR_TESTS = -i2c -io-expander -fpga -piezo -neopixel -attiny -led -radio-sender -radio-receiver

# circleci has 2 cores, but advertises 32, which causes OOMs
ifeq ($(CIRCLECI), true)
	NINJA_FLAGS=-j2
endif

# build a specified target with CMake and Ninja
# usage: $(call cmake_build_target, target, extraCmakeFlags)
define cmake_build_target
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=Debug --target $1 $2 .. && ninja $(NINJA_FLAGS) $1
endef

define cmake_build_target_release
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=Release --target $1 $2 .. && ninja $(NINJA_FLAGS) $1
endef

define cmake_build_target_perf
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=RelWithDebInfo --target $1 $2 .. && ninja $(NINJA_FLAGS) $1
endef

all:
	$(call cmake_build_target, all)

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
	-pkill -f './grsim'
	./run/grsim &
	@echo '!!![WARNING]!!! Multiple soccer instances will not work unless your grSim is broadcasting over the proper IP.'
	@echo 'Please set your grSim broadcast IP to "224.5.23.2:10020", or you will experience issues.'
	./run/soccer -sim -b & sleep 2 && ./run/soccer -sim -y -defend plus
	-pkill -f './grsim'

run-release: all-release
	./run/soccer
run-sim-release: all-release backend-headless-simulator-soccer
rsr: run-sim-release
rrs: rsr
rr: run-release
view: 
	./run/soccer -vlog $(file)

# backend targets to launch soccer with grsim in headless
backend-headless-simulator-soccer:
	-pkill -f './grsim'
	./run/grsim --headless &
	./run/soccer -sim -pbk testing.pbk
# Kill grSim once we unblock
	-pkill -f './grsim'

# backend targets to launch soccer with a grsim window
backend-simulator-soccer:
	-pkill -f './grsim'
	./run/grsim &
	./run/soccer -sim -pbk testing.pbk
# Kill grSim once we unblock
	-pkill -f './grsim'


debug: all
ifeq ($(shell uname), Linux)
	gdb ./run/soccer
else
	lldb ./run/soccer.app
endif

debug-sim: all
	-pkill -f './grsim'
	./run/grsim &
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
	run/test-soccer --gtest_filter=$(TESTS)
test-python: all
	cd soccer/gameplay && ./run_tests.sh
pylint:
	pylint -j8 --reports=n soccer/gameplay
mypy:
	mypy soccer/gameplay

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

STYLIZE_DIFFBASE ?= master
# automatically format code according to our style config defined in .clang-format
pretty:
	@stylize.v1 -i --git_diffbase=$(STYLIZE_DIFFBASE)
# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl -L $${LINK_PREFIX:-file://}clean.patch)\n\n"
	@stylize.v1 --git_diffbase=$(STYLIZE_DIFFBASE) --patch_output "$${CIRCLE_ARTIFACTS:-.}/clean.patch"

pretty-lines:
	@git diff -U0 --no-color $(STYLIZE_DIFFBASE) | clang-format-diff -i -p1
	@git diff -U0 --no-color $(STYLIZE_DIFFBASE) | python3 util/yapf-diff.py -style .style.yapf -i -p1

checkstyle-lines:
	@git diff -U0 --no-color $(STYLIZE_DIFFBASE) | clang-format-diff -p1 | tee /tmp/checkstyle.patch
	@git diff -U0 --no-color $(STYLIZE_DIFFBASE) | python3 util/yapf-diff.py -style .style.yapf -p1 | tee -a /tmp/checkstyle.patch
	@bash -c '[[ ! "$$(cat /tmp/checkstyle.patch)" ]] || (echo "****************************** Checkstyle errors *******************************" && exit 1)'

# Option to use old version of stylize
STYLE_EXCLUDE_DIRS=build \
	external
# automatically format code according to our style config defined in .clang-format
pretty-old:
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS)
# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle-old:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl -L $${LINK_PREFIX:-file://}clean.patch)\n\n"
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS) --check --output_patch_file="/tmp/clean.patch"
