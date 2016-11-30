MAKE_FLAGS = --no-print-directory
TESTS = *
FIRMWR_TESTS = -i2c -io-expander -fpga -piezo -neopixel -attiny -led -radio-sender -radio-receiver

# build a specified target with CMake and Ninja
# usage: $(call cmake_build_target, target, extraCmakeFlags)
define cmake_build_target
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev --target $1 $2 .. && ninja $1
endef

all:
	$(call cmake_build_target, all)

run: all
	./run/soccer
rs: run-sim
run-sim: all
	-pkill -f './simulator --headless'
	./run/simulator --headless &
	./run/soccer -sim -pbk example.pbk
run-sim2play: all
	-pkill -f './simulator --headless'
	./run/simulator --headless &
	./run/soccer -sim -y & ./soccer -sim -b

debug: all
ifeq ($(shell uname), Linux)
	gdb ./run/soccer
else
	lldb ./run/soccer.app
endif

debug-sim: all
	-pkill -f './simulator --headless'
	./run/simulator --headless &
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
	cd soccer && pylint -E gameplay

COV_BUILD_DIR=build/coverage
coverage:
	mkdir -p ${COV_BUILD_DIR}
	cd ${COV_BUILD_DIR} && cmake -GNinja -Wno-dev --target test-soccer \
		-D CMAKE_CXX_FLAGS="--coverage" ../../ && ninja test-soccer
	run/test-soccer		# Kind of hacky, but w/e
	-coveralls -b ${COV_BUILD_DIR} -r . \
		-e ${COV_BUILD_DIR}/tmp/ -e ${COV_BUILD_DIR}/src/ \
		-e ${COV_BUILD_DIR}/simulator/ \
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
	clang-modernize -p build/modernize -include=common,logging,simulator,soccer

apidocs:
	doxygen doc/Doxyfile
	cp doc/doxygen.css api_docs/html/
	@echo "\n=> Open up 'api_docs/html/index.html' in a browser to view a local copy of the documentation"

STYLIZE_DIFFBASE ?= master
STYLE_EXCLUDE_DIRS=build \
	external
# automatically format code according to our style config defined in .clang-format
pretty:
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS)
# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl -L $${LINK_PREFIX:-file://}clean.patch)\n\n"
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS) --check --output_patch_file="$${CIRCLE_ARTIFACTS:-.}/clean.patch"

