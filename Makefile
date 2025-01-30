# Default settings
MIXIN_DIR=local
BUILD_CMD=colcon build --mixin $(MIXIN_DIR)

.PHONY: all debug rel_debug perf ci clean

all:
	$(BUILD_CMD)

debug:
	$(BUILD_CMD)/debug_flags

rel_debug:
	$(BUILD_CMD)/rel_debug

perf:
	$(BUILD_CMD)/perf_flags

ci:
	$(BUILD_CMD)/ci_silent

clean:
	colcon clean
	rm -rf build install log

