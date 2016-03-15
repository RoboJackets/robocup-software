
include(../mbed/mbed_executable.cmake)

# Set the toolchain and compile flags
rj_mbed_env()

# firmware source files
set(base2015_SRC "main.cpp")

rj_add_mbed_executable(base2015 ${base2015_SRC})
target_link_libraries(base2015 common2015)
