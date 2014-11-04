
include(ExternalProject)

# download the latest version of the mbed sdk/library from the official source
# it mostly just contains header files and mbed hardware support
ExternalProject_Add(mbed_library
    HG_REPOSITORY https://developer.mbed.org/users/mbed_official/code/mbed/
    HG_TAG default
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)

set(CMAKE_TOOLCHAIN_FILE mbed_gcc_toolchain.cmake)

# the source files that will go into the robot2015.bin firmware
file(GLOB robot2015_SRC
    "*.cpp"
    )

# include the mbed headers that we downloaded above with the ExternalProject command
set(MBED_LIB_PATH ${CMAKE_CURRENT_BINARY_DIR}/mbed_library-prefix/src/mbed_library)
include_directories(${MBED_LIB_PATH})
include_directories(${MBED_LIB_PATH}/TARGET_LPC1768)
include_directories(${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM)
include_directories(${MBED_LIB_PATH}/TARGET_LPC1768/TARGET_NXP)
include_directories(${MBED_LIB_PATH}/TARGET_LPC1768/TARGET_NXP/TARGET_LPC176X)
include_directories(${MBED_LIB_PATH}/TARGET_LPC1768/TARGET_NXP/TARGET_LPC176X/TARGET_MBED_LPC1768)

# include cross-compiler header files
# find_path(CSPATH arm-none-eabi-g++ PATHS ENV)
set(CSPATH  /usr/lib/arm-none-eabi/include)
message(STATUS "${CSPATH} is where the xcompile toolchain is installed")
include_directories(${CSPATH})
include_directories(${CSPATH}/c++/4.8)

# define some symbols when compiling
set(CC_SYMBOLS "-DTARGET_LPC1768 -DTARGET_M3 -DTARGET_NXP -DTARGET_LPC176X -DTARGET_MBED_LPC1768 -DTOOLCHAIN_GCC_ARM -DTOOLCHAIN_GCC -D__CORTEX_M3 -DARM_MATH_CM3 -DMBED_BUILD_TIMESTAMP=1415078109.67 -D__MBED__=1")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CC_SYMBOLS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CC_SYMBOLS}")

# path to mbed object files
link_directories(${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM)

# use mbed-provided linker script
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -T ${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM/LPC1768.ld")

set(MBED_OBJ
    ${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM/retarget.o
    ${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM/system_LPC17xx.o
    ${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM/board.o
    ${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM/cmsis_nvic.o
    ${MBED_LIB_PATH}/TARGET_LPC1768/TOOLCHAIN_GCC_ARM/startup_LPC17xx.o)

# firmware output: robot2015.bin
add_executable(robot2015 ${robot2015_SRC} ${MBED_OBJ})
add_dependencies(robot2015 mbed_library)
target_link_libraries(robot2015 mbed)
target_link_libraries(robot2015 stdc++ supc++ m c gcc nosys)

# create a robot2015.bin from a robot2015.elf
add_custom_command(TARGET robot2015
    COMMAND arm-none-eabi-objcopy -O binary robot2015 robot2015_mbed.bin
    COMMENT "objcopying to make mbed-compatible firmware")
