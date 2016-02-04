set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  arm)
set(CMAKE_SYSTEM_VERSION    1)
set(ARM_TARGET_ARCH         cortex-m3)

set(CMAKE_C_COMPILER     ${ARM_TARGET_ARCH})
set(CMAKE_CXX_COMPILER   ${ARM_TARGET_ARCH})

# narrow down the search scope of where cmake looks for programs/libraries
# for cross compilation
set(CMAKE_FIND_ROOT_PATH        ${PROJECT_SOURCE_DIR})
# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(ARM_PREFIX                  arm-none-eabi)
find_program(ARM_CC_COMPILER    ${ARM_PREFIX}-gcc)
find_program(ARM_CXX_COMPILER   ${ARM_PREFIX}-g++)

set(AS                        ${ARM_PREFIX}-as)
set(AR                        ${ARM_PREFIX}-ar)
set(OBJCOPY                   ${ARM_PREFIX}-objcopy)
set(OBJDUMP                   ${ARM_PREFIX}-objdump)
set(SIZE                      ${ARM_PREFIX}-size)
set(RANLIB                    ${ARM_PREFIX}-ranlib)

# find_program(LINKER             ${ARM_PREFIX}-ld)
# find_program(OBJCOPY            ${ARM_PREFIX}-objcopy)
# find_program(OBJDUMP            ${ARM_PREFIX}-objdump)
# find_program(GDB                ${ARM_PREFIX}-gdb)
# find_program(SIZE               ${ARM_PREFIX}-size)

# Find the assembly source files and make sure they're compiled using the C compiler
set(CMAKE_ASM_FLAGS "${MCPU_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")
set(CMAKE_ASM_FLAGS_DEBUG "-g -ggdb3" CACHE INTERNAL "asm debug compiler flags")
set(CMAKE_ASM_FLAGS_RELEASE "" CACHE INTERNAL "asm release compiler flags")
