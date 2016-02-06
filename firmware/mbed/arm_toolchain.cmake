CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

set( CMAKE_SYSTEM_NAME       Generic     )
# set( CMAKE_SYSTEM_PROCESSOR  arm         )
set( CMAKE_SYSTEM_VERSION    1           )

# narrow down the search scope of where cmake looks for programs/libraries
# for cross compilation
set( CMAKE_FIND_ROOT_PATH                    ${PROJECT_SOURCE_DIR}   )
# search for programs in the build host directories
set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM       NEVER                   )
# for libraries and headers in the target directories
set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY       ONLY                    )
set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE       ONLY                    )

set( ARM_PREFIX                 arm-none-eabi           )
find_program( ARM_C_COMPILER    ${ARM_PREFIX}-gcc       )
find_program( ARM_CXX_COMPILER  ${ARM_PREFIX}-g++       )
find_program( ARM_RANLIB        ${ARM_PREFIX}-ranlib    )
find_program( ARM_AR            ${ARM_PREFIX}-ar        )
find_program( ARM_AS            ${ARM_PREFIX}-as        )
find_program( ARM_NM            ${ARM_PREFIX}-nm        )
find_program( ARM_LD            ${ARM_PREFIX}-ld        )
find_program( ARM_OBJCOPY       ${ARM_PREFIX}-objcopy   )
find_program( ARM_OBJDUMP       ${ARM_PREFIX}-objdump   )

# let cmake know we plan to use an external toolchain outside
# the scope of the system's default
# set( CMAKE_C_COMPILER_EXTERNAL_TOOLCHAIN true )

# make sure we defien this since we'll be using GCC
add_definitions(-DTOOLCHAIN_GCC)

# Let cmake know we intend to cross compile from the point where
# this file is included onward
set(CMAKE_CROSSCOMPILING true)

# set compiler things that were found above
set( CMAKE_C_COMPILER       ${ARM_C_COMPILER}           )
set( CMAKE_CXX_COMPILER     ${ARM_CXX_COMPILER}         )
set( CMAKE_AR               ${ARM_AR}                   )
set( CMAKE_RANLIB           ${ARM_RANLIB}               )

# Find the assembly source files and make sure they're compiled using the C compiler
# set( CMAKE_ASM_FLAGS "${MCPU_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags" )
set( CMAKE_ASM_FLAGS_DEBUG "-g -ggdb3" CACHE INTERNAL "asm debug compiler flags" )
set( CMAKE_ASM_FLAGS_RELEASE "" CACHE INTERNAL "asm release compiler flags" )
