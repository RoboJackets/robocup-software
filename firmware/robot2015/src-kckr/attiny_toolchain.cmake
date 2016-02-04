set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  avr)
set(AVR_TARGET_ARCH         attiny48a)

set(CMAKE_C_COMPILER_TARGET     ${AVR_TARGET_ARCH})
set(CMAKE_CXX_COMPILER_TARGET   ${AVR_TARGET_ARCH})

# narrow down the search scope of where cmake looks for programs/libraries
# for cross compilation
set(CMAKE_FIND_ROOT_PATH        ${PROJECT_SOURCE_DIR})
# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(AVR_PREFIX                  avr)
find_program(AVR_CC_COMPILER    ${AVR_PREFIX}-gcc)
find_program(AVR_CXX_COMPILER   ${AVR_PREFIX}-g++)
