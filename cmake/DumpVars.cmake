# Include this module to dump a snapshot of the most common build variables where
# the file is included from

# if you are building in-source, this is the same as CMAKE_SOURCE_DIR, otherwise 
# this is the top level directory of your build tree 
message( STATUS "CMAKE_BINARY_DIR:\t\t\t"               "${CMAKE_BINARY_DIR}"                 )
# if you are building in-source, this is the same as CMAKE_CURRENT_SOURCE_DIR, otherwise this 
# is the directory where the compiled or generated files from the current CMakeLists.txt will go to 
message( STATUS "CMAKE_CURRENT_BINARY_DIR:\t\t"         "${CMAKE_CURRENT_BINARY_DIR}"         )
# this is the directory, from which cmake was started, i.e. the top level source directory 
message( STATUS "CMAKE_SOURCE_DIR:\t\t\t"               "${CMAKE_SOURCE_DIR}"                 )
# this is the directory where the currently processed CMakeLists.txt is located in 
message( STATUS "CMAKE_CURRENT_SOURCE_DIR:\t\t"         "${CMAKE_CURRENT_SOURCE_DIR}"         )
# contains the full path to the top level directory of your build tree 
message( STATUS "PROJECT_BINARY_DIR:\t\t\t"             "${PROJECT_BINARY_DIR}"               )
# contains the full path to the root of your project source directory,
# i.e. to the nearest directory where CMakeLists.txt contains the PROJECT() command 
message( STATUS "PROJECT_SOURCE_DIR:\t\t\t"             "${PROJECT_SOURCE_DIR}"               )
# set this variable to specify a common place where CMake should put all executable files
# (instead of CMAKE_CURRENT_BINARY_DIR)
message( STATUS "EXECUTABLE_OUTPUT_PATH:\t\t"           "${EXECUTABLE_OUTPUT_PATH}"           )
# set this variable to specify a common place where CMake should put all libraries 
# (instead of CMAKE_CURRENT_BINARY_DIR)
message( STATUS "LIBRARY_OUTPUT_PATH:\t\t\t"            "${LIBRARY_OUTPUT_PATH}"              )
# tell CMake to search first in directories listed in CMAKE_MODULE_PATH
# when you use FIND_PACKAGE() or INCLUDE()
message( STATUS "CMAKE_MODULE_PATH:\t\t\t"              "${CMAKE_MODULE_PATH}"                )
# this is the complete path of the cmake which runs currently (e.g. /usr/local/bin/cmake) 
message( STATUS "CMAKE_COMMAND:\t\t\t"                  "${CMAKE_COMMAND}"                    )
# this is the CMake installation directory 
message( STATUS "CMAKE_ROOT:\t\t\t\t"                   "${CMAKE_ROOT}"                       )
# this is the filename including the complete path of the file where this variable is used. 
message( STATUS "CMAKE_CURRENT_LIST_FILE:\t\t"          "${CMAKE_CURRENT_LIST_FILE}"          )
# this is linenumber where the variable is used
message( STATUS "CMAKE_CURRENT_LIST_LINE:\t\t"          "${CMAKE_CURRENT_LIST_LINE}"          )
# this is used when searching for include files e.g. using the FIND_PATH() command.
message( STATUS "CMAKE_INCLUDE_PATH:\t\t"               "${CMAKE_INCLUDE_PATH}"               )
# this is used when searching for libraries e.g. using the FIND_LIBRARY() command.
message( STATUS "CMAKE_LIBRARY_PATH:\t\t"               "${CMAKE_LIBRARY_PATH}"               )
# the complete system name, e.g. "Linux-2.4.22", "FreeBSD-5.4-RELEASE" or "Windows 5.1" 
message( STATUS "CMAKE_SYSTEM:\t\t\t"                   "${CMAKE_SYSTEM}"                     )
# the short system name, e.g. "Linux", "FreeBSD" or "Windows"
message( STATUS "CMAKE_SYSTEM_NAME:\t\t\t"              "${CMAKE_SYSTEM_NAME}"                )
# only the version part of CMAKE_SYSTEM 
message( STATUS "CMAKE_SYSTEM_VERSION:\t\t"             "${CMAKE_SYSTEM_VERSION}"             )
# the processor name (e.g. "Intel(R) Pentium(R) M processor 2.00GHz") 
message( STATUS "CMAKE_SYSTEM_PROCESSOR:\t\t"           "${CMAKE_SYSTEM_PROCESSOR}"           )
# is TRUE on all UNIX-like OS's, including Apple OS X and CygWin
message( STATUS "UNIX:\t\t\t\t"                         "${UNIX}"                             )
# is TRUE on Windows, including CygWin 
message( STATUS "WIN32:\t\t\t\t"                        "${WIN32}"                            )
# is TRUE on Apple OS X
message( STATUS "APPLE:\t\t\t\t"                        "${APPLE}"                            )
# is TRUE when using the MinGW compiler in Windows
message( STATUS "MINGW:\t\t\t\t"                        "${MINGW}"                            )
# is TRUE on Windows when using the CygWin version of cmake
message( STATUS "CYGWIN:\t\t\t\t"                       "${CYGWIN}"                           )
# is TRUE on Windows when using a Borland compiler 
message( STATUS "BORLAND:\t\t\t\t"                      "${BORLAND}"                          )
# Microsoft compiler s
message( STATUS "MSVC:\t\t"                             "${MSVC}"                             )
message( STATUS "MSVC_IDE:\t\t"                         "${MSVC_IDE}"                         )
message( STATUS "MSVC60:\t\t"                           "${MSVC60}"                           )
message( STATUS "MSVC70:\t\t"                           "${MSVC70}"                           )
message( STATUS "MSVC71:\t\t"                           "${MSVC71}"                           )
message( STATUS "MSVC80:\t\t"                           "${MSVC80}"                           )
message( STATUS "CMAKE_COMPILER_2005:\t"                "${CMAKE_COMPILER_2005}"              )
# set this to true if you don't want to rebuild the object files if the rules have changed, 
# but not the actual source files or headers (e.g. if you changed the some compiler switches) 
message( STATUS "CMAKE_SKIP_RULE_DEPENDENCY:\t"         "${CMAKE_SKIP_RULE_DEPENDENCY}"       )
# since CMake 2.1 the install rule depends on all, i.e. everything will be built before installing. 
# If you don't like this, set this one to true.
message( STATUS "CMAKE_SKIP_INSTALL_ALL_DEPENDENCY:\t"  "${CMAKE_SKIP_INSTALL_ALL_DEPENDENCY}")
# If set, runtime paths are not added when using shared libraries. Default it is set to OFF
message( STATUS "CMAKE_SKIP_RPATH:\t\t\t"               "${CMAKE_SKIP_RPATH}"                 )
# set this to true if you are using makefiles and want to see the full compile and link 
# commands instead of only the shortened ones 
message( STATUS "CMAKE_VERBOSE_MAKEFILE:\t\t"           "${CMAKE_VERBOSE_MAKEFILE}"           )
# this will cause CMake to not put in the rules that re-run CMake. This might be useful if 
# you want to use the generated build files on another machine. 
message( STATUS "CMAKE_SUPPRESS_REGENERATION:\t"        "${CMAKE_SUPPRESS_REGENERATION}"      )
# the compiler flags for compiling C sources 
message( STATUS "CMAKE_C_FLAGS:\t\t\t"                  "${CMAKE_C_FLAGS}"                    )
# the compiler flags for compiling C++ sources 
message( STATUS "CMAKE_CXX_FLAGS:\t\t\t"                "${CMAKE_CXX_FLAGS}"                  )
# Choose the type of build.  Example: SET(CMAKE_BUILD_TYPE Debug) 
message( STATUS "CMAKE_BUILD_TYPE:\t"                   "${CMAKE_BUILD_TYPE}"                 )
# if this is set to ON, then all libraries are built as shared libraries by default.
message( STATUS "BUILD_SHARED_LIBS:\t"                  "${BUILD_SHARED_LIBS}"                )
# the compiler used for C files 
message( STATUS "CMAKE_C_COMPILER:\t\t\t"               "${CMAKE_C_COMPILER}"                 )
# the compiler used for C++ files 
message( STATUS "CMAKE_CXX_COMPILER:\t\t\t"             "${CMAKE_CXX_COMPILER}"               )
# if the compiler is a variant of gcc
message( STATUS "CMAKE_COMPILER_IS_GNUCC:\t\t"          "${CMAKE_COMPILER_IS_GNUCC}"          )
# if the compiler is a variant of g++
message( STATUS "CMAKE_COMPILER_IS_GNUCXX:\t\t"         "${CMAKE_COMPILER_IS_GNUCXX}"         )
# the tools for creating libraries 
message( STATUS "CMAKE_AR:\t\t\t\t"                     "${CMAKE_AR}"                         )
message( STATUS "CMAKE_RANLIB:\t\t\t"                   "${CMAKE_RANLIB}"                     )
# output directories
message( STATUS "CMAKE_ARCHIVE_OUTPUT_DIRECTORY:\t"     "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}"   )
message( STATUS "CMAKE_LIBRARY_OUTPUT_DIRECTORY:\t"     "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"   )
message( STATUS "CMAKE_OUTPUT_OUTPUT_DIRECTORY:\t"      "${CMAKE_OUTPUT_OUTPUT_DIRECTORY}"    )
# mbed library options
message( STATUS "WITH_MBED_RTOS:\t\t\t"                 "${WITH_MBED_RTOS}"                   )
message( STATUS "WITH_MBED_RPC:\t\t\t"                  "${WITH_MBED_RPC}"                    )
message( STATUS "WITH_MBED_USB:\t\t\t"                  "${WITH_MBED_USB}"                    )
message( STATUS "WITH_MBED_USB_HOST:\t\t\t"             "${WITH_MBED_USB_HOST}"               )
message( STATUS "WITH_MBED_DSP:\t\t\t"                  "${WITH_MBED_DSP}"                    )
message( STATUS "WITH_MBED_ETH:\t\t\t"                  "${WITH_MBED_ETH}"                    )
message( STATUS "WITH_MBED_FATFS:\t\t\t"                "${WITH_MBED_FATFS}"                  )
message( STATUS "WITH_MBED_UBLOX:\t\t\t"                "${WITH_MBED_UBLOX}"                  )
message( STATUS "WITH_MBED_CPPCHECK:\t\t\t"             "${WITH_MBED_CPPCHECK}"                )
# mbed target options
message( STATUS "MBED_TARGET_CORE:\t\t\t"               "${MBED_TARGET_CORE}"                 )
message( STATUS "MBED_TARGET_CORE_LOWERC:\t\t"          "${MBED_TARGET_CORE_LOWERC}"          )
message( STATUS "MBED_TARGET_CORE_UPPERC:\t\t"          "${MBED_TARGET_CORE_UPPERC}"          )
message( STATUS "MBED_TARGET_TOOLCHAINS:\t\t"           "${MBED_TARGET_TOOLCHAINS}"           )
message( STATUS "MBED_TARGET_CODE:\t\t\t"               "${MBED_TARGET_CODE}"                 )
message( STATUS "MBED_TARGET_PROGEN:\t\t\t"             "${MBED_TARGET_PROGEN}"               )
message( STATUS "MBED_TARGET_PROGEN_LOWERC:\t\t"        "${MBED_TARGET_PROGEN_LOWERC}"        )
message( STATUS "MBED_TARGET_PROGEN_UPPERC:\t\t"        "${MBED_TARGET_PROGEN_UPPERC}"        )
message( STATUS "MBED_TARGET_MACROS:\t\t\t"             "${MBED_TARGET_MACROS}"               )
message( STATUS "MBED_TARGET_VENDOR:\t\t\t"             "${MBED_TARGET_VENDOR}"               )
message( STATUS "MBED_TARGET_FAMILY:\t\t"               "${MBED_TARGET_FAMILY}"               )
