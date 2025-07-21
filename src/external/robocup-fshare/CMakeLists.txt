cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(rc-fshare STATIC
    "src/pid.cpp"
    "src/git_version.cpp")

# Robocup software uses it's own eigen find package for some reason, check for this
if (TARGET Eigen3::Eigen)
  # Use the imported target
    target_link_libraries(rc-fshare Eigen3::Eigen)
else (TARGET Eigen3::Eigen)
    target_include_directories(rc-fshare PUBLIC ${EIGEN_INCLUDE_DIR})
endif (TARGET Eigen3::Eigen)


# All headers are assumed to be needed for the interface (PUBLIC)
target_include_directories(rc-fshare PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# -fPIC
set_property(TARGET rc-fshare PROPERTY POSITION_INDEPENDENT_CODE ON)

# don't build unless `make git-version-test` explicitly called
add_executable(git-version-test EXCLUDE_FROM_ALL
    "test/git_version_test.cpp")
target_link_libraries(git-version-test rc-fshare)


# write git_version.cpp with the current git SHA and dirty status
# see http://www.cmake.org/pipermail/cmake/2010-July/038015.html
find_package(Git REQUIRED)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/src/git_version.cpp.in
    "#include \"rc-fshare/git_version.hpp\"\n"
    "const char *git_version_hash = \"@GIT_HASH@\";\n"
    "const char *git_version_short_hash = \"@GIT_SHORT_HASH@\";\n"
    "const char *git_head_date = \"@GIT_DATE@\";\n"
    "const char *git_head_author = \"@GIT_AUTHOR@\";\n"
    "const bool git_version_dirty = (bool)@GIT_DIRTY@;\n"
)
# create version.cmake, which is a cmake script to write git_version.cpp from the template file generated above
file(WRITE ${CMAKE_BINARY_DIR}/version.cmake
    "EXECUTE_PROCESS(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    EXECUTE_PROCESS(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        OUTPUT_VARIABLE GIT_SHORT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    EXECUTE_PROCESS(
        COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd
        OUTPUT_VARIABLE GIT_DATE
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    EXECUTE_PROCESS(
        COMMAND ${GIT_EXECUTABLE} log -1 --format=%aN
        OUTPUT_VARIABLE GIT_AUTHOR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    EXECUTE_PROCESS(
        COMMAND ${GIT_EXECUTABLE} diff --quiet
        RESULT_VARIABLE GIT_DIRTY
    )
    CONFIGURE_FILE(\${SRC} \${DST} @ONLY)
")

# We create the git_version.cpp file in two steps:
# First, we add a custom target that does the actual file generation using the cmake script file above
# CMake considers custom targets to always be "out of date", so they run all of the time.  This is good
# in this situation because the git state may change and we have no way of knowing, so we just rebuild each time.
# The second part is a custom command, which is a way of telling CMake how to build a file.  The custom
# command here "takes credit" for generating git_version.cpp, and adds the custom target as a dependency
# so it does the actual work.
# It'd be nice if CMake had a way to do this as a single custom command/target, but since it doesn't,
# this is the workaround.
add_custom_target(git_version
    ${CMAKE_COMMAND} -D SRC=${CMAKE_CURRENT_BINARY_DIR}/src/git_version.cpp.in
                     -D DST=${CMAKE_CURRENT_BINARY_DIR}/src/git_version.cpp
                     -P ${CMAKE_BINARY_DIR}/version.cmake
    COMMENT "Generating git version c++ file"
)
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/src/git_version.cpp
    DEPENDS git_version
)
