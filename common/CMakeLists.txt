
add_subdirectory(protobuf)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/protobuf")

set(COMMON_SRC
    "Field_Dimensions.cpp"
    "Geometry2d/Circle.cpp"
    "Geometry2d/Line.cpp"
    "Geometry2d/Rect.cpp"
    "Geometry2d/TransformMatrix.cpp"
    "Geometry2d/CompositeShape.cpp"
    "Geometry2d/Polygon.cpp"
    "Geometry2d/Segment.cpp"
    "multicast.cpp"
    "Pid.cpp"
    "Utils.cpp"
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# positon-independent-code flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# build the 'common' static library (and include our protobuf messages in it)
add_library(common STATIC ${COMMON_SRC} git_version.cpp)
target_link_libraries(common proto_messages)
qt5_use_modules(common Core Network Widgets)


# write git_version.cpp with the current git SHA and dirty status
# see http://www.cmake.org/pipermail/cmake/2010-July/038015.html
find_package(Git REQUIRED)
file(WRITE ${CMAKE_BINARY_DIR}/git_version.cpp.in
    "#include <git_version.hpp>\n"
    "const char *git_version_hash = \"@GIT_HASH@\";\n"
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
    ${CMAKE_COMMAND} -D SRC=${CMAKE_BINARY_DIR}/git_version.cpp.in
                     -D DST=${CMAKE_BINARY_DIR}/common/git_version.cpp
                     -P ${CMAKE_BINARY_DIR}/version.cmake
    COMMENT "Generating git version c++ file"
)
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/git_version.cpp
    DEPENDS git_version
)
