
add_subdirectory(protobuf)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/protobuf")


file(GLOB_RECURSE COMMON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE COMMON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# positon-independent-code flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# build the 'common' static library
add_library(common STATIC ${COMMON_SRC} ${COMMON_INCLUDE})
target_link_libraries(common proto_messages)
qt5_use_modules(common Core Network)


# write git_version.h with the current git SHA and dirty status
# see http://www.cmake.org/pipermail/cmake/2010-July/038015.html
find_package(Git)
file(WRITE ${CMAKE_BINARY_DIR}/git_version.h.in
    "static const char git_version_hash[] = \"@GIT_HASH@\";\n"
    "static bool git_version_dirty = (bool)@GIT_DIRTY@;\n"
)
file(WRITE ${CMAKE_BINARY_DIR}/version.cmake
    "EXECUTE_PROCESS(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    EXECUTE_PROCESS(
        COMMAND ${GIT_EXECUTABLE} diff --quiet
        RESULT_VARIABLE GIT_DIRTY
    )
    CONFIGURE_FILE(\${SRC} \${DST} @ONLY)
")
add_custom_target(git_version
    ${CMAKE_COMMAND} -D SRC=${CMAKE_BINARY_DIR}/git_version.h.in
                     -D DST=${CMAKE_BINARY_DIR}/common/git_version.h
                     -P ${CMAKE_BINARY_DIR}/version.cmake
)
add_dependencies(common git_version)
