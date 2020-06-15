#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_common LANGUAGES CXX)

#======================================================================
# Find package
#======================================================================
find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)

#======================================================================
# Define Targets
#======================================================================
add_library(rj_common STATIC)

# position-independent-code flag
target_compile_options(rj_common PRIVATE "-fPIC")

add_subdirectory(src)

#======================================================================
# Dependencies List
#======================================================================
set(RJ_COMMON_DEPS_SYSTEM_INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIRS})

set(RJ_COMMON_DEPS_SYSTEM_LIBRARIES
    Qt5::Core
    Qt5::Network
    Qt5::Widgets)

set(RJ_COMMON_DEPS_INCLUDE_DIRS
    include)

set(RJ_COMMON_DEPS_LIBRARIES
    geometry2d
    rj_constants)

#======================================================================
# Include and Linking
#======================================================================
target_include_directories(rj_common
    SYSTEM PUBLIC
    ${RJ_COMMON_DEPS_SYSTEM_INCLUDE_DIRS})
target_include_directories(rj_common
    PUBLIC
    ${RJ_COMMON_DEPS_INCLUDE_DIRS})

target_link_libraries(rj_common
    PUBLIC
    ${RJ_COMMON_DEPS_SYSTEM_LIBRARIES}
    ${RJ_COMMON_DEPS_LIBRARIES})

#======================================================================
# Packaging
#======================================================================
install(DIRECTORY include/
    DESTINATION include/)

install( TARGETS rj_common
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
