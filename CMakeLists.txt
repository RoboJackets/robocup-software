cmake_minimum_required(VERSION 3.8)
project(rj_robocup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Don't fall back to older versions

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()


# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Google test
enable_testing()
include(SetupGTest)

message("${TARGET}")

add_subdirectory(external/grSim)

# Because we use ninja, we have to explicitly turn on color output for the compiler
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics -Werror=return-stack-address")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-local-addr")

endif()

# Use compiler optimization if we are making a release build.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native")

# Build in debug mode
# TODO: make this easier to specify when running `make` on the command-line
# try using cmake's debug flag support? 'set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")'
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Og")

# Turn some compiler warnings into errors
# TODO determine if we want these flags in the release build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Werror=return-type -Werror=delete-non-virtual-dtor")


# Qt5
find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)
message(STATUS "Found Qt5: ${Qt5Widgets_DIR}")

# Google Protobuf
find_package(Protobuf REQUIRED)
include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIR})

# Remove QT slots/signals/emit keywords
add_definitions(-DQT_NO_KEYWORDS)
# Python
find_package(PythonInterp 3.2 REQUIRED)
find_package(PythonLibs 3.2 REQUIRED)


# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)

# libusb
find_package(libusb-1.0 REQUIRED)

# SDL2
include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)

# Several things depend on the headers in the 'common' directory
include_directories("${PROJECT_SOURCE_DIR}/common") # for headers in common/
include_directories("${PROJECT_BINARY_DIR}/common") # for generated protobuf headers
include_directories("${CMAKE_BINARY_DIR}/common") # for generated protobuf headers


# setup ccache to speed up recompiles.  It's especially useful when switching back and forth
# between branches where you'd normally have to recompile things many times.
# see http://stackoverflow.com/questions/1815688/how-to-use-ccache-with-cmake
if(NOT STATIC_ANALYSIS)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

# run all the other CMakeLists files
add_subdirectory(external)
add_subdirectory(protobuf)
add_subdirectory(common)
add_subdirectory(msgs)
add_subdirectory(soccer)
add_subdirectory(logging)

# Installing Targets
install(TARGETS robocup
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS grSim soccer
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
        soccer/gameplay
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING
            PATTERN "*.py"
            PATTERN "*.pbk")

install(FILES
        soccer/config/realShopField.xml
        DESTINATION share/${PROJECT_NAME}/config
        RENAME soccer-real.cfg)

install(FILES
        soccer/config/sim.xml
        DESTINATION share/${PROJECT_NAME}/config
        RENAME soccer-sim.cfg)

install(FILES
        external/sdlcontrollerdb/gamecontrollerdb.txt
        DESTINATION share/${PROJECT_NAME})

ament_package()
