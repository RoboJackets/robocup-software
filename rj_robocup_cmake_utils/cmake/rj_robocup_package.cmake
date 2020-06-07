#
# Standard RJ Robocup ROS2 package setup
#
# @public
#
macro(rj_robocup_package)
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        message(STATUS "Setting build type to Release as none was specified.")
        set(CMAKE_BUILD_TYPE "Release" CACHE
                STRING "Choose the type of build." FORCE)
        # Set the possible values of build type for cmake-gui
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    endif()

    # Default to C99
    if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
    endif()

    # Default to C++17
    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
    endif()

    option(COVERAGE_ENABLED "Enable code coverage" FALSE)
    if(COVERAGE_ENABLED)
        add_compile_options(--coverage)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
    endif()
endmacro()
