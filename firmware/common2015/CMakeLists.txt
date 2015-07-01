
# set the arm toolchain
set(CMAKE_TOOLCHAIN_FILE ../mbed/arm_toolchain.cmake)
include(../mbed/arm_toolchain.cmake)

set(common2015_SRC
    "drivers/CC1101/CC1101.cpp"
    "modules/CommLink/CommLink.cpp"
    "modules/CommModule/CommModule.cpp"
    "utils/FirmwareCheck/FirmwareCheck.cpp"
    "utils/Logger/logger.cpp"
    "utils/RTOS_Management/FunctionPointerRJ.cpp"
    "utils/RTOS_Management/ThreadHelper.cpp"
)

# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
foreach(mbed_obj ${MBED_OBJECTS})
    add_custom_command(
        OUTPUT ${mbed_obj}
        DEPENDS mbed_libraries
        COMMAND "")
endforeach()

# build the 'common2015' library that contains all the MBED stuff needed for
# both the base station and robot firmware
add_library(common2015 ${common2015_SRC} ${CMAKE_BINARY_DIR}/common/git_version.cpp)
add_dependencies(common2015 mbed_libraries)
target_link_libraries(common2015 ${MBED_OBJECTS})
target_link_libraries(common2015 ${MBED_LIBS})

# only build firmware library if required
set_target_properties(common2015 PROPERTIES EXCLUDE_FROM_ALL TRUE)

# See common/CMakeLists for more info on how this works
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/common/git_version.cpp
    DEPENDS git_version
)
