#
# A version of add_subdirectory() that is specialized for ROS2 packages that only contain message files.
# Instead of performing a normal add_subdirectory(), add_rosidl_subdirectory() will build and install the subdirectory
# at CONFIG time instead of BUILD / INSTALL time, so that dependents of that subdirectory are able to call
# find_package successfully.
#
function(add_rosidl_subdirectory subdirectory)
    # First create the subdirectory in the build folder
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${subdirectory})

    # Command for configuring the cmake project inside the subdirectory
    set(configure_command ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/${subdirectory}
        -B ${CMAKE_CURRENT_BINARY_DIR}/${subdirectory}
        -G ${CMAKE_GENERATOR}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

    set(build_and_install_command ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/${subdirectory}
        --target install)

    # Then call cmake on the subdirectory
    execute_process(COMMAND ${configure_command})

    # Finally build + install it
    execute_process(COMMAND ${build_and_install_command})

    # Also create a target so that it will be invoked on all builds (This is what rosidl_generate_interfaces also does
    # internally).
    add_custom_target(${subdirectory} ALL
        COMMAND ${build_and_install_command})
endfunction()
