cmake_minimum_required(VERSION 3.16)
project(robocup-fshare-wrapper)

# Add external_lib1 as a subdirectory
add_subdirectory(${CMAKE_SOURCE_DIR}/../external_lib1 external_lib1_build)

# Install the library
install(TARGETS rc-fshare EXPORT rc-fshareConfig
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Export CMake package
install(EXPORT rc-fshareConfig NAMESPACE rc-fshare::
    DESTINATION share/rc-fshare/cmake
)
