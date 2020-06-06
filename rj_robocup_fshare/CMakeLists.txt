cmake_minimum_required(VERSION 3.5)
project(rj_robocup_fshare)

find_package(ament_cmake REQUIRED)
find_package(rj_robocup_cmake_utils)

add_subdirectory(external/robocup-fshare)

set(FSHARE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/robocup-fshare/include)
install(DIRECTORY ${FSHARE_INCLUDE_DIR}
        DESTINATION include/
        )

install(TARGETS
        rc-fshare
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

ament_export_include_directories(${FSHARE_INCLUDE_DIR})
ament_export_libraries(rc-fshare)
ament_package()

