cmake_minimum_required(VERSION 3.5)
project(rj_robocup_rrt)

find_package(ament_cmake REQUIRED)
find_package(rj_robocup_cmake_utils)

add_subdirectory(external/rrt)

install(DIRECTORY ${RRT_INCLUDE_DIR}
        DESTINATION include/
        )

install(TARGETS
        RRT
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

ament_export_include_directories(${RRT_INCLUDE_DIR})
ament_export_libraries(RRT)
ament_package()

