cmake_minimum_required(VERSION 3.5)
project(grsim)

find_package(ament_cmake REQUIRED)
find_package(rj_robocup_cmake_utils)

add_subdirectory(external/grSim)

install(TARGETS
        ${app}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

ament_package()

