# Library
add_library(joystick)
target_sources(joystick PRIVATE
        sdl_manual_control.cpp)
target_link_libraries(joystick PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        ${SDL2_LIBRARIES}
        soccer_common)
target_include_directories(joystick SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRECTORIES} ${SDL2_INCLUDE_DIRS})
ament_target_dependencies(joystick PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Node
add_executable(joystick_node
        manual_control_node.cpp
        manual_control_node_main.cpp)
target_link_libraries(joystick_node PUBLIC
        joystick
        config_client)