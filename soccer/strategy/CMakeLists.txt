# Library
add_library(strategy)
target_sources(strategy PRIVATE
        coach/coach_node.cpp
        agent/agent_action_client.cpp
        agent/position/position.cpp
        agent/position/defense.cpp
        agent/position/offense.cpp
        agent/position/goalie.cpp
        agent/position/goal_kicker.cpp
        agent/position/penalty_player.cpp
        agent/position/waller.cpp
        agent/communication/communication.cpp)
target_link_libraries(strategy PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        soccer_common
        rc-fshare)
target_include_directories(strategy SYSTEM PUBLIC)
ament_target_dependencies(strategy PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Node
add_executable(coach_node
        coach/coach_node_main.cpp)
target_link_libraries(coach_node PUBLIC
        strategy)

add_executable(agent_action_client_node
        agent/agent_action_client_main.cpp)
target_link_libraries(agent_action_client_node PUBLIC
        strategy)