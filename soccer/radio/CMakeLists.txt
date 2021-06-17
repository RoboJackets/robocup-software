# Library
add_library(radio)
target_sources(radio PRIVATE
        network_radio.cpp
        packet_convert.cpp
        radio.cpp
        sim_radio.cpp)
target_link_libraries(radio PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        soccer_common
        rc-fshare)
target_include_directories(radio SYSTEM PUBLIC)
ament_target_dependencies(radio PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Node
add_executable(sim_radio_node
        sim_radio_node_main.cpp)
target_link_libraries(sim_radio_node PUBLIC
        radio
        config_client)

add_executable(network_radio_node
        network_radio_node_main.cpp)
target_link_libraries(network_radio_node PUBLIC
        radio
        config_client)
