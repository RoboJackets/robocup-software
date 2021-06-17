# Library
add_library(referee)
target_sources(referee PRIVATE
        external_referee.cpp
        internal_referee.cpp
        referee_base.cpp)
target_link_libraries(referee PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        soccer_common)
target_include_directories(referee SYSTEM PUBLIC)
ament_target_dependencies(referee PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Node
add_executable(internal_referee_node
        internal_referee_main.cpp)
target_link_libraries(internal_referee_node PUBLIC
        referee)

add_executable(external_referee_node
        external_referee_main.cpp)
target_link_libraries(external_referee_node PUBLIC
        referee)
