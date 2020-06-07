# Library and headers
add_library(rj_robocup_network multicast.cpp)
target_include_directories(rj_robocup_network PUBLIC ${COMMON_INCLUDE_DIR})
