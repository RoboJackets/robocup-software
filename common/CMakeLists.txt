project(common)

set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)

# defines geometry2d library
add_subdirectory(geometry2d)

# defines rj_robocup_network library
add_subdirectory(network)

# defines common library
add_subdirectory(common)
