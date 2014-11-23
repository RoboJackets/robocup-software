
include_directories("${PROJECT_SOURCE_DIR}/common")

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

add_executable(simple_logger simple_logger.cpp)
qt5_use_modules(simple_logger Core Network)
target_link_libraries(simple_logger common)
