
include_directories("${PROJECT_SOURCE_DIR}/common")

add_executable(simple_logger simple_logger.cpp)
qt5_use_modules(simple_logger Core Network Widgets)
target_link_libraries(simple_logger common)
