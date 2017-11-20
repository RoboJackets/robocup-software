
include_directories("${PROJECT_SOURCE_DIR}/common")
include_directories("${PROJECT_SOURCE_DIR}/soccer")

add_executable(simple_logger simple_logger.cpp)
qt5_use_modules(simple_logger Core Network Widgets)
target_link_libraries(simple_logger common)

add_executable(log_converter log_converter.cpp)
target_link_libraries(log_converter common)