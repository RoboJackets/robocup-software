include_directories("${PROJECT_SOURCE_DIR}/common")
include_directories("${PROJECT_SOURCE_DIR}/soccer")

add_executable(simple_logger simple_logger.cpp)
qt5_use_modules(simple_logger Core Network Widgets)
target_link_libraries(simple_logger common rc-fshare)

add_executable(log_converter log_converter.cpp)
target_link_libraries(log_converter common rc-fshare)

# We need XML because Context includes configurables,
# which uses XML to represent the tree.
qt5_use_modules(log_converter Xml Core)
