
include_directories("${PROJECT_SOURCE_DIR}/common")
include_directories("${PROJECT_SOURCE_DIR}/soccer")
include_directories("${PROJECT_SOURCE_DIR}/logging")

add_executable(simple_logger simple_logger.cpp)
qt5_use_modules(simple_logger Core Network Widgets)
target_link_libraries(simple_logger common)


set (LOG_CONVERTER_SRC
	"BSVLog.cpp"
	"RJLog.cpp"
	"SSLLog.cpp"
)
add_executable(log_converter ${LOG_CONVERTER_SRC} logConverter.cpp)
target_link_libraries(log_converter common)