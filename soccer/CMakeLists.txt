project (Soccer)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -fno-inline -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
set (USE_CONFIG true)

include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

#Module Include Directories
# Add a new line here for every module used
include_directories (${PROJECT_SOURCE_DIR}/..)

add_subdirectory (${PROJECT_SOURCE_DIR}/../motion ${PROJECT_BINARY_DIR}/motion)
add_subdirectory (${PROJECT_SOURCE_DIR}/../log ${PROJECT_BINARY_DIR}/log)
add_subdirectory (${PROJECT_SOURCE_DIR}/../modeling ${PROJECT_BINARY_DIR}/modeling)
add_subdirectory (${PROJECT_SOURCE_DIR}/../gameplay ${PROJECT_BINARY_DIR}/gameplay)
add_subdirectory (${PROJECT_SOURCE_DIR}/../stateID ${PROJECT_BINARY_DIR}/stateID)

include_directories(${PROJECT_BINARY_DIR}/motion)
include_directories(${PROJECT_BINARY_DIR}/log)
include_directories(${PROJECT_BINARY_DIR}/gameplay)

include_directories(${PROJECT_BINARY_DIR})

set (SRCS
	main.cpp
	Processor.cpp
	MainWindow.cpp
	InputHandler.cpp
	JoystickInput.cpp
    RefereeModule.cpp
)

qt4_automoc (${SRCS})

qt4_wrap_ui (UI_HDRS
    ui/MainWindow.ui
    ui/RefereeTab.ui
)

qt4_add_resources (RC_FILES
	ui/ref_icons.qrc
	ui/main_icons.qrc
)

set (target soccer)
add_executable (${target} ${SRCS} ${MOC_SRCS} ${UI_HDRS} ${RC_FILES})

target_link_libraries(${target} ${PACKET_LIBRARY})
target_link_libraries(${target} ${FRAMEWORK_LIBRARY})
target_link_libraries(${target} ${GEOMETRY_LIBRARY})
target_link_libraries(${target} ${NETWORK_LIBRARY})
target_link_libraries(${target} ${CONFIG_LIBRARY})

target_link_libraries(${target} ${QT_LIBRARIES})

#Module Linking
# Add a new line here for every module used
target_link_libraries(${target} log)
target_link_libraries(${target} motion)
target_link_libraries(${target} modeling)
target_link_libraries(${target} gameplay)
target_link_libraries(${target} stateID)

# External libraries
target_link_libraries(${target} antlr)
target_link_libraries(${target} boost_filesystem-mt)
