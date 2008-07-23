project (radio_test)

add_definitions (-Werror)
add_definitions (-DQT_NO_KEYWORDS)

set (USE_PACKET true)
include ("../../common/CMakeLists.txt")

# needed for qt ui generated header files
include_directories (${PROJECT_BINARY_DIR})
include_directories (${PROJECT_SOURCE_DIR})

# enable QT networking features
set (QT_USE_QTNETWORK true)

# this command finds the Qt4 libraries and sets all required variables
find_package (Qt4 REQUIRED)

# add some useful macros and variables
# QT_USE_FILE is a variable defined by FIND_PACKAGE( Qt4 ). 
# It contains a path to CMake scripts
include (${QT_USE_FILE})

# add all source files here
SET (SRCS main.cpp Serial.cpp Graph.cpp Sample_Thread.cpp Radio.cpp)

qt4_wrap_cpp (MOC_SRCS Graph.hpp)

add_executable (${PROJECT_NAME} ${SRCS} ${MOC_SRCS})

#FIND_PACKAGE prepared QT_LIBRARIES variable for us:
target_link_libraries (${PROJECT_NAME} ${QT_LIBRARIES} ${PACKET})

#add_executable(sim sim.cpp serial.cpp)
