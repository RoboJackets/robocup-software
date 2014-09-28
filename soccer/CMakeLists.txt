
file(GLOB SOCCER_SRC
    "*.cpp"
    "config/*.cpp"
    "gameplay/*.cpp"
    "modeling/*.cpp"
    "motion/*.cpp"
    "planning/*.cpp"
    "radio/*.cpp"
    )

# Exclude the two files that include a main() function - we'll add those in later for their respective executables
list(REMOVE_ITEM SOCCER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/LogViewer.cpp")
list(REMOVE_ITEM SOCCER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")


include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB SOCCER_UIS "${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui")
qt5_wrap_ui(SOCCER_UIS ${SOCCER_UIS})

qt5_add_resources(SOCCER_RSRC ui/main_icons.qrc)



# Python
include_directories(${PYTHON_INCLUDE_DIRS})

# Boost Python
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system python-py34 REQUIRED)

# libusb
include_directories(${LIBUSB_1_INCLUDE_DIRS})


# Build stand-alone soccer dylib
# This is linked into soccer and our unit tests, as well as being a python module
add_library(robocup SHARED ${SOCCER_SRC} ${SOCCER_UIS} ${SOCCER_RSRC})
set_target_properties(robocup PROPERTIES PREFIX "")

target_link_libraries(robocup common)
target_link_libraries(robocup ${PYTHON_LIBRARIES})
qt5_use_modules(robocup Widgets Xml Core OpenGL Network)
target_link_libraries(robocup ${LIBUSB_1_LIBRARIES})
target_link_libraries(robocup ${Boost_LIBRARIES})
target_link_libraries(robocup GL GLU glut)
target_link_libraries(robocup pthread bfd)


# 'soccer' program
add_executable(soccer main.cpp)
target_link_libraries(soccer robocup)

# Unit tests
add_subdirectory(tests)


# build the 'log_viewer' program
qt5_add_resources(LOG_VIEWER_RSRC ui/log_icons.qrc)
qt5_wrap_ui(LOG_VIEWER_UI ui/LogViewer.ui)
set(LOG_VIEWER_SRC
    "LogViewer.cpp"
    "FieldView.cpp"
   	"ProtobufTree.cpp"
   	"StripChart.cpp"
)
add_executable(log_viewer ${LOG_VIEWER_SRC} ${LOG_VIEWER_UI} ${LOG_VIEWER_RSRC})
qt5_use_modules(log_viewer Widgets OpenGL)
target_link_libraries(log_viewer common)
