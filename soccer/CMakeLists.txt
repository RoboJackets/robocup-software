
file(GLOB SOCCER_SRC
    "*.cpp"
    "config/*.cpp"
    "gameplay/*.cpp"
    "modeling/*.cpp"
    "motion/*.cpp"
    "planning/*.cpp"
    "radio/*.cpp"
    )

# the LogViewer executable has a main() in its file, so we exclude it headers
# when we build the executable later, we'll add it back in
list(REMOVE_ITEM SOCCER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/LogViewer.cpp")

include_directories("${PROJECT_SOURCE_DIR}/common")
include_directories("${PROJECT_BINARY_DIR}/common")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB SOCCER_UIS "./ui/*.ui")
qt5_wrap_ui(SOCCER_UIS ${SOCCER_UIS})


qt5_add_resources(SOCCER_RSRC ui/main_icons.qrc)


# 'soccer' program
add_executable(soccer ${SOCCER_SRC} ${SOCCER_UIS} ${SOCCER_RSRC})


# Python
find_package(PythonInterp 3.2 REQUIRED)
target_link_libraries(soccer ${PYTHON_LIBRARIES})
find_package(PythonLibs 3.2 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(soccer ${PYTHON_LIBRARIES})

# Qt
qt5_use_modules(soccer Widgets Xml Core OpenGL Network)

# Our 'common' library
target_link_libraries(soccer common)

# OpenGL
target_link_libraries(soccer GL GLU glut)

# Boost Python
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system python-py34 REQUIRED)
target_link_libraries(soccer ${Boost_LIBRARIES})

# libusb
find_package(libusb-1.0 REQUIRED)
include_directories(${LIBUSB_1_INCLUDE_DIRS})
target_link_libraries(soccer ${LIBUSB_1_LIBRARIES})

# Other
target_link_libraries(soccer pthread bfd)


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