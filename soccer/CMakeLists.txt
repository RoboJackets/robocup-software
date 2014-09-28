


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

# headers from 'common'
include_directories("${PROJECT_SOURCE_DIR}/common")
include_directories("${PROJECT_BINARY_DIR}/common")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB SOCCER_UIS "./ui/*.ui")
file(GLOB SOCCER_QT_QRC "./ui/*.qrc")

include_directories(${CMAKE_CURRENT_BINARY_DIR})

qt5_wrap_ui(SOCCER_UIS_HDRS ${SOCCER_UIS})


# 'soccer' program
add_executable(soccer ${SOCCER_SRC} ${SOCCER_UIS_HDRS})


# Python
find_package(PythonInterp 3.2 REQUIRED)
target_link_libraries(soccer ${PYTHON_LIBRARIES})
find_package(PythonLibs 3.2 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(soccer ${PYTHON_LIBRARIES})

# Qt
qt5_use_modules(soccer Widgets Xml Core OpenGL Network)

# Google Protobuf
target_link_libraries(soccer protobuf)
target_link_libraries(soccer proto_messages)

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
