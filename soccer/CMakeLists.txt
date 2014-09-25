


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


# Python
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


# 'soccer' program
add_executable(soccer ${SOCCER_SRC} ${SOCCER_UIS_HDRS})

# Qt
qt5_use_modules(soccer Widgets Xml Core OpenGL Network)

# Google Protobuf
target_link_libraries(soccer protobuf)

# OpenGL
target_link_libraries(soccer GL GLU glut)

# Boost Python
target_link_libraries(soccer boost_python3 python3)

# libusb
find_package(libusb-1.0 REQUIRED)
include_directories(${LIBUSB_1_INCLUDE_DIRS})
target_link_libraries(soccer ${LIBUSB_1_LIBRARIES})

# Other
target_link_libraries(soccer pthread bfd)
