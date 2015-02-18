
# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

# Qt
set(CMAKE_AUTOMOC ON)

file(GLOB simulator_SRC
    "*.cpp"
    "physics/*.cpp"
    "bullet_opengl/*.cpp"
    )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bullet_opengl)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})    # for Qt automoc files

# qt5 ui files
file(GLOB simulator_UIS ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui)
qt5_wrap_ui(simulator_UIS ${simulator_UIS})

# simulator program
add_executable(simulator ${simulator_SRC} ${simulator_UIS})
target_link_libraries(simulator common BulletDynamics BulletCollision LinearMath)
qt5_use_modules(simulator Widgets Xml Core OpenGL Network)

# bullet physics library
find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIR})
target_link_libraries(simulator ${BULLET_LIBRARIES})

# handle OpenGL stuff separately on OS X vs Linux
target_link_libraries(simulator glut)
if(APPLE)
    link_directories(/opt/X11/lib/)
    include_directories(/opt/X11/include/)

    include_directories(/System/Library/Frameworks)
    find_library(OpenGL_LIBRARY OpenGL)
    target_link_libraries(simulator ${OpenGL_LIBRARY})
else()
    target_link_libraries(simulator GL GLU)
endif()
