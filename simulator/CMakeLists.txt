
# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

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
target_link_libraries(simulator common GL GLU glut BulletDynamics BulletCollision LinearMath)
qt5_use_modules(simulator Widgets Xml Core OpenGL Network)

# bullet physics library
find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIR})
target_link_libraries(simulator ${BULLET_LIBRARIES})
