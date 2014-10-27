
file(GLOB simulator_SRC
    "*.cpp"
    "physics/*.cpp"
    "bullet_opengl/*.cpp"
    )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bullet_opengl)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# qt5 ui files
file(GLOB simulator_UIS ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui)
qt5_wrap_ui(simulator_UIS ${simulator_UIS})

# simulator program
add_executable(simulator ${simulator_SRC} ${simulator_UIS})
target_link_libraries(simulator GL GLU glut BulletDynamics, BulletCollision LinearMath)

# bullet physics library
find_package(bullet REQUIRED)
include_directories(${bullet_HEADERS}) # FIXME: ???
