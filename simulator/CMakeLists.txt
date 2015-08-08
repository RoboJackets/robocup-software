
# Qt
set(CMAKE_AUTOMOC ON)

set(simulator_SRC
    "bullet_opengl/DemoApplication.cpp"
    "bullet_opengl/GL_DialogDynamicsWorld.cpp"
    "bullet_opengl/GL_DialogWindow.cpp"
    "bullet_opengl/GL_ShapeDrawer.cpp"
    "bullet_opengl/GL_Simplex1to4.cpp"
    "bullet_opengl/GLDebugDrawer.cpp"
    "bullet_opengl/GLDebugFont.cpp"
    "bullet_opengl/GlutDemoApplication.cpp"
    "bullet_opengl/GlutStuff.cpp"
    "bullet_opengl/RenderTexture.cpp"
    "physics/Ball.cpp"
    "physics/Entity.cpp"
    "physics/Environment.cpp"
    "physics/FastTimer.cpp"
    "physics/Field.cpp"
    "physics/GlutCamera.cpp"
    "physics/RaycastVehicle.cpp"
    "physics/Robot.cpp"
    "physics/RobotBallController.cpp"
    "physics/SimEngine.cpp"
    "RobotTableModel.cpp"
    "simulator.cpp"
    "SimulatorGLUTThread.cpp"
    "SimulatorWindow.cpp"
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bullet_opengl)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})    # for Qt automoc files

# qt5 ui files
file(GLOB simulator_UIS ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui)
qt5_wrap_ui(simulator_UIS ${simulator_UIS})

# qt 5 resource files
qt5_add_resources(simulator_RSRC ui/main_icons.qrc)

# simulator program
if(APPLE)
    include(BundleUtilities)

    # app icon
    set(OSX_ICON_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/SimIcon.icns)
    set_source_files_properties(${OSX_ICON_FILES} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    set(MACOSX_BUNDLE_ICON_FILE SimIcon)

    set(MACOSX_BUNDLE_GUI_IDENTIFIER "org.robojackets.robocup.simulator")

    add_executable(simulator MACOSX_BUNDLE ${simulator_SRC} ${simulator_UIS} ${simulator_RSRC} ${OSX_ICON_FILES})

    # add a script called "simulator" in the run/ dir that launches the simulator app and passes on all args,
    # so it can be used the same way the simulator executable is used on Linux
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/simulator
        "#!/bin/sh\n"
        "./simulator.app/Contents/MacOS/simulator $@")
    file(INSTALL ${CMAKE_CURRENT_BINARY_DIR}/simulator
        DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        FILE_PERMISSIONS OWNER_READ GROUP_READ OWNER_WRITE GROUP_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE)
else()
    add_executable(simulator ${simulator_SRC} ${simulator_RSRC} ${simulator_UIS})
endif()



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
