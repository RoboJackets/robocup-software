INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
)

LINK_LIBRARIES(
 OpenGLSupport  
 BulletDynamics  
 BulletCollision 
 LinearMath  
 glut 
 GL 
 GLU
)

ADD_EXECUTABLE(AppBasicDemo
	main.cpp
	BasicDemo.cpp 
	BasicDemo.h
)
