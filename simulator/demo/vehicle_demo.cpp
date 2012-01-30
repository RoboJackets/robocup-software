/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include "VehicleDemo.hpp"
#include "GlutCamera.hpp"

static VehicleDemo* gSimpleApplication = 0;

static void glutKeyboardCallback(unsigned char key, int x, int y) {
	gSimpleApplication->keyboardCallback(key, x, y);
}

static void glutKeyboardUpCallback(unsigned char key, int x, int y) {
	gSimpleApplication->keyboardUpCallback(key, x, y);
}

static void glutSpecialKeyboardCallback(int key, int x, int y) {
	gSimpleApplication->specialKeyboard(key, x, y);
}

static void glutSpecialKeyboardUpCallback(int key, int x, int y) {
	gSimpleApplication->specialKeyboardUp(key, x, y);
}

static void glutReshapeCallback(int w, int h) {
	gSimpleApplication->camera()->reshape(w, h);
}

static void glutMoveAndDisplayCallback() {
	gSimpleApplication->clientMoveAndDisplay();
}

static void glutDisplayCallback(void) {
	gSimpleApplication->camera()->displayCallback();
}

int main(int argc, char** argv) {

	// Set up demo
	VehicleDemo* vehicleDemo = new VehicleDemo;
	vehicleDemo->initPhysics();

	// set up glut
	gSimpleApplication = vehicleDemo;
	int width = 640, height = 480;
	const char* title = "Bullet Vehicle Demo";
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow(title);
#ifdef BT_USE_FREEGLUT
	glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

	gSimpleApplication->camera()->myinit();

	glutKeyboardFunc(glutKeyboardCallback);
	glutKeyboardUpFunc(glutKeyboardUpCallback);
	glutSpecialFunc(glutSpecialKeyboardCallback);
	glutSpecialUpFunc(glutSpecialKeyboardUpCallback);

	glutReshapeFunc(glutReshapeCallback);
	glutIdleFunc(glutMoveAndDisplayCallback);
	glutDisplayFunc(glutDisplayCallback);

	glutMoveAndDisplayCallback();

	glutMainLoop();
	return 0;
}
