#include "VehicleDemo.hpp"
#include <GLDebugDrawer.h>
#include <btBulletDynamicsCommon.h>
GLDebugDrawer gDebugDrawer;

static SimpleApplication* gSimpleApplication = 0;

int glutmain(int argc, char **argv, int width, int height, const char* title,
		SimpleApplication* demoApp);

int main(int argc, char** argv) {

	VehicleDemo* vehicleDemo = new VehicleDemo;

	vehicleDemo->initPhysics();
	vehicleDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

	return glutmain(argc, argv, 640, 480,
			"Bullet Vehicle Demo. http://www.continuousphysics.com/Bullet/phpBB2/",
			vehicleDemo);
}

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
	gSimpleApplication->reshape(w, h);
}

static void glutMoveAndDisplayCallback() {
	gSimpleApplication->moveAndDisplay();
}

static void glutDisplayCallback(void) {
	gSimpleApplication->displayCallback();
}

int glutmain(int argc, char **argv, int width, int height, const char* title,
		SimpleApplication* demoApp) {

	gSimpleApplication = demoApp;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow(title);
#ifdef BT_USE_FREEGLUT
	glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

	gSimpleApplication->myinit();

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
