#include <bullet_demo/VehicleDemo.hpp>
#include <bullet_demo/GlutCamera.hpp>

#include <SimulatorThread.hpp>

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

SimulatorThread::SimulatorThread(int argc, char* argv[])
: argv_(argv), argc_(argc)
{

}

void SimulatorThread::run() {
	// Set up demo
	VehicleDemo* vehicleDemo = new VehicleDemo;
	vehicleDemo->initPhysics();

	// set up glut
	gSimpleApplication = vehicleDemo;
	int width = 640, height = 480;
	const char* title = "Bullet Vehicle Demo";
	glutInit(&argc_, argv_);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow(title);

	gSimpleApplication->camera()->myinit();

	glutKeyboardFunc(glutKeyboardCallback);
	glutKeyboardUpFunc(glutKeyboardUpCallback);
	glutSpecialFunc(glutSpecialKeyboardCallback);
	glutSpecialUpFunc(glutSpecialKeyboardUpCallback);

	glutReshapeFunc(glutReshapeCallback);
	glutIdleFunc(glutMoveAndDisplayCallback);
	glutDisplayFunc(glutDisplayCallback);

	glutMoveAndDisplayCallback();

	// Actually start loop
	glutMainLoop();
}

