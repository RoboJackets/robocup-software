#include <bullet_demo/VehicleDemo.hpp>
#include <physics/GlutCamera.hpp>

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

SimulatorThread::SimulatorThread(int argc, char* argv[], const QString& configFile, bool sendShared)
: _argv(argv), _argc(argc), _env(new Environment(configFile, sendShared))
{
}

SimulatorThread::~SimulatorThread() {
	if (_env)
		delete _env;
}

void SimulatorThread::run() {
	_env->init();

	// Set up demo
	VehicleDemo* vehicleDemo = new VehicleDemo;
	vehicleDemo->initPhysics();

	// set up glut
	gSimpleApplication = vehicleDemo;
	int width = 640, height = 480;
	const char* title = "Bullet Vehicle Demo";
	glutInit(&_argc, _argv);
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

