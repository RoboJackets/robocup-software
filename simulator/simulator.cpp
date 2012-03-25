#include "Physics/Environment.hpp"
#include "SimulatorWindow.hpp"

#include <QApplication>
#include <QFile>

#include <stdio.h>
#include <signal.h>

#include <bullet_demo/VehicleDemo.hpp>
#include <bullet_demo/GlutCamera.hpp>

using namespace std;

void quit(int signal)
{
	fprintf(stderr, "Exiting due to signal %d\n", signal);
	exit(0);
}

void usage(const char* prog)
{
	fprintf(stderr, "usage: %s [-c <config file>] [--glut] [--sv]\n", prog);
	fprintf(stderr, "\t--help  Show usage message\n");
	fprintf(stderr, "\t--glut  Use GLUT rendering\n");
	fprintf(stderr, "\t--sv    Use shared vision multicast port\n");
}

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

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	QString configFile = "simulator.cfg";
	bool sendShared = false;
	bool enableGlut = false;

	//loop arguments and look for config file
	for (int i=1 ; i<argc ; ++i)
	{
		if (strcmp(argv[i], "--help") == 0)
		{
			usage(argv[0]);
			return 1;
		} else if (strcmp(argv[i], "--sv") == 0)
		{
			sendShared = true;
		} else if (strcmp(argv[i], "--glut") == 0)
		{
			enableGlut = true;
		} else if (strcmp(argv[i], "-c") == 0)
		{
			++i;
			if (i < argc)
			{
				configFile = argv[i];
			}
			else
			{
				printf ("Expected config file after -c parameter\n");
				return 1;
			}
		} else if ((strcmp(argv[i], "--h") == 0) || (strcmp(argv[i], "--help") == 0))
		{
			usage(argv[0]);
			return 0;
		} else {
			printf("%s is not recognized as a valid flag\n", argv[i]);
			return 1;
		}
	}

	if (enableGlut) {
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

		gSimpleApplication->camera()->myinit();

		glutKeyboardFunc(glutKeyboardCallback);
		glutKeyboardUpFunc(glutKeyboardUpCallback);
		glutSpecialFunc(glutSpecialKeyboardCallback);
		glutSpecialUpFunc(glutSpecialKeyboardUpCallback);

		glutReshapeFunc(glutReshapeCallback);
		glutIdleFunc(glutMoveAndDisplayCallback);
		glutDisplayFunc(glutDisplayCallback);

		glutMoveAndDisplayCallback();
	}

	Environment* env = new Environment(configFile, sendShared);

	struct sigaction act;
	memset(&act, 0, sizeof(act));
	act.sa_handler = quit;
	sigaction(SIGINT, &act, 0);

	SimulatorWindow win(env);
	win.show();
	
	// execute things - TODO: multiple threads?
//	if (enableGlut)
//		glutMainLoop();

	int ret = app.exec();

	// cleanup
	delete env;

	return ret;
}
