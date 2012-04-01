#include "Physics/Environment.hpp"
#include "SimulatorWindow.hpp"

#include <QApplication>
#include <QFile>
#include <QThread>

#include <boost/shared_ptr.hpp>

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

/**
 * Create a new thread to act as a wrapper for the simulation
 */
class BulletDemoThread : public QThread {
protected:
	char ** argv_;
	int argc_;

public:
	typedef boost::shared_ptr<BulletDemoThread> shared_ptr;

	/** need to pass arguments through to glut */
	BulletDemoThread(int argc, char* argv[])
	: argv_(argv), argc_(argc)
	{

	}

private:
	// Re-implement the run function to start the process
	void run() {
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

}; // \class BulletDemoThread

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

	// create the thread for simulation
	BulletDemoThread::shared_ptr bullet_thread;

	Environment* env = new Environment(configFile, sendShared);

	struct sigaction act;
	memset(&act, 0, sizeof(act));
	act.sa_handler = quit;
	sigaction(SIGINT, &act, 0);

	SimulatorWindow win(env);
	win.show();

	// start up environment
	env->init();
	// TODO: start as separate thread

	if (enableGlut) {
		bullet_thread = BulletDemoThread::shared_ptr(new BulletDemoThread(argc, argv));
		bullet_thread->start();
	}
	
	int ret = app.exec();

	// cleanup
	delete env;

	if (enableGlut)
		bullet_thread->wait();

	return ret;
}
