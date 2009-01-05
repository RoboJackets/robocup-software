#include <QApplication>

#include <NxPhysics.h>
#include <stdio.h>

#include "Viewer.hpp"
#include "SimVision.hpp"
#include "Physics/Env.hpp"
#include "Config.hpp"


int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	Env* env = new Env();
	env->start();

	Config cfg("../config/sample.xml", env);

        SimVision simVision(env);
        simVision.start();

	Viewer win(env);
	win.setVisible(true);

        int ret = app.exec();

	//cleanup
	delete env;
        simVision.terminate();
        simVision.wait();

	return ret;
}
