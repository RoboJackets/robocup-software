#include <QApplication>

#include <NxPhysics.h>
#include <stdio.h>

#include "Viewer.hpp"
#include "Physics/Env.hpp"
#include "Config.hpp"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	
	Env* env = new Env();
	env->start();
	
	Config cfg("../config/sample.xml", env);

	Viewer win(env);
	win.setVisible(true);
	
	int ret = app.exec();

	//cleanup
	delete env;
	
	return ret;
}
