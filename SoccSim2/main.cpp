#include <QApplication>

#include <NxPhysics.h>
#include <stdio.h>

#include "Viewer.hpp"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	Viewer win;
	win.setVisible(true);
	
	return app.exec();
}
