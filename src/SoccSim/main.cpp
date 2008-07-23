#include <QApplication>

#include <stdlib.h>
#include <time.h>

#include "SoccSimWindow.hpp"

using namespace std;

int main(int argc,char **argv)
{
	QApplication application(argc,argv);

	srand(time(0));
	
	SoccSimWindow win;
	win.show();
	
	return application.exec();
}
