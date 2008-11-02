#include <QApplication>
#include <Team.h>
#include "MainWindow.hpp"

void usage(const char* prog)
{
	printf("Usage: %s [-b|-y]\n", prog);
}

int main(int argc,char* argv[])
{
        QApplication app(argc, argv);

	Team t = Yellow;

	if (argc != 2 || strlen(argv[1]) != 2)
	{
		usage(argv[0]);
		return -1;
	}

	if (strncmp(argv[1], "-b", 2) == 0)
	{
		t = Blue;
	}
	else if (strncmp(argv[1], "-y", 2) != 0)
	{
		usage(argv[0]);
		return -1;
	}

	MainWindow win(t);
	win.show();

	return app.exec();
}
