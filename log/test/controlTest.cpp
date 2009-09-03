#include <stdio.h>
#include <QApplication>

#include "../LogControl.cpp"
#include "../TreeView.cpp"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	
	TreeView view;
	view.setFixedSize(200,400);
	
	view.show();
	
	return app.exec();
}
