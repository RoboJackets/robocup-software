#include "Graph.hpp"
#include "Radio.hpp"
#include "Sample_Thread.hpp"

#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	
	if (argc != 2)
	{
		printf("Usage: radio_test <port>\n");
		return 1;
	}
	
	Radio *radio = new Radio(argv[1]);
	
	Graph *graph = new Graph();
	Sample_Thread *thread = new Sample_Thread(graph, radio);
	thread->start();
	graph->show();
	
	app.exec();
	
	thread->stop();
	
	return 0;
}
