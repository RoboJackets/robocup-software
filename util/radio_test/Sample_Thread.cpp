#include "Sample_Thread.hpp"
#include "Graph.hpp"
#include "Radio.hpp"

#include <QTime>
#include <QApplication>
#include <QEvent>

using namespace std;

Sample_Thread::Sample_Thread(Graph *graph, Radio *radio)
{
	sna = false;
	_run = true;
	_graph = graph;
	_radio = radio;
}

bool Sample_Thread::event(QEvent *e)
{
	_graph->update();
}

void Sample_Thread::stop()
{
	_run = false;
	wait();
}

void Sample_Thread::run()
{
	vector<int> levels;
	levels.resize(101);
	
	_radio->command_mode(true);
	
	while (_run)
	{
		QTime start = QTime::currentTime();
		
		if (sna)
		{
			_radio->set_channels(0, 0);
			for (int i = 0; i <= 100; ++i)
			{
				levels[i] = _radio->set_channels(i + 1, i + 1);
			}
		} else {
			_radio->set_channels(0, Radio::Off);
			for (int i = 0; i <= 100; ++i)
			{
				levels[i] = _radio->set_channels(i + 1, Radio::Off);
			}
		}
		
		_graph->mutex.lock();
		_graph->levels = levels;
		_graph->mutex.unlock();
		
		qApp->postEvent(this, new QEvent(QEvent::User));
		
		int ms = start.msecsTo(QTime::currentTime());
		printf("%d ms\n", ms);
	}
	
	_radio->set_channels(Radio::Off, Radio::Off);
}
