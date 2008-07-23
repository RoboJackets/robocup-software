#ifndef _SAMPLE_THREAD_HPP_
#define _SAMPLE_THREAD_HPP_

#include <QThread>

class Graph;
class Radio;

class Sample_Thread: public QThread
{
public:
	Sample_Thread(Graph *graph, Radio *radio);
	
	virtual bool event(QEvent *e);
	void stop();
	
	volatile bool sna;
	
protected:
	Radio *_radio;
	Graph *_graph;
	volatile bool _run;
	
	void run();
};

#endif // _SAMPLE_THREAD_HPP_
