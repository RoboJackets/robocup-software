#ifndef _GRAPH_HPP_
#define _GRAPH_HPP_

#include <QWidget>
#include <QMutex>

#include <vector>

class Graph: public QWidget
{
	Q_OBJECT;
	
public:
	Graph(QWidget *parent = 0);
	
	std::vector<int> levels;
	QMutex mutex;
	
protected:
	void paintEvent(QPaintEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void drawInfo(QPainter &p, int n, int x, int y);
	
	int highlight;
};

#endif // _GRAPH_HPP_
