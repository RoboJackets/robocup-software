#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <QWidget>
#include <QPaintEvent>
#include <QPainter>

class Graph : public QWidget
{
    Q_OBJECT;
    public:
	Graph(QWidget* parent = 0);
	~Graph();

    public:
        //Paints a set of vectors onto the graph
	void paintEvent(QPaintEvent* pe);
	int gridSpacingX;
        int gridSpacingY;

    private:
        int _width;
        int _height;

};

#endif
