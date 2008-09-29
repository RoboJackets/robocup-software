#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <QWidget>
#include <QPaintEvent>

class Graph : public QWidget
{
    public:
	Graph(QWidget* parent = 0);
	~Graph();

        //Paints a set of vectors onto the graph
	void paintEvent(QPaintEvent* pe);

    private:
	//position of the object
	float _pos;
        int gridSpacing;
        int xMax;
        int xMin;
        int yMax;
        int yMin;
};

#endif