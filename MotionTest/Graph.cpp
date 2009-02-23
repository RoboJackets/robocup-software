#include "Graph.hpp"

Graph::Graph(QWidget* parent)
        :QWidget(parent)
{
    _width =  width();
    _height = height();
    gridSpacingX = 20;
    gridSpacingY = 20;
}

Graph::~Graph()
{
}

void Graph::paintEvent(QPaintEvent* pe)
{
    int i;
    QPainter painter(this);

    painter.fillRect(pe->rect(),Qt::white);

    for(i = _height; i>0; i-=gridSpacingY)
    {
        painter.drawLine(0,i,_width,i);
    }
    for(i = _width; i>0; i-=gridSpacingX)
    {
        painter.drawLine(i,0,i,_height);
    }
}
