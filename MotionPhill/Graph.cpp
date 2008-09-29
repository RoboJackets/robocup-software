#include "Graph.hpp"

#include <QPainter>

Graph(QWidget* parent = 0)
{
}

void paintEvent(QPaintEvent* pe)
{
    QPainter painter(this);
    painter.fillRect(pe->rect(), Qt::white);
}