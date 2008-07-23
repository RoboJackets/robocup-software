#include <QPaintEvent>
#include <QPainter>
#include <QTime>

#include "Graph.hpp"

Graph::Graph(QWidget *parent): QWidget(parent)
{
	setMouseTracking(true);
	
	levels.resize(101);
	highlight = -1;
	
	QPalette p = palette();
	p.setColor(QPalette::Background, Qt::black);
	setPalette(p);
}

void Graph::paintEvent(QPaintEvent *e)
{
	QPainter p(this);
	
	unsigned int i = 0;
	for (unsigned int j = 1; j < levels.size(); ++j)
	{
		int y1 = height() - levels[i] * height() / 1024;
		int y2 = height() - levels[j] * height() / 1024;
		
		int x1 = width() * i / levels.size();
		int x2 = width() * j / levels.size();
		
		p.setPen(Qt::green);
		p.drawLine(x1, y1, x2, y2);
		
		if (j < (levels.size() - 1))
		{
			int left = levels[i];
			int right = levels[j + 1];
			static const int T = 50;
			if (levels[j] >= (left + T) && levels[j] >= (right + T))
			{
				p.setPen(Qt::gray);
				static const int R = 3;
				p.drawEllipse(x2 - R, y2 - R, R * 2, R * 2);
				drawInfo(p, j, x2 + 10, y2);
			}
		}
		
		i = j;
	}
	
	if (highlight >= 0 && highlight <= 100)
	{
		int x = highlight * width() / levels.size();
		p.setPen(Qt::blue);
		p.drawLine(x, 0, x, height());
		
		p.setPen(Qt::white);
		drawInfo(p, highlight, x + 5, 10);
	}
}

void Graph::drawInfo(QPainter &p, int n, int x, int y)
{
	p.drawText(x, y, QString("Ch %1").arg(n));
	p.drawText(x, y + 15, QString("%1 MHz").arg(902.62 + 0.25 * n));
	p.drawText(x, y + 30, QString("RSSI %1").arg(levels[n]));
}

void Graph::mouseMoveEvent(QMouseEvent *e)
{
	int ch = e->x() * levels.size() / width();
	if (ch != highlight)
	{
		highlight = ch;
		update();
	}
}
