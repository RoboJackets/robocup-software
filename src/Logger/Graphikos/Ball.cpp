#include <Sizes.h>

#include "Ball.hpp"

using namespace Graphikos;

void Ball::paint(QPainter& painter, Geometry::Point2d pos)
{
	Ball::paint(painter, pos.x, pos.y);
}

void Ball::paint(QPainter& painter, float x, float y)
{
	painter.setPen(QColor(0xff, 0x40, 0));
	painter.setBrush(QColor(0xff,0x90,0x00));
	
	painter.drawEllipse(QRectF(-BALL_RADIUS+x, -BALL_RADIUS+y, BALL_DIAM, BALL_DIAM));
}
