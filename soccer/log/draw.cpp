#include "draw.hpp"

#include <Constants.hpp>

#include <QColor>
#include <QRectF>
#include <QLineF>

using namespace Constants;

void drawRobot(QPainter& painter, Team t, int ID, Geometry2d::Point pos, float theta, Team viewTeam, bool haveBall)
{
	painter.setPen(Qt::white);
	painter.setBrush(Qt::NoBrush);
	
	painter.save();
	
	painter.translate(pos.x, pos.y);
	
	painter.save();
    painter.rotate((viewTeam == Blue) ? -90 : 90);
	painter.scale(.008, -.008);
	
	painter.drawText(-5, 6, QString::number(ID));
	
	painter.restore();
	
	if (t == Yellow)
	{
		painter.setPen(Qt::yellow);
	}
	else
	{
		painter.setPen(Qt::blue);
	}
	
	painter.rotate(theta+90);
	
	int span = 40;
	
	int start = span*16 + 90*16;
	int end = 360*16 - (span*2)*16;
	const float r = Constants::Robot::Radius;
	painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
	
	if (haveBall)
	{
		painter.setPen(Qt::red);
		const float r = Constants::Robot::Radius * 0.75f;
		painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
	}
	
	painter.restore();
}

void drawBall(QPainter& painter, Geometry2d::Point pos, Geometry2d::Point vel)
{
	painter.setPen(QColor(0xff, 0x40, 0));
	painter.setBrush(QColor(0xff,0x90,0x00));
	
	painter.drawEllipse(QRectF(-Ball::Radius + pos.x, -Ball::Radius + pos.y, 
			Ball::Diameter, Ball::Diameter));
    
    if (vel.x || vel.y)
    {
        painter.drawLine(pos.toQPointF(), (pos + vel).toQPointF());
    }
}
