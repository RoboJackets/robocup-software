#include "Elements.hpp"

#include <Constants.hpp>

#include <QColor>
#include <QRectF>
#include <QLineF>

using namespace Constants;

void drawRobot(QPainter& painter, Team t, unsigned char ID, Geometry::Point2d pos, float theta)
{	
	painter.setPen(Qt::black);
	painter.setBrush(Qt::NoBrush);
	
	painter.save();
	
	painter.translate(pos.x, pos.y);
	
	painter.save();
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
	
	painter.drawChord(QRectF(-Constants::Robot::Radius, -Constants::Robot::Radius, 
			Constants::Robot::Diameter, Constants::Robot::Diameter), span*16 + 90*16, 360*16 - (span*2)*16);
	
	painter.restore();
}

void drawBall(QPainter& painter, Geometry::Point2d pos)
{
	painter.setPen(QColor(0xff, 0x40, 0));
	painter.setBrush(QColor(0xff,0x90,0x00));
	
	painter.drawEllipse(QRectF(-Ball::Radius + pos.x, -Ball::Radius + pos.y, 
			Ball::Diameter, Ball::Diameter));
}
