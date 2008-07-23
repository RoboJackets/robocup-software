#include <Sizes.h>

#include "Robot.hpp"

#include <QString>

using namespace Graphikos;

void Robot::paint(QPainter& painter, Team t, unsigned char ID, Geometry::Point2d pos, float theta)
{
	Robot::paint(painter, t, ID, pos.x, pos.y, theta);
}

void Robot::paint(QPainter& painter, Team t, unsigned char ID, float x, float y, float theta)
{	
	painter.setPen(Qt::black);
	painter.setBrush(Qt::NoBrush);
	
	painter.save();
	
	painter.translate(x, y);
	
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
	
	painter.drawChord(QRectF(-ROBOT_RADIUS, -ROBOT_RADIUS, ROBOT_DIAM, ROBOT_DIAM), span*16 + 90*16, 360*16 - (span*2)*16);
	
	painter.restore();
}
