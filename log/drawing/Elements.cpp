#include "Elements.hpp"

#include <Constants.hpp>

#include <QColor>
#include <QRectF>
#include <QLineF>

using namespace Constants;

void drawField(QPainter& p)
{
	p.save();
	
	//reset to center
	p.translate(-Floor::Length/2.0, -Floor::Width/2.0);
	
	p.setPen(Qt::transparent);
	p.setBrush(QColor(0,85,0));
	p.drawRect(QRectF(0, 0, Floor::Length, Floor::Width));		
	
	p.translate(Field::Border, Field::Border);
	
	p.setPen(Qt::white);
	p.setBrush(QColor(0,130,0));
	p.drawRect(QRectF(0, 0, Field::Length, Field::Width));
	
	//set brush alpha to 0
	p.setBrush(QColor(0,130,0, 0));
	
	//reset to center
	p.translate(Field::Length/2.0, Field::Width/2.0);
	
	//centerline
	p.drawLine(QLineF(0, Field::Width/2,0, -Field::Width/2.0));
	
	//center circle
	p.drawEllipse(QRectF(-Field::ArcRadius, -Field::ArcRadius, 
		Field::CenterDiameter, Field::CenterDiameter));
	
	p.translate(-Field::Length/2.0, 0);
	
	//goal areas
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius + .175, Field::CenterDiameter, Field::CenterDiameter), -90*16, 90*16);
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius - .175, Field::CenterDiameter, Field::CenterDiameter), 90*16, -90*16);
	p.drawLine(QLineF(Field::ArcRadius, -.175, Field::ArcRadius, .175));
	
	p.translate(Field::Length, 0);
	
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius + .175, Field::CenterDiameter, Field::CenterDiameter), -90*16, -90*16);
	p.drawArc(QRectF(-Field::ArcRadius, -Field::ArcRadius - .175, Field::CenterDiameter, Field::CenterDiameter), 90*16, 90*16);
	p.drawLine(QLineF(-Field::ArcRadius, -.175, -Field::ArcRadius, .175));
		
	// goals
	float x[2] = {0, Field::GoalDepth};
	float y[2] = {Field::GoalWidth/2.0, -Field::GoalWidth/2.0};
	
	p.setPen(Qt::blue);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	
	x[0] -= Field::Length;
	x[1] -= Field::Length + 2 * Field::GoalDepth;
	
	p.setPen(Qt::yellow);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	
	p.restore();
}

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
	
	painter.drawEllipse(QRectF(-Ball::Radius + pos.y, -Ball::Radius + pos.y, 
			Ball::Diameter, Ball::Diameter));
}
