
#include "Field.hpp"
#include <GL/gl.h>
#include <math.h>
#include <Constants.hpp>
#include <QPainterPath>
#include <QColor>
#include <QRectF>
#include <QLineF>

//FIXME - Need to do something about the namespace conflict with constants. Maybe rename this class

using namespace Constants;
using namespace Graphikos;


void Graphikos::Field::paint(QPainter& painter)
{
	painter.save();

	painter.drawRect(0,0,1,1);

	//reset to center
	painter.translate(-Floor::Length/2.0, -Floor::Width/2.0);

	QPainterPath floor;
	floor.addRect(0, 0, Floor::Length, Floor::Width);

	painter.fillPath(floor, QColor(0,100,0));

	painter.translate(Constants::Field::Border, Constants::Field::Border);

	painter.setPen(Qt::white);
	painter.setBrush(QColor(0,130,0));
	painter.drawRect(QRectF(0, 0, Constants::Field::Length, Constants::Field::Width));

	//set brush alpha to 0
	painter.setBrush(QColor(0,130,0, 0));

	//reset to center
	painter.translate(Constants::Field::Length/2.0, Constants::Field::Width/2.0);

	//centerline
	painter.drawLine(QLineF(0, Constants::Field::Width/2,0, -Constants::Field::Width/2));

	//goal areas
	painter.drawArc(QRectF(-Constants::Field::ArcRadius-Constants::Field::Length/2.0, -Constants::Field::ArcRadius + .175, Constants::Field::ArcRadius*2.0, Constants::Field::ArcRadius*2.0), -90*16, 90*16);
	painter.drawArc(QRectF(-Constants::Field::ArcRadius-Constants::Field::Length/2.0, -Constants::Field::ArcRadius - .175, Constants::Field::ArcRadius*2.0, Constants::Field::ArcRadius*2.0), 90*16, -90*16);
	painter.drawLine(QLineF(-Constants::Field::Length/2.0+.5, -.175, -Constants::Field::Length/2.0+.5, .175));

	painter.drawArc(QRectF(-Constants::Field::ArcRadius+Constants::Field::Length/2.0, -Constants::Field::ArcRadius + .175, Constants::Field::ArcRadius*2.0, Constants::Field::ArcRadius*2.0), -90*16,-90*16);
	painter.drawArc(QRectF(-Constants::Field::ArcRadius+Constants::Field::Length/2.0, -Constants::Field::ArcRadius - .175, Constants::Field::ArcRadius*2.0, Constants::Field::ArcRadius*2.0), 90*16, 90*16);
	painter.drawLine(QLineF(Constants::Field::Length/2.0-.5, -.175, Constants::Field::Length/2.0-.5, .175));

	//center circle
	painter.drawEllipse(QRectF(-Constants::Field::ArcRadius, -Constants::Field::ArcRadius, Constants::Field::ArcRadius*2.0, Constants::Field::ArcRadius*2.0));

	// goals
	Field::paintGoal(0, painter);
	Field::paintGoal(1, painter);

	painter.restore();
}

void Graphikos::Field::paintGoal(unsigned char team, QPainter& painter)
{
	float width = Constants::Field::GoalWidth/2.0;

	float x1 = -Constants::Field::Length/2.0;
	float x2 = -Constants::Field::Length/2.0-Constants::Field::GoalDepth;

	float xp = Constants::Field::PenaltyDist - Constants::Field::Length/2.0;

	painter.setPen(Qt::yellow);

	if (team == 1)
	{
		x1 = -x1;
		x2 = -x2;
		xp = -xp;

		painter.setPen(Qt::blue);
	}

	//draw goals
	painter.drawLine(QLineF(x1, width, x2, width));
	painter.drawLine(QLineF(x1, -width, x2, -width));
	painter.drawLine(QLineF(x2, width, x2, -width));

	painter.setPen(Qt::white);

	//penalty mark
	painter.save();
	painter.translate(xp, 0);
	painter.drawEllipse(QRectF(-Constants::Field::PenaltyDiam/2.0, -Constants::Field::PenaltyDiam/2.0, Constants::Field::PenaltyDiam/2.0, Constants::Field::PenaltyDiam/2.0));
	painter.restore();
}
