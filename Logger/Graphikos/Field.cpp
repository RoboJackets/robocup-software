#include <GL/gl.h>
#include <math.h>

#include "Sizes.h"
#include "Field.hpp"

#include <QPainterPath>
#include <QColor>
#include <QRectF>
#include <QLineF>

using namespace Graphikos;

void Field::paint(QPainter& painter)
{
	painter.save();
	
	painter.drawRect(0,0,1,1);
	
	//reset to center
	painter.translate(-FLOOR_LENGTH/2.0, -FLOOR_WIDTH/2.0);
	
	QPainterPath floor;
	floor.addRect(0, 0, FLOOR_LENGTH, FLOOR_WIDTH);
	
	painter.fillPath(floor, QColor(0,100,0));
	
	painter.translate(FIELD_DEADSPACE, FIELD_DEADSPACE);
	
	painter.setPen(Qt::white);
	painter.setBrush(QColor(0,130,0));
	painter.drawRect(QRectF(0, 0, FIELD_LENGTH, FIELD_WIDTH));
	
	//set brush alpha to 0
	painter.setBrush(QColor(0,130,0, 0));
	
	//reset to center
	painter.translate(FIELD_LENGTH/2.0, FIELD_WIDTH/2.0);
	
	//centerline
	painter.drawLine(QLineF(0, FIELD_WIDTH/2,0, -FIELD_WIDTH/2));
	
	//goal areas
	painter.drawArc(QRectF(-ARC_RADIUS-FIELD_LENGTH/2.0, -ARC_RADIUS + .175, ARC_RADIUS*2.0, ARC_RADIUS*2.0), -90*16, 90*16);
	painter.drawArc(QRectF(-ARC_RADIUS-FIELD_LENGTH/2.0, -ARC_RADIUS - .175, ARC_RADIUS*2.0, ARC_RADIUS*2.0), 90*16, -90*16);
	painter.drawLine(QLineF(-FIELD_LENGTH/2.0+.5, -.175, -FIELD_LENGTH/2.0+.5, .175));
	
	painter.drawArc(QRectF(-ARC_RADIUS+FIELD_LENGTH/2.0, -ARC_RADIUS + .175, ARC_RADIUS*2.0, ARC_RADIUS*2.0), -90*16,-90*16);
	painter.drawArc(QRectF(-ARC_RADIUS+FIELD_LENGTH/2.0, -ARC_RADIUS - .175, ARC_RADIUS*2.0, ARC_RADIUS*2.0), 90*16, 90*16);
	painter.drawLine(QLineF(FIELD_LENGTH/2.0-.5, -.175, FIELD_LENGTH/2.0-.5, .175));
	
	//center circle
	painter.drawEllipse(QRectF(-ARC_RADIUS, -ARC_RADIUS, ARC_RADIUS*2.0, ARC_RADIUS*2.0));
	
	// goals
	Field::paintGoal(0, painter);
	Field::paintGoal(1, painter);
	
	painter.restore();
}

void Field::paintGoal(unsigned char team, QPainter& painter)
{	
	float width = GOAL_WIDTH/2.0;
	
	float x1 = -FIELD_LENGTH/2.0;
	float x2 = -FIELD_LENGTH/2.0-GOAL_DEPTH;
	
	float xp = PENALTY_DIST - FIELD_LENGTH/2.0;
	
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
	painter.drawEllipse(QRectF(-PENALTY_DIAM/2.0, -PENALTY_DIAM/2.0, PENALTY_DIAM/2.0, PENALTY_DIAM/2.0));
	painter.restore();
}
