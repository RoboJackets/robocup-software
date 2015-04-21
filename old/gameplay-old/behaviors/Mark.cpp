#include "Mark.hpp"

#include <Constants.hpp>

#include <iostream>

using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::Mark::Mark(GameplayModule *gameplay):
SingleRobotBehavior(gameplay),
_ratio(0.9),
_mark_line_thresh(0.9),
_markRobot(NULL)
{
}

void Gameplay::Behaviors::Mark::ratio(float r)
{
	if (r > 1.0) {
		_ratio = 1.0;
	} else if (r < 0.0) {
		_ratio = 0.0;
	} else {
		_ratio = r;
	}
}

bool Gameplay::Behaviors::Mark::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	if (!ball().valid)
	{
		return false;
	}
	if (_markRobot && _markRobot->visible) {
		// state data
		Point ballPos = ball().pos,
			  // ballVel = ball().vel,
			  pos = robot->pos,
			  markPos = _markRobot->pos;
			  // markVel = _markRobot->vel;
		Point markLineDir = (ballPos-markPos).normalized();
		Segment ballMarkLine(ballPos - markLineDir * Ball_Radius,
				markPos + markLineDir * 2.0 * Robot_Radius);

		state()->drawLine(ballMarkLine, QColor(0, 0, 255, 255), "Mark");
		robot->addText("Mark", QColor(255, 255, 255, 255));

		// we want to get between the mark and the ball as fast as possible, then close in
		float markLineDist = ballMarkLine.distTo(pos);
		Point targetPoint;
		if (markLineDist > _mark_line_thresh) {
			// fast intercept possible passes
			targetPoint = ballMarkLine.nearestPoint(pos);
		} else {
			// drive to the given point specified by ratio
			targetPoint = ballPos + (markPos - ballPos).normalized()* _ratio * ballMarkLine.length();
		}

		state()->drawCircle(_markRobot->pos, Robot_Radius * 1.2, QColor(0.0, 127, 255, 255), "Mark");

		// go there, facing the ball
		robot->approachOpponent(_markRobot->shell(), true);
		robot->move(targetPoint, false);
		robot->face(ballPos);

	} else {
		return false;
	}

	return true;
}
