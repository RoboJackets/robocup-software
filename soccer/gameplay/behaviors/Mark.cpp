#include "Mark.hpp"

#include <Constants.hpp>
#include <LogFrame.hpp>

#include <iostream>

using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::Mark::Mark(GameplayModule *gameplay):
Behavior(gameplay, 1),
_ratio(0.9)
{
}

void Gameplay::Behaviors::Mark::ratio(float r) {
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
	if (_markRobot) {
		// state data
		Point ballPos = ball().pos,
			  ballVel = ball().vel,
			  pos = robot()->pos(),
			  markPos = _markRobot->pos(),
			  markVel = _markRobot->vel();
		Segment ballMarkLine(ballPos, markPos);

		// we want to get between the mark and the ball as fast as possible, then close in
		float markLineDist = ballMarkLine.distTo(pos);
		Point targetPoint;
		float mark_line_thresh = 0.9;
		if (markLineDist > mark_line_thresh) {
			// fast intercept possible passes
			targetPoint = ballMarkLine.nearestPoint(pos);
		} else {
			// drive to the given point specified by ratio
			targetPoint = ballPos + (markPos - ballPos)* _ratio * ballMarkLine.length();
		}

		// go there, facing the ball
		robot()->approachOpp(_markRobot, true);
		robot()->move(targetPoint, false);
		robot()->face(ballPos);

	} else {
		return false;
	}

	return true;
}



