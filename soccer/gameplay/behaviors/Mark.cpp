#include "Mark.hpp"

#include <Constants.hpp>
#include <LogFrame.hpp>

#include <iostream>
using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::Mark::Mark(GameplayModule *gameplay):
Behavior(gameplay, 1)
{
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
			// close in to harass
			// NOTE: this will actually allow their robot to drive us in a direction
			float avgVel = 0.5 * robot()->packet()->config.motion.deg45.velocity;
			float proj_time = markPos.distTo(pos) / avgVel;
			Point markProj = markPos + markVel * proj_time,
				  ballProj = ballPos + ballVel * proj_time,
			targetPoint = markProj + (ballProj-markProj).normalized() * _radius;
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



