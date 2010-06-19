/**
 *  Play takes one robot and drives it in a precomputed path
 */

#include <iostream>
#include "TestRectMotionControl.hpp"

using namespace std;
using namespace Geometry2d;

Gameplay::Plays::TestRectMotionControl::TestRectMotionControl(GameplayModule *gameplay):
	Play(gameplay, 1),
	fsm_state_(Track),
	hWidth_(1.0),
	hHeight_(0.5),
	waitFramesMax_(75),
	pathGoalIdx_(0)
{
	// find the center of our side
	Point center(0.0, Constants::Field::Length/4);

	Point p1(center.x+hWidth_, center.y+hHeight_);
	Point p2(center.x-hWidth_, center.y+hHeight_);
	Point p3(center.x-hWidth_, center.y-hHeight_);
	Point p4(center.x+hWidth_, center.y-hHeight_);

	path_.push_back(p1);
	path_.push_back(p2);
	path_.push_back(p3);
	path_.push_back(p4);
}

bool Gameplay::Plays::TestRectMotionControl::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

bool Gameplay::Plays::TestRectMotionControl::assign(set<Robot *> &available)
{
	takeBest(available);
	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TestRectMotionControl::run()
{
	// pull out the robot pose
	Point pos = robot()->pos();

	int pathGoalThisIdx = pathGoalIdx_;
	int pathGoalNextIdx = (pathGoalIdx_ + 1) % ((int)path_.size());

	Point goal = path_.at(pathGoalThisIdx);

	// check whether to setup a particular point or run a path
	path_t path;
	if (fsm_state_ == Wait)
	{
		if(waitFrames_-- <= 0)
		{
			// go to next state
			pathGoalIdx_ = pathGoalNextIdx;
			goal = path_.at(pathGoalIdx_);
			fsm_state_ = Track;
		}
	}
	else if (fsm_state_ == Track)
	{
		float thresh = 0.1;
		if (pos.distTo(goal) <= thresh)
		{
			fsm_state_ = Wait;
			waitFrames_ = waitFramesMax_;
		}
	}

	// issue the motion command
	robot()->move(goal);

	return true;
}
