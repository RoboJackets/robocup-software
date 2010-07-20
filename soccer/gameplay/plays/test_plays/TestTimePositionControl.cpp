/**
 *  Play takes one robot and drives it in a precomputed path
 */

#include <iostream>
#include "TestTimePositionControl.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestTimePositionControl, "Tests")

Gameplay::Plays::TestTimePositionControl::TestTimePositionControl(GameplayModule *gameplay):
	Play(gameplay, 1),
	fsm_state_(Setup), // start with setup state so robot goes to given point
	radius_(1.5),      // create a circle of given radius
	start_pt_(Point(2.0,1.0)), // start the pattern
	start_time_(0)
{
}

bool Gameplay::Plays::TestTimePositionControl::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

bool Gameplay::Plays::TestTimePositionControl::assign(set<Robot *> &available)
{
	if (!takeBest(available))
	{
	    return false;
	}
	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TestTimePositionControl::run()
{
	// pull out the robot pose and velocity
	Point pos = robot()->pos();
	Point vel = robot()->vel();

	// if in first frame, setup
	float travel_time = 10.0;
	float halfDist = 1.5;
	if (start_time_ == 0.0) {
		// set a better start state to toggle between runs
		if (pos.x > 0)
			start_pt_.x = -halfDist;
		else
			start_pt_.x = halfDist;

		// set the initial timestamp
		start_time_ = Utils::timestamp();

		// set the end time (in delta format)
		//FIXME: set this to generally work
		//travel_time = (start_pt_ - pos).mag()
	}

	// check whether to setup a particular point or run a path
	std::vector<MotionCmd::PathNode> path;

	// send a single point command to the robot at a fixed time
	MotionCmd::PathNode goal;
	goal.pos = start_pt_;
	goal.rot = 0.0;
	goal.time = travel_time; // note that this is a delta in seconds

	// add the point to a path
	path.push_back(goal);

	// issue the motion command
	robot()->move(path, start_time_);

	return true;
}

