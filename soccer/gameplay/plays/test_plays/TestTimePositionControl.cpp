/**
 *  Play takes one robot and drives it in a precomputed path
 */

#include <iostream>
#include "TestTimePositionControl.hpp"

using namespace std;
using namespace Geometry2d;
using namespace Packet;

Gameplay::Plays::TestTimePositionControl::TestTimePositionControl(GameplayModule *gameplay):
	Play(gameplay),
	fsm_state_(Setup), // start with setup state so robot goes to given point
	radius_(1.5),      // create a circle of given radius
	start_pt_(Point(0.0,1.0)), // start the pattern
	start_time_(0)
{
}

bool Gameplay::Plays::TestTimePositionControl::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

void Gameplay::Plays::TestTimePositionControl::assign(set<Robot *> &available)
{
	takeBest(available);
}

bool Gameplay::Plays::TestTimePositionControl::run()
{
	// pull out the robot pose and velocity
	Point pos = robot()->pos();
	Point vel = robot()->vel();

	// fix the start time if not aleady set
	if (start_time_ == 0.0)
		start_time_ = _gameplay->state()->timestamp;

	// check whether to setup a particular point or run a path
	std::vector<MotionCmd::PathNode> path;

	// send a single point command to the robot at a fixed time
	float travel_time = 10.0;
	MotionCmd::PathNode goal;
	goal.pos = start_pt_;
	goal.rot = 0.0;
	goal.time = travel_time; // note that this is a delta in seconds

	// add the point to a path
	path.push_back(goal);

	// issue the motion command
	cout << "Issuing TimePos Move command" << endl;
	robot()->moveTimePos(path, start_time_);

	return true;
}

