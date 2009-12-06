/**
 *  Play takes one robot and drives it in a precomputed path
 */

#include "TestDirectMotionControl.hpp"

using namespace std;

Gameplay::Plays::TestDirectMotionControl::TestDirectMotionControl(GameplayModule *gameplay):
	Play(gameplay),
	fsm_state_(Setup), // start with setup state so robot goes to given point
	radius_(1.5),      // create a circle of given radius
	start_pt_(Geometry2d::Point(1.0,4.0)) // start the pattern
{
}

bool Gameplay::Plays::TestDirectMotionControl::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

void Gameplay::Plays::TestDirectMotionControl::assign(set<Robot *> &available)
{
	takeBest(available);
}

bool Gameplay::Plays::TestDirectMotionControl::run()
{
	// check whether to setup a particular point or run a path
	if (fsm_state_ == Setup) {
		// try a simple point driver command to get to the start point
		path_.clear();
		path_.push_back(start_pt_);
	} else if (fsm_state_ == Track) {
		// PLACEHOLDER: clear the path and remain stationary
		path_.clear();
	}

	// issue the motion command
	robot()->moveExplicit(path_);
	//robot()->faceNone(); // doesn't work
	robot()->face(Geometry2d::Point(), false);

	return true;
}
