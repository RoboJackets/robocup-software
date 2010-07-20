/**
 *  Play takes one robot and drives it in a precomputed path
 */

#include <iostream>
#include "TestDirectMotionControl.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestDirectMotionControl, "Tests")

Gameplay::Plays::TestDirectMotionControl::TestDirectMotionControl(GameplayModule *gameplay):
	Play(gameplay, 1),
	fsm_state_(Setup), // start with setup state so robot goes to given point
	radius_(1.5),      // create a circle of given radius
	start_pt_(Geometry2d::Point(0.0,1.0)) // start the pattern
{
	// find the center of the field
	Point center(0.0, Constants::Field::Length/2);

	// find the first point between the robot start point and the center
	Point deltaPt = (start_pt_ - center).normalized()*radius_;

	// initial point in path
	Point init = center + deltaPt;

	// loop over angle range to create an arc
	float angIncrement = 10;
	float maxAng = 180;
	for (float i = 0; i < maxAng; i += angIncrement) {
		// given angle, and a center, find the next point in the sequence
		Point delta(radius_*sin(DegreesToRadians*i), radius_*cos(DegreesToRadians*i));
		Point newpt = center - delta;
		path_.push_back(newpt);

		//cout << "Point: (" << newpt.x << ", " << newpt.y << ")" << endl;
	}

	//cout << "Generated Path, Size: " << path_.size() << endl;
}

bool Gameplay::Plays::TestDirectMotionControl::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

bool Gameplay::Plays::TestDirectMotionControl::assign(set<Robot *> &available)
{
	if (!takeBest(available))
	{
	    return false;
	}
	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TestDirectMotionControl::run()
{
	// pull out the robot pose
	Point pos = robot()->pos();

	// check whether to setup a particular point or run a path
	path_t path;
	if (fsm_state_ == Setup) {
		// try a simple point driver command to get to the start point
		path.push_back(start_pt_);

		// go onto next state as necessary
		float thresh = 0.1;
		if (pos.distTo(start_pt_) < thresh)
			fsm_state_ = Track;

	} else if (fsm_state_ == Track) {
		// if getting close to the second point, then remove the first point from the path
		if (path_.size() > 2 && pos.distTo(path_[1]) < pos.distTo(path_[0]))
			path_.erase(path_.begin());
		if (path_.size() == 2)
			path_.erase(path_.begin());

		// assign path to robot
		path = path_;
	}

	// issue the motion command
	robot()->move(path);

	return true;
}
