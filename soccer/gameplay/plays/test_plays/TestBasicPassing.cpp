/*
 * TestBasicPassing.cpp
 *
 *  Created on: Nov 8, 2009
 *      Author: alexgc
 */

#include "TestBasicPassing.hpp"

using namespace Geometry2d;
using namespace std;

Gameplay::Plays::TestBasicPassing::TestBasicPassing(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_receiver(gameplay)
{
	_passState = CreateTrajectory;
}

void Gameplay::Plays::TestBasicPassing::assign(set<Robot *> &available)
{
	// assumes only two robots available, takes the first two robots
	if (available.size() < 2)
		throw invalid_argument("Attempting to create a testBasicPassing Play with less than two robots!");
	Robot * r1 = *(available.begin());
	Robot * r2 = *(++available.begin());

	// for the initial setup, send closest robot to ball to be passer
	float dist1 = ball().pos.distTo(r1->pos());
	float dist2 = ball().pos.distTo(r2->pos());
	if (dist1 <= dist2) {
		_passer.assignOne(r1);
		_receiver.assignOne(r2);
	} else {
		_passer.assignOne(r2);
		_receiver.assignOne(r1);
	}
}

bool Gameplay::Plays::TestBasicPassing::run()
{
	if (_passState == CreateTrajectory) {
		// create an initial trajectory
		// one end is the ball
		Point ball;// = ball().pos;
		// other end is on the other side of the field, with a fixed distance
		float center_length = Constants::Field::Length/2;

		// start positioning
		_passState = Positioning;
	}



	return true;
}
