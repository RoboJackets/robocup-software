#include "TheirFreekick.hpp"

#include <iostream>
#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TheirFreekick, "Restarts")

Gameplay::Plays::TheirFreekick::TheirFreekick(GameplayModule *gameplay):
	Play(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_marking1(gameplay),
	_marking2(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);

	// assign general parameters
	float r = 0.3;
	_marking1.ratio(r);
	_marking2.ratio(r);
}

float Gameplay::Plays::TheirFreekick::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.theirFreeKick()) ? 0 : INFINITY;
}

bool Gameplay::Plays::TheirFreekick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_fullback1.robot, available, Geometry2d::Point());
	assignNearest(_fullback2.robot, available, Geometry2d::Point());
	//FIXME - How to choose?
	assignNearest(_marking1.robot, available, Geometry2d::Point());
	assignNearest(_marking2.robot, available, Geometry2d::Point());
	
	if (!ball().valid)
	{
		return false;
	}
	Point ballPos = ball().pos;

	//  determine which robots to mark
	map<float, OpponentRobot*> open_opp;
	BOOST_FOREACH(OpponentRobot * r, state()->opp) {
		// want maximum distance that is less than 3 meters from any of our robots
		Point oppPos = r->pos;
		float ballDist = oppPos.distTo(ballPos);
		float max_relevant_robot_range = 3.0; // meters
		if (ballDist < max_relevant_robot_range) {
			float closestSelfDist = 100.0;
			BOOST_FOREACH(OurRobot * s, state()->self) {
				float selfDist = s->pos.distTo(oppPos);
				if (selfDist < closestSelfDist) {
					closestSelfDist = selfDist;
				}
			}
			if (closestSelfDist < 100.0) {
				// record the robot with distance, sort by open-ness
				open_opp[closestSelfDist] = r;
			}
		}
	}

	// assign targets
	if (open_opp.empty()) {
		// if nothing open, just drive at the ball
		//FIXME - They both drive towards the ball?  What?
		if (_marking1.robot)
		{
			_marking1.robot->move(ballPos);
			_marking1.robot->face(ballPos);
		}
		if (_marking2.robot)
		{
			_marking2.robot->move(ballPos);
			_marking2.robot->face(ballPos);
		}
	} else if (open_opp.size() == 1) {
		if (_marking1.robot)
		{
			_marking1.markRobot(open_opp.begin()->second);
			_marking1.run();
		}
		if (_marking2.robot)
		{
			_marking2.robot->move(ballPos);
			_marking2.robot->face(ballPos);
		}
	} else {
		if (_marking1.robot)
		{
			_marking1.markRobot(open_opp.begin()->second);
			_marking1.run();
		}
		if (_marking2.robot)
		{
			_marking2.markRobot((++open_opp.begin())->second);
			_marking2.run();
		}
	}

	// execute default fullback "wall" behavior
	_fullback1.run();
	_fullback2.run();
	
	return true;
}
