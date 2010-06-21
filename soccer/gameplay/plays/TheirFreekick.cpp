#include "TheirFreekick.hpp"

#include <iostream>
#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

Gameplay::Plays::TheirFreekick::TheirFreekick(GameplayModule *gameplay):
	Play(gameplay, 1),
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

bool Gameplay::Plays::TheirFreekick::applicable()
{
	return gameState().setupRestart() && gameState().theirFreeKick();
}
bool Gameplay::Plays::TheirFreekick::assign(set<Robot *> &available)
{		// FIXME: goes to the wrong side of the mark
	_robots = available;
	
	_fullback1.assign(available);
	_fullback2.assign(available);
	_marking1.assign(available);
	_marking2.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TheirFreekick::run()
{
	Point ballPos = ball().pos;

	//  determine which robots to mark
	map<float, Robot*> open_opp;
	BOOST_FOREACH(Robot * r, _gameplay->opp) {
		// want maximum distance that is less than 3 meters from any of our robots
		Point oppPos = r->pos();
		float ballDist = oppPos.distTo(ballPos);
		float max_relevant_robot_range = 3.0; // meters
		if (ballDist < max_relevant_robot_range) {
			float closestSelfDist = 100.0;
			BOOST_FOREACH(Robot * s, _gameplay->self) {
				float selfDist = s->pos().distTo(oppPos);
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
		_marking1.robot()->move(ballPos);
		_marking1.robot()->face(ballPos);
		_marking2.robot()->move(ballPos);
		_marking2.robot()->face(ballPos);
	} else if (open_opp.size() == 1) {
		_marking1.markRobot(open_opp.begin()->second);
		_marking1.run();
		_marking2.robot()->move(ballPos);
		_marking2.robot()->face(ballPos);
	} else {
		_marking1.markRobot(open_opp.begin()->second);
		_marking1.run();
		_marking2.markRobot((++open_opp.begin())->second);
		_marking2.run();
	}

	// execute default fullback "wall" behavior
	_fullback1.run();
	_fullback2.run();
	
	return true;
}
