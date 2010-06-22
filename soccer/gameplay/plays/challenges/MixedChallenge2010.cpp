#include "MixedChallenge2010.hpp"

#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

Gameplay::Plays::MixedChallenge2010::MixedChallenge2010(GameplayModule *gameplay):
	Play(gameplay, 1),
	_fieldSide(LEFT),   // *************************** Change here for side!
	_state(SETUP),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
}

bool Gameplay::Plays::MixedChallenge2010::applicable()
{
	return true;
}

bool Gameplay::Plays::MixedChallenge2010::assign(set<Robot *> &available)
{
	_robots = available;
	
	_kicker1.assign(available);
	_kicker2.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::MixedChallenge2010::run()
{
	const Point textOffset(-Constants::Robot::Radius*1.3, 0.0);

	// determine which robots are not ours
	Robot * coRobotA = 0, * coRobotB = 0;
	BOOST_FOREACH(Robot * r, _gameplay->self) {
		if (r->exclude) {
			if (coRobotA == 0)
				coRobotA = r;
			else if (coRobotB == 0)
				coRobotB = r;
		}
	}

	switch (_state) {
	case SETUP: {

	} case SHOOTING: {

	} case PASSING: {

	} case RECEIVING: {

	}
	}

	return true;
}
