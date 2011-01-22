#include <boost/foreach.hpp>
#include "BasicOffense.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::BasicOffense, "Playing")

Gameplay::Plays::BasicOffense::BasicOffense(GameplayModule *gameplay):
	Play(gameplay),
	_leftFullback(gameplay, Behaviors::Fullback::Left),
	_rightFullback(gameplay, Behaviors::Fullback::Right),
	_kicker(gameplay),
	_support(gameplay)
{
	_leftFullback.otherFullbacks.insert(&_rightFullback);
	_rightFullback.otherFullbacks.insert(&_leftFullback);
}

float Gameplay::Plays::BasicOffense::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::BasicOffense::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	// defense first - closest to goal
	assignNearest(_leftFullback.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
	assignNearest(_rightFullback.robot, available, Geometry2d::Point(Field_GoalWidth/2, 0.0));

	// determine whether to change offense players
	const float coeff = 0.50; // Determines level of hysteresis
	if (_kicker.robot && _support.robot &&
			_support.robot->pos.distTo(ball().pos) < coeff * _kicker.robot->pos.distTo(ball().pos)) {
		_kicker.robot = NULL;
		_support.robot = NULL;
	}

	// choose offense, we want closest robot to ball to be striker
	assignNearest(_kicker.robot, available, ball().pos);
	assignNearest(_support.robot, available, ball().pos);

	// manually reset any kickers so they keep kicking
	if (_kicker.done())
		_kicker.restart();

	// pick as a mark target the furthest back opposing robot
	// and adjust mark ratio based on field position
	OpponentRobot* bestOpp = NULL;
	float bestDist = Field_Length;
	size_t nrOppClose = 0;
	BOOST_FOREACH(OpponentRobot* opp, state()->opp)
	{
		if (opp->pos.y < bestDist) {
			bestDist = opp->pos.y;
			bestOpp = opp;
		}
		if (opp->pos.y < Field_Length/2)
			++nrOppClose;
	}
	_support.markRobot(bestOpp);
	if (ball().pos.y > Field_Length/2 && nrOppClose)
		_support.ratio(0.7);
	else
		_support.ratio(0.9);

	// execute behaviors
	_kicker.run();
	_support.run();
	_leftFullback.run();
	_rightFullback.run();
	
	return true;
}
