#include <limits>
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

	// use constant value of mark threshold for now
	_support.markLineThresh(1.0);
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

	// project the ball forward (dampened)
	const float proj_time = 0.75; // seconds to project
	const float dampen_factor = 0.9; // accounts for slowing over time
	Geometry2d::Point ballProj = ball().pos + ball().vel * proj_time * dampen_factor;

	// defense first - get closest to goal to choose sides properly
	assignNearest(_leftFullback.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
	assignNearest(_rightFullback.robot, available, Geometry2d::Point(Field_GoalWidth/2, 0.0));

	// determine whether to change offense players
	const float coeff = 0.50; // Determines level of hysteresis
	bool forward_reset = false;
	if (_kicker.robot && _support.robot &&
			_support.robot->pos.distTo(ballProj) < coeff * _kicker.robot->pos.distTo(ballProj)) {
		_kicker.robot = NULL;
		_support.robot = NULL;
		forward_reset = true;
	}

	// choose offense, we want closest robot to ball to be striker
	assignNearest(_kicker.robot, available, ballProj);
	assignNearest(_support.robot, available, ballProj);

	// manually reset any kickers so they keep kicking
	if (_kicker.done())
		_kicker.restart();

	// pick as a mark target the furthest back opposing robot
	// and adjust mark ratio based on field position
	OpponentRobot* bestOpp = NULL;
	float bestDist = numeric_limits<float>::infinity();
	float cur_dist = (_support.markRobot()) ?
			_support.markRobot()->pos.distTo(ballProj) :
			numeric_limits<float>::infinity();
	size_t nrOppClose = 0;
	BOOST_FOREACH(OpponentRobot* opp, state()->opp)
	{
		const float oppPos = opp->pos.y;
		if (opp && opp->visible && oppPos > 0.1) {
			if (oppPos < bestDist) {
				bestDist = opp->pos.y;
				bestOpp = opp;
			}
			if (oppPos < Field_Length/2) {
				++nrOppClose;
			}
		}
	}
	if (!bestOpp) {
		_support.robot->addText("No mark target");
	}

	// use hysteresis for changing of the robot
	const float mark_coeff = 0.9; // how much of an improvement is necessary to switch
	if (bestOpp && bestOpp->visible && (forward_reset || bestDist < cur_dist * mark_coeff))
		_support.markRobot(bestOpp);
	if (ballProj.y > Field_Length/2.0 && nrOppClose)
		_support.ratio(0.7);
	else
		_support.ratio(0.9);

	// Cyan circle around the marked robot
	if (_support.markRobot())
		state()->drawCircle(_support.markRobot()->pos, Robot_Radius * 1.2, QColor(0.0, 127, 255, 255), "BasicOffense");

	// execute behaviors
	if (_kicker.robot) _kicker.run();
	if (_support.robot) _support.run();
	if (_leftFullback.robot) _leftFullback.run();
	if (_rightFullback.robot) _rightFullback.run();
	
	return true;
}
