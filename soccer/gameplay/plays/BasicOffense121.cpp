#include <limits>
#include <boost/foreach.hpp>
#include "BasicOffense121.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::BasicOffense121, "Playing")

Gameplay::Plays::BasicOffense121::BasicOffense121(GameplayModule *gameplay):
	Play(gameplay),
	_fullback(gameplay, Behaviors::Fullback::Center),
	_striker(gameplay),
	_support1(gameplay),
	_support2(gameplay)
{
	// use constant value of mark threshold for now
	_support1.markLineThresh(1.0);
	_support2.markLineThresh(1.0);
}

float Gameplay::Plays::BasicOffense121::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::BasicOffense121::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	// project the ball forward (dampened)
	const float proj_time = 0.75; // seconds to project
	const float dampen_factor = 0.9; // accounts for slowing over time
	Geometry2d::Point ballProj = ball().pos + ball().vel * proj_time * dampen_factor;

	// defense first
	assignNearest(_fullback.robot, available, Geometry2d::Point(0.0, 0.0));

	// determine whether to change offense players
	const float coeff = 0.50; // Determines level of hysteresis
	bool forward_reset = false;
	if (_striker.robot && _support1.robot && _support2.robot &&
		(_support1.robot->pos.distTo(ballProj) < coeff * _striker.robot->pos.distTo(ballProj) ||
		 _support2.robot->pos.distTo(ballProj) < coeff * _striker.robot->pos.distTo(ballProj))) {
		_striker.robot = NULL;
		_support1.robot = NULL;
		_support2.robot = NULL;
		forward_reset = true;
	}

	// choose offense, we want closest robot to ball to be striker
	assignNearest(_striker.robot, available, ballProj);
	assignNearest(_support1.robot, available, ballProj);
	assignNearest(_support2.robot, available, ballProj);

	// manually reset any kickers so they keep kicking
	if (_striker.done())
		_striker.restart();

	// pick as a mark target the furthest back opposing robot
	// and adjust mark ratio based on field position
	// find *two* furthest back opp robots
	OpponentRobot* bestOpp1 = NULL, *bestOpp2 = NULL;
	float bestDist1 = numeric_limits<float>::infinity(), bestDist2 = bestDist1;
	float cur_dist1 = (_support1.markRobot()) ?	_support1.markRobot()->pos.distTo(ballProj) : numeric_limits<float>::infinity();
	float cur_dist2 = (_support2.markRobot()) ?	_support2.markRobot()->pos.distTo(ballProj) : numeric_limits<float>::infinity();
	size_t nrOppClose = 0;
	BOOST_FOREACH(OpponentRobot* opp, state()->opp)
	{
		// use distance from goal rather than just y coordinate to handle corners better
		const float oppDistSq = opp->pos.magsq();
		if (opp && opp->visible && oppDistSq > 0.1) {
			if (oppDistSq <= bestDist1) {
				bestDist2 = bestDist1;
				bestDist1 = oppDistSq;
				bestOpp2 = bestOpp1;
				bestOpp1 = opp;
			} else if (oppDistSq <= bestDist2) {
				bestDist2 = oppDistSq;
				bestOpp2 = opp;
			}


			if (oppDistSq < Field_Length * Field_Length / 4.0) {
				++nrOppClose;
			}
		}
	}
	if (!bestOpp1) {
		_support1.robot->addText("No mark target");
	}
	if (!bestOpp2) {
		_support2.robot->addText("No mark target");
	}

	// use hysteresis for changing of the robot
	const float mark_coeff = 0.9; // how much of an improvement is necessary to switch
	if (bestOpp1 && bestOpp1->visible && (forward_reset || bestDist1 < cur_dist1 * cur_dist1 * mark_coeff))
		_support1.markRobot(bestOpp1);
	if (bestOpp2 && bestOpp2->visible && (forward_reset || bestDist2 < cur_dist2 * cur_dist2 * mark_coeff))
		_support2.markRobot(bestOpp2);
	if (ballProj.y > Field_Length/2.0 && nrOppClose) {
		_support1.ratio(0.7);
		_support2.ratio(0.7);
	}
	else {
		_support1.ratio(0.9);
		_support2.ratio(0.9);
	}


	// execute behaviors
	if (_striker.robot) _striker.run();
	if (_support1.robot) _support1.run();
	if (_support2.robot) _support2.run();
	if (_fullback.robot) _fullback.run();
	
	return true;
}
