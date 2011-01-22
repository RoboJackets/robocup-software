#include <limits>
#include <boost/foreach.hpp>
#include "AggressiveOpen.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::AggressiveOpen, "Playing")

Gameplay::Plays::AggressiveOpen::AggressiveOpen(GameplayModule *gameplay):
	Play(gameplay),
	_leftFullback(gameplay, Behaviors::Fullback::Left),
	_rightFullback(gameplay, Behaviors::Fullback::Right),
	_striker(gameplay),
	_supportPrimary(gameplay),
	_supportSecondary(gameplay)
{
	_leftFullback.otherFullbacks.insert(&_rightFullback);
	_rightFullback.otherFullbacks.insert(&_leftFullback);

	// use constant value of mark threshold for now
	_supportPrimary.markLineThresh(1.0);
	_supportSecondary.markLineThresh(1.0);
}

float Gameplay::Plays::AggressiveOpen::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::AggressiveOpen::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	// project the ball forward (dampened)
	const float proj_time = 0.75; // seconds to project
	const float dampen_factor = 0.9; // accounts for slowing over time
	Geometry2d::Point ballProj = ball().pos + ball().vel * proj_time * dampen_factor;

	// determine whether to be aggressive based on the opponent robots in our area
	vector<OpponentRobot*> homeOpp, farOpp;
	const float field_thresh = Field_Length/2.0;
	float minOppBallDist = numeric_limits<float>::infinity();
	BOOST_FOREACH(OpponentRobot* r, state()->opp) {
		if (r && r->visible) {
			// thresholding
			if (r->pos.y < field_thresh)
				homeOpp.push_back(r);
			else
				farOpp.push_back(r);

			// min ball distance
			float ballDist = r->pos.distTo(ballProj);
			if (ballDist < minOppBallDist)
				minOppBallDist = ballDist;
		}
	}

	// Three main cases:
	//  1: General case  - 1-1-2 configuration - as usual
	//  2: Offense case  - 1-2-1 configuration - if ball, striker and support are on the other side
	//  3: Clearing case - if support and striker are on the other side of the field, and ball is free

	// Clear vs. defense
//	if ((_striker && _striker->robot && _striker->robot->pos.y > field_thresh) && // striker far away
//		(_supportPrimary && _supportPrimary->robot && _supportPrimary->robot->pos.y > field_thresh) && // support far away
//		(minOppBallDist < clear_ball_dist_thresh)) {
//		// execute clear
//		// TODO: setup clear properly to ensure that it will get back out
//	} else {
		// normal defense
		assignNearest(_leftFullback.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
		assignNearest(_rightFullback.robot, available, Geometry2d::Point(Field_GoalWidth/2, 0.0));
//	}

	// determine whether to change offense players
	const float coeff = 0.50; // Determines level of hysteresis
	bool forward_reset = false;
	if (_striker.robot && _supportPrimary.robot &&
			_supportPrimary.robot->pos.distTo(ballProj) < coeff * _striker.robot->pos.distTo(ballProj)) {
		_striker.robot = NULL;
		_supportPrimary.robot = NULL;
		forward_reset = true;
	}

	// choose offense, we want closest robot to ball to be striker
	assignNearest(_striker.robot, available, ballProj);
	assignNearest(_supportPrimary.robot, available, ballProj);

	// manually reset any kickers so they keep kicking
	if (_striker.done())
		_striker.restart();

	// pick as a mark target the furthest back opposing robot
	// and adjust mark ratio based on field position
	OpponentRobot* bestOpp = NULL;
	float bestDist = numeric_limits<float>::infinity();
	float cur_dist = (_supportPrimary.markRobot()) ?
			_supportPrimary.markRobot()->pos.distTo(ballProj) :
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
		_supportPrimary.robot->addText("No mark target");
	}

	// use hysteresis for changing of the robot
	const float mark_coeff = 0.9; // how much of an improvement is necessary to switch
	if (bestOpp && bestOpp->visible && (forward_reset || bestDist < cur_dist * mark_coeff))
		_supportPrimary.markRobot(bestOpp);
	if (ballProj.y > Field_Length/2.0 && nrOppClose)
		_supportPrimary.ratio(0.7);
	else
		_supportPrimary.ratio(0.9);

	// execute behaviors
	if (_striker.robot) _striker.run();
	if (_supportPrimary.robot) _supportPrimary.run();
	if (_leftFullback.robot) _leftFullback.run();
	if (_rightFullback.robot) _rightFullback.run();
	
	return true;
}
