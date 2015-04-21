#include <limits>
#include <iostream>
#include "YankOpen.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::YankOpen, "Playing")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(YankOpen)
	}
}

ConfigDouble *Gameplay::Plays::YankOpen::_offense_hysteresis;
ConfigDouble *Gameplay::Plays::YankOpen::_support_backoff_thresh;
ConfigDouble *Gameplay::Plays::YankOpen::_mark_hysteresis_coeff;
ConfigDouble *Gameplay::Plays::YankOpen::_support_avoid_teammate_radius;
ConfigDouble *Gameplay::Plays::YankOpen::_support_avoid_shot;
ConfigDouble *Gameplay::Plays::YankOpen::_offense_support_ratio;
ConfigDouble *Gameplay::Plays::YankOpen::_defense_support_ratio;
ConfigBool *Gameplay::Plays::YankOpen::_useYank;

void Gameplay::Plays::YankOpen::createConfiguration(Configuration *cfg)
{
	_offense_hysteresis = new ConfigDouble(cfg, "YankOpen/Hystersis Coeff", 0.50);
	_support_backoff_thresh = new ConfigDouble(cfg, "YankOpen/Support Backoff Dist", 1.5);
	_mark_hysteresis_coeff = new ConfigDouble(cfg, "YankOpen/Mark Hystersis Coeff", 0.9);
	_support_avoid_teammate_radius = new ConfigDouble(cfg, "YankOpen/Support Avoid Teammate Dist", 0.5);
	_support_avoid_shot = new ConfigDouble(cfg, "YankOpen/Support Avoid Shot", 0.2);
	_offense_support_ratio = new ConfigDouble(cfg, "YankOpen/Offense Support Ratio", 0.7);
	_defense_support_ratio = new ConfigDouble(cfg, "YankOpen/Defense Support Ratio", 0.9);
	_useYank = new ConfigBool(cfg, "YankOpen/Use Yank", false);
}

Gameplay::Plays::YankOpen::YankOpen(GameplayModule *gameplay):
Play(gameplay),
_leftDefender(gameplay, Behaviors::Defender::Left),
_rightDefender(gameplay, Behaviors::Defender::Right),
_support(gameplay),
_strikerBump(gameplay),
_strikerFling(gameplay),
_strikerYank(gameplay)
{
	_leftDefender.otherDefenders.insert(&_rightDefender);
	_rightDefender.otherDefenders.insert(&_leftDefender);

	// use constant value of mark threshold for now
	_support.markLineThresh(1.0);

	// set targets to the opponent's goal
	static const Point goal_center(0.0, Field_Length);
	_strikerBump.target = goal_center;
	_strikerFling.target = goal_center;
}

float Gameplay::Plays::YankOpen::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::YankOpen::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	// project the ball forward (dampened)
	const float proj_time = 0.75; // seconds to project
	const float dampen_factor = 0.9; // accounts for slowing over time
	Geometry2d::Point ballProj = ball().pos + ball().vel * proj_time * dampen_factor;

	// determine whether to change offense players
	bool forward_reset = false;
//	if (_strikerYank.robot && _support.robot &&
//			_support.robot->pos.distTo(ballProj) < *_offense_hysteresis * _strikerYank.robot->pos.distTo(ballProj)) {
//		_strikerBump.robot = NULL;
//		_strikerYank.robot = NULL;
//		_strikerFling.robot = NULL;
//		_strikerBump.restart();
//		_strikerYank.restart();
//		_strikerFling.restart();
//		_support.robot = NULL;
//		forward_reset = true;
//	}

	// choose the striker first - need to ensure that
	if (assignNearest(_strikerYank.robot, available, ballProj))
	{
		_strikerBump.robot = _strikerYank.robot;
		_strikerFling.robot = _strikerYank.robot;
	}

	// find the nearest opponent to the striker
	OpponentRobot* closestRobotToStriker = 0;
	float closestDistToStriker = numeric_limits<float>::infinity();
	if (_strikerYank.robot) {
		for (OpponentRobot* opp :  state()->opp) {
			if (opp) {
				float d = opp->pos.distTo(_strikerYank.robot->pos);
				if (d < closestDistToStriker) {
					closestDistToStriker = d;
					closestRobotToStriker = opp;
				}
			}
		}
	}
	bool striker_engaged = _strikerYank.robot && closestDistToStriker < *_support_backoff_thresh;

	// defense after striker  - get closest to goal to choose sides properly
	assignNearest(_leftDefender.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
	assignNearest(_rightDefender.robot, available, Geometry2d::Point(Field_GoalWidth/2, 0.0));

	// support last - marginally useful in this scenario
	assignNearest(_support.robot, available, ballProj);

	// pick as a mark target the furthest back opposing robot
	// and adjust mark ratio based on field position
	OpponentRobot* bestOpp = NULL;
	float bestDist = numeric_limits<float>::infinity();
	float cur_dist = (_support.markRobot()) ? _support.markRobot()->pos.distTo(ballProj) : numeric_limits<float>::infinity();

	size_t nrOppClose = 0;
	for (OpponentRobot* opp :  state()->opp)
	{
		const float oppPos = opp->pos.y;
		if (opp && opp->visible && oppPos > 0.1 && !(striker_engaged && opp == closestRobotToStriker)) {
			if (oppPos < bestDist) {
				bestDist = opp->pos.y;
				bestOpp = opp;
			}
			if (oppPos < Field_Length/2) {
				++nrOppClose;
			}
		}
	}
	if (!bestOpp && _support.robot) {
		_support.robot->addText("No mark target");
	}

	// use hysteresis for changing of the robot
	if (bestOpp && bestOpp->visible && (forward_reset || bestDist < cur_dist * (float)*_mark_hysteresis_coeff))
		_support.markRobot(bestOpp);

	// if we are further away, we can mark further from robot
	if (ballProj.y > Field_Length/2.0 && nrOppClose)
		_support.ratio(*_offense_support_ratio);
	else
		_support.ratio(*_defense_support_ratio);

	// adjust obstacles on markers
	if (_strikerYank.robot && _support.robot) {
		unsigned striker = _strikerYank.robot->shell();
		_support.robot->avoidTeammateRadius(striker, *_support_avoid_teammate_radius);
	}

	// manually reset any kickers so they keep kicking
	if (_strikerYank.done())
	{
		_strikerYank.restart();
	}

	if (_strikerBump.done())
	{
		_strikerBump.restart();
	}

	if (_strikerFling.done())
	{
		_strikerFling.restart();
	}

	// execute behaviors
	if (*_useYank && _strikerYank.robot) _strikerYank.run();
	if (!*_useYank && _strikerBump.robot) _strikerBump.run();

	if (_support.robot) _support.run();
	if (_leftDefender.robot) _leftDefender.run();
	if (_rightDefender.robot) _rightDefender.run();

	return true;
}
