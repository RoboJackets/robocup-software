#include <limits>
#include <boost/foreach.hpp>
#include "BasicOffense112.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::BasicOffense112, "Playing")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(BasicOffense112)
	}
}

ConfigDouble *Gameplay::Plays::BasicOffense112::_offense_hysteresis;
ConfigDouble *Gameplay::Plays::BasicOffense112::_support_backoff_thresh;
ConfigDouble *Gameplay::Plays::BasicOffense112::_mark_hysteresis_coeff;
ConfigDouble *Gameplay::Plays::BasicOffense112::_support_avoid_teammate_radius;
ConfigDouble *Gameplay::Plays::BasicOffense112::_support_avoid_shot;
ConfigDouble *Gameplay::Plays::BasicOffense112::_offense_support_ratio;
ConfigDouble *Gameplay::Plays::BasicOffense112::_defense_support_ratio;

void Gameplay::Plays::BasicOffense112::createConfiguration(Configuration *cfg)
{
	_offense_hysteresis = new ConfigDouble(cfg, "BasicOffense112/Hystersis Coeff", 0.50);
	_support_backoff_thresh = new ConfigDouble(cfg, "BasicOffense112/Support Backoff Dist", 1.5);
	_mark_hysteresis_coeff = new ConfigDouble(cfg, "BasicOffense112/Mark Hystersis Coeff", 0.9);
	_support_avoid_teammate_radius = new ConfigDouble(cfg, "BasicOffense112/Support Avoid Teammate Dist", 0.5);
	_support_avoid_shot = new ConfigDouble(cfg, "BasicOffense112/Support Avoid Shot", 0.2);
	_offense_support_ratio = new ConfigDouble(cfg, "BasicOffense112/Offense Support Ratio", 0.7);
	_defense_support_ratio = new ConfigDouble(cfg, "BasicOffense112/Defense Support Ratio", 0.9);
}

Gameplay::Plays::BasicOffense112::BasicOffense112(GameplayModule *gameplay):
Play(gameplay),
_leftFullback(gameplay, Behaviors::Fullback::Left),
_rightFullback(gameplay, Behaviors::Fullback::Right),
_striker(gameplay),
_support(gameplay)
{
	_leftFullback.otherFullbacks.insert(&_rightFullback);
	_rightFullback.otherFullbacks.insert(&_leftFullback);

	// use constant value of mark threshold for now
	_support.markLineThresh(1.0);

}

float Gameplay::Plays::BasicOffense112::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::BasicOffense112::run()
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
	bool forward_reset = false;
	if (_striker.robot && _support.robot &&
			_support.robot->pos.distTo(ballProj) < *_offense_hysteresis * _striker.robot->pos.distTo(ballProj)) {
		_striker.robot = NULL;
		_support.robot = NULL;
		_striker.restart();
		forward_reset = true;
	}

	// choose offense, we want closest robot to ball to be striker
	// FIXME: need to assign more carefully to ensure that there is a robot available to kick
	assignNearestKicker(_striker.robot, available, ballProj);
	assignNearest(_support.robot, available, ballProj);

	// find the nearest opponent to the striker
	OpponentRobot* closestRobotToStriker = 0;
	float closestDistToStriker = numeric_limits<float>::infinity();
	if (_striker.robot) {
		BOOST_FOREACH(OpponentRobot* opp, state()->opp) {
			if (opp) {
				float d = opp->pos.distTo(_striker.robot->pos);
				if (d < closestDistToStriker) {
					closestDistToStriker = d;
					closestRobotToStriker = opp;
				}
			}
		}
	}
	bool striker_engaged = _striker.robot && closestDistToStriker < *_support_backoff_thresh;

	// pick as a mark target the furthest back opposing robot
	// and adjust mark ratio based on field position
	OpponentRobot* bestOpp = NULL;
	float bestDist = numeric_limits<float>::infinity();
	float cur_dist = (_support.markRobot()) ? _support.markRobot()->pos.distTo(ballProj) : numeric_limits<float>::infinity();

	size_t nrOppClose = 0;
	BOOST_FOREACH(OpponentRobot* opp, state()->opp)
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
	if (bestOpp && bestOpp->visible && (forward_reset || bestDist < cur_dist * *_mark_hysteresis_coeff))
		_support.markRobot(bestOpp);

	// if we are further away, we can mark further from robot
	if (ballProj.y > Field_Length/2.0 && nrOppClose)
		_support.ratio(*_offense_support_ratio);
	else
		_support.ratio(*_defense_support_ratio);

	// adjust obstacles on markers
	if (_striker.robot && _support.robot) {
		unsigned striker = _striker.robot->shell();
		_support.robot->avoidTeammateRadius(striker, *_support_avoid_teammate_radius);

		// get out of ways of shots
		if (_striker.robot->pos.nearPoint(ballProj, *_support_avoid_shot)) {
			Polygon shot_obs;
			shot_obs.vertices.push_back(Geometry2d::Point(Field_GoalWidth / 2, Field_Length));
			shot_obs.vertices.push_back(Geometry2d::Point(-Field_GoalWidth / 2, Field_Length));
			shot_obs.vertices.push_back(ballProj);
			_support.robot->localObstacles(ObstaclePtr(new PolygonObstacle(shot_obs)));
		}
	}

	// manually reset any kickers so they keep kicking
	if (_striker.done())
		_striker.restart();

	// execute behaviors
	if (_striker.robot) _striker.run();
	if (_support.robot) _support.run();
	if (_leftFullback.robot) _leftFullback.run();
	if (_rightFullback.robot) _rightFullback.run();

	return true;
}
