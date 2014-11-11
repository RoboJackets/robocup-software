#include <limits>

#include "BasicOffense122.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::BasicOffense122, "Playing")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(BasicOffense122)
	}
}

ConfigDouble *Gameplay::Plays::BasicOffense122::_offense_hysteresis;
ConfigDouble *Gameplay::Plays::BasicOffense122::_support_backoff_thresh;
ConfigDouble *Gameplay::Plays::BasicOffense122::_mark_hysteresis_coeff;
ConfigDouble *Gameplay::Plays::BasicOffense122::_support_avoid_teammate_radius;
ConfigDouble *Gameplay::Plays::BasicOffense122::_support_avoid_shot;
ConfigDouble *Gameplay::Plays::BasicOffense122::_offense_support_ratio;
ConfigDouble *Gameplay::Plays::BasicOffense122::_defense_support_ratio;
ConfigBool   *Gameplay::Plays::BasicOffense122::_use_line_kick;
ConfigBool   *Gameplay::Plays::BasicOffense122::_defense_first;

void Gameplay::Plays::BasicOffense122::createConfiguration(Configuration *cfg)
{
	_offense_hysteresis = new ConfigDouble(cfg, "BasicOffense122/Hystersis Coeff", 0.50);
	_support_backoff_thresh = new ConfigDouble(cfg, "BasicOffense122/Support Backoff Dist", 1.5);
	_mark_hysteresis_coeff = new ConfigDouble(cfg, "BasicOffense122/Mark Hystersis Coeff", 0.9);
	_support_avoid_teammate_radius = new ConfigDouble(cfg, "BasicOffense122/Support Avoid Teammate Dist", 0.5);
	_support_avoid_shot = new ConfigDouble(cfg, "BasicOffense122/Support Avoid Shot", 0.2);
	_offense_support_ratio = new ConfigDouble(cfg, "BasicOffense122/Offense Support Ratio", 0.7);
	_defense_support_ratio = new ConfigDouble(cfg, "BasicOffense122/Defense Support Ratio", 0.9);
	_use_line_kick = new ConfigBool(cfg, "BasicOffense122/Enable Line Kick", true);
	_defense_first = new ConfigBool(cfg, "BasicOffense122/Defense First", true);
}

Gameplay::Plays::BasicOffense122::BasicOffense122(GameplayModule *gameplay):
Play(gameplay),
_leftDefender(gameplay, Behaviors::Defender::Left),
_rightDefender(gameplay, Behaviors::Defender::Right),
_striker(gameplay),
_support1(gameplay),
_support2(gameplay)
{
	_leftDefender.otherDefenders.insert(&_rightDefender);
	_rightDefender.otherDefenders.insert(&_leftDefender);

	// use constant value of mark threshold for now
	_support1.markLineThresh(1.0);
	_support2.markLineThresh(1.0);
}

float Gameplay::Plays::BasicOffense122::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::BasicOffense122::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	// project the ball forward (dampened)
	const float proj_time = 0.75; // seconds to project
	const float dampen_factor = 0.9; // accounts for slowing over time
	Geometry2d::Point ballProj = ball().pos + ball().vel * proj_time * dampen_factor;

	// Get a striker first to ensure there a robot that can kick
	assignNearestKicker(_striker.robot, available, ballProj);

	if(*_defense_first) {
	// defense first - get closest to goal to choose sides properly
		assignNearest(_leftDefender.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
		assignNearest(_rightDefender.robot, available, Geometry2d::Point(Field_GoalWidth/2, 0.0));

	// choose offense, we want closest robot to ball to be striker
		assignNearest(_support1.robot, available, ballProj);
		if (_support1.robot)
			_support1.robot->addText("Support 1");
		assignNearest(_support2.robot, available, ballProj);
		if (_support2.robot)
			_support2.robot->addText("Support 2");
	} else {
	// choose offense, we want closest robot to ball to be striker
		assignNearest(_support1.robot, available, ballProj);
		if (_support1.robot)
			_support1.robot->addText("Support 1");
		assignNearest(_support2.robot, available, ballProj);
		if (_support2.robot)
			_support2.robot->addText("Support 2");
		
	// defense first - get closest to goal to choose sides properly
		assignNearest(_leftDefender.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
		assignNearest(_rightDefender.robot, available, Geometry2d::Point(Field_GoalWidth/2, 0.0));
	}

	// determine whether to change offense players
	bool forward_reset = false;
	if (_striker.robot && _support1.robot && _support2.robot &&
			(_support1.robot->pos.distTo(ballProj) < *_offense_hysteresis * _striker.robot->pos.distTo(ballProj)
				|| _support2.robot->pos.distTo(ballProj) < *_offense_hysteresis * _striker.robot->pos.distTo(ballProj))) {
		_striker.robot = NULL;
		_support1.robot = NULL;
		_support2.robot = NULL;
		_striker.restart();
		forward_reset = true;
	}

	// find the nearest opponent to the striker
	OpponentRobot* closestRobotToStriker = 0;
	float closestDistToStriker = numeric_limits<float>::infinity();
	if (_striker.robot) {
		for (OpponentRobot* opp :  state()->opp) {
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
	// find *two* furthest back opp robots
	OpponentRobot* bestOpp1 = NULL, *bestOpp2 = NULL;
	float bestDist1 = numeric_limits<float>::infinity(), bestDist2 = bestDist1;
	size_t nrOppClose = 0;
	for (OpponentRobot* opp :  state()->opp)
	{
		// use distance from goal rather than just y coordinate to handle corners better
		const float oppDistSq = opp->pos.magsq();
		if (opp && opp->visible &&
				   oppDistSq > 0.1 &&
				   !(striker_engaged && opp == closestRobotToStriker)
				   ) {
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
	if (!bestOpp1 && _support1.robot) {
		_support1.robot->addText("No mark target");
		if(_striker.robot){
			// if nothing to mark, hang behind the striker on other side of the field
			_support1.robot->addText("Static Support");
			Point support_goal = _striker.robot->pos;
			support_goal.x *= -1.0;
			if (fabs(support_goal.x) < 0.2)
			{
				support_goal.x = (support_goal.x < 0.0) ? -1.0 : 1.0;
			}

			if (ballProj.y > Field_Length/2.0 && nrOppClose)
				support_goal.y = max(support_goal.y * (double)*_offense_support_ratio, 0.3);
			else
				support_goal.y = max(support_goal.y * (double)*_defense_support_ratio, 0.3);

			_support1.robot->move(support_goal);
			_support1.robot->face(ballProj);
		}
	}
	if (!bestOpp2 && _support2.robot) {
		_support2.robot->addText("No mark target");
		//TODO: something useful like wait for a rebound kick
	}
	bool marking_active_1 = bestOpp1 != 0;
	bool marking_active_2 = bestOpp2 != 0;

	// use hysteresis for changing of the robot
	float cur_dist1 = (_support1.markRobot()) ?	_support1.markRobot()->pos.distTo(ballProj) : numeric_limits<float>::infinity();
	float cur_dist2 = (_support2.markRobot()) ?	_support2.markRobot()->pos.distTo(ballProj) : numeric_limits<float>::infinity();
	if (bestOpp1 && bestOpp1->visible && (forward_reset || bestDist1 < cur_dist1 * cur_dist1 * (float)*_mark_hysteresis_coeff))
		_support1.markRobot(bestOpp1);
	if (bestOpp2 && bestOpp2->visible && (forward_reset || bestDist2 < cur_dist2 * cur_dist2 * (float)*_mark_hysteresis_coeff))
		_support2.markRobot(bestOpp2);

	// if we are further away, we can mark further from robot
	if (ballProj.y > Field_Length/2.0 && nrOppClose) {
		_support1.ratio(*_offense_support_ratio);
		_support2.ratio(*_offense_support_ratio);
	}
	else {
		_support1.ratio(*_defense_support_ratio);
		_support2.ratio(*_defense_support_ratio);
	}

	// adjust obstacles on striker and support
	if (_striker.robot) {
		unsigned striker = _striker.robot->shell();
		if (_support1.robot)
			_support1.robot->avoidTeammateRadius(striker, *_support_avoid_teammate_radius);
		if (_support2.robot)
			_support2.robot->avoidTeammateRadius(striker, *_support_avoid_teammate_radius);

		// get out of ways of shots
		if (_striker.robot->pos.nearPoint(ballProj, *_support_avoid_shot)) {
			Polygon shot_obs;
			shot_obs.vertices.push_back(Geometry2d::Point(Field_GoalWidth / 2, Field_Length));
			shot_obs.vertices.push_back(Geometry2d::Point(-Field_GoalWidth / 2, Field_Length));
			shot_obs.vertices.push_back(ballProj);
			if(_support1.robot)
				_support1.robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(shot_obs)));
			if(_support2.robot)
				_support2.robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(shot_obs)));
		}
	}

	// manually reset any kickers so they keep kicking
	if (_striker.done())
		_striker.restart();

	// set flags for inner behaviors
	_striker.use_line_kick = *_use_line_kick;
	if (_striker.robot)
	{
		_striker.calculateChipPower(_striker.robot->pos.distTo(ball().pos));
	}

	// execute behaviors
	if (_striker.robot) _striker.run();
	if (marking_active_1 && _support1.robot) _support1.run();
	if (marking_active_2 && _support2.robot) _support2.run();
	if (_leftDefender.robot) _leftDefender.run();
	if (_rightDefender.robot) _rightDefender.run();

	return true;
}
