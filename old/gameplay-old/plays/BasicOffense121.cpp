#include <limits>

#include "BasicOffense121.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::BasicOffense121, "Playing")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(BasicOffense121)
	}
}

ConfigDouble *Gameplay::Plays::BasicOffense121::_offense_hysteresis;
ConfigDouble *Gameplay::Plays::BasicOffense121::_support_backoff_thresh;
ConfigDouble *Gameplay::Plays::BasicOffense121::_mark_hysteresis_coeff;
ConfigDouble *Gameplay::Plays::BasicOffense121::_support_avoid_teammate_radius;
ConfigDouble *Gameplay::Plays::BasicOffense121::_support_avoid_shot;
ConfigDouble *Gameplay::Plays::BasicOffense121::_offense_support_ratio;
ConfigDouble *Gameplay::Plays::BasicOffense121::_defense_support_ratio;

void Gameplay::Plays::BasicOffense121::createConfiguration(Configuration *cfg)
{
	_offense_hysteresis = new ConfigDouble(cfg, "BasicOffense121/Hystersis Coeff", 0.50);
	_support_backoff_thresh = new ConfigDouble(cfg, "BasicOffense121/Support Backoff Dist", 1.5);
	_mark_hysteresis_coeff = new ConfigDouble(cfg, "BasicOffense121/Mark Hystersis Coeff", 0.9);
	_support_avoid_teammate_radius = new ConfigDouble(cfg, "BasicOffense121/Support Avoid Teammate Dist", 0.5);
	_support_avoid_shot = new ConfigDouble(cfg, "BasicOffense121/Support Avoid Shot", 0.2);
	_offense_support_ratio = new ConfigDouble(cfg, "BasicOffense121/Offense Support Ratio", 0.7);
	_defense_support_ratio = new ConfigDouble(cfg, "BasicOffense121/Defense Support Ratio", 0.9);
}

Gameplay::Plays::BasicOffense121::BasicOffense121(GameplayModule *gameplay):
	Play(gameplay),
	_defender(gameplay, Behaviors::Defender::Center),
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
	assignNearest(_defender.robot, available, Geometry2d::Point(0.0, 0.0));

	// determine whether to change offense players
	bool forward_reset = false;
	if (_striker.robot && _support1.robot && _support2.robot &&
		(_support1.robot->pos.distTo(ballProj) < *_offense_hysteresis * _striker.robot->pos.distTo(ballProj) ||
		 _support2.robot->pos.distTo(ballProj) < *_offense_hysteresis * _striker.robot->pos.distTo(ballProj))) {
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
	for (OpponentRobot* opp :  state()->opp)
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
	if (!bestOpp1 && _support1.robot) {
		_support1.robot->addText("No mark target");
	}
	if (!bestOpp2 && _support2.robot) {
		_support2.robot->addText("No mark target");
	}

	// use hysteresis for changing of the robot
	if (bestOpp1 && bestOpp1->visible && (forward_reset || bestDist1 < cur_dist1 * cur_dist1 * (float)*_mark_hysteresis_coeff))
		_support1.markRobot(bestOpp1);
	if (bestOpp2 && bestOpp2->visible && (forward_reset || bestDist2 < cur_dist2 * cur_dist2 * (float)*_mark_hysteresis_coeff))
		_support2.markRobot(bestOpp2);

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
			if(_support1.robot)
				_support2.robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(shot_obs)));
		}
	}

	// execute behaviors
	if (_striker.robot) _striker.run();
	if (_support1.robot) _support1.run();
	if (_support2.robot) _support2.run();
	if (_defender.robot) _defender.run();
	
	return true;
}
