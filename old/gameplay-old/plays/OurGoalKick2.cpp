#include "OurGoalKick2.hpp"
#include <RobotConfig.hpp>

#include <iostream>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurGoalKick2, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurGoalKick2)
	}
}

ConfigDouble *Gameplay::Plays::OurGoalKick2::_downFieldRange;
ConfigDouble *Gameplay::Plays::OurGoalKick2::_minChipRange;
ConfigDouble *Gameplay::Plays::OurGoalKick2::_maxChipRange;
ConfigInt *Gameplay::Plays::OurGoalKick2::_chipper_power;
ConfigInt *Gameplay::Plays::OurGoalKick2::_kicker_power;
ConfigDouble *Gameplay::Plays::OurGoalKick2::_field_length_mult;
ConfigDouble *Gameplay::Plays::OurGoalKick2::_field_width_mult;
ConfigDouble *Gameplay::Plays::OurGoalKick2::_field_length_off;
ConfigDouble *Gameplay::Plays::OurGoalKick2::_field_width_off;

void Gameplay::Plays::OurGoalKick2::createConfiguration(Configuration *cfg)
{
	_chipper_power = new ConfigInt(cfg, "OurGoalKick2/Chipper Power", 255);
	_kicker_power = new ConfigInt(cfg, "OurGoalKick2/Kicker Power", 255);
	_downFieldRange = new ConfigDouble(cfg, "OurGoalKick2/Downfield Range", Field_Length / 2.);
	_minChipRange = new ConfigDouble(cfg, "OurGoalKick2/Min Chip Range", 0.3);
	_maxChipRange = new ConfigDouble(cfg, "OurGoalKick2/Max Chip Range", 3.0);
	_field_length_mult = new ConfigDouble(cfg, "OurGoalKick2/Centers Length Multiplier", 0.6);
	_field_length_off = new ConfigDouble(cfg, "OurGoalKick2/Centers Length Offset", 0.0);
	_field_width_mult = new ConfigDouble(cfg, "OurGoalKick2/Centers Width Multiplier", 0.15);
	_field_width_off = new ConfigDouble(cfg, "OurGoalKick2/Centers Width Offset", 0.0);
}

Gameplay::Plays::OurGoalKick2::OurGoalKick2(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_center1(gameplay),
	_center2(gameplay),
	_defender1(gameplay, Behaviors::Defender::Left),
	_defender2(gameplay, Behaviors::Defender::Right),
	_pdt(gameplay, &_kicker)
{
	_defender2.otherDefenders.insert(&_defender1);
	_defender1.otherDefenders.insert(&_defender2);
	hasChipper = false;
}

float Gameplay::Plays::OurGoalKick2::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	Point ballPos = gameplay->state()->ball.pos;
	return (gs.setupRestart() && gs.ourDirect() && ballPos.y < 1.0) ? 1 : INFINITY;
}

bool Gameplay::Plays::OurGoalKick2::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	// Assign Kicker
	if (!assignNearestChipper(_kicker.robot, available, ball().pos))
	{
		if(!assignNearest(_kicker.robot, available, ball().pos))
		{
			// Could not get a robot to play with :(
			return false;
		}
		hasChipper = false;
	}
	else
	{
		hasChipper = true;
	}

	assignNearest(_center1.robot, available, Point(-Field_Width/2.0,Field_Length));
	assignNearest(_center2.robot, available, Point( Field_Width/2.0,Field_Length));

	assignNearest(_defender1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_defender2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));

	// choose a target for the kick
	// if straight shot on goal is available, take it
	Segment goal(Point(-Field_GoalWidth/2.0, Field_Length), Point(Field_GoalWidth/2.0, Field_Length));
	Segment target = goal;
	if (_kicker.findShot(goal, target, hasChipper, Ball_Radius * 3.0))
	{
		// in case there's an easy shot
		_kicker.override_aim = true;
		_kicker.setTarget(target);
		_kicker.use_chipper = false;

		// center stays out of the way
		Polygon shot_obs;
		shot_obs.vertices.push_back(target.pt[0]);
		shot_obs.vertices.push_back(target.pt[1]);
		shot_obs.vertices.push_back(ball().pos);
		if(_center1.robot)
			_center1.robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(shot_obs)));
		if(_center2.robot)
			_center2.robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(shot_obs)));
		_center1.target = Point(0.0, 1.5);
		_center2.target = Point(1.0, 1.5);
	} else
	{
		// Normal case: Chip lightly towards the middle-ish.
		_kicker.use_chipper = hasChipper;
		_kicker.forceChip = hasChipper;

		_center1.target = Point(
				-Field_Width * _field_width_mult->value() + _field_width_off->value(),
				Field_Length * _field_length_mult->value() + _field_length_off->value()
				);
		_center2.target = Point(
				Field_Width * _field_width_mult->value() + _field_width_off->value(),
				Field_Length * _field_length_mult->value() + _field_length_off->value()
				);

		_kicker.setTarget(Segment(_center1.target, _center2.target));
	}

	// setup kicker from parameters - want to use chipper when possible
	_kicker.use_line_kick = true;
	if(hasChipper)
		_kicker.chip_power = *_chipper_power;
	else
		_kicker.kick_power = *_kicker_power;
	_kicker.minChipRange = *_minChipRange;
	_kicker.maxChipRange = *_maxChipRange;

	_pdt.backoff.robots.clear();
	_pdt.backoff.robots.insert(_kicker.robot);

	_pdt.run();
	_center1.run();
	_center2.run();
	_defender1.run();
	_defender2.run();
	
	return _pdt.keepRunning();
}
