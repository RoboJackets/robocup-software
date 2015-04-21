#include "OurGoalKick.hpp"
#include <RobotConfig.hpp>

#include <iostream>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurGoalKick, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurGoalKick)
	}
}

ConfigDouble *Gameplay::Plays::OurGoalKick::_downFieldRange;
ConfigDouble *Gameplay::Plays::OurGoalKick::_minChipRange;
ConfigDouble *Gameplay::Plays::OurGoalKick::_maxChipRange;
ConfigInt *Gameplay::Plays::OurGoalKick::_chipper_power;

void Gameplay::Plays::OurGoalKick::createConfiguration(Configuration *cfg)
{
	_chipper_power = new ConfigInt(cfg, "OurGoalKick/Chipper Power", 255);
	_downFieldRange = new ConfigDouble(cfg, "OurGoalKick/Downfield Range", Field_Length / 2.);
	_minChipRange = new ConfigDouble(cfg, "OurGoalKick/Min Chip Range", 0.3);
	_maxChipRange = new ConfigDouble(cfg, "OurGoalKick/Max Chip Range", 3.0);
}

Gameplay::Plays::OurGoalKick::OurGoalKick(GameplayModule *gameplay):
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
}

float Gameplay::Plays::OurGoalKick::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	Point ballPos = gameplay->state()->ball.pos;
	bool chipper_available = false;
	for (OurRobot * r :  gameplay->playRobots())
	{
		if (r && r->chipper_available())
		{
			chipper_available = true;
			break;
		}
	}
	return (gs.setupRestart() && gs.ourDirect() && chipper_available && ballPos.y < 1.0) ? 1 : INFINITY;
}

bool Gameplay::Plays::OurGoalKick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	// pick the chip kicker - will only chip, otherwise OurGoalKick will not run, see score()
	if (!assignNearestChipper(_kicker.robot, available, ball().pos))
	{
		return false;
	}

	assignNearest(_center1.robot, available, _center1.target);
	assignNearest(_center2.robot, available, _center2.target);

	// choose a target for the kick
	// if straight shot on goal is available, take it
	Segment goal(Point(-Field_GoalWidth/2.0, Field_Length), Point(Field_GoalWidth/2.0, Field_Length));
	Segment target = goal;
	if (_kicker.findShot(goal, target, false, 0.3))
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
		// normal case: chip towards opposite (left/right) side
		Segment downfield;
		if (ball().pos.x < 0)
		{
			// right side
			downfield = Segment(Point(Field_Width/2.0, *_downFieldRange),
													Point(Field_Width/2.0, Field_Length));
		} else
		{
			downfield = Segment(Point(-Field_Width/2.0, *_downFieldRange),
													Point(-Field_Width/2.0, Field_Length));
		}
		_kicker.setTarget(downfield);

		// drive the center for a pass
		double center_x_coord = (ball().pos.x < 0) ? 1.0 : -1.0;
		_center1.target = Point(center_x_coord, *_downFieldRange - Robot_Radius);
		_center2.target = Point(center_x_coord, *_downFieldRange*3.0/2.0+Field_Length/2.0 - Robot_Radius);
		_kicker.use_chipper = true;
	}

	// setup kicker from parameters - want to use chipper when possible
	_kicker.use_line_kick = true;
	_kicker.kick_power = *_chipper_power;
	_kicker.minChipRange = *_minChipRange;
	_kicker.maxChipRange = *_maxChipRange;

	_pdt.backoff.robots.clear();
	_pdt.backoff.robots.insert(_kicker.robot);
	assignNearest(_defender1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_defender2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));
	
	_pdt.run();
	_center1.run();
	_center2.run();
	_defender1.run();
	_defender2.run();
	
	return _pdt.keepRunning();
}
