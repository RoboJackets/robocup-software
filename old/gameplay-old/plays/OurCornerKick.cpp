#include "OurCornerKick.hpp"
#include <RobotConfig.hpp>

#include <iostream>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurCornerKick, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurCornerKick)
	}
}

ConfigDouble *Gameplay::Plays::OurCornerKick::_targetSegmentWidth;
ConfigDouble *Gameplay::Plays::OurCornerKick::_minChipRange;
ConfigDouble *Gameplay::Plays::OurCornerKick::_maxChipRange;
ConfigInt *Gameplay::Plays::OurCornerKick::_chipper_power;

void Gameplay::Plays::OurCornerKick::createConfiguration(Configuration *cfg)
{
	_chipper_power = new ConfigInt(cfg, "OurCornerKick/Chipper Power", 127);
	_targetSegmentWidth = new ConfigDouble(cfg, "OurCornerKick/Window Width", Field_Length / 4.);
	_minChipRange = new ConfigDouble(cfg, "OurCornerKick/Min Chip Range", 0.3);
	_maxChipRange = new ConfigDouble(cfg, "OurCornerKick/Max Chip Range", 3.0);
}

Gameplay::Plays::OurCornerKick::OurCornerKick(GameplayModule *gameplay):
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

	_center1.target = Point(0.0, Field_Length /2.0);
}

float Gameplay::Plays::OurCornerKick::score ( Gameplay::GameplayModule* gameplay )
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
	return (gs.setupRestart() && gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 1.0)) ? 1 : INFINITY;
}

bool Gameplay::Plays::OurCornerKick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	// pick the chip kicker - want to chip most of the time
	if (!assignNearestChipper(_kicker.robot, available, ball().pos))
	{
		return false;
	}

	assignNearest(_center1.robot, available, Point(-0.5, Field_Length));
	assignNearest(_center2.robot, available, Point(0.5, Field_Length));

	// choose a target for the kick
	// if straight shot on goal is available, take it
	float goal_x = (ball().pos.x < 0) ? Field_GoalWidth / 2.0 : -Field_GoalWidth / 2.0;
	Segment target(Point(goal_x, Field_Length), Point(goal_x, Field_Length - *_targetSegmentWidth));

	// camp first robot to receive "pass"
	_center1.target.x = (ball().pos.x < 0) ? Field_GoalWidth / 2.0 + 0.5 : -(Field_GoalWidth / 2.0 + 0.5);
	_center1.target.y = Field_Length - *_targetSegmentWidth / 2.0;

	// camp second side for TODO: short "pass"
	_center2.target.x = (ball().pos.x < 0) ? -(Field_GoalWidth / 2.0 + 0.5) : Field_GoalWidth / 2.0 + 0.5 ;
	_center2.target.y = Field_Length - *_targetSegmentWidth / 2.0;

	// setup kicker from parameters - want to use chipper when possible
	_kicker.setTarget(target);
	_kicker.use_chipper = true;
	_kicker.use_line_kick = true;
	_kicker.calculateChipPower(_center1.target.distTo(ball().pos));
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
