#include "DemoKickMeasurement.hpp"

#include <stdio.h>

using namespace std;

using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoKickMeasurement, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoKickMeasurement)
	}
}

ConfigBool *Gameplay::Plays::DemoKickMeasurement::_use_chipper;
ConfigInt  *Gameplay::Plays::DemoKickMeasurement::_kick_power;

void Gameplay::Plays::DemoKickMeasurement::createConfiguration(Configuration* cfg)
{
	_use_chipper  = new ConfigBool(cfg, "DemoKickMeasurement/Enable Chipping", false);
	_kick_power = new ConfigInt(cfg, "DemoKickMeasurement/Kicker Power", 127);
}

Gameplay::Plays::DemoKickMeasurement::DemoKickMeasurement(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
	msr_index = 0;
	kicked = false;
}

bool Gameplay::Plays::DemoKickMeasurement::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_kicker.robot, available, Geometry2d::Point());

	Geometry2d::Point ballPos = ball().pos;

	// if we have kicked, we want to track the ball
	if (!kicked && _kicker.done() &&  ball().valid &&
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		kicked = true;
	}

	if(kicked){
		if(msr_index <100){
			position[msr_index] = ball().pos;
			velocity[msr_index] = ball().vel;
			time[msr_index] = ball().time;
			msr_index ++;

			printf("Ball Pos X: %f", ball().pos.x);
			printf("Ball Pos Y: %f", ball().pos.y);
			printf("Velocity X: %f", ball().vel.x);
			printf("Velocity Y: %f", ball().vel.y);
		}
	}

	_kicker.use_chipper = *_use_chipper;
	_kicker.kick_power = *_kick_power;

	_kicker.run();

	return true;
}
