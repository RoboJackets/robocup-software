//TODO: Prevent backup off the field
//TODO: have receiver intercept ball
//TODO: add kicked status for when kicked but not received yet?
//TODO: add linear function for kicker power

#include <iostream>
#include "PassReceive.hpp"

#include <algorithm>

using namespace std;

namespace Gameplay {
namespace Behaviors {
REGISTER_CONFIGURABLE(PassReceive)
}
}

namespace Gameplay {
namespace Behaviors {
ConfigInt *Gameplay::Behaviors::PassReceive::_backup_time;
ConfigDouble *Gameplay::Behaviors::PassReceive::_backup_speed;

void Gameplay::Behaviors::PassReceive::createConfiguration(Configuration *cfg)
{
	_backup_time = new ConfigInt(cfg, "PassReceive/Backup Time", 100000);
	_backup_speed = new ConfigDouble(cfg, "PassReceive/Backup Speed", 0.05);
}

Gameplay::Behaviors::PassReceive::PassReceive(GameplayModule *gameplay):
	TwoRobotBehavior(gameplay),
	_passer(gameplay)
{
	passUseChip = false;
	passUseLine = true;
	passPower = 32;
	readyTime = 0;
}

bool Gameplay::Behaviors::PassReceive::done()
{
	return _state == State_Done;
}

void Gameplay::Behaviors::PassReceive::reset()
{
	readyTime = 0;
	_state = State_Setup;
}

bool Gameplay::Behaviors::PassReceive::run()
{
	if(!robot1 || !robot2)
	{
		return false;
	}
	_passer.robot = robot1;

	_passer.use_chipper = passUseChip;
	_passer.use_line_kick = passUseLine;
	_passer.kick_power = passPower;

	//state changes
	if(_state == State_Setup)
	{
		if(_passer.kickReady)
		{
			_state = State_Ready;
		}
	}
	else if(_state == State_Ready)
	{
		if((state()->timestamp - rea127dyTime) > *_backup_time)
		{
			_state = State_Execute;
		}
	}
	else if(_state == State_Execute)
	{
		if(_passer.done())
		{
			_state = State_Done;
		}
	}


	//control
	_passer.setTarget(robot2->kickerBar());
	robot2->face(_passer.robot->pos);

	if(_state == State_Setup)
	{
		_passer.enableKick = false;
		_passer.run();
	}
	else if(_state == State_Ready)
	{
		if(readyTime == 0)
		{
			readyTime = state()->timestamp;
		}

		robot2->bodyVelocity(Geometry2d::Point(-(*_backup_speed),0));
		robot2->angularVelocity(0);

		_passer.run();
	}
	else if(_state == State_Execute)
	{
		robot2->bodyVelocity(Geometry2d::Point(-(*_backup_speed),0));
		robot2->angularVelocity(0);
		_passer.enableKick = true;
		_passer.run();
	}
	else if(_state == State_Done)
	{
		readyTime = 0;
		_passer.restart();
	}



	return true;
}

}
}
