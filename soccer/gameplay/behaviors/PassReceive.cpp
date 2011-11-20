//TODO: have receiver intercept ball
//TODO: add kicked status for when kicked but not received yet?

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
ConfigDouble *Gameplay::Behaviors::PassReceive::_done_thresh;
ConfigDouble *Gameplay::Behaviors::PassReceive::_kick_power_constant;

void Gameplay::Behaviors::PassReceive::createConfiguration(Configuration *cfg)
{
	_backup_time = new ConfigInt(cfg, "PassReceive/Backup Time", 100000);
	_backup_speed = new ConfigDouble(cfg, "PassReceive/Backup Speed", 0.05);
	_done_thresh = new ConfigDouble(cfg, "PassReceive/Done Thresh", 0.1);
	_kick_power_constant = new ConfigDouble(cfg, "PassReceive/Kick Power Constant", 10);
}

Gameplay::Behaviors::PassReceive::PassReceive(GameplayModule *gameplay):
	TwoRobotBehavior(gameplay),
	_passer(gameplay)
{
	passUseChip = false;
	passUseLine = true;
	passPower = 32;
	readyTime = 0;
	enable = true;
}

bool Gameplay::Behaviors::PassReceive::done()
{
	return _state == State_Done;
}

bool Gameplay::Behaviors::PassReceive::kicked()
{
	return _state == State_Kicked;
}

void Gameplay::Behaviors::PassReceive::setEnable(bool b)
{
	enable = b;
}
void Gameplay::Behaviors::PassReceive::reset()
{
	readyTime = 0;
	_state = State_Setup;
}

uint8_t Gameplay::Behaviors::PassReceive::calcKickPower(int d)
{
	return d * (*_kick_power_constant);
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
	if(_state == State_Setup && enable)
	{
		if(_passer.kickReady)
		{
			_state = State_Ready;
		}
	}
	else if(_state == State_Ready)
	{
		if((state()->timestamp - readyTime) > *_backup_time)
		{
			_state = State_Execute;
		}
	}
	else if(_state == State_Execute)
	{
		if(_passer.done())
		{
			_state = State_Kicked;
		}
	}
	else if(_state == State_Kicked)
	{
		if(ball().pos.distTo(robot2->pos) < *_done_thresh)
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

		if(robot2->pos.y > 0 && robot2->pos.y < Field_Length && robot2->pos.x > -Field_Width && robot2->pos.x < Field_Width)
		{
			robot2->bodyVelocity(Geometry2d::Point(-(*_backup_speed),0));
			robot2->angularVelocity(0);
		}

		_passer.run();
	}
	else if(_state == State_Execute)
	{
		_passer.enableKick = true;
		_passer.kick_power = calcKickPower(robot1->pos.distTo(robot2->pos));
		_passer.run();
	}
	else if(_state == State_Kicked)
	{
		Geometry2d::Point p1 = ball().pos;
		Geometry2d::Point p2 = ball().pos + ball().vel * 3;
		Geometry2d::Point p3 = robot2->pos;

		float u  = ((p3.x - p1.x) * (p2.x - p1.x) + (p3.y - p1.y) * (p2.y - p1.y)) / (p2 - p1).mag();

		Geometry2d::Point intercept(p1.x + u * (p2.x - p1.x),p1.y + u * (p2.y - p1.y));
		robot2->move(intercept, true);
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
