#include "KickMeasurement.hpp"

#include <Utils.hpp>
#include <stdio.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::KickMeasurement, "Test")

static const double robot_w = 2 * M_PI * 2;
static const double test_w = 2 * M_PI * 0.5;

static const double WAIT_TIME = 12;

Gameplay::Plays::KickMeasurement::KickMeasurement(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
	_time = 0;
	_lastKickTime = 0;
	_state = State_Setup;
	_kickPower=255;
	_file.open(("Kick_Exp_-" + QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log").toStdString()).c_str(), fstream::out);
	_file << str(format("%f %f 0 0 0 0\n") % test_w % robot_w);

	_robotPos = Geometry2d::Point(0,1);
}

bool Gameplay::Plays::KickMeasurement::run()
{
	if (_gameplay->playRobots().empty())
	{
		return false;
	}

	if (_time == 0)
	{
		_time = state()->timestamp;
		return true;
	}

	OurRobot *robot = *_gameplay->playRobots().begin();
	if (!robot && !robot->visible)
	{
		return false;
	}

	double curTime = (state()->timestamp - _time) / 1000000.0;

	if(_state==State_Wait)
	{
		if( curTime-_lastKickTime < WAIT_TIME )
			_state=State_Wait;
		else
		{
			_state = State_Setup;
		}
	}
	else if(_state == State_Setup)
	{
		robot->move(_robotPos, true);
		robot->face(Geometry2d::Point(0,10));
		if(robot->pos.distTo(_robotPos) < 0.01 && robot->angle == 90)
		{
			_state = State_Grab;
		}
	}
	else if(_state == State_Grab)
	{
		robot->worldVelocity(Geometry2d::Point(0,1) * 0.3);
		if(robot->pos.distTo(_robotPos) > 0.5)
		{
			_state = State_Setup;
		}
		else if(robot->hasBall())
		{
			_state = State_Kick;
		}
	}
	else if (_state == State_Kick){
		if(!_kicked && robot->hasBall())
		{
			robot->kick(_kickPower);
			_fireDelta = curTime;
			_kicked = true;
		}
		else if (_kicked && !robot->hasBall())
		{
			_fireDelta = curTime - _fireDelta;

			_lastKickTime = curTime;

			_file << str(format("%f %d %f\n") % curTime % (int)_kickPower % _fireDelta);

			_state = State_Wait;

			_kicked = false;
		}
	}
	return true;
}
