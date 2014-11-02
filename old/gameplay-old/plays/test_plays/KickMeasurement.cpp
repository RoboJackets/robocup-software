#include "KickMeasurement.hpp"

#include <Utils.hpp>
#include <stdio.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::KickMeasurement, "Test")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(KickMeasurement)
	}
}

static const double robot_w = 2 * M_PI * 2;
static const double test_w = 2 * M_PI * 0.5;

static const double WAIT_TIME = 12;
static const double MEASURE_TIME = 10;
static const double GRAB_TIME = 0.1;

ConfigBool *Gameplay::Plays::KickMeasurement::_ready_signal;

void Gameplay::Plays::KickMeasurement::createConfiguration(Configuration *cfg)
{
	_ready_signal = new ConfigBool(cfg, "KickMeasurement/Ready Signal", true);
}

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

	_lastGrabTime = 0;

	_grabbed = false;
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

	set<OurRobot *> available = _gameplay->playRobots();

	OurRobot *robot = 0;
	assignNearest(robot, available, Geometry2d::Point(0,0));

	if (!robot || !robot->visible)
	{
		return false;
	}

	double curTime = (state()->timestamp - _time) / 1000000.0;

	if(_state == State_Setup)
		robot->addText(QString("Setup"));
	if(_state == State_Ready)
		robot->addText(QString("Ready"));
	if(_state == State_Grab)
		robot->addText(QString("Grab"));
	if(_state == State_Kick)
		robot->addText(QString("Kick"));
	if(_state == State_Measure)
		robot->addText(QString("Measure"));

	if(_state == State_Setup)
	{
		robot->move(_robotPos, true);
		robot->face(Geometry2d::Point(0,10));
		if(robot->pos.distTo(_robotPos) < 0.1 && robot->angle - 90 < 5)
		{
			if(ball().pos.distTo(robot->pos) < 0.5)
				_state = State_Ready;
		}
	}
	else if(_state == State_Ready)
		{
			if(*_ready_signal){
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

		if(_grabbed){
			if(robot->hasBall()){
				if( curTime - _lastGrabTime > GRAB_TIME){
					_state = State_Kick;
					_grabbed = false;
				}
			}else{
				_grabbed = false;
			}
		}else{
			if(robot->hasBall()){
				_grabbed = true;
				_lastGrabTime = curTime;
			}
		}
	}
	else if (_state == State_Kick){
		if(!_kicked && robot->hasBall())
		{
			robot->kick(_kickPower);
			_fireDelta = curTime;
			_kicked = true;
		}
		else if (_kicked && !robot->hasBall()) //!robot->hasBall())
		{
			_fireDelta = curTime - _fireDelta;

			_lastKickTime = curTime;

			_file << str(format("%f %d %f\n") % curTime % (int)_kickPower % _fireDelta);

			_state = State_Measure;

			_kicked = false;
		}
	}
	else if (_state == State_Measure){

		_file << str(format("%f %f %f %f\n") % curTime % ball().vel.mag() % ball().pos.x % ball().pos.y);

		if (curTime-_lastKickTime < MEASURE_TIME){
			if(curTime - _lastKickTime > 1.0 && ball().vel.mag() < 0.3)
				_state = State_Setup;
		}else{
			_state = State_Setup;
		}
	}
	return true;
}
