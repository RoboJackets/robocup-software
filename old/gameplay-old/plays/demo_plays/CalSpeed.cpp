#include "CalSpeed.hpp"

#include <stdio.h>

using namespace std;

using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::CalSpeed, "Demos")

Gameplay::Plays::CalSpeed::CalSpeed(GameplayModule *gameplay):
	Play(gameplay)
{
	_speed = 0;
	_startTime = _gameplay->state()->timestamp;
	_lastAngle = 0;
	_average = 0;
	_count = 0;
	
	printf("---- CalSpeed ----\n");
}

float Gameplay::Plays::CalSpeed::score(GameplayModule *gameplay)
{
	return gameplay->state()->gameState.playing() ? 0 : INFINITY;
}

bool Gameplay::Plays::CalSpeed::run()
{
	const unsigned int Step_Time = 2000000;
	
	OurRobot *robot = *_gameplay->playRobots().begin();
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	const uint64_t now = _gameplay->state()->timestamp;
	
	int out;
	if (_speed <= 127)
	{
		if ((now - _startTime) >= Step_Time)
		{
			printf("%3d %f\n", _speed, _average);
			
			++_speed;
			_startTime = now;
			_average = 0;
			_count = 0;
			
			if (_speed > 127)
			{
				printf("---- CalSpeed done ----\n");
			}
		}
		out = _speed;
		robot->addText(QString().sprintf("CalSpeed: %3d", out));
	} else {
		// Finished all measurements
		robot->addText("CalSpeed: done");
		out = 0;
	}
	
	float dtime = (now - _lastTime) / 1.0e6;
	_lastTime = now;
	
	float w = fixAngleDegrees(robot->angle - _lastAngle) / dtime;
	
	_average = (_average * _count + w) / (_count + 1);
	_lastAngle = robot->angle;
	++_count;
	
	robot->cmd.planner = MotionCmd::DirectMotor;
	robot->cmd.direct_motor_cmds.resize(4);
	for (int i = 0; i < 4; ++i)
	{
		robot->cmd.direct_motor_cmds[i] = out;
	}
	
	return true;
}
