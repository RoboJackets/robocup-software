#include "TestLatency.hpp"

#include <Utils.hpp>
#include <stdio.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

REGISTER_PLAY(Gameplay::Plays::TestLatency)

Gameplay::Plays::TestLatency::TestLatency(GameplayModule *gameplay):
	Play(gameplay)
{
	_startTime = Utils::timestamp();
	_lastTime = _startTime;
	_lastAngle = 0;
	_first = true;
	
	_file.open("latency.txt", fstream::out);
}

bool Gameplay::Plays::TestLatency::run()
{
	if (_gameplay->playRobots().empty())
	{
		return false;
	}
	
	uint64_t now = state()->timestamp;
	float dtime = (now - _lastTime) / 1000000.0f;
	_lastTime = now;
	
	uint64_t t = now - _startTime;
	float w = sin(t / 1000000.0 * M_PI) * 4 * M_PI;
	OurRobot *robot = *_gameplay->playRobots().begin();
	if (!robot->visible)
	{
		return false;
	}
	robot->angularVelocity(w);
	
	float a = robot->angle * M_PI / 180;
	float delta = Utils::fixAngleRadians(a - _lastAngle) / dtime;
	_lastAngle = a;
	
	if (_first)
	{
		_first = false;
	} else {
		_file << str(format("%f %f %f %f\n") % w % delta % robot->angle % dtime);
	}
	
	return true;
}
