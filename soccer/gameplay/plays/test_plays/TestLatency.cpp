#include "TestLatency.hpp"

#include <stdio.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

REGISTER_PLAY(Gameplay::Plays::TestLatency)

Gameplay::Plays::TestLatency::TestLatency(GameplayModule *gameplay):
	Play(gameplay)
{
	_startTime = Utils::timestamp();
	_total = 0;
	_lastAngle = 0;
	_angle = 0;
	
	_file.open("latency.txt", fstream::out);
}

bool Gameplay::Plays::TestLatency::run()
{
	if (_gameplay->robots().empty())
	{
		return false;
	}
	
	uint64_t t = Utils::timestamp() - _startTime;
	float w = sin(t / 1000000.0 * M_PI);
	Robot *r = *_gameplay->robots().begin();
	r->directVelocityCommands(Geometry2d::Point(), w * 80);
	
	float a = r->angle();
	float delta = a - _lastAngle;
	if (delta < -180)
	{
		delta += 360;
	} else if (delta > 180)
	{
		delta -= 360;
	}
	_angle += delta;
	_lastAngle = a;
	
	_file << str(format("%f %f\n") % w % _angle);
	
	return true;
}
