#include "TestLatency.hpp"

#include <Utils.hpp>
#include <stdio.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace Geometry2d;

REGISTER_PLAY(Gameplay::Plays::TestLatency)

static const double robot_w = 2 * M_PI * 2;
static const double test_w = 2 * M_PI * 0.5;

Gameplay::Plays::TestLatency::TestLatency(GameplayModule *gameplay):
	Play(gameplay)
{
	_startTime = 0;
	
	_file.open("latency.txt", fstream::out);
	_file << str(format("%f %f 0 0 0 0\n") % test_w % robot_w);
}

bool Gameplay::Plays::TestLatency::run()
{
	if (_gameplay->playRobots().empty())
	{
		return false;
	}
	
	OurRobot *robot = *_gameplay->playRobots().begin();
	if (!robot->visible)
	{
		return false;
	}
	
	if (_startTime == 0)
	{
		_startTime = state()->timestamp;
		return true;
	}
	
	double t = (state()->timestamp - _startTime) / 1000000.0 * test_w;
	double w_i = sin(t) * robot_w;
#if 1
	#warning angularVelocity() was depracated, so this doesn't do what it used to anymore
	// robot->angularVelocity(w_i);
	double obs = robot->angleVel * M_PI / 180.0;
#else
	robot->worldVelocity(Point(w_i, 0));
	double obs = robot->vel.x;
#endif
	_file << str(format("%f %f %d %d %d %d\n") % t % obs % robot->radioRx().encoders(0) % robot->radioRx().encoders(1) % robot->radioRx().encoders(2) % robot->radioRx().encoders(3));
	
	return true;
}
