/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#include "TestKicks.hpp"
#include <boost/foreach.hpp>

using namespace std;

Gameplay::Plays::TestKicks::TestKicks(GameplayModule *gameplay):
	Play(gameplay),
	_test(Deg0StationaryKick),
	swichTest(true)
{
}

bool Gameplay::Plays::TestKicks::applicable()
{
	return true;
}

bool Gameplay::Plays::TestKicks::assign(set<Robot *> &available)
{
	_robots = available;
	takeAll(available);
	return true;
}

bool Gameplay::Plays::TestKicks::run()
{
	Robot* robot;

	switch(_test){
		case(Deg0StationaryKick):
				robot = getRobotWithId(0);
		break;
		default:

		break;
	}
	_test = (_test+1)%(NumTests);


	/** Replace this with your code to run every frame */
	return true;
}

Robot* Gameplay::Plays::TestKicks::getRobotWithId(int id)
{
	BOOST_FOREACH(Robot* robot, _robots)
	{
		if(id == robot->shell){
			return robot;
		}
	}
	return 0;
}
