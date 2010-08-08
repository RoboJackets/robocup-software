#include "DemoKicks.hpp"
#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoKicks, "Demos")

Gameplay::Plays::DemoKicks::DemoKicks(GameplayModule *gameplay):
	Play(gameplay),
	_test(Deg0StationaryKick),
	swichDemo(true)
{
	set<Robot *> available = gameplay->robots();
	_robots = available;
	takeAll(available);
}

bool Gameplay::Plays::DemoKicks::run()
{
	Robot* robot;

	switch(_test){
		case(Deg0StationaryKick):
				robot = getRobotWithId(0);
		break;
		default:

		break;
	}
	_test = (_test+1)%(NumDemos);


	/** Replace this with your code to run every frame */
	return true;
}

Robot* Gameplay::Plays::DemoKicks::getRobotWithId(int id)
{
	BOOST_FOREACH(Robot* robot, _robots)
	{
		if(id == robot->shell){
			return robot;
		}
	}
	return 0;
}
