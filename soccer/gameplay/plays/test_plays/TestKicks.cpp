/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#include "TestKicks.hpp"

using namespace std;

Gameplay::Plays::TestKicks::TestKicks(GameplayModule *gameplay):
	Play(gameplay),
	_test(Deg0StationaryKick)
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
				//robot =
		break;
	}
	_test = (_test+1)%(NumTests);


	/** Replace this with your code to run every frame */
	return true;
}
