#include "TheirFreekick.hpp"

#include <boost/foreach.hpp>

using namespace std;

Gameplay::Plays::TheirFreekick::TheirFreekick(GameplayModule *gameplay):
	Play(gameplay, 1),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_marking1(gameplay),
	_marking2(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

bool Gameplay::Plays::TheirFreekick::applicable()
{
	return gameState().setupRestart() && gameState().theirFreeKick();
}
bool Gameplay::Plays::TheirFreekick::assign(set<Robot *> &available)
{
	_robots = available;
	
	_fullback1.assign(available);
	_fullback2.assign(available);
	_marking1.assign(available);
	_marking2.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TheirFreekick::run()
{
	//  determine which robots to mark
//	Robot *mark1 = 0, *mark2 = 0;
//	map<unsigned int, Robot*> open_opp;
//	BOOST_FOREACH(Robot * r, _gameplay->opp) {
//		if ()
//	}

	// execute marking
	_marking1.run();
	_marking2.run();

	// execute default fullback "wall" behavior
	_fullback1.run();
	_fullback2.run();
	
	return true;
}
