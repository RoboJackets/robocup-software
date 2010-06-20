#include "TheirKickoff.hpp"

using namespace std;

Gameplay::Plays::TheirKickoff::TheirKickoff(GameplayModule *gameplay):
	Play(gameplay, 1),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_back1(gameplay),
	_back2(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

bool Gameplay::Plays::TheirKickoff::applicable()
{
	return gameState().setupRestart() && gameState().theirKickoff();
}

bool Gameplay::Plays::TheirKickoff::assign(set<Robot *> &available)
{
	_robots = available;
	
	_fullback1.assign(available);
	_fullback2.assign(available);
	_back1.assign(available);
	_back2.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TheirKickoff::run()
{
	_fullback1.run();
	_fullback2.run();
	_back1.run();
	_back2.run();
	
	return true;
}
