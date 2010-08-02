#include "TheirKickoff.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TheirKickoff, "Restarts")

Gameplay::Plays::TheirKickoff::TheirKickoff(GameplayModule *gameplay):
	Play(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_idle(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

bool Gameplay::Plays::TheirKickoff::applicable(const std::set<Robot *> &robots)
{
	return gameState().setupRestart() && gameState().theirKickoff();
}

bool Gameplay::Plays::TheirKickoff::assign(set<Robot *> &available)
{
	_robots = available;
	
	_fullback1.assign(available);
	_fullback2.assign(available);
	_idle.assign(available);

	return true;
}

bool Gameplay::Plays::TheirKickoff::run()
{
	_fullback1.run();
	_fullback2.run();
	_idle.run();
	
	return true;
}
