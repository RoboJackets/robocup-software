#include "TheirKickoff.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TheirKickoff, "Restarts")

Gameplay::Plays::TheirKickoff::TheirKickoff(GameplayModule *gameplay):
	Play(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_idle(gameplay)
{
	set<Robot *> available = gameplay->robots();
	
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
	
	_robots = available;
	
	_fullback1.assign(available);
	_fullback2.assign(available);
	_idle.assign(available);
}

float Gameplay::Plays::TheirKickoff::score ( Gameplay::GameplayModule* gameplay )
{
	return (gameplay->state()->gameState.setupRestart() && gameplay->state()->gameState.theirKickoff()) ? 0 : INFINITY;
}

bool Gameplay::Plays::TheirKickoff::run()
{
	_fullback1.run();
	_fullback2.run();
	_idle.run();
	
	return true;
}
