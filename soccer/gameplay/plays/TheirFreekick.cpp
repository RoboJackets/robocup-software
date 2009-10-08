#include "TheirFreekick.hpp"

using namespace std;

Gameplay::Plays::TheirFreekick::TheirFreekick(GameplayModule *gameplay):
	Play(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_halfback1(gameplay),
	_halfback2(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

bool Gameplay::Plays::TheirFreekick::applicable()
{
	return gameState().setupRestart() && gameState().theirFreeKick();
}

void Gameplay::Plays::TheirFreekick::assign(set<Robot *> &available)
{
	_fullback1.assign(available);
	_fullback2.assign(available);
	_halfback1.assign(available);
	_halfback2.assign(available);
}

bool Gameplay::Plays::TheirFreekick::run()
{
	_fullback1.run();
	_fullback2.run();
	_halfback1.run();
	_halfback2.run();
	
	return true;
}
