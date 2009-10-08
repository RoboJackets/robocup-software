#include "Offense.hpp"

using namespace std;

Gameplay::Plays::Offense::Offense(GameplayModule *gameplay):
	Play(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Left),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

bool Gameplay::Plays::Offense::applicable()
{
	//FIXME - Offense/defense
	return _gameplay->state()->gameState.playing();
}

void Gameplay::Plays::Offense::assign(set<Robot *> &available)
{
	_kicker1.assign(available);
	_kicker2.assign(available);
	_fullback1.assign(available);
	_fullback2.assign(available);
}

bool Gameplay::Plays::Offense::run()
{
	_kicker1.run();
	_kicker2.run();
	_fullback1.run();
	_fullback2.run();
	
	return true;
}
