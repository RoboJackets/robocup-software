#include "TheirFreekick.hpp"

using namespace std;

Gameplay::Plays::TheirFreekick::TheirFreekick(GameplayModule *gameplay):
	Play(gameplay, 4),
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
bool Gameplay::Plays::TheirFreekick::assign(set<Robot *> &available)
{
	if(!_fullback1.assign(available)){return false;};
	if(!_fullback2.assign(available)){return false;};
	if(!_halfback1.assign(available)){return false;};
	if(!_halfback2.assign(available)){return false;};

	_robots.insert(_fullback1.robot());
	_robots.insert(_fullback2.robot());
	_robots.insert(_halfback1.robot());
	_robots.insert(_halfback2.robot());

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TheirFreekick::run()
{
	_fullback1.run();
	_fullback2.run();
	_halfback1.run();
	_halfback2.run();
	
	return true;
}
