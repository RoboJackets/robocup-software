#include "Defense.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::Defense, "Playing")

Gameplay::Plays::Defense::Defense(GameplayModule *gameplay):
	Play(gameplay, 1),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

bool Gameplay::Plays::Defense::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	bool gameplayApplicable = _gameplay->state()->stateID.posession == SystemState::DEFENSE;

	return refApplicable && gameplayApplicable;
}

bool Gameplay::Plays::Defense::assign(set<Robot *> &available)
{
	_robots = available;
	
	_fullback1.assign(available);
	_fullback2.assign(available);
	_kicker1.assign(available);
	_kicker2.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::Defense::run()
{
	_fullback1.run();
	_fullback2.run();
	_kicker1.run();
	_kicker2.run();
	
	return true;
}
