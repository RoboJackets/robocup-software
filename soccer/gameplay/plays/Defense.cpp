#include "Defense.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::Defense, "Playing")

Gameplay::Plays::Defense::Defense(GameplayModule *gameplay):
	Play(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
	set<Robot *> available = gameplay->robots();
	
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);

	_robots = available;
	
	_fullback1.assign(available);
	_fullback2.assign(available);
	_kicker1.assign(available);
	_kicker2.assign(available);
}

float Gameplay::Plays::Defense::score ( GameplayModule* gameplay )
{
	bool refApplicable = gameplay->state()->gameState.playing();
	bool gameplayApplicable = gameplay->state()->stateID.posession == SystemState::DEFENSE;

	return (refApplicable && gameplayApplicable) ? 0 : INFINITY;
}

bool Gameplay::Plays::Defense::run()
{
	_fullback1.run();
	_fullback2.run();
	_kicker1.run();
	_kicker2.run();
	
	return true;
}
