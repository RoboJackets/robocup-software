#include "AggressiveZoneOffense.hpp"

using namespace std;

REGISTER_PLAY(Gameplay::Plays::AggressiveZoneOffense)

Gameplay::Plays::AggressiveZoneOffense::AggressiveZoneOffense(GameplayModule *gameplay):
	Play(gameplay),
	_fullback(gameplay, Behaviors::Fullback::Center),
	_offense(gameplay)
{
}

bool Gameplay::Plays::AggressiveZoneOffense::applicable(const std::set<Robot *> &robots)
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	bool gameplayApplicable = _gameplay->state()->stateID.posession == SystemState::OFFENSE ||
						      _gameplay->state()->stateID.posession == SystemState::FREEBALL;

	return robots.size() >= 4 && refApplicable && gameplayApplicable;
}

bool Gameplay::Plays::AggressiveZoneOffense::assign(set<Robot *> &available)
{
	if(!_fullback.assign(available)) {return false;}
	if(!_offense.assign(available)) {return false;}

	return true;
}

bool Gameplay::Plays::AggressiveZoneOffense::run()
{
	// setup offense
	_offense.run();

	// run the defender
	_fullback.run();
	
	return true;
}
