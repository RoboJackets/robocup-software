#include "AggressiveZoneOffense.hpp"

using namespace std;

REGISTER_PLAY(Gameplay::Plays::AggressiveZoneOffense)

Gameplay::Plays::AggressiveZoneOffense::AggressiveZoneOffense(GameplayModule *gameplay):
	Play(gameplay),
	_fullback(gameplay, Behaviors::Fullback::Center),
	_offense(gameplay)
{
	set<Robot *> available = gameplay->robots();
	
	_fullback.assign(available);
	_offense.assign(available);
}

float Gameplay::Plays::AggressiveZoneOffense::score(GameplayModule *gameplay)
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	bool gameplayApplicable = _gameplay->state()->stateID.posession == SystemState::OFFENSE ||
						      _gameplay->state()->stateID.posession == SystemState::FREEBALL;

	return gameplay->robots().size() >= 4 && refApplicable && gameplayApplicable;
}

bool Gameplay::Plays::AggressiveZoneOffense::run()
{
	// setup offense
	_offense.run();

	// run the defender
	_fullback.run();
	
	return true;
}
