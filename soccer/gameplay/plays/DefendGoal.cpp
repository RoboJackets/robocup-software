#include "DefendGoal.hpp"

using namespace std;

REGISTER_PLAY(Gameplay::Plays::DefendGoal)

Gameplay::Plays::DefendGoal::DefendGoal(GameplayModule *gameplay):
	Play(gameplay),
	_defenseState(DefendAndDefend),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_kicker1(gameplay),
	_kicker2(gameplay),
	_goalDefender(gameplay)
{
	set<Robot *> available = gameplay->robots();
	
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);

	_goalDefender.assign(available);
}

float Gameplay::Plays::DefendGoal::score ( GameplayModule* gameplay )
{
	bool refApplicable = gameplay->state()->gameState.playing();
	bool gameplayApplicable = true; //gameplay->state()->stateID.posession == Packet::LogFrame::DEFENSE;

	return (refApplicable && gameplayApplicable) ? 0 : INFINITY;
}

bool Gameplay::Plays::DefendGoal::run()
{
	//_fullback1.run();
	//_fullback2.run();
	//_kicker1.run();
	//_kicker2.run();
	_goalDefender.run();
	return true;
}
