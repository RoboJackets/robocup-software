#include "DefendGoal.hpp"

using namespace std;

REGISTER_PLAY(Gameplay::Plays::DefendGoal)

Gameplay::Plays::DefendGoal::DefendGoal(GameplayModule *gameplay):
	Play(gameplay,4),
	_defenseState(DefendAndDefend),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_kicker1(gameplay),
	_kicker2(gameplay),
	_goalDefender(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

bool Gameplay::Plays::DefendGoal::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	bool gameplayApplicable = true; //_gameplay->state()->stateID.posession == Packet::LogFrame::DEFENSE;

	return refApplicable && gameplayApplicable;
}

bool Gameplay::Plays::DefendGoal::assign(set<Robot *> &available)
{
	if(!_goalDefender.assign(available)){return false;}
	//if(!_fullback1.assign(available)){return false;};
	//if(!_fullback2.assign(available)){return false;};
	//if(!_kicker1.assign(available)){return false;};
	//if(!_kicker2.assign(available)){return false;};

	//_robots.insert(_fullback1.robot());
	//_robots.insert(_fullback2.robot());
	//_robots.insert(_kicker1.robot());
	//_robots.insert(_kicker2.robot());

	//return _robots.size() >= _minRobots;
	return true;
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
