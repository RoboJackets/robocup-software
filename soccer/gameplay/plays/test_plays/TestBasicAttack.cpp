#include "TestBasicAttack.hpp"

using namespace std;

Gameplay::Plays::TestBasicAttack::TestBasicAttack(GameplayModule *gameplay):
	Play(gameplay), _kicker(gameplay)
{
}

bool Gameplay::Plays::TestBasicAttack::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

void Gameplay::Plays::TestBasicAttack::assign(set<Robot *> &available)
{
	_kicker.assign(available);
}

bool Gameplay::Plays::TestBasicAttack::run()
{
	_kicker.run();
	return true;
}
