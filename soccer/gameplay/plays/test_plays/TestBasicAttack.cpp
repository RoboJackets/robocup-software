#include "TestBasicAttack.hpp"

#include <boost/foreach.hpp>

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
	// remove non-visible robots
	// this prevents bug with 2-robot tests where play would do nothing
	BOOST_FOREACH(Robot *r, available){
		if(!r->visible()){
			available.erase(r);
		}
	}

	_kicker.assign(available);
}

bool Gameplay::Plays::TestBasicAttack::run()
{
	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::Kick::Done)
		_kicker.restart();

	// run the kick play
	_kicker.run();
	return true;
}
