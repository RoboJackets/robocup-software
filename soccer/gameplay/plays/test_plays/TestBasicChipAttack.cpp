#include "TestBasicChipAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestBasicChipAttack, "Tests")

Gameplay::Plays::TestBasicChipAttack::TestBasicChipAttack(GameplayModule *gameplay):
	Play(gameplay, 1), _kicker(gameplay)
{
}

bool Gameplay::Plays::TestBasicChipAttack::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

bool Gameplay::Plays::TestBasicChipAttack::assign(set<Robot *> &available)
{
	// remove non-visible robots
	// this prevents bug with 2-robot tests where play would do nothing
	BOOST_FOREACH(Robot *r, available){
		if(!r->visible()){
			available.erase(r);
		}
	}

	_kicker.assign(available);
	return _robots.size() >= _minRobots && robot()->hasChipper();
}

bool Gameplay::Plays::TestBasicChipAttack::run()
{
	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::Kick::Done)
		_kicker.restart();

	// run the kick play
	_kicker.kickType(Behaviors::Kick::CHIP);
	_kicker.run();
	return true;
}
