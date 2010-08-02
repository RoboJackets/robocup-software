#include "DemoBasicOneTouchChipAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoBasicOneTouchChipAttack, "Demos")

Gameplay::Plays::DemoBasicOneTouchChipAttack::DemoBasicOneTouchChipAttack(GameplayModule *gameplay):
	Play(gameplay, 1), _kicker(gameplay)
{
}

bool Gameplay::Plays::DemoBasicOneTouchChipAttack::assign(set<Robot *> &available)
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

bool Gameplay::Plays::DemoBasicOneTouchChipAttack::run()
{
	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::OneTouchKick::Done)
		_kicker.restart();

	// run the kick play
	_kicker.kickType(Behaviors::OneTouchKick::CHIP);
	_kicker.run();
	return true;
}
