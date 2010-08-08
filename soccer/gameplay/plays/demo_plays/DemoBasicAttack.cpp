#include "DemoBasicAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoBasicAttack, "Demos")

Gameplay::Plays::DemoBasicAttack::DemoBasicAttack(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
	set<Robot *> available = gameplay->robots();
	// remove non-visible robots
	// this prevents bug with 2-robot tests where play would do nothing
	BOOST_FOREACH(Robot *r, available){
		if(!r->visible()){
			available.erase(r);
		}
	}

	_kicker.assign(available);
}

bool Gameplay::Plays::DemoBasicAttack::run()
{
	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::Kick::Done)
		_kicker.restart();

	// run the kick play
	_kicker.run();
	return true;
}
