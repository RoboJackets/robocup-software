#include "DemoBasicOneTouchAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;
using namespace Gameplay::Behaviors;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoBasicOneTouchAttack, "Demos")

Gameplay::Plays::DemoBasicOneTouchAttack::DemoBasicOneTouchAttack(GameplayModule *gameplay):
	Play(gameplay, 1), _kicker(gameplay)
{
}

bool Gameplay::Plays::DemoBasicOneTouchAttack::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	return refApplicable;
}

bool Gameplay::Plays::DemoBasicOneTouchAttack::assign(set<Robot *> &available)
{
	// remove non-visible robots
	// this prevents bug with 2-robot tests where play would do nothing
	BOOST_FOREACH(Robot *r, available){
		if(!r->visible()){
			available.erase(r);
		}
	}

	_kicker.assign(available);
	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::DemoBasicOneTouchAttack::run()
{
	// set the aiming type
	_kicker.aimType(Kick::ONETOUCH);

	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::Kick::Done)
		_kicker.restart();

	// run the kick play
	_kicker.run();
	return true;
}
