#include "DemoBasicAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoBasicAttack, "Demos")

Gameplay::Plays::DemoBasicAttack::DemoBasicAttack(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
}

bool Gameplay::Plays::DemoBasicAttack::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_kicker.robot, available, Geometry2d::Point());
	
	// if we have kicked, we want to reset
	if (_kicker.done())
		_kicker.restart();

	_kicker.run();
	
	return true;
}
