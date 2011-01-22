#include "MightyMight.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::MightyMight, "Playing")

Gameplay::Plays::MightyMight::MightyMight(GameplayModule *gameplay):
	Play(gameplay),
	_leftFullback(gameplay, Behaviors::Fullback::Left),
	_rightFullback(gameplay, Behaviors::Fullback::Right),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
	_leftFullback.otherFullbacks.insert(&_rightFullback);
	_rightFullback.otherFullbacks.insert(&_leftFullback);
}

float Gameplay::Plays::MightyMight::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::MightyMight::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	// defense first - closest to goal
	assignNearest(_leftFullback.robot, available, Geometry2d::Point());
	assignNearest(_rightFullback.robot, available, Geometry2d::Point());

	// choose offense, we want both robots to attack
	assignNearest(_kicker1.robot, available, ball().pos);
	assignNearest(_kicker2.robot, available, ball().pos);

	// manually reset any kickers so they keep kicking
	if (_kicker1.done())
		_kicker1.restart();
	if (_kicker2.done())
		_kicker2.restart();

	// execute kickers dumbly - FIXME: make this smarter to keep them from colliding
	_kicker1.run();
	_kicker2.run();

	// run standard fullback behavior
	_leftFullback.run();
	_rightFullback.run();
	
	return true;
}
