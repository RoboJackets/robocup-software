#include "TheirKickoff.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TheirKickoff, "Restarts")

Gameplay::Plays::TheirKickoff::TheirKickoff(GameplayModule *gameplay):
	Play(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_idle(gameplay)
{
	_fullback1.otherFullbacks.insert(&_fullback2);
	_fullback2.otherFullbacks.insert(&_fullback1);
}

float Gameplay::Plays::TheirKickoff::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.theirKickoff()) ? 0 : INFINITY;
}

bool Gameplay::Plays::TheirKickoff::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_fullback1.robot, available, Geometry2d::Point());
	assignNearest(_fullback2.robot, available, Geometry2d::Point());
	_idle.robots = available;
	
	_fullback1.run();
	_fullback2.run();
	_idle.run();
	
	return true;
}
