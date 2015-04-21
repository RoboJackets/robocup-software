#include "TheirKickoff.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TheirKickoff, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(TheirKickoff)
	}
}

void Gameplay::Plays::TheirKickoff::createConfiguration(Configuration *cfg)
{

}

Gameplay::Plays::TheirKickoff::TheirKickoff(GameplayModule *gameplay):
	Play(gameplay),
	_defender1(gameplay, Behaviors::Defender::Left),
	_defender2(gameplay, Behaviors::Defender::Right),
	_idle(gameplay)
{
	_defender1.otherDefenders.insert(&_defender2);
	_defender2.otherDefenders.insert(&_defender1);
}

float Gameplay::Plays::TheirKickoff::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.theirKickoff()) ? 0 : INFINITY;
}

bool Gameplay::Plays::TheirKickoff::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_defender1.robot, available, Geometry2d::Point());
	assignNearest(_defender2.robot, available, Geometry2d::Point());
	_idle.robots = available;
	
	_defender1.run();
	_defender2.run();
	_idle.run();
	
	return true;
}
