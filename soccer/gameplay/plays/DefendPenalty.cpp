#include "DefendPenalty.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DefendPenalty, "Restarts")

Gameplay::Plays::DefendPenalty::DefendPenalty(GameplayModule *gameplay):
	Play(gameplay),
	_idle1(gameplay),
	_idle2(gameplay),
	_idle3(gameplay),
	_idle4(gameplay),
	_idle5(gameplay)
{
	_idle1.target = Geometry2d::Point(1.5, 1.3);
	_idle2.target = Geometry2d::Point(1.5, 1.6);
	_idle3.target = Geometry2d::Point(1.5, 1.9);
	_idle4.target = Geometry2d::Point(1.5, 2.2);
	_idle5.target = Geometry2d::Point(1.5, 2.5);
}

float Gameplay::Plays::DefendPenalty::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.theirPenalty()) ? 0 : INFINITY;
}

bool Gameplay::Plays::DefendPenalty::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_idle1.robot, available, _idle1.target);
	assignNearest(_idle2.robot, available, _idle2.target);
	assignNearest(_idle3.robot, available, _idle3.target);
	assignNearest(_idle4.robot, available, _idle4.target);
	assignNearest(_idle5.robot, available, _idle5.target);
	
	_idle1.run();
	_idle2.run();
	_idle3.run();
	_idle4.run();
	_idle5.run();
	
	return true;
}
