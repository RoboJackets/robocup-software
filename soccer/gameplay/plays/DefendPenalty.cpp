#include "DefendPenalty.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DefendPenalty, "Restarts")

Gameplay::Plays::DefendPenalty::DefendPenalty(GameplayModule *gameplay):
	Play(gameplay, 1),
	_idle1(gameplay),
	_idle2(gameplay),
	_idle3(gameplay),
	_idle4(gameplay)
{
}

bool Gameplay::Plays::DefendPenalty::applicable()
{
	return gameState().setupRestart() && gameState().theirPenalty();
}

bool Gameplay::Plays::DefendPenalty::assign(set<Robot *> &available)
{
	_robots = available;
	
	_idle1.target = Geometry2d::Point(1.5, 1);
	_idle2.target = Geometry2d::Point(1.5, 1.5);
	_idle3.target = Geometry2d::Point(1.5, 2);
	_idle4.target = Geometry2d::Point(1.5, 2.5);
	
	_idle1.assign(available);
	_idle2.assign(available);
	_idle3.assign(available);
	_idle4.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::DefendPenalty::run()
{
	_idle1.run();
	_idle2.run();
	_idle3.run();
	_idle4.run();
	
	return true;
}
