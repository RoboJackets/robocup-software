#include "DefendPenalty.hpp"

using namespace std;

Gameplay::Plays::DefendPenalty::DefendPenalty(GameplayModule *gameplay):
	Play(gameplay, 4),
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
	_idle1.target = Geometry2d::Point(1.5, 1);
	_idle2.target = Geometry2d::Point(1.5, 1.5);
	_idle3.target = Geometry2d::Point(1.5, 2);
	_idle4.target = Geometry2d::Point(1.5, 2.5);
	
	if(!_idle1.assign(available)){return false;};
	if(!_idle2.assign(available)){return false;};
	if(!_idle3.assign(available)){return false;};
	if(!_idle4.assign(available)){return false;};

	_robots.insert(_idle1.robot());
	_robots.insert(_idle2.robot());
	_robots.insert(_idle3.robot());
	_robots.insert(_idle4.robot());

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
