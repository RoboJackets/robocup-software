#include "KickPenalty.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::KickPenalty, "Restarts")

Gameplay::Plays::KickPenalty::KickPenalty(GameplayModule *gameplay):
	Play(gameplay, 1),
	_kicker(gameplay),
	_idle1(gameplay),
	_idle2(gameplay),
	_idle3(gameplay)
{
}

bool Gameplay::Plays::KickPenalty::applicable()
{
	return gameState().setupRestart() && gameState().ourPenalty();
}

bool Gameplay::Plays::KickPenalty::assign(set<Robot *> &available)
{
	_idle1.target = Geometry2d::Point(1.5, 1);
	_idle2.target = Geometry2d::Point(1.5, 1.5);
	_idle3.target = Geometry2d::Point(1.5, 2);
	
	_kicker.assign(available);
	_idle1.assign(available);
	_idle2.assign(available);
	_idle3.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::KickPenalty::run()
{
	_kicker.run();
	_idle1.run();
	_idle2.run();
	_idle3.run();
	
	return true;
}
