#include "ZoneOffense.hpp"

using namespace std;

Gameplay::Behaviors::ZoneOffense::ZoneOffense(GameplayModule *gameplay)
: Behavior(gameplay, 2)
{
	_leftAttack = 0;
	_rightAttack = 0;
	_midfielder = 0;
}

bool Gameplay::Behaviors::ZoneOffense::assign(std::set<Robot *> &available) {
	takeAll(available);
//	return _robots.size() >= _minRobots;
	return true;
}

bool Gameplay::Behaviors::ZoneOffense::run() {
	return true;
}
