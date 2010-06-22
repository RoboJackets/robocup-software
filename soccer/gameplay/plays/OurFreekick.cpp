#include "OurFreekick.hpp"

using namespace std;

Gameplay::Plays::OurFreekick::OurFreekick(GameplayModule *gameplay):
	Play(gameplay, 1),
	_kicker(gameplay),
	_center(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Left),
	_pdt(gameplay, &_kicker)
{
}

bool Gameplay::Plays::OurFreekick::applicable()
{
	return (gameState().setupRestart() && gameState().ourFreeKick()) || _pdt.keepRunning();
}

bool Gameplay::Plays::OurFreekick::assign(set<Robot *> &available)
{
	_robots = available;
	_center.target = _gameplay->centerMatrix() * Geometry2d::Point(0, 1.5);
	
	_pdt.assign(available);
	_center.assign(available);
	_fullback1.assign(available);
	_fullback2.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::OurFreekick::run()
{
	_kicker.aimType(Behaviors::Kick::ONETOUCH);
	_kicker.setVScale(0.3, 0.2); // drive slowly until close to ball
	_pdt.run();
	_center.run();
	_fullback1.run();
	_fullback2.run();
	
	return true;
}
