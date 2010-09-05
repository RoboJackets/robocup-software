#include "OurFreekick.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurFreekick, "Restarts")

Gameplay::Plays::OurFreekick::OurFreekick(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_center(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Left),
	_pdt(gameplay, &_kicker)
{
	_center.target = _gameplay->centerMatrix() * Geometry2d::Point(0, 1.5);
}

float Gameplay::Plays::OurFreekick::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.ourFreeKick()) ? 0 : INFINITY;
}

bool Gameplay::Plays::OurFreekick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_kicker.robot, available, ball().pos);
	_pdt.backoff.robots.clear();
	_pdt.backoff.robots.insert(_kicker.robot);
	assignNearest(_center.robot, available, _center.target);
	assignNearest(_fullback1.robot, available, Geometry2d::Point());
	assignNearest(_fullback2.robot, available, Geometry2d::Point());
	
// 	_kicker.aimType(Behaviors::Kick::ONETOUCH);
// 	_kicker.setVScale(0.3, 0.2); // drive slowly until close to ball
	_pdt.run();
	_center.run();
	_fullback1.run();
	_fullback2.run();
	
	return _pdt.keepRunning();
}
