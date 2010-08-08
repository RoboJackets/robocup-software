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
	set<Robot *> available = gameplay->robots();
	_robots = available;
	_center.target = _gameplay->centerMatrix() * Geometry2d::Point(0, 1.5);
	
	_pdt.assign(available);
	_center.assign(available);
	_fullback1.assign(available);
	_fullback2.assign(available);
}

float Gameplay::Plays::OurFreekick::score ( Gameplay::GameplayModule* gameplay )
{
	bool ok = (gameplay->state()->gameState.setupRestart() && gameplay->state()->gameState.ourFreeKick());
	return ok ? 0 : INFINITY;
}

bool Gameplay::Plays::OurFreekick::run()
{
	_kicker.aimType(Behaviors::Kick::ONETOUCH);
	_kicker.setVScale(0.3, 0.2); // drive slowly until close to ball
	_pdt.run();
	_center.run();
	_fullback1.run();
	_fullback2.run();
	
	return _pdt.keepRunning();
}
