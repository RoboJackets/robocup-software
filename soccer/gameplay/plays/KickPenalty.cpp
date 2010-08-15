#include "KickPenalty.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::KickPenalty, "Restarts")

Gameplay::Plays::KickPenalty::KickPenalty(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_idle1(gameplay),
	_idle2(gameplay),
	_idle3(gameplay)
{
	set<Robot *> available = gameplay->robots();
	
	_idle1.target = Geometry2d::Point(1.5, 1);
	_idle2.target = Geometry2d::Point(1.5, 1.5);
	_idle3.target = Geometry2d::Point(1.5, 2);
	
	_kicker.assign(available);
	_idle1.assign(available);
	_idle2.assign(available);
	_idle3.assign(available);
}

float Gameplay::Plays::KickPenalty::score ( Gameplay::GameplayModule* gameplay )
{
	return (gameplay->state()->gameState.setupRestart() && gameplay->state()->gameState.ourPenalty()) ? 0 : INFINITY;
}

bool Gameplay::Plays::KickPenalty::run()
{
	_kicker.run();
	_idle1.run();
	_idle2.run();
	_idle3.run();
	
	return true;
}
