#include "Stopped.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::Stopped, "Restarts")

Gameplay::Plays::Stopped::Stopped(GameplayModule *gameplay):
	Play(gameplay),
	_idle(gameplay),
	_left(gameplay, Behaviors::Fullback::Left),
	_right(gameplay, Behaviors::Fullback::Right)
{
	set<Robot *> available = _gameplay->robots();
	
	_left.otherFullbacks.insert(&_right);
	_right.otherFullbacks.insert(&_left);
	
	_robots = available;
	
	_left.assign(available);
	_right.assign(available);
	_idle.assign(available);
}

float Gameplay::Plays::Stopped::score ( Gameplay::GameplayModule* gameplay )
{
	return gameplay->state()->gameState.stopped() ? 0 : INFINITY;
}

bool Gameplay::Plays::Stopped::run()
{
	_idle.run();
	_left.run();
	_right.run();
	
	return true;
}
