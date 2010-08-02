#include "Stopped.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::Stopped, "Restarts")

Gameplay::Plays::Stopped::Stopped(GameplayModule *gameplay):
	Play(gameplay),
	_idle(gameplay),
	_left(gameplay, Behaviors::Fullback::Left),
	_right(gameplay, Behaviors::Fullback::Right)
{
	_left.otherFullbacks.insert(&_right);
	_right.otherFullbacks.insert(&_left);
}

bool Gameplay::Plays::Stopped::applicable(const std::set<Robot *> &robots)
{
	return _gameplay->state()->gameState.stopped();
}

bool Gameplay::Plays::Stopped::assign(set<Robot *> &available)
{
	_robots = available;
	
	_left.assign(available);
	_right.assign(available);
	_idle.assign(available);

	return true;
}

bool Gameplay::Plays::Stopped::run()
{
	_idle.run();
	_left.run();
	_right.run();
	
	return true;
}
