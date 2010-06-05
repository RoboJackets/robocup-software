#include "Chip.hpp"

using namespace std;

Gameplay::Behaviors::Chip::Chip(GameplayModule *gameplay):
    Behavior(gameplay)
{
}

bool Gameplay::Behaviors::Chip::assign(set<Robot *> &available)
{
	_robots.clear(); // clear existing robots
	takeBest(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Behaviors::Chip::run()
{
	// Dummy implementation - just turns on the chip flag
	uint8_t strength = 255;
	robot()->chip(strength);

	return true;
}

float Gameplay::Behaviors::Chip::score(Robot* robot)
{
	return 0.0;
}
