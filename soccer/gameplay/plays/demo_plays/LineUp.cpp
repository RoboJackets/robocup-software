#include "LineUp.hpp"

#include <algorithm>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::LineUp, "Demos")

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::LineUp::LineUp(GameplayModule* gameplay):
	Play(gameplay)
{
}

float Gameplay::Plays::LineUp::score(GameplayModule* gameplay)
{
	return 0;
}

bool Gameplay::Plays::LineUp::run()
{
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
	{
		// No robots
		return false;
	}
	
	// Make a list of robots, sorted by shell
	vector<OurRobot *> robots(playRobots.size());
	copy(playRobots.begin(), playRobots.end(), robots.begin());
	sort(robots.begin(), robots.end(), shellLessThan);
	
	for (unsigned int i = 0; i < robots.size(); ++i)
	{
		Geometry2d::Point dest = Geometry2d::Point(-1.9, 0.2 + i * 0.32);
		robots[i]->move(dest);
		robots[i]->face(dest + Geometry2d::Point(0, 1));
	}
	
	return true;
}
