#include "Follow.hpp"

#include <algorithm>
#include <stdio.h>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::Follow, "Tests")

static bool shellLessThan(Gameplay::Robot *r1, Gameplay::Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::Follow::Follow(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::Follow::run()
{
	if (_gameplay->robots().empty())
	{
		// No robots
		return false;
	}
	
	float backoff = 0.1;
	
	// Make a list of robots, sorted by shell
	vector<Robot *> robots(_gameplay->robots().size());
	copy(_gameplay->robots().begin(), _gameplay->robots().end(), robots.begin());
	sort(robots.begin(), robots.end(), shellLessThan);
	
	// Put the manual robot in front
	int manualID = _gameplay->manualID();
	for (unsigned int i = 0; i < robots.size(); ++i)
	{
		if (robots[i]->shell() == manualID)
		{
			swap(robots[i], robots[0]);
			break;
		}
	}
	
	if (manualID < 0)
	{
		// No manual robot: first robot follows the ball
		Geometry2d::Point ballPos = ball().pos;
		Geometry2d::Point cur = robots[0]->pos();
		Geometry2d::Point dest = ballPos - (ballPos - cur).normalized() * (Constants::Robot::Radius + Constants::Ball::Radius + backoff);
		robots[0]->move(dest);
	}
	
	// robot[0] is manual.
	// robot[i] follows robot[i-1].
	for (unsigned int i = 1; i < robots.size(); ++i)
	{
		Geometry2d::Point leader = robots[i - 1]->pos();
		Geometry2d::Point cur = robots[i]->pos();
		// Stay a small distance behind the leader
		Geometry2d::Point dest = leader - (leader - cur).normalized() * (Constants::Robot::Diameter + backoff);
		robots[i]->move(dest);
	}
	
	return true;
}
