#include "TestPlay.hpp"

#include <algorithm>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestPlay, "Demos")

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::TestPlay::TestPlay(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::TestPlay::run()
{
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
	{
		// No robots
		return false;
	}
	

	float backoff = 0.0;
	
	// Make a list of robots, sorted by shell
	vector<OurRobot *> robots(playRobots.size());
	copy(playRobots.begin(), playRobots.end(), robots.begin());
	sort(robots.begin(), robots.end(), shellLessThan);
	
	SystemState *sState;
	 sState = state();
	
	// No manual robot: first robot follows the ball
	Geometry2d::Point ballPos = ball().pos;
	Geometry2d::Point cur0 = robots[0]->pos;
	Geometry2d::Point dest0 = ballPos - (ballPos - cur0).normalized() * (Robot_Radius + Ball_Radius + backoff);

	robots[0]->move(dest0);
	robots[0]->face(ballPos);
	
	
	unsigned int counter = 0;

	if(robots.size() <= 4)
	{
		counter = robots.size();
	}
	else
	{
		counter = 4;
	}
	for (unsigned int i = 1; i < counter; ++i)
	{
		Geometry2d::Point leader = robots[i - 1]->pos;
		Geometry2d::Point dest = ballPos - (ballPos - leader).perpCW().normalized() * (Robot_Diameter + Ball_Radius + backoff);
		robots[i]->move(dest);
		robots[i]->face(ballPos);
	}
	
	return true;
}
