#include "FollowTheLeader.hpp"

#include <algorithm>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::FollowTheLeader, "Demos")

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::FollowTheLeader::FollowTheLeader(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::FollowTheLeader::run()
{
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
	{
		// No robots
		return false;
	}
	/*
	float backoff = 0.1;
	
	// Make a list of robots, sorted by shell
	vector<OurRobot *> robots(playRobots.size());
	copy(playRobots.begin(), playRobots.end(), robots.begin());
	sort(robots.begin(), robots.end(), shellLessThan);
	
	// Put the manual robot in front
	int manualID = _gameplay->manualID();
	for (unsigned int i = 0; i < robots.size(); ++i)
	{
		if ((int)robots[i]->shell() == manualID)
		{
			swap(robots[i], robots[0]);
			break;
		}
	}

	if (manualID < 0)
	{
		// No manual robot: first robot follows the ball
		Geometry2d::Point ballPos = ball().pos;
		Geometry2d::Point cur = robots[0]->pos;
		Geometry2d::Point dest = ballPos - (ballPos - cur).normalized() * (Robot_Radius + Ball_Radius + backoff);
		robots[0]->move(dest);
		robots[0]->face(ballPos);
	}
	
	// robot[0] is manual.
	// robot[i] follows robot[i-1].
	Geometry2d::Point ballPos = ball().pos;
	Geometry2d::Point RZeroPos = robots[0]->pos;
	for (unsigned int i = 1; i < robots.size(); ++i)
	{	Geometry2d::Point facingTheBall = ballPos - RZeroPos;

		Geometry2d::Point temp = facingTheBall.perpCW().normalized();
		Geometry2d::Point leader = robots[i - 1]->pos;
		Geometry2d::Point cur = robots[i]->pos;
		// Stay a small distance behind the leader
		Geometry2d::Point dest = RZeroPos + temp * (Robot_Diameter + i* backoff);
		robots[i]->move(dest);
		robots[i]->face(ballPos);
	}*/
	
	return true;
}
