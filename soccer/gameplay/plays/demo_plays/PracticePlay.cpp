/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#include "PracticePlay.hpp"

#include <algorithm>
#include <stdio.h>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::PracticePlay, "Demos")

Gameplay::Plays::PracticePlay::PracticePlay(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
}

float Gameplay::Plays::PracticePlay::score(GameplayModule *gameplay)
{
	return 0;
}

bool Gameplay::Plays::PracticePlay::run()
{
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
		{
			// No robots
			return false;
		}
	vector<OurRobot *> robot(playRobots.size());
	copy(playRobots.begin(),playRobots.end(),robot.begin());_kicker.restart();

	int closestI = 0, currentDist = 1000;
		Geometry2d::Point ballPos = ball().pos;
		for (unsigned int i = 0; i < robot.size(); ++i) {
			Geometry2d::Point robotPos = robot[i]->pos;
			Geometry2d::Point vec = robotPos-ballPos;
			int dist = vec.x*vec.x+vec.y*vec.y;
			if (dist < currentDist) {
				currentDist = dist;
				closestI = i;
			}
		}

	OurRobot * closest= (robot[closestI]);
	_kicker.robot = closest;
	bool in = _kicker.run();
	if (!in)
		_kicker.restart();
	return true;
}

