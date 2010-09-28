/**
 *  Example play: This play is causes the closest robot to the ball
 *  to approach and kick it. Its logic isn't perfect. It generally
 *  works when the robot approaches the ball from the left.
 */

#include "KickWithClosestPlay.hpp"

#include <algorithm>
#include <stdio.h>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::KickWithClosestPlay, "Demos")

Gameplay::Plays::KickWithClosestPlay::KickWithClosestPlay(GameplayModule *gameplay):
	Play(gameplay)
{
	reached = false;
}

float Gameplay::Plays::KickWithClosestPlay::score(GameplayModule *gameplay)
{
	return 0;
}

bool Gameplay::Plays::KickWithClosestPlay::run()
{
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
	{
		// No robots
		return false;
	}
	vector<OurRobot *> robot(playRobots.size());
	copy(playRobots.begin(),playRobots.end(),robot.begin());

	//Finds the closest robot
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
	OurRobot * closest= robot[closestI];

	//Finds the point to which to move the robot to kick it in the goal
	Geometry2d::Point target(0, Field_Length);
	Geometry2d::Point vec = ballPos-target;
	Geometry2d::Point moveto = ballPos+(ballPos-target).normalized()*(Robot_Radius+Ball_Radius);
	state()->drawLine(moveto,target);

	//Moves the robot to the point to kick.
	closest->face(target);
	if (!reached) {
		closest->move(moveto);
		closest->addText("Approach");
	} else  {
		closest->addText("Kick");
		if (ball().pos.y >= Field_Length)
			reached =false;
		closest->kick(255);
		closest->move(ballPos);

	}
	//Checks if the ball has reached moveTo
	float err = Robot_Radius * 1.5;
	if (ball().pos.x+err >= closest->pos.x && ball().pos.x-err <= closest->pos.x)
		if (ball().pos.y+(moveto.y-ballPos.y) >= closest->pos.y && ball().pos.y-err <= closest->pos.y)
			reached = true;

	//Checks if the ball has been kicked based on its velocity
	Geometry2d::Point ballVel = ball().vel;
	float vel = ballVel.x*ballVel.x + ballVel.y*ballVel.y;
	if (vel > 9)
		reached = false;
	return true;
}

