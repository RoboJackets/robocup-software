
#include "RunLaps.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::RunLaps, "Demo")

Gameplay::Plays::RunLaps::RunLaps(GameplayModule *gameplay):
	Play(gameplay)
{
}

float Gameplay::Plays::RunLaps::score(GameplayModule *gameplay)
{
	return 0;
}

bool Gameplay::Plays::RunLaps::run()
{
	set<OurRobot *> available = _gameplay->playRobots();

	//	target point
	float fudgeFactor = .15;
	float maxX = Field_Width / 2.0 - Robot_Radius - fudgeFactor;
	float x = _targetingLeft ? -maxX : maxX;
	float y = 0.5;
	Point target(x, y);


	OurRobot *robot;
	assignNearest(robot, available, startPt);

	if (!robot) return false;


	






	return true;
}
