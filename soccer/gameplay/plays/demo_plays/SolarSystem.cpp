#include "SolarSystem.hpp"

#include <algorithm>

using namespace std;
using namespace Geometry2d;


REGISTER_PLAY_CATEGORY(Gameplay::Plays::SolarSystem, "Demos")

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::SolarSystem::SolarSystem(GameplayModule* gameplay):
	Play(gameplay)
{
}

float Gameplay::Plays::SolarSystem::score(GameplayModule* gameplay)
{
	return 0;
}

bool Gameplay::Plays::SolarSystem::run()
{
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
	{
		// No robots
		return false;
	}
	


	Point center(0, 3);




	// Make a list of robots, sorted by shell
	vector<OurRobot *> robots(playRobots.size());
	copy(playRobots.begin(), playRobots.end(), robots.begin());
	// sort(robots.begin(), robots.end(), shellLessThan);
	

	float radiusIncrement = .35;
	float startRadius = 0;
	const maxRadiusError = .05;


	for (unsigned int i = 0; i < robots.size(); ++i)
	{
		OurRobot *bot = robots[i];

		float radius = startRadius + i * radiusIncrement;
		Point offset = bot->pos - center;
		Point direction = offset.direction();
		Point targetOffset = direction * radius;

		Point targetLocation = center + targetOffset;

		Point errorVector = bot->pos - targetLocation;
		float error = errorVector.mag();


		if ( error < maxRadiusError ) {
			////////


			
			

		} else {
			////////////
		}


		Point dest = Point(-1.9 + i * 0.28, 2);
		robots[i]->move(dest);
		robots[i]->face(dest + Point(0, 1));
	}
	
	return true;
}

