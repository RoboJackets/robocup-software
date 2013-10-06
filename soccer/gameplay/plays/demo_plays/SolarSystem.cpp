#include "SolarSystem.hpp"

#include <algorithm>

using namespace std;
using namespace Geometry2d;


REGISTER_PLAY_CATEGORY(Gameplay::Plays::SolarSystem, "Demos")


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
	

	float radiusIncrement = .35;
	float startRadius = 0;
	const float maxRadiusError = .05;


	for (unsigned int i = 0; i < robots.size(); ++i)
	{
		OurRobot *bot = robots[i];

		if ( i == 0 ) {
			Point faceTarget(1, 0);

			float angle = bot->angle;
			angle -= 15;

			faceTarget.rotate(angle);
			faceTarget += center;

			bot->face(faceTarget);
		}


		float radius = startRadius + i * radiusIncrement;
		Point offset = bot->pos - center;
		Point direction = offset.normalized();
		Point targetOffset = direction * radius;

		Point targetLocation = center + targetOffset;

		Point errorVector = bot->pos - targetLocation;
		float error = errorVector.mag();


		if ( true ) {
			//	alternate direction
			float dTheta = 4;
			if ( i % 2 == 0 ) {
				dTheta *= -1;
				// dTheta = dTheta * -1;
			}

			targetOffset.rotate(Point(0, 0), dTheta);
			targetLocation = targetOffset + center;
			bot->move(targetLocation);
		} else {
			bot->move(targetLocation);
		}


		state()->drawCircle(center, radius, Qt::red, "SolarSystem");

	}
	
	return true;
}

