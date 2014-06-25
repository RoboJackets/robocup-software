#include "DynamicNavigationTC.hpp"

#include <algorithm>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DynamicNavigationTC, "Demos")

namespace Gameplay
{
	namespace Plays
	{
	REGISTER_CONFIGURABLE(DynamicNavigationTC)
	}
}

ConfigDouble *Gameplay::Plays::DynamicNavigationTC::_opp_avoid_radius;
ConfigDouble *Gameplay::Plays::DynamicNavigationTC::_goal_X;
ConfigDouble *Gameplay::Plays::DynamicNavigationTC::_goal_Y;
ConfigDouble *Gameplay::Plays::DynamicNavigationTC::_center_X;

void Gameplay::Plays::DynamicNavigationTC::createConfiguration(Configuration *cfg)
{
	_opp_avoid_radius = new ConfigDouble(cfg, "DynamicNavigationTC/Opponent Avoid Radius", 0.5);
	_goal_X = new ConfigDouble(cfg, "DynamicNavigationTC/Goal X", 0.70);
	_goal_Y = new ConfigDouble(cfg, "DynamicNavigationTC/Goal Y", 0.70);
	_center_X = new ConfigDouble(cfg, "DynamicNavigationTC/Center X", 0.15);
}

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::DynamicNavigationTC::DynamicNavigationTC(GameplayModule *gameplay):
			Play(gameplay)
{
	{
		// Center obstacle
		PolygonObstacle *c = new PolygonObstacle;
		c->polygon.vertices.push_back(Point(*_center_X, 5.3));
		c->polygon.vertices.push_back(Point(-*_center_X, 5.3));
		c->polygon.vertices.push_back(Point(-*_center_X, 0.75));
		c->polygon.vertices.push_back(Point(*_center_X, 0.75));
		centerObstacle[0] = std::shared_ptr<Obstacle>(c);

		c = new PolygonObstacle;
		c->polygon.vertices.push_back(Point(*_center_X / 4, 5.3));
		c->polygon.vertices.push_back(Point(-*_center_X / 4, 5.3));
		c->polygon.vertices.push_back(Point(-*_center_X / 4, 0.75));
		c->polygon.vertices.push_back(Point(*_center_X / 4, 0.75));
		centerObstacle[1] = std::shared_ptr<Obstacle>(c);
	}

	{
		float width = Field_GoalWidth/2.0f;
		float depth = Field_GoalDepth;
		// goal obstacles
		PolygonObstacle *c = new PolygonObstacle;
		c->polygon.vertices.push_back(Point(-width, 0));
		c->polygon.vertices.push_back(Point(-width, -depth));
		c->polygon.vertices.push_back(Point(width, -depth));
		c->polygon.vertices.push_back(Point(width, 0));
		goalObstacle[0] = std::shared_ptr<Obstacle>(c);

		c = new PolygonObstacle;
		c->polygon.vertices.push_back(Point(-width, Field_Length));
		c->polygon.vertices.push_back(Point(-width, Field_Length+depth));
		c->polygon.vertices.push_back(Point(width, Field_Length+depth));
		c->polygon.vertices.push_back(Point(width, Field_Length));
		goalObstacle[1] = std::shared_ptr<Obstacle>(c);
	}

	{
		// Quadrant obstacles
		//
		// Looking at the field view:
		//   0 3
		//   1 2
		const float x = Field_Width / 2 + Field_Border;
		const float y1 = Field_Length / 2;
		const float y2 = Field_Length + Field_Border;

		PolygonObstacle *q = new PolygonObstacle;
		q->polygon.vertices.push_back(Point(x, y1));
		q->polygon.vertices.push_back(Point(0, y1));
		q->polygon.vertices.push_back(Point(0, y2));
		q->polygon.vertices.push_back(Point(x, y2));
		quadrantObstacle[0] = std::shared_ptr<Obstacle>(q);

		q = new PolygonObstacle;
		q->polygon.vertices.push_back(Point(0, y1));
		q->polygon.vertices.push_back(Point(-x, y1));
		q->polygon.vertices.push_back(Point(-x, y2));
		q->polygon.vertices.push_back(Point(0, y2));
		quadrantObstacle[1] = std::shared_ptr<Obstacle>(q);

		q = new PolygonObstacle;
		q->polygon.vertices.push_back(Point(0, 0));
		q->polygon.vertices.push_back(Point(-x, 0));
		q->polygon.vertices.push_back(Point(-x, y1));
		q->polygon.vertices.push_back(Point(0, y1));
		quadrantObstacle[2] = std::shared_ptr<Obstacle>(q);

		q = new PolygonObstacle;
		q->polygon.vertices.push_back(Point(x, 0));
		q->polygon.vertices.push_back(Point(0, 0));
		q->polygon.vertices.push_back(Point(0, y1));
		q->polygon.vertices.push_back(Point(x, y1));
		quadrantObstacle[3] = std::shared_ptr<Obstacle>(q);
	}

}

bool Gameplay::Plays::DynamicNavigationTC::run()
{
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
	{
		// No robots
		return false;
	}

	float backoff = 0.1;

	// Make a list of robots, sorted by shell
	vector<OurRobot *> robots(playRobots.size());
	copy(playRobots.begin(), playRobots.end(), robots.begin());
	sort(robots.begin(), robots.end(), shellLessThan);

	// first robot is the motion leader
	Geometry2d::Point ballPos = ball().pos;
	Geometry2d::Point cur = robots[0]->pos;
	Geometry2d::Point dest = ballPos - (ballPos - cur).normalized() * (Robot_Radius + Ball_Radius + backoff);
	robots[0]->move(dest);
	robots[0]->face(ballPos);

	OurRobot * leader = robots[0];

	// increase the size of opponent obstacles
	for (unsigned int i = 0; i < robots.size(); ++i)
	{
		robots[i]->avoidAllOpponentRadius(*_opp_avoid_radius);
	}

	leader->localObstacles(centerObstacle[0]);
	leader->localObstacles(centerObstacle[1]);

	leader->localObstacles(goalObstacle[0]);
	leader->localObstacles(goalObstacle[1]);

	Point pos = leader->pos;
	if (pos.x >= 0)
	{
		if (pos.y > (Field_Length / 2))
		{
			// In quadrant 0
			leader->localObstacles(quadrantObstacle[1]);
			leader->move(Point(- *_goal_X, Field_Length - *_goal_Y));
		} else {
			// In quadrant 3
			leader->localObstacles(quadrantObstacle[0]);
			leader->move(Point(*_goal_X, Field_Length - *_goal_Y));
		}
	} else {
		if (pos.y > (Field_Length / 2))
		{
			// In quadrant 1
			leader->localObstacles(quadrantObstacle[2]);
			leader->move(Point(- *_goal_X, *_goal_Y));
		} else {
			// In quadrant 2
			leader->localObstacles(quadrantObstacle[3]);
			leader->move(Point(*_goal_X, *_goal_Y));
		}
	}

	// robot[0] is manual.
	// robot[i] follows robot[i-1].
	for (unsigned int i = 1; i < robots.size(); ++i)
	{
		Geometry2d::Point leader = robots[i - 1]->pos;
		Geometry2d::Point cur = robots[i]->pos;
		// Stay a small distance behind the leader
		Geometry2d::Point dest = leader - (leader - cur).normalized() * (Robot_Diameter + backoff);
		robots[i]->move(dest);
		robots[i]->face(leader);
	}

	return true;
}
