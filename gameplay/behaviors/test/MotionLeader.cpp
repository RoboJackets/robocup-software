#include "MotionLeader.hpp"
#include "../../Role.hpp"

#include <LogFrame.hpp>

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::MotionLeader> behavior("motion_leader");

static const float Goal_X = 0.75;
static const float Center_X = 0.15;

Gameplay::Behaviors::MotionLeader::MotionLeader(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role)
{
    {
        // Center obstacle
        PolygonObstacle *c = new PolygonObstacle;
        c->polygon.vertices.push_back(Geometry2d::Point(Center_X, 5.3));
        c->polygon.vertices.push_back(Geometry2d::Point(-Center_X, 5.3));
        c->polygon.vertices.push_back(Geometry2d::Point(-Center_X, 0.75));
        c->polygon.vertices.push_back(Geometry2d::Point(Center_X, 0.75));
        centerObstacle[0] = ObstaclePtr(c);

        c = new PolygonObstacle;
        c->polygon.vertices.push_back(Geometry2d::Point(Center_X / 4, 5.3));
        c->polygon.vertices.push_back(Geometry2d::Point(-Center_X / 4, 5.3));
        c->polygon.vertices.push_back(Geometry2d::Point(-Center_X / 4, 0.75));
        c->polygon.vertices.push_back(Geometry2d::Point(Center_X / 4, 0.75));
        centerObstacle[1] = ObstaclePtr(c);
    }

    {
    	float width = Constants::Field::GoalWidth/2.0f;
    	float depth = Constants::Field::GoalDepth;
    	// goal obstacles
    	PolygonObstacle *c = new PolygonObstacle;
		c->polygon.vertices.push_back(Geometry2d::Point(-width, 0));
		c->polygon.vertices.push_back(Geometry2d::Point(-width, -depth));
		c->polygon.vertices.push_back(Geometry2d::Point(width, -depth));
		c->polygon.vertices.push_back(Geometry2d::Point(width, 0));
		goalObstacle[0] = ObstaclePtr(c);

		c = new PolygonObstacle;
		c->polygon.vertices.push_back(Geometry2d::Point(-width, Constants::Field::Length));
		c->polygon.vertices.push_back(Geometry2d::Point(-width, Constants::Field::Length+depth));
		c->polygon.vertices.push_back(Geometry2d::Point(width, Constants::Field::Length+depth));
		c->polygon.vertices.push_back(Geometry2d::Point(width, Constants::Field::Length));
		goalObstacle[1] = ObstaclePtr(c);
    }

    {
        // Quadrant obstacles
        //
        // Looking at the field view:
        //   0 3
        //   1 2
        const float x = Constants::Field::Width / 2 + Constants::Field::Border;
        const float y1 = Constants::Field::Length / 2;
        const float y2 = Constants::Field::Length + Constants::Field::Border;

        PolygonObstacle *q = new PolygonObstacle;
        q->polygon.vertices.push_back(Geometry2d::Point(x, y1));
        q->polygon.vertices.push_back(Geometry2d::Point(0, y1));
        q->polygon.vertices.push_back(Geometry2d::Point(0, y2));
        q->polygon.vertices.push_back(Geometry2d::Point(x, y2));
        quadrantObstacle[0] = ObstaclePtr(q);

        q = new PolygonObstacle;
        q->polygon.vertices.push_back(Geometry2d::Point(0, y1));
        q->polygon.vertices.push_back(Geometry2d::Point(-x, y1));
        q->polygon.vertices.push_back(Geometry2d::Point(-x, y2));
        q->polygon.vertices.push_back(Geometry2d::Point(0, y2));
        quadrantObstacle[1] = ObstaclePtr(q);

        q = new PolygonObstacle;
        q->polygon.vertices.push_back(Geometry2d::Point(0, 0));
        q->polygon.vertices.push_back(Geometry2d::Point(-x, 0));
        q->polygon.vertices.push_back(Geometry2d::Point(-x, y1));
        q->polygon.vertices.push_back(Geometry2d::Point(0, y1));
        quadrantObstacle[2] = ObstaclePtr(q);

        q = new PolygonObstacle;
        q->polygon.vertices.push_back(Geometry2d::Point(x, 0));
        q->polygon.vertices.push_back(Geometry2d::Point(0, 0));
        q->polygon.vertices.push_back(Geometry2d::Point(0, y1));
        q->polygon.vertices.push_back(Geometry2d::Point(x, y1));
        quadrantObstacle[3] = ObstaclePtr(q);
    }
}

void Gameplay::Behaviors::MotionLeader::run()
{
#if 0
    for (int i = 0; i < Constants::Robots_Per_Team; ++i)
    {
        gameplay()->state()->obstacles[i].add(centerObstacle[0]);
        gameplay()->state()->obstacles[i].add(centerObstacle[1]);
    }
#else
    obstacles()->add(centerObstacle[0]);
    obstacles()->add(centerObstacle[1]);

    obstacles()->add(goalObstacle[0]);
    obstacles()->add(goalObstacle[1]);
#endif

    Geometry2d::Point pos = robot()->state()->pos;
    if (pos.x >= 0)
    {
        if (pos.y > (Constants::Field::Length / 2))
        {
            // In quadrant 0
            obstacles()->add(quadrantObstacle[1]);
            robot()->state()->cmd.goalPosition = Geometry2d::Point(Goal_X, Constants::Field::Length / 4);
        } else {
            // In quadrant 3
            obstacles()->add(quadrantObstacle[0]);
            robot()->state()->cmd.goalPosition = Geometry2d::Point(-Goal_X, Constants::Field::Length / 4);
        }
    } else {
        if (pos.y > (Constants::Field::Length / 2))
        {
            // In quadrant 1
            obstacles()->add(quadrantObstacle[2]);
            robot()->state()->cmd.goalPosition = Geometry2d::Point(Goal_X, Constants::Field::Length * 3 / 4);
        } else {
            // In quadrant 2
            obstacles()->add(quadrantObstacle[3]);
            robot()->state()->cmd.goalPosition = Geometry2d::Point(-Goal_X, Constants::Field::Length * 3 / 4);
        }
    }
}
