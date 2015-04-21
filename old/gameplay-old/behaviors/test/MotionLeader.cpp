#include "MotionLeader.hpp"

using namespace Geometry2d;

Gameplay::Behaviors::MotionLeader::MotionLeader(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
    {
        // Center obstacle
        PolygonObstacle *c = new PolygonObstacle;
        c->polygon.vertices.push_back(Point(Center_X, 5.3));
        c->polygon.vertices.push_back(Point(-Center_X, 5.3));
        c->polygon.vertices.push_back(Point(-Center_X, 0.75));
        c->polygon.vertices.push_back(Point(Center_X, 0.75));
        centerObstacle[0] = std::shared_ptr<Obstacle>(c);

        c = new PolygonObstacle;
        c->polygon.vertices.push_back(Point(Center_X / 4, 5.3));
        c->polygon.vertices.push_back(Point(-Center_X / 4, 5.3));
        c->polygon.vertices.push_back(Point(-Center_X / 4, 0.75));
        c->polygon.vertices.push_back(Point(Center_X / 4, 0.75));
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

bool Gameplay::Behaviors::MotionLeader::run()
{
#if 0
    for (int i = 0; i < Robots_Per_Team; ++i)
    {
        gameplay()->state()->obstacles[i].add(centerObstacle[0]);
        gameplay()->state()->obstacles[i].add(centerObstacle[1]);
    }
#else
    robot->localObstacles(centerObstacle[0]);
    robot->localObstacles(centerObstacle[1]);

    robot->localObstacles(goalObstacle[0]);
    robot->localObstacles(goalObstacle[1]);
#endif

    Point pos = robot->pos;
    if (pos.x >= 0)
    {
        if (pos.y > (Field_Length / 2))
        {
            // In quadrant 0
            robot->localObstacles(quadrantObstacle[1]);
            robot->move(Point(-Goal_X, Field_Length - Goal_Y));
        } else {
            // In quadrant 3
            robot->localObstacles(quadrantObstacle[0]);
            robot->move(Point(Goal_X, Field_Length - Goal_Y));
        }
    } else {
        if (pos.y > (Field_Length / 2))
        {
            // In quadrant 1
            robot->localObstacles(quadrantObstacle[2]);
            robot->move(Point(-Goal_X, Goal_Y));
        } else {
            // In quadrant 2
            robot->localObstacles(quadrantObstacle[3]);
            robot->move(Point(Goal_X, Goal_Y));
        }
    }
    
    return true;
}
