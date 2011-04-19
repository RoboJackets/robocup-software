#include <framework/Obstacle.hpp>
#include <Constants.hpp>

#include <boost/foreach.hpp>

using namespace boost;

Obstacle::Obstacle()
{
}

Obstacle::~Obstacle()
{
}

////////

ObstacleGroup::~ObstacleGroup()
{
    clear();
}

void ObstacleGroup::clear()
{
    _obstacles.clear();
}

void ObstacleGroup::add(ObstaclePtr obs)
{
    _obstacles.push_back(obs);
}

void ObstacleGroup::add(const ObstacleGroup& group)
{
	_obstacles.insert(_obstacles.end(), group._obstacles.begin(), group._obstacles.end());
}

////////

CircleObstacle::CircleObstacle(Geometry2d::Point center, float radius):
    circle(center, radius)
{
}

bool CircleObstacle::hit(const Geometry2d::Point &pt)
{
    return pt.nearPoint(circle.center, circle.radius() + Robot_Radius);
}

bool CircleObstacle::hit(const Geometry2d::Segment &seg)
{
    return seg.nearPoint(circle.center, circle.radius() + Robot_Radius);
}

////////

bool PolygonObstacle::hit(const Geometry2d::Point &pt)
{
    return polygon.nearPoint(pt, Robot_Radius);
}

bool PolygonObstacle::hit(const Geometry2d::Segment &seg)
{
    return polygon.nearSegment(seg, Robot_Radius);
}
