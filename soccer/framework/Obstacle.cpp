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
	if (obs)
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

bool CircleObstacle::hit(const Geometry2d::Point &pt) const
{
    return pt.nearPoint(circle.center, circle.radius() + Robot_Radius);
}

bool CircleObstacle::hit(const Geometry2d::Segment &seg) const
{
    return seg.nearPoint(circle.center, circle.radius() + Robot_Radius);
}

Geometry2d::Point CircleObstacle::closestEscape(const Geometry2d::Point& pose) const
{
	// no change if this is close
	if (!hit(pose))
		return pose;

	// if exactly at center, return point closest to origin
	if (circle.center.nearPoint(pose, 0.01))
		return pose - pose.normalized() * circle.radius();

	// find closest point on circle
	Geometry2d::Point d = pose - circle.center;
	return circle.center + d.normalized() * circle.radius();
}

////////

bool PolygonObstacle::hit(const Geometry2d::Point &pt) const
{
    return polygon.nearPoint(pt, Robot_Radius);
}

bool PolygonObstacle::hit(const Geometry2d::Segment &seg) const
{
    return polygon.nearSegment(seg, Robot_Radius);
}

Geometry2d::Point PolygonObstacle::closestEscape(const Geometry2d::Point& pose) const
{
	if (!hit(pose))
		return pose;

	// find closest segment and go to it
	Geometry2d::Point bestPt, lastPt = polygon.vertices.back();
	float closestDist = 1000;
	for (size_t i=0; i<polygon.vertices.size(); ++i) {
		Geometry2d::Point curPt = polygon.vertices[i];
		Geometry2d::Line line(lastPt, curPt);
		Geometry2d::Point pt = line.nearestPoint(pose);
		float dist = pt.distTo(pose);
		if (dist < closestDist) {
			bestPt = pt;
			closestDist = dist;
		}
	}
	return bestPt;
}
