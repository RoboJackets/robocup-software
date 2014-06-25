#include <planning/Obstacle.hpp>
#include <Constants.hpp>

#include <boost/foreach.hpp>

using namespace boost;

// bool CircleObstacle::hit(const Geometry2d::Point &pt) const
// {
//     return pt.nearPoint(circle.center, circle.radius() + Robot_Radius);
// }

// bool CircleObstacle::hit(const Geometry2d::Segment &seg) const
// {
//     return seg.nearPoint(circle.center, circle.radius() + Robot_Radius);
// }

// ////////

// bool PolygonObstacle::hit(const Geometry2d::Point &pt) const
// {
//     return polygon.nearPoint(pt, Robot_Radius);
// }

// bool PolygonObstacle::hit(const Geometry2d::Segment &seg) const
// {
//     return polygon.nearSegment(seg, Robot_Radius);
// }

