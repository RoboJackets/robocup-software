#include "TrajectoryGen.hpp"
#include <Constants.hpp>

using namespace Trajectory;

TrajectoryGen::TrajectoryGen() :
    Module("TrajectoryGen"), _running(false)
{
    _waypoints.append(Point2d(0,0));
    _currWaypoint = _waypoints.end();
}

TrajectoryGen::~TrajectoryGen()
{

}

void TrajectoryGen::run()
{
    if(_running)
    {
        _state->self[0].cmdPos = *_currWaypoint;
//         printf("Waypoint x %f\n", _state->self[0].cmdPos.x);
//         printf("Waypoint y %f\n", _state->self[0].cmdPos.y);
//         printf("_waypoints.begin %d\n", _waypoints.begin());
//         printf("_waypoint.end %d\n", _waypoints.end());
//         printf("_currWaypoint %d\n", _currWaypoint);
        if(_currWaypoint  > _waypoints.begin())
        {
            _currWaypoint--;
        }
    }
}


void TrajectoryGen::setPaths(QVector<RobotPath::Path> paths)
{
    _paths = paths;

    _waypoints.clear();

    Q_FOREACH(RobotPath::Path p, _paths)
    {
        switch(p.type)
        {
            case RobotPath::Line:
                break;
            case RobotPath::Arc:
                break;
            case RobotPath::Start:
                _waypoints.append(convertUnits(p.points[0]));
                //printf("Waypoint x %f\n", _waypoints[0].x);
                //printf("Waypoint y %f\n", _waypoints[0].y);
                break;
            case RobotPath::BezierCurve:
                break;
            case RobotPath::Close:
                break;
        }
        _waypointMutex.lock();
        _currWaypoint = _waypoints.end();
        _waypointMutex.unlock();
//         printf("_currWaypoint set to %d\n", _currWaypoint);
    }
}

Point2d TrajectoryGen::convertUnits(QPointF point)
{
    Point2d retPoint;
    retPoint.y = (point.x() * (Constants::Floor::Length/802.0)) - Constants::Field::Border;
    retPoint.x = (point.y() * (Constants::Floor::Width/556.0)) - Constants::Field::Border;

    retPoint.x = retPoint.x - Constants::Field::Width/2;

    if(_state->isBlue)
    {
        retPoint.y = Constants::Field::Length - retPoint.y;
    }

    return retPoint;
}