#include "TrajectoryGen.hpp"
#include <Constants.hpp>

using namespace Trajectory;

TrajectoryGen::TrajectoryGen() :
    Module("TrajectoryGen"), _running(false)
{
    _waypoints.append(Point2d(0,3));
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
        Geometry::Point2d error = _state->self[0].cmdPos - _state->self[0].pos;
        Geometry::Point2d deadband(0.05,0.05);
//         printf("error x %f y %f\n",error.x, error.y);
        if(error.mag() < deadband.mag())
        {
            _currWaypoint++;
//             printf("Waypoint x %f y %f\n", _currWaypoint->x, _currWaypoint->y);
        }
//         printf("_waypoints.begin %d\n", _waypoints.begin());
//         printf("_waypoint.end %d\n", _waypoints.end());
//         printf("_currWaypoint %d\n", _currWaypoint);

    }
    else
    {
        _state->self[0].cmdPos = _state->self[0].pos;
    }
}


void TrajectoryGen::setPaths(QVector<RobotPath::Path> paths)
{
    _paths = paths;

    if(!_paths.isEmpty())
    {
        _waypoints.clear();
        Q_FOREACH(RobotPath::Path p, _paths)
        {
            Geometry::Point2d startPoint, endPoint, tempPoint;
            float m, delta_x, delta_y;
            switch(p.type)
            {
                case RobotPath::Line:
                    startPoint = convertPoint(p.points[1]);
                    endPoint = convertPoint(p.points[0]);
                    m = (startPoint.y - endPoint.y)/(startPoint.x - endPoint.x);

                    _waypoints.append(startPoint);

                    delta_x=fabs(startPoint.x - endPoint.x)/50;
                    delta_y=fabs(startPoint.y - endPoint.y)/50;
                    tempPoint.x = startPoint.x;
                    for(int i = 0; i<50; i++)
                    {
                        //Parameterized lines (duh!!!)
                        tempPoint.x = startPoint.x + delta_x*i;
                        tempPoint.y = startPoint.y + delta_y*i;
                        _waypoints.append(tempPoint);
                    }
//                     printf("End point x %f y %f\n", endPoint.x, endPoint.y);
                    break;
                case RobotPath::Arc:
                    break;
                case RobotPath::Start:
                    _waypoints.append(convertPoint(p.points[0]));
//                     printf("Waypoint x %f\n", _waypoints[0].x);
//                     printf("Waypoint y %f\n", _waypoints[0].y);
                    break;
                case RobotPath::BezierCurve:
                    break;
                case RobotPath::Close:
                    break;
            }
//             printf("Waypoint x %f y %f\n", _waypoints.last().x, _waypoints.last().y);
        }
        _currWaypoint = _waypoints.begin();
//         printf("_currWaypoint set to %f %f\n", _currWaypoint->x, _currWaypoint->y);
    }
}

Point2d TrajectoryGen::convertPoint(QPointF point)
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