#include "TrajectoryGen.hpp"
#include <Constants.hpp>
#include <Team.h>
#include <math.h>

using namespace Trajectory;

TrajectoryGen::TrajectoryGen(float maxV, float maxA) :
    Module("TrajectoryGen"), _running(false), _maxV(maxV), _maxA(maxA)
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
        Geometry::Point2d currPos = _state->self[0].pos;

        if(*_currWaypoint != _goalPos)
        {
            int trajLength;
            _goalPos = *_currWaypoint;
            _startPos = _state->self[0].pos;

            //Reset variables
            trajLength = _startPos.distTo(_goalPos);
            _vel = _state->self[0].vel;
            _trajTimer = 0;
            _t1 = _maxV/_maxA;
            _dt1 = (1/2)*_maxA*(_t1*_t1);
            _t12 = (trajLength - 2*_dt1) / _maxV;
            _totalTime = 2*_t1 + _t12;
            _t2 = _t12 + _t1;

            if(_t12 < 1)
            {
                //Robot won't reach max speed over the length of the trajectory
                //Switch to triangular velocity profile
                _t1 = (int)sqrt(trajLength/_maxA);
                _totalTime = 2*_t1;
                _t1 = _t2;
            }
        }

        //Generate pos and velocity command based on trajectory
        Geometry::Point2d cmdPos;
        Geometry::Point2d cmdVel;
        if(_trajTimer < _t1)
        {
            cmdPos.x = _startPos.x + (1/2)*_maxA*(_trajTimer*_trajTimer);
            cmdVel.x = _maxA*_trajTimer;
        }
        else if(_trajTimer > _t1 && _trajTimer < _t2)
        {
            cmdPos.x = _startPos.x + (1/2)*_maxA*(_t1*_t1) + _maxV*_trajTimer;
            cmdVel.x = _maxV;
        }
        else if(_trajTimer > _t2 && !areWeThereYet())
        {
            cmdPos.x = _startPos.x + (1/2)*_maxA*(_t1*_t1) + _maxV*(_t12) - (1/2)*_maxA*(_trajTimer*_trajTimer);
            cmdVel.x = -_maxA*_trajTimer;
        }

        _state->self[0].cmd.pos = cmdPos;
        _state->self[0].cmd.v_ff = cmdVel;

        printf("%f,\n", cmdPos.x);
        printf("%f,\n", cmdVel.x);
//         _state->self[0].cmdPos = *_currWaypoint;
//         Geometry::Point2d error = _state->self[0].cmdPos - _state->self[0].pos;
//         Geometry::Point2d deadband(0.05,0.05);
//         if(error.mag() < deadband.mag())
//         {
//             if(_nextWaypoint < _waypoints.end())
//             {
//                 _currWaypoint++;
//                 _nextWaypoint++;
//             }
//         }
    }
    else
    {
        _state->self[0].cmd.pos = _state->self[0].pos;
    }
    printf("Hello\n");
}


void TrajectoryGen::setPaths(QVector<RobotPath::Path> paths)
{
    _paths = paths;

    if(!_paths.isEmpty())
    {
        _waypoints.clear();
        Q_FOREACH(RobotPath::Path p, _paths)
        {
            Geometry::Point2d startPoint, endPoint, tempPoint, dist;
            float m, delta_x, delta_y;
            int numPathPoints;
            switch(p.type)
            {
                case RobotPath::Line:
                    startPoint = convertPoint(p.points[1]);
                    endPoint = convertPoint(p.points[0]);
                    m = (startPoint.y - endPoint.y)/(startPoint.x - endPoint.x);

                    _waypoints.append(startPoint);

                    //determine number of points
                    dist = startPoint - endPoint;
                    numPathPoints = (int) (dist.mag() / 0.10);

                    delta_x=fabs(startPoint.x - endPoint.x)/numPathPoints;
                    delta_y=fabs(startPoint.y - endPoint.y)/numPathPoints;
                    tempPoint.x = startPoint.x;
                    for(int i = 0; i<numPathPoints; i++)
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
                    break;
                case RobotPath::BezierCurve:
                    break;
                case RobotPath::Close:
                    break;
            }
//             printf("Waypoint x %f y %f\n", _waypoints.last().x, _waypoints.last().y);
        }
        _currWaypoint = _waypoints.begin();
        _nextWaypoint = _currWaypoint;
        _nextWaypoint++;
//         printf("_waypoints.begin %f %f\n", _waypoints.begin()->x, _waypoints.begin()->y);
//         printf("_waypoint.end %f %f\n", _waypoints.end()->x, _waypoints.end()->y);

    }
}

Point2d TrajectoryGen::convertPoint(QPointF point)
{
    Point2d retPoint;
    retPoint.y = (point.x() * (Constants::Floor::Length/802.0)) - Constants::Field::Border;
    retPoint.x = (point.y() * (Constants::Floor::Width/556.0)) - Constants::Field::Border;

    retPoint.x = retPoint.x - Constants::Field::Width/2;

    if(_state->team == Blue)
    {
        retPoint.y = Constants::Field::Length - retPoint.y;
        retPoint.x = -retPoint.x;
    }

    return retPoint;
}


void TrajectoryGen::trapGen(bool accelerate, int t)
{
    static Geometry::Point2d previousVel;

    if(accelerate)
    {
        _vel.x += _maxA*t;
        _vel.y += _maxA*t;
    }
    else
    {
        _vel.x += -_maxA*t;
        _vel.y += -_maxA*t;
    }


    if(_vel.x >= _maxV)
    {
        _vel.x = _maxV;
    }

    if(_vel.y >= _maxV)
    {
        _vel.y = _maxV;
    }
}

bool TrajectoryGen::areWeThereYet()
{
    bool weThere = false;
    Geometry::Point2d error = _goalPos - _state->self[0].pos;
    Geometry::Point2d deadband(0.05,0.05);
    if(error.mag() < deadband.mag())
    {
        weThere = true;
    }
    return weThere;
}
