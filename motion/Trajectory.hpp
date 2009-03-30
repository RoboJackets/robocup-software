#ifndef _TRAJECTORY_HPP_
#define _TRAJECTORY_HPP_

#include <Geometry/Point2d.hpp>
#include <QVector>
#include <QPointF>

class Trajectory
{
    public:
        typedef struct
        {
            Geometry::Point2d pos;
            Geometry::Point2d v_ff;
        } TrajectoryCmd;

        typedef struct
        {
            float t1,t12, t2, tf, dt1;
        } TrajectoryParams;

    public:
        Trajectory(float maxV, float maxA);
        ~Trajectory();

        TrajectoryCmd run(Geometry::Point2d currPos, float t);
        bool areWeThereYet();
        void setTrajectory(Geometry::Point2d goalPoint, Geometry::Point2d startPos, Geometry::Point2d currVel);
        TrajectoryParams calcTime(float maxA, float maxV, float length);

    private:
        Geometry::Point2d _goalPos;
        Geometry::Point2d _startPos;
        Geometry::Point2d _vel;
        Geometry::Point2d _currPos;

        /** Max Velocity on the path in m/s **/
        float _maxV;

        /** Max accel on path in m/s^2 **/
        float _maxA;

        float  _t1, _t12, _t2, _dt1, _tf, _ax, _ay, _vx, _vy;
};
#endif
