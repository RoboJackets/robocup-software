#ifndef _TRAJECTORYGEN_HPP_
#define _TRAJECTORYGEN_HPP_

#include <Geometry/Point2d.hpp>
#include <QVector>
#include <QPointF>

#include "framework/Module.hpp"
#include "RobotPath.hpp"

namespace Trajectory
{
    class TrajectoryGen : public QObject, public Module
    {
        Q_OBJECT;

    public:
        TrajectoryGen(float maxV, float maxA);
        ~TrajectoryGen();

        virtual void run();
        void trapGen(bool accelerate, int t);
        Point2d convertPoint(QPointF point);
        bool areWeThereYet();

    public Q_SLOTS:
        void runModule() { _running = true; }
        void stopModule() { _running = false; }
        void pixelFieldSize(Geometry::Point2d size) { _pixelFieldSize = size; }
        void setPaths(QVector<RobotPath::Path> paths);
    private:
        QVector<RobotPath::Path> _paths;
        QVector<Geometry::Point2d> _waypoints;

        Geometry::Point2d _pixelFieldSize;

        QVector<Geometry::Point2d>::iterator _currWaypoint;
        QVector<Geometry::Point2d>::iterator _nextWaypoint;
        bool _running;

        Geometry::Point2d _goalPos;
        Geometry::Point2d _startPos;
        Geometry::Point2d _vel;

        /** Max Velocity on the path **/
        float _maxV;

        /** Max accel on path **/
        float _maxA;

        /** Scaled time on trajectory. Goes from 0 to 1 **/
        unsigned int _trajTimer;

        unsigned int _t1,_t12,_t2;
        float _dt1;
        unsigned int _totalTime;
    };
}
#endif
