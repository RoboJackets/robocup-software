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
        TrajectoryGen();
        ~TrajectoryGen();

        virtual void run();

    public Q_SLOTS:
        void runModule()
        {
            _running = true;
        }

        void stopModule()
        {
            _running = false;
        }

        void pixelFieldSize(Geometry::Point2d size)
        {
            _pixelFieldSize = size;
        }

        Point2d convertPoint(QPointF point);
        void setPaths(QVector<RobotPath::Path> paths);

    private:
        QVector<RobotPath::Path> _paths;
        QVector<Geometry::Point2d> _waypoints;

        Point2d _pixelFieldSize;

        QVector<Geometry::Point2d>::iterator _currWaypoint;
        QVector<Geometry::Point2d>::iterator _nextWaypoint;
        bool _running;
    };
}
#endif
