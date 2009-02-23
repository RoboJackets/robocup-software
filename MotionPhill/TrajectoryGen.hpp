#ifndef _TRAJECTORYGEN_HPP_
#define _TRAJECTORYGEN_HPP_

#include <QVector>
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
        void runModule();
        void stopModule();
        void setPath(QVector<RobotPath::Path> paths)
        {
            _paths = paths;
        }


    private:

        QVector<RobotPath::Path> _paths;
        bool _running;
    };
}
#endif
