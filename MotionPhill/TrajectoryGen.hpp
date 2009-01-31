#ifndef _TRAJECTORYGEN_HPP_
#define _TRAJECTORYGEN_HPP_

#include <QObject>
#include "framework/Module.hpp"

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

        private:
            bool _running;
    };
}
#endif
