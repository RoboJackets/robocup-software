#include "PivotPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include <cmath>
#include <boost/range/irange.hpp>
#include "RRTPlanner.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool PivotPathPlanner::shouldReplan(MotionInstant startInstant,
                                    const MotionCommand* cmd,
                                    const MotionConstraints& motionConstraints,
                                    const Geometry2d::ShapeSet* obstacles,
                                    const Path* prevPath) {
    PivotCommand command = *dynamic_cast<const PivotCommand*>(cmd);
    // TODO Implement This
    return true;
}

std::unique_ptr<Path> PivotPathPlanner::run(
    MotionInstant startInstant, const MotionCommand* cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    EscapeObstaclesPathPlanner escapePlanner;
    EmptyCommand emptyCommand;
    // TODO implement actual Pivoting
    return escapePlanner.run(startInstant, &emptyCommand, motionConstraints,
                             obstacles, std::move(prevPath));
}
}
