#include "PivotPathPlanner.hpp"
#include <Configuration.hpp>
#include <boost/range/irange.hpp>
#include <cmath>
#include "EscapeObstaclesPathPlanner.hpp"
#include "RRTPlanner.hpp"
#include "TrapezoidalPath.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool PivotPathPlanner::shouldReplan(MotionInstant startInstant,
                                    const MotionCommand* cmd,
                                    const MotionConstraints& motionConstraints,
                                    const Geometry2d::ShapeSet& obstacles,
                                    const Path* prevPath) {
    PivotCommand command = *dynamic_cast<const PivotCommand*>(cmd);
    debugThrow("Unfinished Class");
    // TODO Implement This
    return true;
}

std::unique_ptr<Path> PivotPathPlanner::run(
    MotionInstant startInstant, const MotionCommand* cmd,
    const MotionConstraints& motionConstraints, Geometry2d::ShapeSet& obstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles,
    std::unique_ptr<Path> prevPath) {
    // TODO implement actual Pivoting
    debugThrow("Unfinished Class");

    EscapeObstaclesPathPlanner escapePlanner;
    EmptyCommand emptyCommand;
    return escapePlanner.run(startInstant, &emptyCommand, motionConstraints,
                             obstacles, dynamicObstacles, std::move(prevPath));
}
}
