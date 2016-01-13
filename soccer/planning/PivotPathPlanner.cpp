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

REGISTER_CONFIGURABLE(PivotPathPlanner);

ConfigDouble* PivotPathPlanner::_targetVelChangeReplanThreshold;

void PivotPathPlanner::createConfiguration(Configuration* cfg) {
    _targetVelChangeReplanThreshold = new ConfigDouble(
        cfg, "PivotPathPlanner/velChangeReplanThreshold", 0.05);
}

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
    return escapePlanner.run(startInstant, &emptyCommand, motionConstraints,
                             obstacles, std::move(prevPath));
    // return nullptr;

    if (cmd->getCommandType() != Planning::MotionCommand::Pivot) {
        debugThrow("Pivot PathPlanner doesn't support this commandType.");
        return nullptr;
    }

    // If the start point is in an obstacle, escape from it
    if (obstacles->hit(startInstant.pos)) {
        EscapeObstaclesPathPlanner escapePlanner;
        EmptyCommand emptyCommand;
        return escapePlanner.run(startInstant, &emptyCommand, motionConstraints,
                                 obstacles, std::move(prevPath));
    }

    PivotCommand command = *static_cast<const PivotCommand*>(cmd);
    Point pivotTarget = command.pivotTarget;
    startInstant.pos;
    float r = Robot_Radius;
    // const float FudgeFactor = *_robot->config->pivotVelMultiplier;
    // float speed = RadiansToDegrees(r * targetW * FudgeFactor);
    // Point vel(speed, 0);

    // the robot body coordinate system is wierd...
    // vel.rotate(-M_PI_2);

    //_targetBodyVel(vel);
    vector<Point> points;
    Point vi;
    Point vf;

    return RRTPlanner::generateVelocityPath(points, *obstacles,
                                            motionConstraints, vi, vf);
}

}  // namespace Planning
