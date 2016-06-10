#include "LineKickPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include <Geometry2d/Util.hpp>
#include "RRTPlanner.hpp"
#include "motion/TrapezoidalMotion.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool LineKickPlanner::shouldReplan(
    const SinglePlanRequest& planRequest) const {
    const MotionConstraints& motionConstraints =
        planRequest.robotConstraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    const auto& command = dynamic_cast<const PivotCommand&>(planRequest.cmd);

    if (!prevPath) {
        return true;
    } else {
        // TODO ashaw37: make this better
        float radius = command.radius;
        auto pivotPoint = command.pivotPoint;
        auto pivotTarget = command.pivotTarget;
        auto endTarget =
            pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
        float targetChange = (prevPath->end().motion.pos - endTarget).mag();

        if (targetChange > SingleRobotPathPlanner::goalChangeThreshold()) {
            // return true;
        }
    }
    return false;
}

std::unique_ptr<Path> LineKickPlanner::run(SinglePlanRequest& planRequest) {

    const float ballAvoidDistance = 0.1;

    auto prevAnglePath = dynamic_cast<AngleFunctionPath*>(planRequest.prevPath.get());

    const auto& command = dynamic_cast<const LineKickCommand&>(planRequest.cmd);

    const MotionInstant& startInstant = planRequest.startInstant;
    const auto& motionConstraints = planRequest.robotConstraints.mot;
    const auto& rotationConstraints = planRequest.robotConstraints.rot;
    auto& obstacles = planRequest.obstacles;
    const auto& systemState = planRequest.systemState;
    const auto& ball = systemState.ball;
    const auto& robotConstraints = planRequest.robotConstraints;

    float timeEstimate;
    if (prevAnglePath) {
        float timeIntoPath = RJ::TimestampToSecs(RJ::timestamp() - prevAnglePath->startTime());
        timeEstimate = planRequest.prevPath->getDuration() - timeIntoPath;
    } else {
        timeEstimate = Trapezoidal::getTime(0, ball.pos.distTo(startInstant.pos),
                                            robotConstraints.mot.maxSpeed, robotConstraints.mot.maxAcceleration,
                                            startInstant.vel.mag(), 0.5);
    }

    if (timeEstimate<0) {
        debugLog("timeEstimate<0");
        timeEstimate = 0;
    }

    MotionInstant target = ball.predict(RJ::SecsToTimestamp(timeEstimate) + RJ::timestamp());
    //target.pos = ball.pos;
    target.vel = (command.target - target.pos).normalized(1.0);
    if(std::abs(target.vel.angleBetween((target.pos - startInstant.pos)))>DegreesToRadians(30)) {
        obstacles.add(make_shared<Circle>(target.pos, ballAvoidDistance));
        obstacles.add(make_shared<Circle>(ball.pos, ballAvoidDistance));
        target.pos -= target.vel.normalized(ballAvoidDistance);
        //printf("wrongSide");
    }
    unique_ptr<Path> prevPath;
    if (prevAnglePath) {
        prevPath = std::move(prevAnglePath->path);
    } else {
        prevPath = nullptr;
    }
    auto request = SinglePlanRequest(startInstant, PathTargetCommand(target),
        robotConstraints, obstacles, planRequest.dynamicObstacles, systemState, std::move(prevPath));
    auto path = rrtPlanner.run(request);

    return make_unique<AngleFunctionPath>(std::move(path), angleFunctionForCommandType(FacePointCommand(command.target)));

    return path;
}
}
