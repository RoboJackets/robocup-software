#include "InterceptPlanner.hpp"

#include <Configuration.hpp>
#include <Constants.hpp>
#include <motion/TrapezoidalMotion.hpp>

#include "MotionInstant.hpp"
#include "RRTPlanner.hpp"

namespace Planning {
std::unique_ptr<Path> InterceptPlanner::run(PlanRequest& planRequest) {
    const InterceptCommand& command =
        dynamic_cast<const InterceptCommand&>(*planRequest.motionCommand);

    // Start state for the specified robot
    const MotionInstant& startInstant = planRequest.start;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;

    SystemState& systemState = planRequest.context->state;
    const Ball& ball = systemState.ball;

    // Time for ball to hit target point
    // Target point is projected into ball vel line
    Geometry2d::Point targetPosOnLine;
    RJ::Seconds ballToPointTime =
        ball.estimateTimeTo(command.target, &targetPosOnLine) - RJ::now();

    // Try to use the same point if it's pretty close
    // Improves consistency overall
    if ((targetPosOnLine - prevPathTarget).mag() < Robot_Radius / 2) {
        targetPosOnLine = prevPathTarget;
    } else {
        prevPathTarget = targetPosOnLine;
    }

    // vector from robot to target
    Geometry2d::Point botToTarget = (targetPosOnLine - startInstant.pos);
    // Normalized vector from robot to target
    Geometry2d::Point botToTargetNorm = botToTarget.normalized();

    // Max speed we can reach given the distance to target and constant
    // acceleration If we don't constrain the speed, there is a velocity
    // discontinuity in the middle of the path
    double maxSpeed = std::min(
        startInstant.vel.mag() +
            sqrt(2 * motionConstraints.maxAcceleration * botToTarget.mag()),
        motionConstraints.maxSpeed);

    // Try to shortcut
    // If we are almost in the way of the ball already, just stay still
    // Saves some frames trying to compute that
    // Also is much more consistent
    if (botToTarget.mag() < Robot_Radius / 2) {
        MotionInstant finalStoppingMotion(targetPosOnLine,
                                          Geometry2d::Point(0, 0));

        std::unique_ptr<MotionCommand> directCommand =
            std::make_unique<DirectPathTargetCommand>(finalStoppingMotion);

        PlanRequest request = PlanRequest(
            planRequest.context, startInstant, std::move(directCommand),
            planRequest.constraints, nullptr, planRequest.obstacles,
            planRequest.dynamicObstacles, planRequest.shellID);

        std::unique_ptr<Path> path = directPlanner.run(request);
        path->setDebugText("AtPoint");

        return std::make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }

    // Scale the end velocity by % of max velocity to see if we can reach the
    // target at the same time as the ball
    std::unique_ptr<Path> path;

    for (double mag = 0.0; mag <= 1.0; mag += .05) {
        MotionInstant finalStoppingMotion(
            targetPosOnLine, mag * maxSpeed * botToTarget.normalized());

        std::unique_ptr<MotionCommand> directCommand =
            std::make_unique<DirectPathTargetCommand>(finalStoppingMotion);

        auto request = PlanRequest(
            planRequest.context, startInstant, std::move(directCommand),
            planRequest.constraints, nullptr, planRequest.obstacles,
            planRequest.dynamicObstacles, planRequest.shellID);

        path = directPlanner.run(request);

        // First path where we can reach the point at or before the ball
        // If the end velocity is not 0, you should reach the point as close
        // to the ball time as possible to just ram it
        if (path->getDuration() <= ballToPointTime) {
            path->setDebugText(
                "RT " + QString::number(path->getDuration().count(), 'g', 2) +
                " BT " + QString::number(path->getDuration().count(), 'g', 2));

            return std::make_unique<AngleFunctionPath>(
                std::move(path),
                angleFunctionForCommandType(FacePointCommand(ball.pos)));
        }
    }

    // We couldn't get to the target point in time
    // Just give up and do the max vel across ball vel
    // Which ends up being the path after the final loop
    path->setDebugText("GivingUp");

    return std::make_unique<AngleFunctionPath>(
        std::move(path),
        angleFunctionForCommandType(FacePointCommand(ball.pos)));
}
}  // namespace Planning
