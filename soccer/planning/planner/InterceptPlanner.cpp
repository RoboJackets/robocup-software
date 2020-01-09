#include "planning/planner/InterceptPlanner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "MotionCommand.hpp"
#include <Configuration.hpp>
#include <Constants.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include "planning/trajectory/VelocityProfiling.hpp"
#include "time.hpp"

namespace Planning {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

Trajectory InterceptPlanner::plan(PlanRequest&& request) {
    const InterceptCommand& command = std::get<InterceptCommand>(request.motionCommand)
    RobotInstant startInstant = request.start;
    MotionConstraints motionConstraints = request.constraints.mot;
    RotationConstraints rotationConstraints = request.constraints.rot;
    const Ball& ball = request.context->state.ball;

    // Time for ball to hit target point
    // Target point is projected into ball vel line
    Point targetPosOnLine;
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
    Point botToTarget = (targetPosOnLine - startInstant.pose.position());

    // Max speed we can reach given the distance to target and constant
    // acceleration If we don't constrain the speed, there is a velocity
    // discontinuity in the middle of the path
    double maxSpeed = std::min(startInstant.velocity.linear().mag() +
            sqrt(2 * motionConstraints.maxAcceleration * botToTarget.mag()),
        motionConstraints.maxSpeed);

    // Try to shortcut
    // If we are almost in the way of the ball already, just stay still
    // Saves some frames trying to compute that
    // Also is much more consistent
    if (botToTarget.mag() < Robot_Radius / 2) {
        RobotInstant finalStoppingMotion(Pose{targetPosOnLine, targetPosOnLine.angleTo(ball.pos)}, {}, RJ::now());

        PlanRequest pathTargetRequest = PlanRequest(
            request.context, startInstant, PathTargetCommand{finalStoppingMotion},
            request.constraints, Trajectory{{}}, request.static_obstacles,
            request.dynamic_obstacles, request.shellID);

        Trajectory path = pathTargetPlanner.plan(std::move(pathTargetRequest));
        PlanAngles(path, startInstant, AngleFns::facePoint(ball.pos), rotationConstraints);
        path.setDebugText("AtPoint");
        return std::move(path);
    }

    // Scale the end velocity by % of max velocity to see if we can reach the
    // target at the same time as the ball
    Trajectory path{{}};

    for (double mag = 0.0; mag <= 1.0; mag += .05) {
        RobotInstant finalStoppingMotion{Pose{targetPosOnLine, targetPosOnLine.angleTo(ball.pos)}, Twist{mag * maxSpeed * botToTarget.normalized(),0}, RJ::now()};

        auto pathTargetRequest = PlanRequest(
            request.context, startInstant, PathTargetCommand{finalStoppingMotion},
            request.constraints, Trajectory{{}}, request.static_obstacles,
            request.dynamic_obstacles, request.shellID);

        path = pathTargetPlanner.plan(std::move(pathTargetRequest));

        // First path where we can reach the point at or before the ball
        // If the end velocity is not 0, you should reach the point as close
        // to the ball time as possible to just ram it
        if (path.duration() <= ballToPointTime) {
            path.setDebugText(
                "RT " + QString::number(path.duration().count(), 'g', 2) +
                " BT " + QString::number(path.duration().count(), 'g', 2));
            PlanAngles(path, startInstant, AngleFns::facePoint(ball.pos), rotationConstraints);
            return std::move(path);
        }
    }

    // We couldn't get to the target point in time
    // Just give up and do the max vel across ball vel
    // Which ends up being the path after the final loop
    path.setDebugText("GivingUp");
    PlanAngles(path, startInstant, AngleFns::facePoint(ball.pos), rotationConstraints);
    return std::move(path);
}
}  // namespace Planning
