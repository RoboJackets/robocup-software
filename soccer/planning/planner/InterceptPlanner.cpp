#include "InterceptPlanner.hpp"

#include <Configuration.hpp>
#include <Constants.hpp>
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {

Trajectory InterceptPlanner::plan(PlanRequest&& planRequest) {
    InterceptCommand command = std::get<InterceptCommand>(planRequest.motionCommand);

    // Start state for the specified robot
    RobotInstant startInstant = planRequest.start;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;

    BallState ball = planRequest.world_state->ball;

    // Time for ball to hit target point
    // Target point is projected into ball vel line
    Geometry2d::Point targetPosOnLine;
    RJ::Seconds ballToPointTime =
        ball.query_seconds_to(command.target, &targetPosOnLine);

    // vector from robot to target
    Geometry2d::Point botToTarget = (targetPosOnLine - startInstant.pose.position());

    // Max speed we can reach given the distance to target and constant
    // acceleration If we don't constrain the speed, there is a velocity
    // discontinuity in the middle of the path
    double maxSpeed = std::min(
        startInstant.velocity.linear().mag() +
        sqrt(2 * motionConstraints.maxAcceleration * botToTarget.mag()),
        motionConstraints.maxSpeed);

    // Scale the end velocity by % of max velocity to see if we can reach the
    // target at the same time as the ball
    Trajectory trajectory;

    for (double mag = 0.0; mag <= 1.0; mag += .05) {
        RobotInstant finalStoppingMotion(
            Geometry2d::Pose(targetPosOnLine, startInstant.pose.heading()),
            Geometry2d::Twist(mag * maxSpeed * botToTarget.normalized(), 0),
            RJ::now());

        trajectory = CreatePath::simple(startInstant, finalStoppingMotion, planRequest.constraints.mot);

        // First path where we can reach the point at or before the ball
        // If the end velocity is not 0, you should reach the point as close
        // to the ball time as possible to just ram it
        if (trajectory.duration() <= ballToPointTime) {
            trajectory.setDebugText(
                "RT " + QString::number(trajectory.duration().count(), 'g', 2) +
                " BT " + QString::number(trajectory.duration().count(), 'g', 2));

            PlanAngles(trajectory, startInstant,
                       AngleFns::facePoint(ball.position),
                       planRequest.constraints.rot);
            return trajectory;
        }
    }

    // We couldn't get to the target point in time
    // Just give up and do the max vel across ball vel
    // Which ends up being the path after the final loop
    trajectory.setDebugText("GivingUp");

    PlanAngles(trajectory, startInstant,
               AngleFns::facePoint(ball.position),
               planRequest.constraints.rot);

    return trajectory;
}

}  // namespace Planning
