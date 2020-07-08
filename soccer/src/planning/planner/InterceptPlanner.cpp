#include "InterceptPlanner.hpp"

#include <rj_constants/constants.hpp>

#include "Configuration.hpp"
#include "planning/Instant.hpp"
#include "planning/primitives/AnglePlanning.hpp"
#include "planning/primitives/CreatePath.hpp"
#include "planning/primitives/RRTUtil.hpp"
#include "planning/primitives/VelocityProfiling.hpp"

namespace Planning {

Trajectory InterceptPlanner::plan(const PlanRequest& planRequest) {
    InterceptCommand command =
        std::get<InterceptCommand>(planRequest.motionCommand);

    // Start state for the specified robot
    RobotInstant startInstant = planRequest.start;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;

    BallState ball = planRequest.world_state->ball;

    // Time for ball to hit target point
    // Target point is projected into ball velocity line
    Geometry2d::Point targetPosOnLine;
    RJ::Seconds ballToPointTime =
        ball.query_seconds_near(command.target, &targetPosOnLine);

    // vector from robot to target
    Geometry2d::Point botToTarget = (targetPosOnLine - startInstant.position());

    // Max speed we can reach given the distance to target and constant
    // acceleration If we don't constrain the speed, there is a velocity
    // discontinuity in the middle of the path
    double maxSpeed = std::min(
        startInstant.linear_velocity().mag() +
            sqrt(2 * motionConstraints.maxAcceleration * botToTarget.mag()),
        motionConstraints.maxSpeed);

    // Scale the end velocity by % of max velocity to see if we can reach the
    // target at the same time as the ball
    Trajectory trajectory;

    int num_iterations = 20;
    for (int i = 0; i <= num_iterations; i++) {
        double mag = i * 0.05;

        LinearMotionInstant finalStoppingMotion{
            targetPosOnLine, mag * maxSpeed * botToTarget.normalized()};

        trajectory = CreatePath::simple(
            startInstant.linear_motion(), finalStoppingMotion,
            planRequest.constraints.mot, startInstant.stamp);

        // First path where we can reach the point at or before the ball
        // If the end velocity is not 0, you should reach the point as close
        // to the ball time as possible to just ram it
        if (trajectory.duration() <= ballToPointTime) {
            std::ostringstream debug_text_out;
            debug_text_out.precision(2);
            debug_text_out << "Time " << trajectory.duration().count();
            trajectory.setDebugText(debug_text_out.str());

            PlanAngles(&trajectory, startInstant,
                       AngleFns::facePoint(ball.position),
                       planRequest.constraints.rot);
            trajectory.stamp(RJ::now());
            return trajectory;
        }
    }

    // We couldn't get to the target point in time
    // Just give up and do the max velocity across ball velocity
    // Which ends up being the path after the final loop
    trajectory.setDebugText("GivingUp");

    PlanAngles(&trajectory, startInstant, AngleFns::facePoint(ball.position),
               planRequest.constraints.rot);
    trajectory.stamp(RJ::now());

    return trajectory;
}

}  // namespace Planning
