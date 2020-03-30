#include "InterceptPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "PlanRequest.hpp"
#include "Planner.hpp"
namespace Planning {
using namespace Geometry2d;
Trajectory InterceptPlanner::plan(PlanRequest&& request) {
    RobotInstant startInstant = request.start;
    const Ball& ball = request.context->state.ball;
    Point targetPoint = std::get<InterceptCommand>(request.motionCommand).target;

    RJ::Seconds ballToPointTime =
            ball.estimateTimeTo(targetPoint, &targetPoint) - RJ::now();
    if ((targetPoint - _prevTargetPoint).mag() < Robot_Radius / 2) {
        targetPoint = _prevTargetPoint;
    } else {
        _prevTargetPoint = targetPoint;
    }
    Point botToTarget = (targetPoint - startInstant.pose.position());

    // Try to shortcut
    // If we are almost in the way of the ball already, just stay still
    // Saves some frames and is much more consistent
    RobotInstant targetInstant{Pose{targetPoint, 0}, Twist{}, RJ::Time{0s}};
    if (botToTarget.mag() < Robot_Radius / 2) {
        auto path = CreatePath::simple(startInstant, targetInstant, request.constraints.mot);
        PlanAngles(path, startInstant, AngleFns::facePoint(ball.pos), request.constraints.rot);
        path.setDebugText("AtPoint");
        return std::move(path);
    }

    RJ::Time startTime = RJ::now();
    // Find the minimum viable target velocity using Brute Force
    // a lower target velocity is safer because we are more likely to stay there
    Trajectory path{{}};
    constexpr int interpolations = 21;
    for (int i = 0; i < interpolations; i++) {
        double percent = static_cast<double>(i) / (interpolations-1);
        targetInstant.velocity.linear() =
                percent * request.constraints.mot.maxSpeed * botToTarget.norm();
        path = CreatePath::simple(startInstant, targetInstant, request.constraints.mot);
        PlanAngles(path, startInstant, AngleFns::facePoint(ball.pos), request.constraints.rot);
        if (path.duration() <= ballToPointTime) {
            break;
        }
    }
    printf("Intercept brute force took %d sec\n", RJ::Seconds{RJ::now()-startTime}.count());
    return std::move(path);
}
}  // namespace Planning
