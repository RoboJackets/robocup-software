#include "CollectPathPlanner.hpp"
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <Geometry2d/Pose.hpp>
#include "Configuration.hpp"
#include <vector>

namespace Planning {

REGISTER_CONFIGURABLE(CollectPathPlanner);

ConfigDouble* CollectPathPlanner::_ballSpeedApproachDirectionCutoff;
ConfigDouble* CollectPathPlanner::_approachAccelScalePercent;
ConfigDouble* CollectPathPlanner::_controlAccelScalePercent;
ConfigDouble* CollectPathPlanner::_approachDistTarget;
ConfigDouble* CollectPathPlanner::_touchDeltaSpeed;
ConfigDouble* CollectPathPlanner::_velocityControlScale;
ConfigDouble* CollectPathPlanner::_distCutoffToControl;
ConfigDouble* CollectPathPlanner::_velCutoffToControl;
ConfigDouble* CollectPathPlanner::_distCutoffToApproach;
ConfigDouble* CollectPathPlanner::_stopDistScale;
ConfigDouble* CollectPathPlanner::_targetPointAveragingGain;

void CollectPathPlanner::createConfiguration(Configuration* cfg) {
    _ballSpeedApproachDirectionCutoff = new ConfigDouble(
        cfg, "Capture/Collect/ballSpeedApproachDirectionCutoff", 0.1);
    _approachAccelScalePercent =
        new ConfigDouble(cfg, "Capture/Collect/approachAccelScalePercent", 0.7);
    _controlAccelScalePercent =
        new ConfigDouble(cfg, "Capture/Collect/controlAccelScalePercent", 0.8);
    _approachDistTarget =
        new ConfigDouble(cfg, "Capture/Collect/approachDistTarget", 0.04);
    _touchDeltaSpeed =
        new ConfigDouble(cfg, "Capture/Collect/touchDeltaSpeed", 0.1);
    _velocityControlScale =
        new ConfigDouble(cfg, "Capture/Collect/velocityControlScale", 1);
    _distCutoffToControl =
        new ConfigDouble(cfg, "Capture/Collect/distCutoffToControl", 0.05);
    _velCutoffToControl =
        new ConfigDouble(cfg, "Capture/Collect/velCutoffToControl", 1);
    _distCutoffToApproach =
        new ConfigDouble(cfg, "Capture/Collect/distCutoffToApproach", 0.3);
    _stopDistScale = new ConfigDouble(cfg, "Capture/Collect/stopDistScale", 1);
    _targetPointAveragingGain =
        new ConfigDouble(cfg, "Capture/Collect/targetPointAveragingGain", 0.8);
}

// the Collect path consists of segments for Course, Fine, and Control
// a ---------------- b --- c -------- d
//      Course          Fine   Control
// The Course segment aims to get to the ball as fast as possible
// The Fine segment aims to capture the ball without it bouncing off
// The Control segment moves through the ball to fully possess it
// and stop the robot (if it has non-zero velocity at point c)
enum class CollectState {
    Course,
    Fine,
    Control,
    None
};
std::string collectStateToString(CollectState state) {
    switch(state) {
        case CollectState::Course: return "Course";
        case CollectState::Fine: return "Fine";
        case CollectState::Control: return "Control";
        default: return "None";
    }
}
Trajectory CollectPathPlanner::plan(PlanRequest&& request) {
    using Geometry2d::Point;
    using Geometry2d::Pose;
    using Geometry2d::Twist;

    static CollectState prevState = CollectState::None;
    const Ball& ball = request.context->state.ball;
    const CollectCommand& command = std::get<CollectCommand>(request.motionCommand);
    RotationConstraints rotationConstraints = request.constraints.rot;
    MotionConstraints& motionConstraints = request.constraints.mot;
    RobotInstant startInstant = request.start;
    Point startPoint = startInstant.pose.position();

    const bool veeredOff = veeredOffPath(request);
    RJ::Seconds invalidTime = 0s;
    const bool hitObstacle = request.prevTrajectory.hit(request.obstacles, startInstant.stamp - request.prevTrajectory.begin_time(), &invalidTime);

    // Initialize the filter to the ball velocity so there's less ramp up
    if (!averageBallVel) {
        averageBallVel = ball.vel;
    } else {
        averageBallVel = *_targetPointAveragingGain * *averageBallVel +
                         (1 - *_targetPointAveragingGain) * ball.vel;
    }

    Point approachDirection = ball.vel.mag() < *_ballSpeedApproachDirectionCutoff ?
            (ball.pos - startPoint).norm() : averageBallVel->norm();

    double courseThreshold = *_approachDistTarget + Robot_MouthRadius;
    RobotInstant pathTarget;
    double distance = ball.pos.distTo(startPoint);
    Point targetHitVel = *averageBallVel + approachDirection * *_touchDeltaSpeed;
    double fineSpeed = std::min(targetHitVel.mag(), motionConstraints.maxSpeed);
    CollectState currentState = CollectState::None;
    bool replan = false;
    if (distance > courseThreshold) {
        // Course Segment
        currentState = CollectState::Course;
        Point targetPoint = ball.pos - approachDirection * courseThreshold;
        pathTarget = RobotInstant{Pose{ targetPoint, approachDirection.angle() },
            Twist{ approachDirection * fineSpeed, 0 }, RJ::now() };


        if(RJ::Seconds(RJ::now() - request.prevTrajectory.begin_time()).count() > 0.2) {
            replan = true;
        }
    } else if (distance > Robot_MouthRadius) {
        // Fine Segment
        currentState = CollectState::Fine;
        //todo(Ethan) should be dot product ?
        motionConstraints.maxSpeed = fineSpeed;
        motionConstraints.maxAcceleration *= *_approachAccelScalePercent;
        Point targetPoint = ball.pos - approachDirection * Robot_MouthRadius;
        pathTarget = RobotInstant{Pose{ targetPoint, approachDirection.angle() },
                                  Twist{ approachDirection * fineSpeed * 0.3, 0 }, RJ::now() };
    } else {
        currentState = CollectState::Control;
        motionConstraints.maxAcceleration *= *_controlAccelScalePercent;
        //todo(Ethan) handle balls coming at us with Settle?
        //kinematics: v^2 = v0^2 + 2 * a * deltaX
        double stoppingDist = fineSpeed * fineSpeed / (2 * motionConstraints.maxAcceleration);
        //travel a bit more at constant speed then decelerate to stop
        Point targetPoint = ball.pos + approachDirection * (stoppingDist + *_approachDistTarget);
        pathTarget = RobotInstant{Pose{ targetPoint, approachDirection.angle() },
                                  Twist{}, RJ::now() };
    }
    if(currentState != prevState || veeredOff || hitObstacle) {
        replan = true;
    }
    prevState = currentState;
    if(replan) {
        request.motionCommand = PathTargetCommand{pathTarget};
        Trajectory trajectory = pathTargetPlanner.plan(std::move(request));
        trajectory.setDebugText((std::string("Collect: ") + collectStateToString(currentState)).c_str());
        std::function<double(Point, Point, double)> angleFunction =
                [approachDirection](Point pos, Point vel, double angle) {
            return approachDirection.angle();
        };
        PlanAngles(trajectory, startInstant, angleFunction, rotationConstraints);
        return std::move(trajectory);
    } else {
        return std::move(request.prevTrajectory);
    }
}
}  // namespace Planning
