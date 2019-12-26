#include "CollectPathPlanner.hpp"
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <Geometry2d/Pose.hpp>
#include "Configuration.hpp"
#include <vector>
#include "Robot.hpp"

namespace Planning {

    REGISTER_CONFIGURABLE(CollectPathPlanner);

    ConfigDouble *CollectPathPlanner::_ballSpeedApproachDirectionCutoff;
    ConfigDouble *CollectPathPlanner::_approachAccelScalePercent;
    ConfigDouble *CollectPathPlanner::_controlAccelScalePercent;
    ConfigDouble *CollectPathPlanner::_approachDistTarget;
    ConfigDouble *CollectPathPlanner::_touchDeltaSpeed;
    ConfigDouble *CollectPathPlanner::_velocityControlScale;
    ConfigDouble *CollectPathPlanner::_distCutoffToControl;
    ConfigDouble *CollectPathPlanner::_velCutoffToControl;
    ConfigDouble *CollectPathPlanner::_distCutoffToApproach;
    ConfigDouble *CollectPathPlanner::_stopDistScale;
    ConfigDouble *CollectPathPlanner::_targetPointAveragingGain;

    void CollectPathPlanner::createConfiguration(Configuration *cfg) {
        //todo(Ethan) fix these
        _ballSpeedApproachDirectionCutoff = new ConfigDouble(
                cfg, "Capture/Collect/ballSpeedApproachDirectionCutoff", 0.1);
        _approachAccelScalePercent =
                new ConfigDouble(cfg,
                                 "Capture/Collect/approachAccelScalePercent",
                                 0.7);
        _controlAccelScalePercent =
                new ConfigDouble(cfg,
                                 "Capture/Collect/controlAccelScalePercent",
                                 0.8);
        _approachDistTarget =
                new ConfigDouble(cfg, "Capture/Collect/approachDistTarget",
                                 0.04);
        _touchDeltaSpeed =
                new ConfigDouble(cfg, "Capture/Collect/touchDeltaSpeed", 0.1);
        _velocityControlScale =
                new ConfigDouble(cfg, "Capture/Collect/velocityControlScale",
                                 1);
        _distCutoffToControl =
                new ConfigDouble(cfg, "Capture/Collect/distCutoffToControl",
                                 0.05);
        _velCutoffToControl =
                new ConfigDouble(cfg, "Capture/Collect/velCutoffToControl", 1);
        _distCutoffToApproach =
                new ConfigDouble(cfg, "Capture/Collect/distCutoffToApproach",
                                 0.3);
        _stopDistScale = new ConfigDouble(cfg, "Capture/Collect/stopDistScale",
                                          1);
        _targetPointAveragingGain =
                new ConfigDouble(cfg,
                                 "Capture/Collect/targetPointAveragingGain",
                                 0.8);
    }

    std::vector<CollectPathPlanner::CollectState> CollectPathPlanner::collectStates{
            Num_Shells, CollectState::Course};

    using Geometry2d::Point;
    using Geometry2d::Pose;
    using Geometry2d::Twist;

    // todo(Ethan) use Settle Planner if the ball is moving directly toward us
    // Collect Planner will plan a Bezier that intersects the ball, resulting in
    // smacking the ball along the way to the target
    Trajectory CollectPathPlanner::plan(PlanRequest &&request) {
        const Ball &ball = request.context->state.ball;
        RotationConstraints rotationConstraints = request.constraints.rot;
        MotionConstraints &motionConstraints = request.constraints.mot;
        RobotInstant startInstant = request.start;
        Point startPoint = startInstant.pose.position();

        // Initialize the filter to the ball velocity so there's less ramp up
        if (!averageBallVel) {
            averageBallVel = ball.vel;
        } else {
            averageBallVel = applyLowPassFilter(*averageBallVel, ball.vel,
                                                1 - *_targetPointAveragingGain);
        }
        const unsigned int shell = request.shellID;
        CollectState &state = collectStates[shell];
        Point approachDirection =
                (ball.vel.mag() < *_ballSpeedApproachDirectionCutoff ||
                 state == CollectState::Control) ?
                (ball.pos - startPoint).norm() : averageBallVel->norm();
        CollectState prevState = collectStates[shell];
        processStateTransitions(ball, *request.context->state.self[shell],
                                startInstant, state);
        switch (state) {
            case CollectState::Course:
                return course(std::move(request), approachDirection);
            case CollectState::Fine:
                return fine(std::move(request), approachDirection);
            case CollectState::Control:
                if (prevState == CollectState::Control &&
                    !request.prevTrajectory.empty()) {
                    // the ball typically bounces after Fine,
                    // then rolls back into us during Control
                    // so this tells the robot to wait for the ball
                    return std::move(request.prevTrajectory);
                }
                std::cout << "Control Path Replan" << std::endl;
                return control(std::move(request), approachDirection);
            default:
                debugThrow("Invalid CollectState");
                return Trajectory{{}};
        }
    }

    Trajectory
    CollectPathPlanner::course(PlanRequest &&request, Point approachDirection) {
        RotationConstraints rotationConstraints = request.constraints.rot;
        RobotInstant startInstant = request.start;
        const Ball &ball = request.context->state.ball;

        Point targetPoint =
                ball.pos -
                approachDirection * (*_approachDistTarget + Robot_MouthRadius);
        Point targetVel = fineVelocity(approachDirection);
        RobotInstant pathTarget{
                Pose{targetPoint, approachDirection.angle()},
                Twist{targetVel, 0}, RJ::now()};
        request.motionCommand = PathTargetCommand{pathTarget};
        Trajectory trajectory = pathTargetPlanner.plan(std::move(request));
        PlanAngles(trajectory, startInstant, AngleFns::tangent,
                   rotationConstraints);
        trajectory.setDebugText("Course");
        return std::move(trajectory);
    }

    Trajectory
    CollectPathPlanner::fine(PlanRequest &&request, Point approachDirection) {
        MotionConstraints &motionConstraints = request.constraints.mot;
        RotationConstraints rotationConstraints = request.constraints.rot;
        RobotInstant startInstant = request.start;

        motionConstraints.maxAcceleration *= *_approachAccelScalePercent;
        const Ball &ball = request.context->state.ball;
        Point targetPoint = ball.pos - approachDirection * Robot_MouthRadius;
        Point targetVel = fineVelocity(approachDirection);
        motionConstraints.maxSpeed = std::min(motionConstraints.maxSpeed,
                                              targetVel.mag());
        RobotInstant pathTarget{
                Pose{targetPoint, approachDirection.angle()},
                Twist{targetVel, 0},
                RJ::now()};
        request.motionCommand = PathTargetCommand{pathTarget};
        Trajectory trajectory = pathTargetPlanner.plan(std::move(request));
        PlanAngles(trajectory, startInstant, AngleFns::facePoint(ball.pos),
                   rotationConstraints);
        trajectory.setDebugText("Fine");
        return std::move(trajectory);
    }

    Trajectory CollectPathPlanner::control(PlanRequest &&request,
                                           Point approachDirection) {
        MotionConstraints &motionConstraints = request.constraints.mot;
        RotationConstraints rotationConstraints = request.constraints.rot;
        RobotInstant startInstant = request.start;

        motionConstraints.maxAcceleration *= *_controlAccelScalePercent;
        motionConstraints.maxSpeed = std::min(motionConstraints.maxSpeed,
                                              startInstant.velocity.linear().mag());
        //kinematics: v^2 = v0^2 + 2 * a * deltaX
        double stoppingDist = std::pow(motionConstraints.maxSpeed, 2) /
                              (2 * motionConstraints.maxAcceleration);
        //decelerate to stop
        Point targetPoint =
                startInstant.pose.position() + approachDirection * stoppingDist;
        RobotInstant pathTarget{
                Pose{targetPoint, approachDirection.angle()},
                Twist{}, RJ::now()};
        request.motionCommand = PathTargetCommand{pathTarget};
        Trajectory trajectory = pathTargetPlanner.plan(std::move(request));
        PlanAngles(trajectory, startInstant, AngleFns::faceAngle(approachDirection.angle()),
                   rotationConstraints);
        trajectory.setDebugText("Control");
        return std::move(trajectory);
    }

    void CollectPathPlanner::processStateTransitions(const Ball &ball,
                                                     const OurRobot &robot,
                                                     const RobotInstant &startInstant,
                                                     CollectState &state) {
        Point startPoint = startInstant.pose.position();
        // Do the transitions
        float dist = startPoint.distTo(ball.pos) - Robot_MouthRadius;
        float speedDiff =
                startInstant.velocity.linear().distTo(*averageBallVel) -
                *_touchDeltaSpeed;
        if (dist > *_approachDistTarget + Robot_MouthRadius) {
            //Start at Course
            state = CollectState::Course;
        } else if (state == CollectState::Course) {
            // Course --> Fine
            state = CollectState::Fine;
        }
        if (dist < *_distCutoffToControl && speedDiff < *_velCutoffToControl &&
            state == CollectState::Fine && robot.hasBall()) {
            // Fine --> Control
            state = CollectState::Control;
        }
        //TODO do something with ball sense?
    }

}  // namespace Planning
