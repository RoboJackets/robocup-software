#include "CollectPlanner.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/planner/PlanRequest.hpp"
namespace Planning {
ConfigDouble* CollectPlanner::_bufferDistBeforeContact;
ConfigDouble* CollectPlanner::_bufferDistAfterContact;
ConfigDouble* CollectPlanner::_ballSpeedApproachDirectionCutoff;
ConfigDouble* CollectPlanner::_touchDeltaSpeed;
ConfigDouble* CollectPlanner::_targetChangeThreshold;
ConfigDouble* CollectPlanner::_controlAccelScalePercent;
ConfigDouble* CollectPlanner::_distCutoffToControl;

REGISTER_CONFIGURABLE(CollectPlanner);

void CollectPlanner::createConfiguration(Configuration* cfg) {
    _bufferDistBeforeContact = new ConfigDouble(cfg, "Capture/Collect/bufferDistBeforeContact");
    _bufferDistAfterContact = new ConfigDouble(cfg, "Capture/Collect/bufferDistAfterContact");
    _ballSpeedApproachDirectionCutoff = new ConfigDouble(cfg, "Capture/Collect/ballSpeedApproachDirectionCutoff");
    _touchDeltaSpeed = new ConfigDouble(cfg, "Capture/Collect/touchDeltaSpeed");
    _targetChangeThreshold = new ConfigDouble(cfg, "Capture/Collect/targetChangeThreshold");
    _controlAccelScalePercent = new ConfigDouble(cfg, "Capture/Collect/controlAccelScalePercent");
    _distCutoffToControl = new ConfigDouble(cfg, "Capture/Collect/distCutoffToControl");
}

using namespace Geometry2d;
Trajectory CollectPlanner::plan(PlanRequest&& request) {
    processStateTransitions(request);
    switch(_collectStates[request.shellID]) {
        case CollectState::Course:
            return course(std::move(request));
        case CollectState::Fine:
            return fine(std::move(request));
        case CollectState::Control:
            return control(std::move(request));
    }
    debugThrow("Error: CollectPlanner state machine failed.");
    return Trajectory{{}};
}
Trajectory CollectPlanner::course(PlanRequest&& request) {
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    const Ball& ball = request.context->state.ball;
    Point approachDir = findApproachDirection(request);
    Point noisyTargetPoint = ball.pos - approachDir * (Ball_Radius + Robot_MouthRadius + *_bufferDistBeforeContact);
    Point noisyTargetVel = approachDir * (ball.vel.dot(approachDir) + *_touchDeltaSpeed);
    auto& targetInstant = _courseTargetInstants[request.shellID];
    if(!targetInstant || targetInstant->pose.position().distTo(noisyTargetPoint) > *_targetChangeThreshold) {
        targetInstant = RobotInstant{Pose{noisyTargetPoint,0}, Twist{noisyTargetVel,0}, RJ::Time{0s}};
    }
    request.motionCommand = PathTargetCommand{*targetInstant};
    request.static_obstacles.add(std::make_shared<Circle>(ball.pos, Ball_Radius));
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::faceAngle(approachDir.angle()), rotationConstraints);
    return std::move(path);
}
Trajectory CollectPlanner::fine(PlanRequest&& request) {
    Point approachDir = findApproachDirection(request);
    const Ball& ball = request.context->state.ball;
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    Point targetPoint = ball.pos - approachDir * (Ball_Radius + Robot_MouthRadius);
    request.constraints.mot.maxSpeed = ball.vel.dot(approachDir) + *_touchDeltaSpeed;
    Point targetVel = request.constraints.mot.maxSpeed * approachDir;
    RobotInstant targetInstant{Pose{targetPoint, 0}, Twist{targetVel, 0}, RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{targetInstant};
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::faceAngle(approachDir.angle()), rotationConstraints);
    return std::move(path);
}
Trajectory CollectPlanner::control(PlanRequest&& request) {
    Point approachDir = findApproachDirection(request);
    RotationConstraints rotationConstraints = request.constraints.rot;
    RobotInstant startInstant = request.start;
    // move with constant negative acceleration until stop
    double accel = request.constraints.mot.maxAcceleration * *_controlAccelScalePercent;
    double stoppingDist = std::pow(startInstant.velocity.linear().mag(), 2) / (2 * accel);
    Point targetPoint = startInstant.pose.position() + stoppingDist * approachDir;
    RobotInstant targetInstant{Pose{targetPoint, 0}, {}, RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{targetInstant};
    request.constraints.mot.maxSpeed = std::min(startInstant.velocity.linear().mag(),
            request.constraints.mot.maxSpeed);
    request.constraints.mot.maxAcceleration = accel;
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::faceAngle(approachDir.angle()), rotationConstraints);
    return std::move(path);
}
void CollectPlanner::processStateTransitions(const PlanRequest& request) {
    const Ball& ball = request.context->state.ball;
    RobotInstant startInstant = request.start;
    double distToBall = (startInstant.pose.position() - ball.pos).mag();
    double distToBallAtContact = Ball_Radius + Robot_MouthRadius;
    CollectState& collectState = _collectStates[request.shellID];
    const double distCutoffToFine = *_bufferDistBeforeContact + 0.0765;
    // course -> fine
    if(collectState == CollectState::Course) {
        if (distToBall < distCutoffToFine + distToBallAtContact) {
            collectState = CollectState::Fine;
        }
    }
    // fine -> control
    if(collectState == CollectState::Fine) {
        if(distToBall < *_distCutoffToControl + distToBallAtContact) {
            collectState = CollectState::Control;
        }
    }
    // backward transitions
    bool fineToCourse = collectState == CollectState::Fine && distToBall > distCutoffToFine + distToBallAtContact + 0.2;
    bool controlToCourse = collectState == CollectState::Control && distToBall > *_distCutoffToControl + distToBallAtContact + 0.072;
    if(fineToCourse || controlToCourse) {
        _courseTargetInstants[request.shellID] = std::nullopt;
        collectState = CollectState::Course;
    }
}
}