#include "SettlePathPlanner.hpp"

#include <algorithm>
#include <cmath>

#include "Configuration.hpp"
#include "Constants.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/PathSmoothing.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "Robot.hpp"
#include "motion/TrapezoidalMotion.hpp"

namespace Planning {

    REGISTER_CONFIGURABLE(SettlePathPlanner);

    ConfigDouble *SettlePathPlanner::_ballSpeedPercentForDampen;
    ConfigDouble *SettlePathPlanner::_searchStartDist;
    ConfigDouble *SettlePathPlanner::_searchEndDist;
    ConfigDouble *SettlePathPlanner::_searchIncDist;
    ConfigDouble *SettlePathPlanner::_interceptBufferTime;
    ConfigDouble *SettlePathPlanner::_targetPointGain;
    ConfigDouble *SettlePathPlanner::_ballVelGain;
    ConfigDouble *SettlePathPlanner::_shortcutDist;
    ConfigDouble *SettlePathPlanner::_maxBallVelForPathReset;
    ConfigDouble *SettlePathPlanner::_maxBallAngleForReset;
    ConfigDouble *SettlePathPlanner::_maxBounceAngle;

    void SettlePathPlanner::createConfiguration(Configuration *cfg) {
        _ballSpeedPercentForDampen = new ConfigDouble(
                cfg, "Capture/Settle/ballSpeedPercentForDampen", 0.1);  // %
        _searchStartDist =
                new ConfigDouble(cfg, "Capture/Settle/searchStartDist",
                                 0.0);  // m
        _searchEndDist =
                new ConfigDouble(cfg, "Capture/Settle/searchEndDist",
                                 7.0);  // m
        _searchIncDist =
                new ConfigDouble(cfg, "Capture/Settle/searchIncDist",
                                 0.2);  // m
        _interceptBufferTime =
                new ConfigDouble(cfg, "Capture/Settle/interceptBufferTime",
                                 0.0);  // %
        _targetPointGain = new ConfigDouble(cfg,
                                            "Capture/Settle/targetPointGain",
                                            0.5);  // gain between 0 and 1
        _ballVelGain = new ConfigDouble(cfg, "Capture/Settle/ballVelGain",
                                        0.5);  // gain between 0 and 1
        _shortcutDist = new ConfigDouble(cfg, "Capture/Settle/shortcutDist",
                                         Robot_Radius);  // m
        _maxBallVelForPathReset = new ConfigDouble(
                cfg, "Capture/Settle/maxBallVelForPathReset", 2);  // m/s
        _maxBallAngleForReset = new ConfigDouble(
                cfg, "Capture/Settle/maxBallAngleForReset", 20);  // Deg
        _maxBounceAngle =
                new ConfigDouble(cfg, "Capture/Settle/maxBounceAngle",
                                 45);  // Deg
    }

    using Geometry2d::Point;
    using Geometry2d::Pose;
    using Geometry2d::Twist;

    std::vector<SettlePathPlanner::SettleState> SettlePathPlanner::settleStates{
            Num_Shells, SettleState::Intercept};

    std::vector<RJ::Time> SettlePathPlanner::planTimes{Num_Shells,
                                                       RJ::now() - 60s};

    Trajectory SettlePathPlanner::plan(PlanRequest &&request) {
        SystemState &systemState = request.context->state;
        const unsigned int shell = request.shellID;
        const Ball &ball = request.context->state.ball;
        RobotInstant startInstant = request.start;
        const MotionConstraints &motionConstraints = request.constraints.mot;
        const RotationConstraints &rotationConstraints = request.constraints.rot;

        // Initialize the filter to the ball velocity so there's less ramp up
        bool ballAngleJumped = averageBallVel->angleBetween(ball.vel) >
                               *_maxBallAngleForReset * M_PI / 180.0;
        bool ballVelJumped =
                averageBallVel->distTo(ball.vel) > *_maxBallVelForPathReset;
        if (!averageBallVel || ballAngleJumped || ballVelJumped) {
            averageBallVel = ball.vel;
        } else {
            averageBallVel = applyLowPassFilter(*averageBallVel, ball.vel,
                                                *_ballVelGain);
        }

        SettleState& state = settleStates[shell];
        SettleState prevState = state;
        processStateTransitions(ball, *request.context->state.self[shell],
                                startInstant, state);
        RJ::Time prevTime = planTimes[shell];
        switch (state) {
            case SettleState::Intercept:
                if (prevState == SettleState::Intercept &&
                    RJ::Seconds(RJ::now() - prevTime) < 0.2s &&
                    !request.prevTrajectory.empty()) {
                    return std::move(request.prevTrajectory);
                }
                planTimes[shell] = RJ::now();
                return intercept(std::move(request));
            case SettleState::Dampen:
                if(prevState == SettleState::Dampen && !request.prevTrajectory.empty()) {
                    return std::move(request.prevTrajectory);
                }
                planTimes[shell] = RJ::now();
                return dampen(std::move(request));
            default:
                debugThrow("Invalid SettleState");
                return Trajectory{{}};
        }
    }

    Trajectory SettlePathPlanner::intercept(PlanRequest &&request) {
        RobotInstant startInstant = request.start;
        RotationConstraints rotationConstraints = request.constraints.rot;
        const Ball &ball = request.context->state.ball;
        const Geometry2d::Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
        Trajectory result = request.prevTrajectory.empty() ? Trajectory{{startInstant}} : std::move(request.prevTrajectory);
        // Try find best point to intercept using brute force method
        // where we check ever X distance along the ball velocity vector
        //
        // Disallow points outside the field
        double distance = _searchStartDist->value();
        Point targetPoint = ball.pos;
        Point targetVel = *averageBallVel * *_ballSpeedPercentForDampen;
        double targetAngle = fixAngleRadians(averageBallVel->angle() + M_PI);
        RJ::Seconds minPathTime = 1000000s;
        while (distance < *_searchEndDist) {
            Point futureBallPoint =
                    ball.pos + averageBallVel->normalized(distance);
            if (fieldRect.containsPoint(futureBallPoint)) {
                RJ::Seconds futureBallTime = ball.estimateTimeTo(futureBallPoint, &targetPoint) -
                        RJ::now();
                RobotInstant pathTarget{Pose{targetPoint, targetAngle},
                                        Twist{targetVel, 0}, RJ::now()};
                PlanRequest pathTargetRequest{request.context, startInstant,
                                              PathTargetCommand{pathTarget},
                                              request.constraints,
                                              Trajectory{{}}, request.static_obstacles, request.dynamic_obstacles,
                                              request.shellID};
                Trajectory path = pathTargetPlanner.plan(std::move(pathTargetRequest));
                if(path.duration() < minPathTime) {
                    minPathTime = path.duration();
                    result = std::move(path);
                }
            }
            distance += *_searchIncDist;
        }
        // Angle Planning strategy: steer tangent to the path for as long as
        // possible, then turn at max speed at the end of the path
        double rotationAngle = fixAngleRadians(
                targetAngle - startInstant.pose.heading());
        RJ::Seconds rotationTime{
                Trapezoidal::getTime(rotationAngle, rotationAngle,
                                     rotationConstraints.maxSpeed,
                                     rotationConstraints.maxAccel,
                                     startInstant.velocity.angular(),
                                     0)};
        //todo(Ethan) probably change angle planning to be in terms of Time so this doesn't have to be so gross
        RobotInstant beginRotInstant =
                result.duration() < rotationTime ? result.first()
                                                 : *result.evaluate(
                        result.duration() - rotationTime);
        double beginRotDist = beginRotInstant.pose.position().distTo(
                targetPoint);
        std::function<double(Point, Point, double)> angleFunction =
                [=](Point pos, Point vel, double angle) {
                    static bool begin = false;
                    if (pos.distTo(targetPoint) < beginRotDist) begin = true;
                    return begin ? targetAngle : (vel.angle() + M_PI);
                };
        PlanAngles(result, startInstant, angleFunction,
                   rotationConstraints);
        result.setDebugText("Intercept");
        return std::move(result);
    }

    Trajectory SettlePathPlanner::dampen(PlanRequest &&request) {
        const RobotInstant &startInstant = request.start;
        MotionConstraints &motionConstraints = request.constraints.mot;
        const RotationConstraints &rotationConstraints = request.constraints.rot;
        const Ball &ball = request.context->state.ball;
        double currentSpeed = startInstant.velocity.linear().mag();
        motionConstraints.maxSpeed = std::min({motionConstraints.maxSpeed,
                                               averageBallVel->mag() *
                                               *_ballSpeedPercentForDampen,
                                               currentSpeed});
        currentSpeed = std::min(currentSpeed, motionConstraints.maxSpeed);
        double stoppingDistance = currentSpeed * currentSpeed /
                                  (2 * motionConstraints.maxAcceleration);
        Point targetPoint =
                ball.pos + averageBallVel->normalized(stoppingDistance);
        double targetAngle = fixAngleRadians(averageBallVel->angle() + M_PI);
        RobotInstant pathTarget{Pose{targetPoint, targetAngle}, Twist{},
                                RJ::now()};
        PlanRequest pathTargetRequest{request.context, startInstant,
                                      PathTargetCommand{pathTarget},
                                      request.constraints, Trajectory{{}},
                                      request.static_obstacles, request.dynamic_obstacles, request.shellID};
        Trajectory result = pathTargetPlanner.plan(
                std::move(pathTargetRequest));
        std::function<double(Point, Point, double)> angleFunction =
                [=](Point pos, Point vel, double angle) {
                    return targetAngle;
                };
        PlanAngles(result, startInstant, angleFunction,
                   rotationConstraints);
        result.setDebugText("Dampen");
        assert(!result.empty());
        return std::move(result);
    }

    void SettlePathPlanner::processStateTransitions(const Ball &ball,
                                                    const OurRobot &robot,
                                                    const RobotInstant &currentInstant,
                                                    SettleState &state) {
        Geometry2d::Line ballLine{ball.pos, ball.pos + *averageBallVel};
        Point currentPoint = currentInstant.pose.position();
        double angle = averageBallVel->angleBetween(currentPoint - ball.pos);
        double distance = ballLine.distTo(currentPoint);
        if (angle < M_PI / 2.0 && distance < Robot_MouthWidth / 2.0 && ball.pos.distTo(currentPoint) < Robot_MouthRadius + 0.04) {
            state = SettleState::Dampen;
        }
        //todo(Ethan) fix this?
        if (ball.pos.distTo(currentPoint) > 2 * Robot_Radius + Ball_Radius &&
            averageBallVel->mag() > 0.2 && angle > M_PI / 2) {
            state = SettleState::Intercept;
        }
    }
}  // namespace Planning
