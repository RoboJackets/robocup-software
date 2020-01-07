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
#include "planning/trajectory/RRTUtil.hpp"

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
    std::vector<std::optional<Point>> SettlePathPlanner::averageGoalPoints(Num_Shells);

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

    // Try find best point to intercept using brute force method
    // where we check ever X distance along the ball velocity vector
    //
    // Disallow points outside the field
    Point SettlePathPlanner::bruteForceGoal(const PlanRequest& request) {
        double distance = *_searchStartDist;
        const Ball& ball = request.context->state.ball;
        Point targetPoint;
        double targetAngle = fixAngleRadians(averageBallVel->angle() + M_PI);
        const Geometry2d::Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
        Point startPoint = request.start.pose.position();
        RJ::Time curTime = RJ::now();
        Trajectory path{{}};
        int its = 0;
        while (distance < *_searchEndDist) {
            Point futureBallPoint =
                    ball.pos + averageBallVel->normalized(distance);
            RJ::Seconds futureBallTime = ball.estimateTimeTo(futureBallPoint, &targetPoint) -
                                         curTime;
            if(!fieldRect.containsPoint(targetPoint)) {
                targetPoint = projectPointIntoField(targetPoint, fieldRect, ball.pos);
                break;
            }
            RobotInstant pathTarget{Pose{targetPoint, targetAngle},
                                    {}, RJ::now()};
            path = RRTTrajectory(request.start, pathTarget, request.constraints.mot, request.static_obstacles);
            if(!path.empty() && path.duration() * *_interceptBufferTime <= futureBallTime) {
                break;
            }
            distance += *_searchIncDist;
            its++;
        }
        printf("brute force took %.3f sec, its: %d\n", RJ::Seconds(RJ::now()-curTime).count(), its);
        return targetPoint;
    }

    constexpr double rotAccelScale = 0.8; // range: [0,1]
    Trajectory SettlePathPlanner::intercept(PlanRequest &&request) {
        const Ball& ball = request.context->state.ball;
        request.static_obstacles.add(std::make_shared<Geometry2d::Circle>(ball.pos, Robot_Radius + Ball_Radius));
        RobotInstant startInstant = request.start;
        RotationConstraints rotationConstraints = request.constraints.rot;
        std::optional<Point>& averageGoalPoint = averageGoalPoints[request.shellID];
        Point jitteryGoalPoint = bruteForceGoal(request);
        if (averageGoalPoint) {
            averageGoalPoint = jitteryGoalPoint;
        } else {
            averageGoalPoint = applyLowPassFilter(*averageGoalPoint,
                                                    jitteryGoalPoint,
                                                    *_targetPointGain);
        }
        double goalAngle = fixAngleRadians(averageBallVel->angle() + M_PI);
        RobotInstant goalInstant{Pose{*averageGoalPoint, goalAngle},
                                 Twist{*_ballSpeedPercentForDampen *
                                       *averageBallVel, 0}, RJ::now()};
        //now use the PathTargetPlanner's mostly reliable replan strategy
        PlanRequest pathTargetRequest{request.context, startInstant,
                                      PathTargetCommand{goalInstant},
                                      request.constraints,
                                      std::move(request.prevTrajectory),
                                      request.static_obstacles,
                                      request.dynamic_obstacles,
                                      request.shellID};
        Trajectory result = pathTargetPlanner.plan(
                std::move(pathTargetRequest));
        // Angle Planning strategy: steer tangent to the path for as long as
        // possible, then turn at max speed at the end of the path
        // the idea is that the robots move faster when tangent to their path
        // not sure if this is correct tho. TODO: test this irl
        double rotationAngle = fixAngleRadians(
                goalInstant.pose.heading() - startInstant.pose.heading());
        RJ::Seconds rotationTime{
                Trapezoidal::getTime(rotationAngle, rotationAngle,
                                     rotationConstraints.maxSpeed,
                                     rotationConstraints.maxAccel * rotAccelScale,
                                     startInstant.velocity.angular(),
                                     0)};
        RJ::Time rotStartTime = result.begin_time() + std::max(RJ::Seconds(0), result.duration()-rotationTime);
        AngleFunction angleFunction =
                [=](const RobotInstant& instant) {
                    static bool begin = false;
                    if (instant.stamp > rotStartTime - 0.001s) {
                        begin = true;
                    }
                    if(begin) {
                        return goalInstant.pose.heading();
                    } else {
                        return instant.velocity.linear().angle() + M_PI;
                    }
                };
        PlanAngles(result, startInstant, angleFunction, rotationConstraints);
        result.setDebugText("Intercept");
        assert(!result.empty());
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
        PlanAngles(result, startInstant, AngleFns::faceAngle(targetAngle),
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
