#include "LineKickPlanner.hpp"
#include <Configuration.hpp>
#include <Geometry2d/Util.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include "EscapeObstaclesPathPlanner.hpp"
#include "RRTPlanner.hpp"
#include "motion/TrapezoidalMotion.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool LineKickPlanner::shouldReplan(const PlanRequest& planRequest) const {
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    const auto& command =
        dynamic_cast<const PivotCommand&>(*planRequest.motionCommand);

    if (!prevPath) {
        return true;
    } else {
        // TODO ashaw37: make this better
        float radius = command.radius;
        auto pivotPoint = command.pivotPoint;
        auto pivotTarget = command.pivotTarget;
        auto endTarget =
            pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
        float targetChange = (prevPath->end().motion.pos - endTarget).mag();

        // if (targetChange > SingleRobotPathPlanner::goalChangeThreshold()) {
        //     return true;
        // }
    }
    return false;
}

std::unique_ptr<Path> LineKickPlanner::run(PlanRequest& planRequest) {
    const float ApproachSpeed = 1.0;

    const float ballAvoidDistance = 0.05;

    auto prevAnglePath =
        dynamic_cast<AngleFunctionPath*>(planRequest.prevPath.get());

    const auto& command =
        dynamic_cast<const LineKickCommand&>(*planRequest.motionCommand);

    const MotionInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    const auto& rotationConstraints = planRequest.constraints.rot;
    auto& obstacles = planRequest.obstacles;
    auto& systemState = planRequest.systemState;
    const auto& ball = systemState.ball;
    const auto& robotConstraints = planRequest.constraints;
    auto& dynamicObstacles = planRequest.dynamicObstacles;

    auto ballObstacles = obstacles;
    const RJ::Time curTime = RJ::now();
    ballObstacles.add(
        make_shared<Circle>(ball.predict(curTime).pos, ballAvoidDistance));
    unique_ptr<InterpolatedPath> prevPath;
    if (prevAnglePath && prevAnglePath->path) {
        prevPath = std::unique_ptr<InterpolatedPath>(
            dynamic_cast<InterpolatedPath*>(prevAnglePath->path.release()));
        if (prevPath->waypoints.size() <= 1) {
            prevPath = nullptr;
        }
    }

    if (!prevPath) {
        finalApproach = false;
    }

    if (finalApproach && prevPath && targetKickPos) {
        RJ::Seconds timeIntoPath = curTime - prevPath->startTime();

        MotionInstant target = prevPath->end().motion;
        RJ::Time time = ball.estimateTimeTo(*targetKickPos);
        auto timeLeft = prevPath->getDuration() - timeIntoPath;

        if (timeLeft < RJ::Seconds(-0.3) || timeLeft > RJ::Seconds(5.0)) {
            finalApproach = false;
            prevPath = nullptr;
        } else if (timeLeft < RJ::Seconds(0)) {
            prevPath->setDebugText("reuse past done " +
                                   QString::number(timeLeft.count()));
            return make_unique<AngleFunctionPath>(
                std::move(prevPath),
                angleFunctionForCommandType(FacePointCommand(command.target)));
        } else {
            RJ::Seconds timeForBall = time - curTime;
            prevPath->slow(timeForBall / timeLeft, timeIntoPath);
            prevPath->setDebugText("reuse final " +
                                   QString::number(timeForBall.count()) + " " +
                                   QString::number(timeLeft.count()));
            return make_unique<AngleFunctionPath>(
                std::move(prevPath),
                angleFunctionForCommandType(FacePointCommand(command.target)));
        }
    }

    if (ball.vel.mag() < 0.2) {
        MotionInstant target(ball.pos);
        target.vel = (command.target - target.pos).normalized(ApproachSpeed);
        auto ballPath = ball.path(curTime);
        unique_ptr<Path> path;
        if (std::abs(target.vel.angleBetween((target.pos - startInstant.pos))) >
            DegreesToRadians(50)) {
            target.pos -=
                target.vel.normalized(ballAvoidDistance * 3.0f + Robot_Radius);

            std::unique_ptr<MotionCommand> command =
                std::make_unique<PathTargetCommand>(target);

            auto request = PlanRequest(systemState, startInstant,
                                       std::move(command), robotConstraints,
                                       std::move(prevPath), ballObstacles,
                                       dynamicObstacles, planRequest.shellID);
            path = rrtPlanner.run(request);
        } else {
            std::unique_ptr<MotionCommand> command =
                std::make_unique<PathTargetCommand>(target);
            auto request =
                PlanRequest(systemState, startInstant, std::move(command),
                            robotConstraints, std::move(prevPath), obstacles,
                            dynamicObstacles, planRequest.shellID);
            path = rrtPlanner.run(request);
        }
        targetKickPos = boost::none;
        path->setDebugText("Slow ball");
        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(command.target)));
    }

    if (prevPath && targetKickPos &&
        prevPath->getDuration() < RJ::Seconds(1.0)) {
        MotionInstant target;
        RJ::Time time = ball.estimateTimeTo(*targetKickPos, &target.pos);

        targetKickPos = target.pos;
        target.vel = (command.target - target.pos).normalized(ApproachSpeed);
        target.pos -= target.vel.normalized(Robot_Radius + Ball_Radius * 2);

        RJ::Seconds timeToHit = time - curTime;
        if (timeToHit > prevPath->getDuration() &&
            timeToHit < RJ::Seconds(2.0)) {
            vector<Point> points{startInstant.pos, target.pos};
            finalApproach = true;
            auto path =
                RRTPlanner::generatePath(points, obstacles, motionConstraints,
                                         startInstant.vel, target.vel);
            path->setDebugText(
                "FinalPath" + QString::number(path->getDuration().count()) +
                " " + QString::number(timeToHit.count()) + " " +
                QString::number(time.time_since_epoch().count()));
            path->slow(timeToHit / path->getDuration());
            return make_unique<AngleFunctionPath>(
                std::move(path),
                angleFunctionForCommandType(FacePointCommand(command.target)));
        }
    }

    QString debug = "";
    for (auto t = RJ::Seconds(0); t < RJ::Seconds(6); t += RJ::Seconds(0.1)) {
        MotionInstant ballNow = ball.predict(curTime + t);
        MotionInstant target(ballNow.pos);
        targetKickPos = target.pos;
        target.vel = (command.target - target.pos).normalized(ApproachSpeed);
        target.pos -= target.vel.normalized(Robot_Radius + Ball_Radius * 2);

        vector<Point> points{startInstant.pos, target.pos};
        if (std::abs(target.vel.angleBetween((target.pos - startInstant.pos))) >
            DegreesToRadians(60)) {
            auto dist = target.pos.distTo(startInstant.pos);
            points = {
                startInstant.pos,
                target.pos -
                    target.vel.normalized(min(
                        dist / 2, (double)Robot_Radius * 2 + Ball_Radius * 2)),
                target.pos};

            obstacles.add(
                make_shared<Circle>(ballNow.pos, Robot_Radius + Ball_Radius));
            debug = "additional ";
        }
        if (Geometry2d::Segment(ball.pos, target.pos).distTo(startInstant.pos) <
            Robot_Radius) {
            debug = "meh";
            // break;
        }
        auto path = RRTPlanner::generatePath(
            points, obstacles, motionConstraints, startInstant.vel, target.vel);
        RJ::Seconds hitTime;

        if (path) {
            if (path->getDuration() <= t) {
                if (path->hit(obstacles, RJ::Seconds::zero(), &hitTime)) {
                    continue;
                }
                float multiplier = t / path->getDuration();
                path->setDebugText(
                    "FoundPath" + debug +
                    QString::number(path->getDuration().count()));
                // if (path->getDuration() < 0.7) {
                //    path->slow(multiplier);

                //    path->setDebugText(debug +
                //    QString::number(path->getDuration()) + " slow");
                //}
                return make_unique<AngleFunctionPath>(
                    std::move(path), angleFunctionForCommandType(
                                         FacePointCommand(command.target)));
            }
        }
    }

    MotionInstant ballNow = ball.predict(curTime);
    MotionInstant target(ballNow.pos);
    target.vel = (command.target - target.pos).normalized(ApproachSpeed);
    target.pos -= target.vel.normalized(Robot_Radius * 3);

    // obstacles.add(make_shared<Circle>(target.pos, ballAvoidDistance));
    auto ballPath = ball.path(curTime);
    dynamicObstacles.push_back(DynamicObstacle(ballPath.get(), Ball_Radius));
    std::unique_ptr<MotionCommand> rrtCommand =
        std::make_unique<PathTargetCommand>(target);

    auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                               robotConstraints, std::move(prevPath), obstacles,
                               dynamicObstacles, planRequest.shellID);
    auto path = rrtPlanner.run(request);
    path->setDebugText("Gives ups");

    targetKickPos = boost::none;
    return make_unique<AngleFunctionPath>(
        std::move(path),
        angleFunctionForCommandType(FacePointCommand(command.target)));
}
}
