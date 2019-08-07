#include "LineKickPlanner.hpp"
#include <Configuration.hpp>
#include <Geometry2d/Util.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include "EscapeObstaclesPathPlanner.hpp"
#include "RRTPlanner.hpp"
#include "motion/TrapezoidalMotion.hpp"
#include "CompositePath.hpp"
#include "MotionInstant.hpp"

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
    const float ApproachSpeed = 0.25;

    const float ballAvoidDistance = 0.05;

    auto prevAnglePath =
        dynamic_cast<AngleFunctionPath*>(planRequest.prevPath.get());

    const auto& command =
        dynamic_cast<const LineKickCommand&>(*planRequest.motionCommand);

    const MotionInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    const auto& rotationConstraints = planRequest.constraints.rot;
    auto& obstacles = planRequest.obstacles;
    auto context = planRequest.context;
    auto& systemState = context->state;
    const auto& ball = systemState.ball;
    const auto& robotConstraints = planRequest.constraints;
    auto& dynamicObstacles = planRequest.dynamicObstacles;

    auto ballObstacles = obstacles;
    const RJ::Time curTime = RJ::now();
    ballObstacles.add(
        make_shared<Circle>(ball.predict(curTime).pos, ballAvoidDistance));
    unique_ptr<Path> prevPath;
    if (prevAnglePath && prevAnglePath->path) {
        prevPath = std::move(prevAnglePath->path);
    }

    if (!prevPath) {
        finalApproach = false;
    }

    if (finalApproach && prevPath && targetKickPos) {
        RJ::Seconds timeIntoPath = curTime - prevPath->startTime();

        MotionInstant target = prevPath->end().motion;
        RJ::Time time = ball.estimateTimeTo(*targetKickPos);
        auto timeLeft = prevPath->getSlowedDuration() - timeIntoPath;

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
            prevPath->slow(timeLeft / timeForBall, timeIntoPath);
            prevPath->setDebugText("reuse final slow " +
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
            DegreesToRadians(10)) {
            target.pos -=
                target.vel.normalized(ballAvoidDistance * 2 + Robot_Radius);
            if (prevPath &&
                target.pos.distTo(prevPath->end().motion.pos) <
                    SingleRobotPathPlanner::goalPosChangeThreshold() &&
                reusePathCount < 20) {
                target.vel = prevPath->end().motion.vel;
                reusePathCount++;
            } else {
                reusePathCount = 0;
            }

            auto command = std::make_unique<PathTargetCommand>(target);
            auto request = PlanRequest(context, startInstant,
                                       std::move(command), robotConstraints,
                                       std::move(prevPath), ballObstacles,
                                       dynamicObstacles, planRequest.shellID);
            path = rrtPlanner.run(request);
        } else {
            if (prevPath &&
                target.pos.distTo(prevPath->end().motion.pos) <
                    SingleRobotPathPlanner::goalPosChangeThreshold() &&
                reusePathCount < 20) {
                target.vel = prevPath->end().motion.vel;
                reusePathCount++;
            } else {
                reusePathCount = 0;
            }

            target.pos += target.vel.normalized(Robot_Radius);
            auto command = std::make_unique<PathTargetCommand>(target);
            auto request =
                PlanRequest(context, startInstant, std::move(command),
                            robotConstraints, std::move(prevPath), obstacles,
                            dynamicObstacles, planRequest.shellID);

            path = rrtPlanner.run(request);
        }
        targetKickPos = std::nullopt;
        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(command.target)));
    }

    if (prevPath && targetKickPos) {
        auto timeInto = RJ::now() - prevPath->startTime();
        auto timeLeft = prevPath->getDuration() - timeInto;

        MotionInstant target;
        RJ::Time time = ball.estimateTimeTo(*targetKickPos, &target.pos);
        RJ::Seconds timeToHit = time - curTime;
        if (timeLeft < RJ::Seconds(1.0)) {
            target.vel =
                (command.target - target.pos).normalized(ApproachSpeed);
            target.pos -= target.vel.normalized(Robot_Radius + Ball_Radius * 2);

            if (true) {
                vector<Point> points{startInstant.pos, target.pos};
                finalApproach = true;
                auto path = RRTPlanner::generatePath(
                    points, obstacles, motionConstraints, startInstant.vel,
                    target.vel);

                if (path) {
                    path->setDebugText(
                        "FinalPath" +
                        QString::number(path->getSlowedDuration().count()) +
                        " " + QString::number(timeToHit.count()) + " " +
                        QString::number(time.time_since_epoch().count()));

                    return make_unique<AngleFunctionPath>(
                        std::move(path), angleFunctionForCommandType(
                                             FacePointCommand(command.target)));
                }
            }
        }

        if (!prevPath->hit(obstacles, timeInto) &&
            timeLeft - 1000ms < timeToHit && reusePathCount < 10) {
            reusePathCount++;
            Point nearPoint;
            prevPath->setDebugText("Reuse prevPath");
            if (ball.estimateTimeTo(prevPath->end().motion.pos, &nearPoint) >=
                RJ::now() + timeLeft - 1000ms) {
                return make_unique<AngleFunctionPath>(
                    std::move(prevPath), angleFunctionForCommandType(
                                             FacePointCommand(command.target)));
            }
        }
    }

    std::unique_ptr<Path> partialPath = nullptr;
    RJ::Seconds partialPathTime = 0ms;
    auto tmpStartInstant = startInstant;
    const auto partialReplanLeadTime = RRTPlanner::getPartialReplanLeadTime();
    if (prevPath) {
        auto timeInto = RJ::now() - prevPath->startTime();
        if (timeInto < prevPath->getDuration() - partialReplanLeadTime * 2) {
            partialPath =
                prevPath->subPath(0ms, timeInto + partialReplanLeadTime);
            partialPathTime = partialReplanLeadTime;
            tmpStartInstant = partialPath->end().motion;
        }
    }

    QString debug = "";
    for (auto t = RJ::Seconds(0.4); t < RJ::Seconds(6); t += RJ::Seconds(0.2)) {
        auto tempObstacles = obstacles;
        MotionInstant ballNow = ball.predict(curTime + t);
        MotionInstant target(ballNow.pos);
        targetKickPos = target.pos;
        target.vel = (command.target - target.pos).normalized(ApproachSpeed);
        target.pos -= target.vel.normalized(Robot_Radius + Ball_Radius * 2);

        vector<Point> points{tmpStartInstant.pos, target.pos};
        if (std::abs(target.vel.angleBetween(
                (target.pos - tmpStartInstant.pos))) > DegreesToRadians(60)) {
            auto dist = target.pos.distTo(tmpStartInstant.pos);
            points = {tmpStartInstant.pos,
                      target.pos -
                          target.vel.normalized(Robot_Radius * 2.0 +
                                                Ball_Radius * 2.0),
                      target.pos};

            // tempObstacles.add(
            //    make_shared<Circle>(ballNow.pos, Robot_Radius + Ball_Radius));
            // debug = "additional ";
        }
        if (Geometry2d::Segment(ball.pos, target.pos)
                .distTo(tmpStartInstant.pos) < Robot_Radius) {
            debug = "meh";
            // break;
        }

        std::unique_ptr<Path> path =
            RRTPlanner::generatePath(points, tempObstacles, motionConstraints,
                                     tmpStartInstant.vel, target.vel);
        RJ::Seconds hitTime;

        if (path) {
            if (path->getDuration() + partialPathTime <= t) {
                if (path->hit(tempObstacles, RJ::Seconds::zero(), &hitTime)) {
                    continue;
                }

                if (partialPath) {
                    path = make_unique<CompositePath>(std::move(partialPath),
                                                      std::move(path));
                    path->setStartTime(prevPath->startTime());
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
                reusePathCount = 0;
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

    auto request = PlanRequest(context, startInstant, std::move(rrtCommand),
                               robotConstraints, std::move(prevPath), obstacles,
                               dynamicObstacles, planRequest.shellID);
    auto path = rrtPlanner.run(request);
    path->setDebugText("Gives ups");

    targetKickPos = std::nullopt;
    return make_unique<AngleFunctionPath>(
        std::move(path),
        angleFunctionForCommandType(FacePointCommand(command.target)));
}
}
