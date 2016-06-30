#include "LineKickPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include <Geometry2d/Util.hpp>
#include "RRTPlanner.hpp"
#include "motion/TrapezoidalMotion.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool LineKickPlanner::shouldReplan(
    const SinglePlanRequest& planRequest) const {
    const MotionConstraints& motionConstraints =
        planRequest.robotConstraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    const auto& command = dynamic_cast<const PivotCommand&>(planRequest.cmd);

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

        if (targetChange > SingleRobotPathPlanner::goalChangeThreshold()) {
            // return true;
        }
    }
    return false;
}

std::unique_ptr<Path> LineKickPlanner::run(SinglePlanRequest& planRequest) {
    const float ApproachSpeed = 1.0;

    const float ballAvoidDistance = 0.05;

    auto prevAnglePath = dynamic_cast<AngleFunctionPath*>(planRequest.prevPath.get());

    const auto& command = dynamic_cast<const LineKickCommand&>(planRequest.cmd);

    const MotionInstant& startInstant = planRequest.startInstant;
    const auto& motionConstraints = planRequest.robotConstraints.mot;
    const auto& rotationConstraints = planRequest.robotConstraints.rot;
    auto& obstacles = planRequest.obstacles;
    const auto& systemState = planRequest.systemState;
    const auto& ball = systemState.ball;
    const auto& robotConstraints = planRequest.robotConstraints;
    auto& dynamicObstacles = planRequest.dynamicObstacles;

    auto ballObstacles = obstacles;
    ballObstacles.add(make_shared<Circle>(ball.predict(RJ::timestamp()).pos, ballAvoidDistance));
    unique_ptr<InterpolatedPath> prevPath;
    if (prevAnglePath && prevAnglePath->path) {
        prevPath = std::unique_ptr<InterpolatedPath>(dynamic_cast<InterpolatedPath*>(prevAnglePath->path.release()));
        if (prevPath->waypoints.size() <= 1) {
            prevPath = nullptr;
        }
    }

    if (!prevPath) {
        finalApproach = false;
    }

    if (finalApproach && prevPath && targetKickPos) {
        float timeIntoPath = RJ::TimestampToSecs(RJ::timestamp() - prevPath->startTime());


        MotionInstant target = prevPath->end().motion;
        RJ::Time time = ball.estimateTimeTo(*targetKickPos);
        float timeLeft = prevPath->getDuration() - timeIntoPath;

        if (timeLeft < -0.3 || timeLeft > 5.0) {
            finalApproach = false;
            prevPath = nullptr;
        } else if (timeLeft < 0) {
            prevPath->setDebugText("reuse past done " + QString::number(timeLeft));

            assert(prevPath!=nullptr);
            return make_unique<AngleFunctionPath>(std::move(prevPath), angleFunctionForCommandType(FacePointCommand(command.target)));
        } else {
            float timeForBall = RJ::TimestampToSecs(time - RJ::timestamp());
            prevPath->slow(timeForBall / timeLeft, timeIntoPath);
            prevPath->setDebugText("reuse final " + QString::number(timeForBall) + " " + QString::number(timeLeft));

            assert(prevPath!=nullptr);
            return make_unique<AngleFunctionPath>(std::move(prevPath),
                                                  angleFunctionForCommandType(FacePointCommand(command.target)));
        }
    }

    if (ball.vel.mag() < 0.2) {
        MotionInstant target(ball.pos);
        target.vel = (command.target - target.pos).normalized(ApproachSpeed);
        auto ballPath = ball.path(RJ::timestamp());
        unique_ptr<Path> path;
        if (std::abs(target.vel.angleBetween((target.pos - startInstant.pos))) > DegreesToRadians(50)) {
            target.pos -= target.vel.normalized(ballAvoidDistance * 3.0f + Robot_Radius);
            auto request = SinglePlanRequest(startInstant, PathTargetCommand(target),
                                            robotConstraints, ballObstacles, dynamicObstacles, systemState,
                                            std::move(prevPath));
            path = rrtPlanner.run(request);
        } else {
            auto request = SinglePlanRequest(startInstant, PathTargetCommand(target),
                                            robotConstraints, obstacles, dynamicObstacles, systemState,
                                            std::move(prevPath));
            path = rrtPlanner.run(request);
        }
        targetKickPos = boost::none;
        path->setDebugText("Slow ball");

        assert(path!=nullptr);
        return make_unique<AngleFunctionPath>(std::move(path),
                                              angleFunctionForCommandType(FacePointCommand(command.target)));
    }

    if (prevPath && targetKickPos && prevPath->getDuration() < 1.0) {
        MotionInstant target;
        RJ::Time time = ball.estimateTimeTo(*targetKickPos, &target.pos);

        targetKickPos = target.pos;
        target.vel = (command.target - target.pos).normalized(ApproachSpeed);
        target.pos -= target.vel.normalized(Robot_Radius+Ball_Radius*2);

        auto timeToHit = RJ::TimestampToSecs(time - RJ::timestamp());
        if (timeToHit>prevPath->getDuration() && timeToHit < 2.0) {
            vector<Point> points{startInstant.pos, target.pos};
            finalApproach = true;
            auto path = RRTPlanner::generatePath(points, obstacles, motionConstraints, startInstant.vel, target.vel);
            path->setDebugText("FinalPath" + QString::number(path->getDuration())
                               + " " + QString::number(timeToHit) + " " + QString::number(time));
            path->slow(timeToHit/path->getDuration());

            assert(path!=nullptr);
            return make_unique<AngleFunctionPath>(std::move(path), angleFunctionForCommandType(FacePointCommand(command.target)));
        }
    }

    QString debug = "";
    for (float t=0; t< 4; t+= 0.1) {
        MotionInstant ballNow = ball.predict(RJ::SecsToTimestamp(t) + RJ::timestamp());
        MotionInstant target(ballNow.pos);
        targetKickPos = target.pos;
        target.vel = (command.target - target.pos).normalized(ApproachSpeed);
        target.pos -= target.vel.normalized(Robot_Radius+Ball_Radius*2);

        vector<Point> points{startInstant.pos, target.pos};
        if(std::abs(target.vel.angleBetween((target.pos - startInstant.pos)))>DegreesToRadians(60)) {
            auto dist = target.pos.distTo(startInstant.pos);
            points = {startInstant.pos, target.pos - target.vel.normalized(min(dist/2, Robot_Radius * 2 + Ball_Radius*2)), target.pos};

            obstacles.add(make_shared<Circle>(ballNow.pos, Robot_Radius + Ball_Radius));
            debug = "additional ";
        }
        if (Geometry2d::Segment(ball.pos, target.pos).distTo(startInstant.pos) < Robot_Radius) {
            debug = "meh";
            break;
        }
        auto path = RRTPlanner::generatePath(points, obstacles, motionConstraints, startInstant.vel, target.vel);
        float hitTime;

        if (path) {
            if (path->getDuration() <= t) {
                if (path->hit(obstacles, hitTime, 0)) {
                    continue;
                }
                float multiplier = t/path->getDuration();
                path->setDebugText("FoundPath" + debug + QString::number(path->getDuration()));
                //if (path->getDuration() < 0.7) {
                //    path->slow(multiplier);

                //    path->setDebugText(debug + QString::number(path->getDuration()) + " slow");
                //}
                assert(path!=nullptr);

                return make_unique<AngleFunctionPath>(std::move(path), angleFunctionForCommandType(FacePointCommand(command.target)));
            }
        }

    }

    MotionInstant ballNow = ball.predict(1.0 + RJ::timestamp());
    MotionInstant target(ballNow.pos);
    target.vel = (command.target - target.pos).normalized(ApproachSpeed);
    target.pos -= target.vel.normalized(Robot_Radius*3);

    //obstacles.add(make_shared<Circle>(target.pos, ballAvoidDistance));
    auto ballPath = ball.path(RJ::timestamp());
    dynamicObstacles.push_back(DynamicObstacle(ballPath.get(), Ball_Radius));
    auto request = SinglePlanRequest(startInstant, PathTargetCommand(target),
                                     robotConstraints, obstacles, dynamicObstacles, systemState,
                                     std::move(prevPath));
    auto path = rrtPlanner.run(request);
    path->setDebugText("Gives ups");


    targetKickPos = boost::none;
    assert(path!=nullptr);
    return make_unique<AngleFunctionPath>(std::move(path), angleFunctionForCommandType(FacePointCommand(command.target)));
}
}
