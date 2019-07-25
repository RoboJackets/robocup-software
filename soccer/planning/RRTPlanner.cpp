#include "RRTPlanner.hpp"
#include <Constants.hpp>
#include <Utils.hpp>
#include <rrt/planning/Path.hpp>
#include <rrt/Tree.hpp>
#include "EscapeObstaclesPathPlanner.hpp"
#include "RRTUtil.hpp"
#include "RoboCupStateSpace.hpp"
#include "motion/TrapezoidalMotion.hpp"
#include "CompositePath.hpp"
#include "Configuration.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace Geometry2d;

namespace Planning {
REGISTER_CONFIGURABLE(RRTPlanner);

ConfigDouble* RRTPlanner::_partialReplanLeadTime;

void RRTPlanner::createConfiguration(Configuration* cfg) {
    _partialReplanLeadTime = new ConfigDouble(
        cfg, "RRTPlanner/partialReplanLeadTime", 0.2, "partialReplanLeadTime");
}

RRTPlanner::RRTPlanner(int minIterations, int maxIterations)
    : _minIterations(minIterations),
      _maxIterations(maxIterations),
      SingleRobotPathPlanner(true) {}

bool veeredOffPath(Point currentPos, const Path& path,
                   MotionConstraints motionConstraints) {
    RJ::Seconds timeIntoPath =
        (RJ::now() - path.startTime()) + RJ::Seconds(1) / 60;

    std::optional<RobotInstant> optTarget = path.evaluate(timeIntoPath);
    // If we went off the end of the path, use the end for calculations.
    MotionInstant target = optTarget ? optTarget->motion : path.end().motion;

    // invalidate path if current position is more than the replanThreshold away
    // from where it's supposed to be right now
    auto pathError = (target.pos - currentPos).mag();
    auto replanThreshold = *motionConstraints._replan_threshold;
    return replanThreshold != 0 && pathError > replanThreshold;
}

bool goalChanged(const MotionInstant& goal, const Path& prevPath) {
    double goalPosDiff = (prevPath.end().motion.pos - goal.pos).mag();
    double goalVelDiff = (prevPath.end().motion.vel - goal.vel).mag();
    // cout<<"posDif"<<(prevPath.end().motion.pos - goal.pos)<<endl;
    //    cout<<"vel1Dif"<<(prevPath.end().motion.vel)<<endl;
    // cout<<"vel2Dif"<<goal.vel<<endl;
    return goalPosDiff > SingleRobotPathPlanner::goalPosChangeThreshold() ||
           goalVelDiff > SingleRobotPathPlanner::goalVelChangeThreshold();
}

bool RRTPlanner::shouldReplan(const PlanRequest& planRequest,
                              const vector<DynamicObstacle> dynamicObs,
                              string* debugOut) const {
    const ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    const Planning::PathTargetCommand& command =
        dynamic_cast<const Planning::PathTargetCommand&>(
            *planRequest.motionCommand);

    const auto& goal = command.pathGoal;

    if (SingleRobotPathPlanner::shouldReplan(planRequest)) {
        if (debugOut) {
            *debugOut = "SingleRobotPathInvalidate";
        }
        return true;
    }

    // if the destination of the current path is greater than X m away
    // from the target destination, we invalidate the path. This
    // situation could arise if the path destination changed.
    double goalPosDiff = (prevPath->end().motion.pos - goal.pos).mag();
    double goalVelDiff = (prevPath->end().motion.vel - goal.vel).mag();
    if (goalPosDiff > goalPosChangeThreshold() ||
        goalVelDiff > goalVelChangeThreshold()) {
        // FIXME: goalChangeThreshold shouldn't be used for velocities as it
        // is above
        if (debugOut) {
            *debugOut = "GoalChanged";
        }
        return true;
    }

    if (prevPath->pathsIntersect(dynamicObs, RJ::now(), nullptr, nullptr)) {
        if (debugOut) {
            *debugOut = "DynamicIntersect";
        }
        return true;
    }

    return false;
}

const int maxContinue = 10;

std::unique_ptr<Path> RRTPlanner::run(PlanRequest& planRequest) {
    const MotionInstant& start = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;
    const auto& dynamicObstacles = planRequest.dynamicObstacles;

    // This planner only works with commands of type 'PathTarget'
    assert(planRequest.motionCommand->getCommandType() ==
           Planning::MotionCommand::PathTarget);
    const Planning::PathTargetCommand& pathTargetCommand =
        dynamic_cast<const Planning::PathTargetCommand&>(
            *planRequest.motionCommand);

    MotionInstant goal = pathTargetCommand.pathGoal;
    vector<DynamicObstacle> actualDynamic;
    splitDynamic(obstacles, actualDynamic, dynamicObstacles);

    // Simple case: no path
    if (start.pos == goal.pos) {
        auto path = make_unique<InterpolatedPath>();
        path->setStartTime(RJ::now());
        path->waypoints.emplace_back(MotionInstant(start.pos, Point()),
                                     RJ::Seconds::zero());
        path->setDebugText("Invalid Basic Path");
        return std::move(path);
    }

    // Locate a goal point that is obstacle-free
    std::optional<Point> prevGoal;
    if (prevPath) prevGoal = prevPath->end().motion.pos;
    goal.pos = EscapeObstaclesPathPlanner::findNonBlockedGoal(
        goal.pos, prevGoal, obstacles);

    string debugOut;

    const auto partialReplanTime = RJ::Seconds(*_partialReplanLeadTime);

    enum ReplanState { Reuse, FullReplan, PartialReplan, CheckBetter };

    ReplanState replanState = Reuse;

    if (!prevPath || veeredOffPath(start.pos, *prevPath, motionConstraints)) {
        replanState = FullReplan;
    }

    if (replanState == Reuse) {
        const auto timeIntoPrevPath = RJ::now() - prevPath->startTime();

        RJ::Seconds invalidTime;
        if (prevPath->hit(obstacles, timeIntoPrevPath, &invalidTime)) {
            replanState = PartialReplan;
            debugOut = "hitObstacle";
        } else if (prevPath->pathsIntersect(actualDynamic, RJ::now(), nullptr,
                                            &invalidTime)) {
            replanState = PartialReplan;
            debugOut = "pathsIntersect";
        } else if (goalChanged(goal, *prevPath)) {
            invalidTime = prevPath->getDuration();
            replanState = PartialReplan;
            debugOut += "goalChanged";
        }

        if (replanState == PartialReplan) {
            const auto timeFromInvalid = invalidTime - timeIntoPrevPath;
            if (timeFromInvalid < partialReplanTime * 2) {
                replanState = FullReplan;
                debugOut += " Too soon";
            }
        }
    }

    std::unique_ptr<Path> path;

    if (replanState == Reuse) {
        if (reusePathTries >= maxContinue) {
            reusePathTries = 0;
            const auto timeIntoPrevPath = RJ::now() - prevPath->startTime();
            if (prevPath->getDuration() - timeIntoPrevPath >
                partialReplanTime * 2) {
                replanState = CheckBetter;
            }
        }
        reusePathTries++;
    }

    if (replanState == CheckBetter || replanState == PartialReplan) {
        const auto timeIntoPrevPath = RJ::now() - prevPath->startTime();
        auto subPath =
            prevPath->subPath(0ms, timeIntoPrevPath + partialReplanTime);
        RobotInstant newStart = subPath->end();

        std::optional<vector<Point>> biasWaypoints;
        if (replanState == PartialReplan) {
            biasWaypoints = vector<Point>();
            biasWaypoints->push_back(start.pos + start.vel * 0.2);
            // if (auto point = prevPath->evaluate(300ms)) {
            //     biasWaypoints->push_back(point->motion.pos);
            // }
            // if (auto point = prevPath->evaluate(500ms)) {
            //     biasWaypoints->push_back(point->motion.pos);
            // }
            // if (auto point = prevPath->evaluate(1000ms)) {
            //     biasWaypoints->push_back(point->motion.pos);
            // }
            for (auto it = prevPath->iterator(RJ::now(), 100ms);
                 it->operator*().motion.pos != prevPath->end().motion.pos;
                 it->operator++()) {
                auto instant = it->operator*();
                biasWaypoints->push_back(instant.motion.pos);
            }
        }

        auto newSubPath = generateRRTPath(
            newStart.motion, goal, motionConstraints, obstacles, actualDynamic,
            planRequest.context, planRequest.shellID, biasWaypoints);
        if (newSubPath) {
            path = make_unique<CompositePath>(std::move(subPath),
                                              std::move(newSubPath));
            path->setStartTime(prevPath->startTime());
            path->setDebugText(
                QString::fromStdString("partialReplan" + debugOut));

            if (replanState == CheckBetter) {
                RJ::Seconds remaining = prevPath->getDuration() -
                                        (RJ::now() - prevPath->startTime());
                if (remaining > path->getDuration()) {
                    path->setDebugText("Found better path");
                    return path;
                } else {
                    replanState = Reuse;
                }
            } else {
                return path;
            }
        } else if (replanState == PartialReplan) {
            debugOut = "Partial Failed";
            replanState = FullReplan;
        } else {
            replanState = Reuse;
        }
    }

    if (replanState == Reuse) {
        path = std::move(prevPath);
        path->setDebugText("Reusing");
        return path;
    } else if (replanState == FullReplan) {
        path = generateRRTPath(start, goal, motionConstraints, obstacles,
                               actualDynamic, planRequest.context,
                               planRequest.shellID);
        if (path) {
            path->setDebugText(
                QString::fromStdString("FullReplan." + debugOut));
        } else {
            path = InterpolatedPath::emptyPath(start.pos);
        }
        return path;
    } else {
        string type = [](auto replanState) {
            switch (replanState) {
                case Reuse:
                    return "Reuse";
                case FullReplan:
                    return "FullReplan";
                case CheckBetter:
                    return "CheckBetter";
                case PartialReplan:
                    return "PartialReplan";
                default:
                    return "No idea what replanState";
            }
        }(replanState);
        debugThrow("Logic Error. Rip. State:" + type);
        return InterpolatedPath::emptyPath(start.pos);
    }
}

std::unique_ptr<InterpolatedPath> RRTPlanner::generateRRTPath(
    const MotionInstant& start, const MotionInstant& goal,
    const MotionConstraints& motionConstraints, ShapeSet& origional,
    const std::vector<DynamicObstacle> dyObs, Context* context,
    unsigned shellID, const optional<vector<Point>>& biasWayPoints) {
    const int tries = 10;
    ShapeSet obstacles = origional;
    unique_ptr<InterpolatedPath> lastPath;
    for (int i = 0; i < tries; i++) {
        // Run bi-directional RRT to generate a path.
        auto points = runRRT(start, goal, motionConstraints, obstacles, context,
                             shellID, biasWayPoints);

        // Check if Planning or optimization failed
        if (points.size() < 2) {
            // debugLog("RRTPlanning Failed");
            break;
        }

        // Generate and return a cubic bezier path using the waypoints
        auto path = generateCubicBezier(points, obstacles, motionConstraints,
                                        start.vel, goal.vel);

        if (!path) {
            // debugLog("RRT Vel Planning Failed");
            break;
        }

        RJ::Seconds hitTime;
        Point hitLocation;
        bool hit = path->pathsIntersect(dyObs, path->startTime(), &hitLocation,
                                        &hitTime);
        if (hit) {
            obstacles.add(
                make_shared<Circle>(hitLocation, Robot_Radius * 1.5f));
            lastPath = std::move(path);
        } else {
            return std::move(path);
        }
    }
    // debugLog("Generate Failed 10 times");
    return lastPath;
}

vector<Point> RRTPlanner::runRRT(MotionInstant start, MotionInstant goal,
                                 const MotionConstraints& motionConstraints,
                                 const ShapeSet& obstacles, Context* context,
                                 unsigned shellID,
                                 const optional<vector<Point>>& biasWaypoints) {
    vector<Point> straight =
        runRRTHelper(start, goal, motionConstraints, obstacles, context,
                     shellID, biasWaypoints, true);
    if (straight.size() != 0) {
        return straight;
    }
    return runRRTHelper(start, goal, motionConstraints, obstacles, context,
                        shellID, biasWaypoints, false);
}

vector<Point> RRTPlanner::runRRTHelper(
    MotionInstant start, MotionInstant goal,
    const MotionConstraints& motionConstraints, const ShapeSet& obstacles,
    Context* context, unsigned shellID,
    const optional<vector<Point>>& biasWaypoints, bool straightLine) {
    // Initialize bi-directional RRT

    auto stateSpace = make_shared<RoboCupStateSpace>(
        Field_Dimensions::Current_Dimensions, obstacles);
    RRT::BiRRT<Point> biRRT(stateSpace, Point::hash, 2);
    reusePathTries++;
    biRRT.setStartState(start.pos);
    biRRT.setGoalState(goal.pos);

    // If trying to plan a straight path, plan a straight path. Otherwise, run
    // normal RRT.
    if (straightLine) {
        // Set the step size to be the distance between the start and goal.
        biRRT.setStepSize(stateSpace->distance(start.pos, goal.pos));
        // Plan straight toward the goal.
        biRRT.setGoalBias(1);
        // Try up to five times. If unsuccessful after five tries, there
        // probably doesn't exist
        // a straight path.
        biRRT.setMinIterations(0);
        biRRT.setMaxIterations(5);
    } else {
        biRRT.setStepSize(*RRTConfig::StepSize);
        biRRT.setMinIterations(_minIterations);
        biRRT.setMaxIterations(_maxIterations);
        biRRT.setGoalBias(*RRTConfig::GoalBias);

        if (biasWaypoints) {
            biRRT.setWaypoints(*biasWaypoints);
            biRRT.setWaypointBias(*RRTConfig::WaypointBias);
        }
    }

    bool success = biRRT.run();
    if (!success) return vector<Point>();

    if (*RRTConfig::EnableRRTDebugDrawing) {
        DrawBiRRT(biRRT, &context->debug_drawer, shellID);
    }

    vector<Point> points = biRRT.getPath();

    // Optimize out uneccesary waypoints
    RRT::SmoothPath(points, *stateSpace);

    return points;
}

double getTime(vector<Point> path, int index,
               const MotionConstraints& motionConstraints, double startSpeed,
               double endSpeed) {
    double length = 0;
    double startLength = 0;
    for (int i = 1; i < path.size(); i++) {
        length += path[i - 1].distTo(path[i]);
        if (index == i) {
            startLength = length;
        }
    }
    return Trapezoidal::getTime(startLength, length, motionConstraints.maxSpeed,
                                motionConstraints.maxAcceleration, startSpeed,
                                endSpeed);
}
double getTime(InterpolatedPath& path, int index,
               const MotionConstraints& motionConstraints, double startSpeed,
               double endSpeed) {
    return Trapezoidal::getTime(
        path.length(0, index), path.length(), motionConstraints.maxSpeed,
        motionConstraints.maxAcceleration, startSpeed, endSpeed);
}

std::unique_ptr<InterpolatedPath> RRTPlanner::generatePath(
    const std::vector<Point>& points, const ShapeSet& obstacles,
    const MotionConstraints& motionConstraints, Point vi, Point vf) {
    return generateCubicBezier(points, obstacles, motionConstraints, vi, vf);
}

vector<CubicBezierControlPoints> RRTPlanner::generateNormalCubicBezierPath(
    const vector<Point>& points, const MotionConstraints& motionConstraints,
    Point vi, Point vf) {
    size_t length = points.size();
    size_t curvesNum = length - 1;

    const double directionDistance = 0.4;
    vector<Point> startDirections;
    vector<Point> endDirections;

    const double pathWeight = 0.1;
    Point pathDirection =
        (vi + (points[1] - points[0]).normalized(pathWeight)).normalized();
    // Point pathDirection = (points[1] - points[0]).normalized(pathWeight);
    startDirections.push_back(
        (pathDirection).normalized((points[1] - points[0]).mag() * 0.1));
    for (int i = 1; i < length - 1; i++) {
        const Point difference = points[i + 1] - points[i - 1];
        endDirections.push_back(difference.normalized(
            (points[i] - points[i - 1]).mag() * directionDistance));
        startDirections.push_back(difference.normalized(
            (points[i + 1] - points[i]).mag() * directionDistance));
    }
    Point endPathDirection =
        (points[points.size() - 1] - points[points.size() - 2])
            .normalized(pathWeight) +
        vf;
    endDirections.push_back((endPathDirection)
                                .normalized((points[points.size() - 1] -
                                             points[points.size() - 2]).mag() *
                                            directionDistance));

    vector<CubicBezierControlPoints> path;

    for (int i = 0; i < curvesNum; i++) {
        Point p0 = points[i];
        Point p3 = points[i + 1];
        Point p1 = p0 + startDirections[i];
        Point p2 = p3 - endDirections[i];
        path.emplace_back(p0, p1, p2, p3);
    }
    return path;
}

vector<CubicBezierControlPoints> RRTPlanner::generateCubicBezierPath(
    const vector<Geometry2d::Point>& points,
    const MotionConstraints& motionConstraints, Geometry2d::Point vi,
    Geometry2d::Point vf, const std::optional<vector<double>>& times) {
    size_t length = points.size();
    size_t curvesNum = length - 1;
    vector<double> pointsX(length);
    vector<double> pointsY(length);
    vector<double> ks(length - 1);
    vector<double> ks2(length - 1);

    for (int i = 0; i < length; i++) {
        pointsX[i] = points[i].x();
        pointsY[i] = points[i].y();
    }
    const double startSpeed = vi.mag();

    const double endSpeed = vf.mag();

    if (times) {
        assert(times->size() == points.size());
        for (int i = 0; i < curvesNum; i++) {
            ks[i] = 1.0 / (times->at(i + 1) - times->at(i));
            ks2[i] = ks[i] * ks[i];
            if (std::isnan(ks[i])) {
                debugThrow(
                    "Something went wrong. Points are too close to each other "
                    "probably");
                return vector<CubicBezierControlPoints>();
            }
        }
    } else {
        for (int i = 0; i < curvesNum; i++) {
            ks[i] = 1.0 / (getTime(points, i + 1, motionConstraints, startSpeed,
                                   endSpeed) -
                           getTime(points, i, motionConstraints, startSpeed,
                                   endSpeed));
            ks2[i] = ks[i] * ks[i];
            if (std::isnan(ks[i])) {
                debugThrow(
                    "Something went wrong. Points are too close to each other "
                    "probably");
                return vector<CubicBezierControlPoints>();
            }
        }
    }

    VectorXd solutionX =
        RRTPlanner::cubicBezierCalc(vi.x(), vf.x(), pointsX, ks, ks2);
    VectorXd solutionY =
        RRTPlanner::cubicBezierCalc(vi.y(), vf.y(), pointsY, ks, ks2);

    if (solutionX.hasNaN() || solutionY.hasNaN()) {
        return vector<CubicBezierControlPoints>();
    }

    vector<CubicBezierControlPoints> path;

    for (int i = 0; i < curvesNum; i++) {
        Point p0 = points[i];
        Point p1 = Point(solutionX(i * 2), solutionY(i * 2));
        Point p2 = Point(solutionX(i * 2 + 1), solutionY(i * 2 + 1));
        Point p3 = points[i + 1];
        path.emplace_back(p0, p1, p2, p3);
    }
    return path;
}

double oneStepLimitAcceleration(double maxAceleration, double d1, double v1,
                                double c1, double d2, double v2, double c2,
                                double minMaxSpeed = 0) {
    double d = std::abs(d2 - d1);
    double deltaSpeed = v2 - v1;
    if (deltaSpeed < 0) {
        return v2;
    }

    // Isolated maxSpeed based on Curvature should already be taken care of
    double c = max(c1, c2);
    double a = maxAceleration;

    // acceleration = (v2-v1)/t;
    // t = distance/((v1+v2)/2)
    // acceleration = (v2-v1)/(distance/((v1+v2)/2))
    // acceleration = (v2-v1)(v1+v2)/2)/distance
    // acceleration^2 = ((v2-v1)((v1+v2)/2)/(distance))^2 + (v^2*curvature)^2
    // a^2 = ((b-v)((v+b)/2)/(d))^2 + (b^2*c)^2
    // http://www.wolframalpha.com/input/?i=solve+for+b+where+a%5E2+%3D+((b-v)((v%2Bb)%2F2)%2F(d))%5E2+%2B+(v%5E2*c)%5E2
    double part = d * d * (a * a - c * c * pow(v1, 4));
    if (part >= 0) {
        double possible = sqrt(v1 * v1 + 2 * sqrt(part));
        return std::min(v2, possible);
    }

    double maxSpeed = std::sqrt(a * d * 2 + v1 * v1);
    maxSpeed = std::max(minMaxSpeed, maxSpeed);
    return std::min(v2, maxSpeed);
}

/**
 * Generates a Cubic Bezier Path based on Albert's random Bezier Velocity Path
 * Algorithm
 */
std::vector<InterpolatedPath::Entry> RRTPlanner::generateVelocityPath(
    const std::vector<CubicBezierControlPoints>& controlPoints,
    const MotionConstraints& motionConstraints, Point vi, Point vf,
    int interpolations) {
    // Interpolate Through Bezier Path
    vector<Point> newPoints, newPoints1stDerivative, newPoints2ndDerivative;
    vector<double> newPointsCurvature, newPointsDistance, newPointsSpeed;

    double totalDistance = 0;
    const double maxAceleration = motionConstraints.maxAcceleration;
    const double& maxSpeed = motionConstraints.maxSpeed;

    int asd = 0;
    for (const CubicBezierControlPoints& controlPoint : controlPoints) {
        Point p0 = controlPoint.p0;
        Point p1 = controlPoint.p1;
        Point p2 = controlPoint.p2;
        Point p3 = controlPoint.p3;
        for (int j = 0; j < interpolations; j++) {
            double t = (((double)j / (double)(interpolations)));
            Geometry2d::Point pos =
                pow(1.0 - t, 3) * p0 + 3.0 * pow(1.0 - t, 2) * t * p1 +
                3 * (1.0 - t) * pow(t, 2) * p2 + pow(t, 3) * p3;

            // Derivitive 1
            // 3 k (-(A (-1 + k t)^2) + k t (2 C - 3 C k t + D k t) + B (1 - 4 k
            // t + 3 k^2 t^2))
            Point d1 = 3 * pow(1 - t, 2) * (p1 - p0) +
                       6 * (1 - t) * t * (p2 - p1) + 3 * pow(t, 2) * (p3 - p2);

            // Derivitive 2
            // https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B.C3.A9zier_curves
            // B''(t) = 6(1-t)(P2 - 2*P1 + P0) + 6*t(P3 - 2 * P2 + P1)
            Point d2 =
                6 * (1 - t) * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1);

            // https://en.wikipedia.org/wiki/Curvature#Local_expressions
            // K = |x'*y'' - y'*x''| / (x'^2 + y'^2)^(3/2)
            double curvature =
                std::abs(d1.x() * d2.y() - d1.y() * d2.x()) /
                std::pow(std::pow(d1.x(), 2) + std::pow(d1.y(), 2), 1.5);

            // Handle 0 velocity case
            if (std::isnan(curvature)) {
                curvature = 0;
            }

            assert(curvature >= 0);
            if (!newPoints.empty()) {
                double distance = pos.distTo(newPoints.back());
                totalDistance += distance;
            }
            newPointsDistance.push_back(totalDistance);

            newPoints.push_back(pos);
            newPoints1stDerivative.push_back(d1);
            newPoints2ndDerivative.push_back(d2);

            newPointsCurvature.push_back(curvature);

            // Isolated maxSpeed based on Curvature
            // Curvature = 1/Radius of Curvature
            // vmax = sqrt(acceleartion/abs(Curvature))

            const auto maxCentripetalAcceleration =
                motionConstraints.maxCentripetalAcceleration;
            double constantMaxSpeed =
                std::sqrt(maxCentripetalAcceleration / curvature);
            //            constantMaxSpeed =std::max(constantMaxSpeed, 0.5f);
            // cout<<asd<< " " << constantMaxSpeed<<endl;
            asd++;
            newPointsSpeed.push_back(std::min(constantMaxSpeed, maxSpeed));
        }
    }
    // Get last point in Path
    CubicBezierControlPoints lastControlPoint = controlPoints.back();

    Point p0 = lastControlPoint.p0;
    Point p1 = lastControlPoint.p1;
    Point p2 = lastControlPoint.p2;
    Point p3 = lastControlPoint.p3;

    Point pos = p3;
    Point d1 = vf;
    Geometry2d::Point d2 = 6 * (1) * (p3 - 2 * p2 + p1);
    double curvature = std::abs(d1.x() * d2.y() - d1.y() * d2.x()) /
                       std::pow(std::pow(d1.x(), 2) + std::pow(d1.y(), 2), 1.5);

    // handle 0 velcoity case
    if (std::isnan(curvature)) {
        curvature = 0;
    }

    totalDistance += pos.distTo(newPoints.back());
    newPoints.push_back(pos);
    newPoints1stDerivative.push_back(vf);
    newPoints2ndDerivative.push_back(d2);
    newPointsCurvature.push_back(curvature);

    newPointsDistance.push_back(totalDistance);

    newPointsSpeed[0] = vi.mag();
    newPointsSpeed.push_back(vf.mag());

    // Velocity Profile Generation
    // Forward Smoothing
    const double size = newPoints.size();
    assert(size == newPoints.size());
    assert(size == newPoints1stDerivative.size());
    assert(size == newPoints2ndDerivative.size());
    assert(size == newPointsDistance.size());
    assert(size == newPointsSpeed.size());
    assert(size == newPointsCurvature.size());

    // Generate Velocity Profile from Interpolation of Bezier Path
    // Forward Constraints

    for (int i = 1; i < size; i++) {
        int i1 = i - 1;
        int i2 = i;
        newPointsSpeed[i2] = oneStepLimitAcceleration(
            maxAceleration, newPointsDistance[i1], newPointsSpeed[i1],
            newPointsCurvature[i1], newPointsDistance[i2], newPointsSpeed[i2],
            newPointsCurvature[i2], 0.1);
    }

    // Backwards Constraints
    for (int i = size - 2; i >= 0; i--) {
        int i1 = i + 1;
        int i2 = i;
        newPointsSpeed[i2] = oneStepLimitAcceleration(
            maxAceleration, newPointsDistance[i1], newPointsSpeed[i1],
            newPointsCurvature[i1], newPointsDistance[i2], newPointsSpeed[i2],
            newPointsCurvature[i2], 0.1);
    }

    RJ::Seconds totalTime(0);
    vector<InterpolatedPath::Entry> entries;

    for (int i = 0; i < size; i++) {
        double currentSpeed = newPointsSpeed[i];
        if (i != 0) {
            auto distance = newPointsDistance[i] - newPointsDistance[i - 1];
            double averageSpeed = (currentSpeed + newPointsSpeed[i - 1]) / 2.0;
            auto deltaT = RJ::Seconds(distance / averageSpeed);
            totalTime += deltaT;
        }

        Point point = newPoints[i];
        Point vel = newPoints1stDerivative[i].normalized();
        entries.emplace_back(MotionInstant(point, vel * currentSpeed),
                             totalTime);
    }
    return entries;
}

std::unique_ptr<Planning::InterpolatedPath> RRTPlanner::generateCubicBezier(
    const std::vector<Point>& points, const ShapeSet& obstacles,
    const MotionConstraints& motionConstraints, Point vi, Point vf) {
    const int interpolations = 40;

    size_t length = points.size();
    size_t curvesNum = length - 1;
    if (curvesNum <= 0) {
        debugThrow("The path doesn't have enough points");
        return nullptr;
    }

    //    vector<CubicBezierControlPoints> controlPoints =
    //        generateNormalCubicBezierPath(points, motionConstraints, vi, vf);

    vector<CubicBezierControlPoints> controlPoints =
        generateCubicBezierPath(points, motionConstraints, vi, vf);

    if (controlPoints.empty()) {
        return nullptr;
    }

    vector<InterpolatedPath::Entry> entries = generateVelocityPath(
        controlPoints, motionConstraints, vi, vf, interpolations);

    std::unique_ptr<InterpolatedPath> path = make_unique<InterpolatedPath>();
    path->waypoints = entries;
    path->setStartTime(RJ::now());
    return path;
}

VectorXd RRTPlanner::cubicBezierCalc(double vi, double vf,
                                     vector<double>& points, vector<double>& ks,
                                     vector<double>& ks2) {
    int curvesNum = points.size() - 1;

    if (curvesNum == 1) {
        VectorXd vector(2);
        vector[0] = vi / (3.0 * ks[0]) + points[0];
        vector[1] = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);
        return vector;
    } else {
        int matrixSize = curvesNum * 2;
        MatrixXd equations = MatrixXd::Zero(matrixSize, matrixSize);
        VectorXd answer(matrixSize);
        equations(0, 0) = 1;
        answer(0) = vi / (3.0 * ks[0]) + points[0];
        equations(1, matrixSize - 1) = 1;
        answer(1) = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);

        int i = 2;
        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2 + 1) = ks[n];
            equations(i, n * 2 + 2) = ks[n + 1];
            answer(i) = (ks[n] + ks[n + 1]) * points[n + 1];
            i++;
        }

        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2) = ks2[n];
            equations(i, n * 2 + 1) = -2 * ks2[n];
            equations(i, n * 2 + 2) = 2 * ks2[n + 1];
            equations(i, n * 2 + 3) = -ks2[n + 1];
            answer(i) = points[n + 1] * (ks2[n + 1] - ks2[n]);
            i++;
        }

        HouseholderQR<MatrixXd> solver(equations);
        VectorXd solution = solver.solve(answer);
        return solution;
    }
}

RJ::Seconds RRTPlanner::getPartialReplanLeadTime() {
    return RJ::Seconds(*_partialReplanLeadTime);
}

}  // namespace Planning
