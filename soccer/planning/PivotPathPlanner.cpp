#include "PivotPathPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include "RRTPlanner.hpp"
#include "Geometry2d/Util.hpp"
using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(PivotPathPlanner);

ConfigDouble* PivotPathPlanner::_pivotRadiusMultiplier;

void PivotPathPlanner::createConfiguration(Configuration* cfg) {
    _pivotRadiusMultiplier =
        new ConfigDouble(cfg, "Pivot/radius", 1.0,
                         "Multiplier for the pivotRadius. PivotRadius = "
                         "RobotRadius * multiplier");
}

bool PivotPathPlanner::shouldReplan(
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

        if (targetChange > 0.1) {
            return true;
        }
        if (prevPath->getDuration() -
                RJ::TimestampToSecs(RJ::timestamp() - prevPath->startTime()) <
            -0.5) {
            return true;
        }
    }
    return false;
}

std::unique_ptr<Path> PivotPathPlanner::run(SinglePlanRequest& planRequest) {
    const MotionInstant& startInstant = planRequest.startInstant;
    const auto& motionConstraints = planRequest.robotConstraints.mot;
    const auto& rotationConstraints = planRequest.robotConstraints.rot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    const auto& command = dynamic_cast<const PivotCommand&>(planRequest.cmd);

    if (shouldReplan(planRequest)) {
        // float radius = command.radius;
        float radius = (float)_pivotRadiusMultiplier->value() * Robot_Radius;
        auto pivotPoint = command.pivotPoint;
        auto pivotTarget = command.pivotTarget;
        auto endTarget =
            pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
        vector<Point> points;

        // maxSpeed = maxRadians * radius
        MotionConstraints newConstraints = planRequest.robotConstraints.mot;
        newConstraints.maxSpeed = std::min(
            newConstraints.maxSpeed, rotationConstraints.maxSpeed / 2 * radius);

        float startAngle = pivotPoint.angleTo(startInstant.pos);
        float targetAngle = pivotPoint.angleTo(endTarget);
        float change = fixAngleRadians(targetAngle - startAngle);

        const int interpolations = 10;

        points.push_back(startInstant.pos);
        for (int i = 1; i <= interpolations; i++) {
            float percent = (float)i / interpolations;
            float angle = startAngle + change * percent;
            Point point =
                Point::direction(angle).normalized(radius) + pivotPoint;
            points.push_back(point);
        }
        unique_ptr<InterpolatedPath> path = RRTPlanner::generatePath(
            points, obstacles, newConstraints, startInstant.vel, Point(0, 0));
        // for (auto &waypoint: path->waypoints) {
        // waypoint.angle = AngleInstant::
        //}
        std::function<AngleInstant(MotionInstant)> function =
            [pivotPoint, pivotTarget](MotionInstant instant) {
                auto angleToPivot = instant.pos.angleTo(pivotPoint);
                auto angleToPivotTarget = instant.pos.angleTo(pivotTarget);

                if (abs(angleToPivot - angleToPivotTarget) <
                    DegreesToRadians(10)) {
                    return AngleInstant(angleToPivotTarget);
                } else {
                    return AngleInstant(angleToPivot);
                }
            };
        return make_unique<AngleFunctionPath>(move(path), function);
        ;
    } else {
        return std::move(prevPath);
    }
}
}
