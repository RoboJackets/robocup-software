#include "PivotPathPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/PathSmoothing.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include <Geometry2d/Pose.hpp>
#include <Constants.hpp>
#include <vector>
#include <memory>

namespace Planning {
    using Geometry2d::Point;

    REGISTER_CONFIGURABLE(PivotPathPlanner);

    ConfigDouble* PivotPathPlanner::_pivotRadiusMultiplier;

    void PivotPathPlanner::createConfiguration(Configuration *cfg) {
        _pivotRadiusMultiplier = new ConfigDouble(cfg, "Pivot/radius", 1.0,
        "Multiplier for the pivotRadius. PivotRadius = RobotRadius * multiplier");
    }

    Trajectory PivotPathPlanner::plan(PlanRequest&& request) {
        if(!isApplicable(request.motionCommand)) {
            throw std::invalid_argument("Error in PivotPathPlanner: invalid motionCommand; must be a PivotCommand.");
        }
        bool replanRequired = shouldReplan(request);
        bool targetIsDifferent = targetChanged(request);
        bool pathTooOld = request.prevTrajectory.duration()
                          - (RJ::now() - request.prevTrajectory.begin_time()) < RJ::Seconds(-0.5);

        RobotInstant startInstant = request.start;
        RobotConstraints constraints = request.constraints;
        PivotCommand command = std::get<PivotCommand>(request.motionCommand);
        Trajectory result = std::move(request.prevTrajectory);
        auto state_space = std::make_shared<RoboCupStateSpace>(
                Field_Dimensions::Current_Dimensions, std::move(request.obstacles));


        if(replanRequired || targetIsDifferent || pathTooOld) {

            // float radius = command.radius;
            float radius = Robot_Radius * _pivotRadiusMultiplier->value();
            Point pivotPoint = command.pivotPoint;
            double targetAngle = command.targetAngle;
            std::vector<Point> points;

            // maxSpeed = maxRadians * radius
            MotionConstraints motionConstraints = constraints.mot;
            motionConstraints.maxSpeed =
                    std::min(motionConstraints.maxSpeed,
                             constraints.rot.maxSpeed * radius) * .5;
            Point startPoint = startInstant.pose.position();
            float startAngle = pivotPoint.angleTo(startPoint);
            float change = fixAngleRadians(targetAngle - startAngle);

            const int interpolations = 10;

            points.push_back(startPoint);
            for (int i = 1; i <= interpolations; i++) {
                float percent = (float)i / interpolations;
                float angle = startAngle + change * percent;
                Point point =
                        Point::direction(angle).normalized(radius) + pivotPoint;
                points.push_back(point);
            }
            BezierPath bezier(points, startInstant.velocity.linear(), Point(0, 0), motionConstraints);
            result = ProfileVelocity(bezier, startInstant.velocity.linear().mag(), 0, motionConstraints);
            std::function<double(Point, Point, double)> angleFunction =
                [&](Point pos, Point vel_linear, double angle) -> double {
//                    auto angleToPivot = pos.angleTo(pivotPoint);
//                    auto angleToPivotTarget = angle - targetAngle;
//
//                    if (abs(angleToPivot - angleToPivotTarget) <
//                        10 * M_PI / 180) {
//                        return angleToPivotTarget;
//                    } else {
//                        return angleToPivot;
//                    }
return pivotPoint.angleTo(pos);
                };
            PlanAngles(result, startInstant, angleFunction, constraints.rot);
            result.setDebugText("Pivot (New)");
        } else {
            result.setDebugText("Pivot (Old)");
        }
        return std::move(result);
    }

    bool PivotPathPlanner::targetChanged(const PlanRequest& request) const {
        if(request.prevTrajectory.empty()) return false;
        PivotCommand command = std::get<PivotCommand>(request.motionCommand);
        Point prevTargetPoint = request.prevTrajectory.last().pose.position();
        Point newTargetPoint = command.pivotPoint + Point::direction(command.targetAngle)
                .normalized(command.radius * _pivotRadiusMultiplier->value());
        return (newTargetPoint - prevTargetPoint).mag() > 0.1;
    }
}