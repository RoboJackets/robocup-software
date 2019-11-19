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
    REGISTER_CONFIGURABLE(PivotPathPlanner);

    ConfigDouble* PivotPathPlanner::_pivotRadiusMultiplier;

    void PivotPathPlanner::createConfiguration(Configuration *cfg) {
        _pivotRadiusMultiplier = new ConfigDouble(cfg, "Pivot/radius", 1.0,
        "Multiplier for the pivotRadius. PivotRadius = RobotRadius * multiplier");
    }

    Trajectory PivotPathPlanner::plan(PlanRequest&& request) {
        if(!isApplicable(request.motionCommand)) {
            throw std::invalid_argument("Error in PathTargetPlanner: invalid motionCommand; must be a PivotCommand.");
        }

        using Geometry2d::Point;

        RobotInstant current_instant;
        current_instant.pose = request.start.pose;
        current_instant.velocity = request.start.velocity;
        current_instant.stamp = request.start.timestamp;

        Trajectory result = std::move(request.prevTrajectory);
        if(shouldReplan(request)) {
            PivotCommand& command = std::get<PivotCommand>(request.motionCommand);
            auto state_space = std::make_shared<RoboCupStateSpace>(
                    Field_Dimensions::Current_Dimensions, std::move(request.obstacles));

            // float radius = command.radius;
            float radius = (float)_pivotRadiusMultiplier->value() * Robot_Radius;
            Point pivotPoint = command.pivotPoint;
            double targetAngle = command.targetAngle;
            std::vector<Point> points;

            // maxSpeed = maxRadians * radius
            MotionConstraints motionConstraints = request.constraints.mot;
            motionConstraints.maxSpeed =
                    std::min(motionConstraints.maxSpeed,
                             request.constraints.rot.maxSpeed * radius) * .5;
            Point current_point = request.start.pose.position();
            float startAngle = pivotPoint.angleTo(current_point);
            float change = fixAngleRadians(targetAngle - startAngle);

            const int interpolations = 10;

            points.push_back(current_point);
            for (int i = 1; i <= interpolations; i++) {
                float percent = (float)i / interpolations;
                float angle = startAngle + change * percent;
                Point point =
                        Point::direction(angle).normalized(radius) + pivotPoint;
                points.push_back(point);
            }
            BezierPath bezier(points, current_instant.velocity.linear(), Point(0,0), motionConstraints);
            result = ProfileVelocity(bezier, current_instant.velocity.linear().mag(), 0, motionConstraints);
            std::function<double(Point, Point, double)> angleFunction =
                [&](Point pos, Point vel_linear, double angle) -> double {
                    auto angleToPivot = pos.angleTo(pivotPoint);
                    auto angleToPivotTarget = angle - targetAngle;

                    if (abs(angleToPivot - angleToPivotTarget) <
                        10 * M_PI / 180) {
                        return angleToPivotTarget;
                    } else {
                        return angleToPivot;
                    }
                };
            PlanAngles(result, RobotState{current_instant.pose, current_instant.velocity, current_instant.stamp}, angleFunction, request.constraints.rot);
        }
        result.setDebugText("Pivot");
        return std::move(result);
    }
}