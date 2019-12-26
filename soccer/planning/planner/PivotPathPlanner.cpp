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

    std::vector<RJ::Time> PivotPathPlanner::planTimes{Num_Shells, RJ::now()-60s};

    Trajectory PivotPathPlanner::plan(PlanRequest&& request) {
        if(!isApplicable(request.motionCommand)) {
            throw std::invalid_argument("Error in PivotPathPlanner: invalid motionCommand; must be a PivotCommand.");
        }
        PivotCommand command = std::get<PivotCommand>(request.motionCommand);
        Point pivotPoint = command.pivotPoint;
        Point pivotTarget = command.pivotTarget;
        RobotInstant startInstant = request.start;
        RobotConstraints constraints = request.constraints;
        Trajectory& prevTrajectory = request.prevTrajectory;
        RJ::Time& prevTime = planTimes[request.shellID];
        auto state_space = std::make_shared<RoboCupStateSpace>(
                Field_Dimensions::Current_Dimensions, request.static_obstacles);
        double radius = Robot_Radius * _pivotRadiusMultiplier->value();

        bool targetIsDifferent = false;
        if(prevTrajectory.num_instants() > 0) {
            Point prevTargetPoint = prevTrajectory.last().pose.position();
            Point newTargetPoint = pivotPoint + (pivotPoint-pivotTarget).normalized(radius);
            targetIsDifferent = (newTargetPoint - prevTargetPoint).mag() > 0.1;
        }

        bool pathTooOld = false;// RJ::now() - prevTime > 0.5s;

        if(prevTrajectory.num_instants() < 2 || targetIsDifferent || pathTooOld) {
            prevTime = RJ::now();
            double targetAngle = pivotTarget.angleTo(pivotPoint);

            // maxSpeed = maxRadians * radius
            MotionConstraints motionConstraints = constraints.mot;
            motionConstraints.maxSpeed =
                    std::min(motionConstraints.maxSpeed,
                             constraints.rot.maxSpeed * radius) * .5;
            double startAngle = pivotPoint.angleTo(startInstant.pose.position());
            double change = fixAngleRadians(targetAngle - startAngle);

            const int interpolations = 10;
            std::vector points{startInstant.pose.position()};
            for (int i = 1; i <= interpolations; i++) {
                float percent = (float)i / interpolations;
                float angle = startAngle + change * percent;
                Point point = Point::direction(angle).normalized(radius) + pivotPoint;
                points.push_back(point);
            }
            BezierPath bezier(points, startInstant.velocity.linear(), Point(0, 0), motionConstraints);
            Trajectory result = ProfileVelocity(bezier, startInstant.velocity.linear().mag(), 0, motionConstraints);
            if(!result.empty()) {
                AngleFunction angleFunction =
                        [pivotPoint, pivotTarget](const RobotInstant& instant) -> double {
                            double angleToPivot = instant.pose.position().angleTo(pivotPoint);
                            double angleToPivotTarget = instant.pose.position().angleTo(pivotTarget);
                            if (abs(fixAngleRadians(angleToPivot - angleToPivotTarget)) <
                                10.0 * M_PI / 180.0) {
                                // when we're close to the aim direction, we use the actual pivotTarget
                                // this is necessary because Gameplay seems to kick early/late
                                // sometimes so it's important to maintain our aim for more
                                // than just the final instant of the trajectory.
                                return angleToPivotTarget;
                            } else {
                                return angleToPivot;
                            }
                        };
                PlanAngles(result, startInstant, angleFunction, constraints.rot);
                result.setDebugText("Pivot (New)");
                return std::move(result);
            }
        }
        prevTrajectory.setDebugText("Pivot (Old)");
        return std::move(prevTrajectory);
    }
}