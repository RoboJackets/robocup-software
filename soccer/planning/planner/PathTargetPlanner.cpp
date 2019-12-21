#include "planning/planner/EscapeObstaclesPathPlanner.hpp"
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <Geometry2d/Pose.hpp>
#include <vector>
#include <rrt/planning/Path.hpp>
#include "Robot.hpp"
#include "planning/DynamicObstacle.hpp"

namespace Planning {

    REGISTER_CONFIGURABLE(PathTargetPlanner);

    ConfigDouble *PathTargetPlanner::_partialReplanLeadTime;

    void PathTargetPlanner::createConfiguration(Configuration *cfg) {
        _partialReplanLeadTime = new ConfigDouble(
                cfg, "PathTargetPlanner/partialReplanLeadTime", 0.2,
                "partialReplanLeadTime");
    }

    using Geometry2d::Point;
    using Geometry2d::Pose;
    using Geometry2d::Twist;

    Trajectory
    PathTargetPlanner::planWithoutAngles(Planning::PlanRequest &&request) {
        RobotInstant pathTarget;
        const MotionCommand& cmd = request.motionCommand;
        const bool veeredOff = veeredOffPath(request);
        Geometry2d::ShapeSet staticObstacles = std::move(request.static_obstacles);
        std::vector<DynamicObstacle> dynamicObstacles = std::move(request.dynamic_obstacles);
        Trajectory prevTrajectory = std::move(request.prevTrajectory);
        RobotInstant startInstant = request.start;
        RobotConstraints constraints = request.constraints;
        if(std::holds_alternative<PathTargetCommand>(cmd)) {
            pathTarget = std::get<PathTargetCommand>(cmd).pathGoal;
        } else if (std::holds_alternative<WorldVelTargetCommand>(cmd)) {
            Twist velTarget = std::get<WorldVelTargetCommand>(cmd).worldVel;
            pathTarget.velocity = velTarget;
            pathTarget.pose.position() = request.start.pose.position() + velTarget.linear().normalized(Robot_Diameter);
            pathTarget.pose.heading() = velTarget.linear().angle();
            constraints.mot.maxSpeed = velTarget.linear().mag();
        }

        // adjust the requested goal out of an obstacle if necessary
        std::optional<Point> prevGoal;
        if (!prevTrajectory.empty()) {
            prevGoal = prevTrajectory.last().pose.position();
        }
        Point &goalPoint = pathTarget.pose.position();
        goalPoint = EscapeObstaclesPathPlanner::findNonBlockedGoal(goalPoint,
                                                                   prevGoal,
                                                                   staticObstacles);

        // Simple case: no path
        if (startInstant.pose.position() == goalPoint) {
            std::vector<RobotInstant> instants;
            instants.emplace_back(startInstant.pose, Twist(), RJ::now());
            Trajectory result{std::move(instants)};
            result.setDebugText("RRT Basic");
            return std::move(result);
        }


        enum ReplanState {
            Reuse, FullReplan, PartialReplan, CheckBetter
        };
        ReplanState replanState = Reuse;
        RobotInstant goalInstant = pathTarget;

        const RJ::Seconds timeIntoTrajectory =
                RJ::now() - prevTrajectory.begin_time();
        const RJ::Seconds timeRemaining =
                prevTrajectory.duration() - timeIntoTrajectory;
        RJ::Seconds invalidTime = 0s;
        if (prevTrajectory.empty() || veeredOff) {
            replanState = FullReplan;
        } else if (prevTrajectory.intersects(dynamicObstacles, RJ::now(), nullptr, &invalidTime)) {
            replanState = PartialReplan;
        } else if (prevTrajectory.hit(staticObstacles, timeIntoTrajectory,
                                      &invalidTime)) {
            replanState = PartialReplan;
        } else if (goalChanged(prevTrajectory.last(), goalInstant)) {
            invalidTime = prevTrajectory.duration();
            replanState = PartialReplan;
        }
        const RJ::Seconds partialReplanTime(*_partialReplanLeadTime);
        if (invalidTime - timeIntoTrajectory < partialReplanTime * 2 &&
            replanState == PartialReplan) {
            replanState = FullReplan;
        }

        if (replanState == Reuse) {
            if (counter >= 10) {
                counter = 0;
                if (timeRemaining > partialReplanTime * 2) {
                    replanState = CheckBetter;
                }
            }
            counter++;
        }

        auto stateSpace = std::make_shared<RoboCupStateSpace>(
                Field_Dimensions::Current_Dimensions, std::move(staticObstacles));
        if (replanState == CheckBetter || replanState == PartialReplan) {
            Trajectory preTrajectory = prevTrajectory.subTrajectory(0ms,
                                                                    timeIntoTrajectory +
                                                                    partialReplanTime);
            RobotInstant middleInstant = preTrajectory.last();
            std::vector<Point> biasWaypoints;
            if (replanState == PartialReplan) {
                biasWaypoints.push_back(startInstant.pose.position() +
                                        startInstant.velocity.linear() * 0.2);
                //todo(Ethan) shouldn't this be less than or equal because rn it doesn't add result.end()
                for (auto it = prevTrajectory.iterator(RJ::now(), 100ms);
                     (*it).stamp < prevTrajectory.end_time(); ++it) {
                    biasWaypoints.push_back((*it).pose.position());
                }
            }
            counter++;
            std::vector<Point> postPoints = GenerateRRT(
                    middleInstant.pose.position(), goalInstant.pose.position(),
                    stateSpace, biasWaypoints);
            Trajectory postTrajectory{{}};
            if (!postPoints.empty()) {
                RRT::SmoothPath(postPoints, *stateSpace);
                BezierPath postBezier(postPoints,
                                      middleInstant.velocity.linear(),
                                      goalInstant.velocity.linear(),
                                      constraints.mot);
                postTrajectory = ProfileVelocity(postBezier,
                                                 middleInstant.velocity.linear().mag(),
                                                 goalInstant.velocity.linear().mag(),
                                                 constraints.mot,
                                                 middleInstant.stamp);
            }
            if (!postTrajectory.empty()) {
                Trajectory comboPath = Trajectory(std::move(preTrajectory),
                                                  std::move(postTrajectory));
                if (replanState == CheckBetter) {
                    if (timeRemaining > comboPath.duration()) {
                        std::cout << "Found A better Path!!!!" << std::endl;
                        assert(!comboPath.empty());
                        return std::move(comboPath);
                    } else {
                        replanState = Reuse;
                    }
                } else {
                    assert(!comboPath.empty());
                    return std::move(comboPath);
                }
            } else if (replanState == PartialReplan) {
                replanState = FullReplan;
            } else {
                replanState = Reuse;
            }
        }

        if (replanState == FullReplan) {
            counter++;
            std::vector<Point> newPoints = GenerateRRT(
                    startInstant.pose.position(), goalInstant.pose.position(),
                    stateSpace);
            Trajectory result{{}};
            if(!newPoints.empty()) {
                RRT::SmoothPath(newPoints, *stateSpace);
                BezierPath new_bezier(newPoints, startInstant.velocity.linear(),
                                      goalInstant.velocity.linear(),
                                      constraints.mot);
                result = ProfileVelocity(new_bezier,
                                                    startInstant.velocity.linear().mag(),
                                                    goalInstant.velocity.linear().mag(),
                                                    constraints.mot);
                result.setDebugText("RRT Full");
            }
            if(!result.empty()) {
                return std::move(result);
            }
        }
        return prevTrajectory.empty() ? Trajectory{{startInstant}} : std::move(prevTrajectory);
    }

    Trajectory PathTargetPlanner::plan(PlanRequest &&request) {
        RobotInstant startInstant = request.start;
        RotationConstraints rotationConstraints = request.constraints.rot;
        Trajectory path = planWithoutAngles(std::move(request));
        assert(!path.empty());
        std::function<double(Point, Point, double)> angleFunction =
                [](Point pos, Point vel_linear, double angle) -> double {
                    return vel_linear.angle();
                };
        PlanAngles(path, startInstant, angleFunction, rotationConstraints);
        return std::move(path);
    }

    bool PathTargetPlanner::goalChanged(const RobotInstant &prevGoal,
                                        const RobotInstant &goal) const {
        double goalPosDiff = (prevGoal.pose.position() -
                              goal.pose.position()).mag();
        double goalVelDiff = (prevGoal.velocity.linear() -
                              goal.velocity.linear()).mag();
        return goalPosDiff > Planner::goalPosChangeThreshold()
               || goalVelDiff > Planner::goalVelChangeThreshold();
    }

} // namespace Planning
