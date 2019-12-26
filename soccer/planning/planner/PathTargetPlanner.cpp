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


    Trajectory PathTargetPlanner::plan(PlanRequest &&request) {
        //setup angle function and motion command
        AngleFunction angleFunction;
        if (std::holds_alternative<WorldVelTargetCommand>(request.motionCommand)) {
            Point targetVelLinear = std::get<WorldVelTargetCommand>(request.motionCommand).worldVel.linear();
            Point targetPoint = request.start.pose.position() + targetVelLinear.normalized(Robot_Diameter);
            RobotInstant pathTarget{Pose{targetPoint, targetVelLinear.angle()}, Twist{targetVelLinear, 0}, RJ::now()};
            request.motionCommand = PathTargetCommand{pathTarget};
            request.constraints.mot.maxSpeed = targetVelLinear.mag();
            angleFunction = AngleFns::facePoint(targetPoint);
        } else {
            RJ::Time t0 = request.prevTrajectory.begin_time();
            angleFunction = AngleFns::tangent;
        }
        std::optional<double> angle_override =
                request.context->robot_intents[request.shellID].angle_override;
        if(angle_override) {
            angleFunction = AngleFns::faceAngle(*angle_override);
        }

        const Trajectory& prevTrajectory = request.prevTrajectory;

        // adjust the requested goal out of an obstacle if necessary
        std::optional<Point> prevGoal;
        if (!prevTrajectory.empty()) {
            prevGoal = prevTrajectory.last().pose.position();
        }
        RobotInstant& goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Point& goalPoint = goalInstant.pose.position();
        goalPoint = EscapeObstaclesPathPlanner::findNonBlockedGoal(goalPoint,
                                                                   prevGoal,
                                                                   request.static_obstacles);
        // Simple case: no path
        if (request.start.pose.position() == goalPoint) {
            std::vector<RobotInstant> instants;
            instants.emplace_back(request.start.pose, Twist(), RJ::now());
            Trajectory result{std::move(instants)};
            result.setDebugText("RRT Basic");
            return std::move(result);
        }
        if (prevTrajectory.empty() || veeredOffPath(request)) {
            printf("full replan -> ");
            return fullReplan(std::move(request), angleFunction);
        }

        const RJ::Seconds timeIntoTrajectory =
                RJ::now() - prevTrajectory.begin_time();
        const RJ::Seconds timeRemaining =
                prevTrajectory.duration() - timeIntoTrajectory;

        RJ::Seconds invalidTime;
        //note: the dynamic check is expensive, so we shortcut it sometimes
        bool shouldPartialReplan = prevTrajectory.hit(request.static_obstacles, timeIntoTrajectory, &invalidTime)
                || prevTrajectory.intersects(request.dynamic_obstacles, RJ::now(), nullptr, &invalidTime);
        if(!shouldPartialReplan && goalChanged(prevTrajectory.last(), goalInstant)) {
            shouldPartialReplan = true;
            invalidTime = prevTrajectory.duration();
        }
        if (shouldPartialReplan) {
            if (invalidTime - timeIntoTrajectory < RJ::Seconds(*_partialReplanLeadTime) * 2) {
//                if(!prevTrajectory.empty()) {
//                    // This handles moving targets. We don't want to use request.start
//                    // because repeatedly replanning using the robot's current instant
//                    // causes the robot to wander off course
//                    std::optional<RobotInstant> inst = prevTrajectory.evaluate(RJ::now());
//                    if(inst) request.start = *inst;
//                }
                printf("full replan -> ");
                return fullReplan(std::move(request), angleFunction);
            }
            printf("partial replan -> ");
            return partialReplan(std::move(request), angleFunction);
        }
        if (RJ::now() - prevTimes[request.shellID] > 0.2s && timeRemaining > RJ::Seconds(*_partialReplanLeadTime * 2)) {
            printf("check better -> ");
            return checkBetter(std::move(request));
        }
        return reuse(std::move(request));
    }

    Trajectory PathTargetPlanner::reuse(PlanRequest&& request) {
        printf("reuse\n");
        return request.prevTrajectory.empty() ? Trajectory{{request.start}} : std::move(request.prevTrajectory);
    }

    Trajectory PathTargetPlanner::checkBetter(PlanRequest&& request) {
        Trajectory& prevTrajectory = request.prevTrajectory;
        std::shared_ptr<RoboCupStateSpace> stateSpace = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, std::move(request.static_obstacles));
        RobotInstant goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        const RJ::Seconds timeIntoTrajectory =
                RJ::now() - prevTrajectory.begin_time();
        Trajectory preTrajectory = partialPath(prevTrajectory);
        Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
        if (!postTrajectory.empty()) {
            Trajectory comboPath{std::move(preTrajectory),std::move(postTrajectory)};
            if (prevTrajectory.duration() - timeIntoTrajectory > comboPath.duration()) {
                std::cout << "Found A better Path!!!!" << std::endl;
                prevTimes[request.shellID] = RJ::now();
                return std::move(comboPath);
            }
        }
        return reuse(std::move(request));
    }

    Trajectory PathTargetPlanner::partialReplan(PlanRequest&& request, AngleFunction angleFunction) {
        RobotInstant goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Trajectory& prevTrajectory = request.prevTrajectory;
        std::vector<Point> biasWaypoints;
        for (auto it = prevTrajectory.iterator(RJ::now(), 100ms);
             (*it).stamp < prevTrajectory.end_time(); ++it) {
            biasWaypoints.push_back((*it).pose.position());
        }
        Trajectory preTrajectory = partialPath(prevTrajectory);
        Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles, biasWaypoints);
        if (postTrajectory.empty()) {
            std::cout << "Partial Failed --> Full Replan" << std::endl;
            return fullReplan(std::move(request), angleFunction);
        }
        if(anglePlanningEnabled) {
            PlanAngles(postTrajectory, preTrajectory.last(), angleFunction, request.constraints.rot);
        }
        Trajectory comboPath = Trajectory(std::move(preTrajectory),
                                          std::move(postTrajectory));
        prevTimes[request.shellID] = RJ::now();
        return std::move(comboPath);
    }

    Trajectory PathTargetPlanner::fullReplan(PlanRequest&& request, AngleFunction angleFunction) {
        const RobotInstant& goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Trajectory path = RRTTrajectory(request.start, goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
        if(path.empty()) {
            return reuse(std::move(request));
        }
        if(anglePlanningEnabled) {
            PlanAngles(path, request.start, angleFunction, request.constraints.rot);
        }
        prevTimes[request.shellID] = RJ::now();
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
