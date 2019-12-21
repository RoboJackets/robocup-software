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

    std::vector<RJ::Time> PathTargetPlanner::prevTimes{Num_Shells, RJ::now()-60s};

    Trajectory
    PathTargetPlanner::planWithoutAngles(Planning::PlanRequest &&request) {
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
            return fullReplan(std::move(request));
        }

        const RJ::Seconds timeIntoTrajectory =
                RJ::now() - prevTrajectory.begin_time();
        const RJ::Seconds timeRemaining =
                prevTrajectory.duration() - timeIntoTrajectory;

        RJ::Seconds invalidTime = prevTrajectory.duration();
        bool dynamicHit = prevTrajectory.intersects(request.dynamic_obstacles, RJ::now(), nullptr, &invalidTime);
        bool staticHit = prevTrajectory.hit(request.static_obstacles, timeIntoTrajectory, &invalidTime);
        bool changeOfGoal = goalChanged(prevTrajectory.last(), goalInstant);
        if (dynamicHit || staticHit || changeOfGoal) {
            if (invalidTime - timeIntoTrajectory < RJ::Seconds(*_partialReplanLeadTime * 2)) {
                return fullReplan(std::move(request));
            }
            return partialReplan(std::move(request));
        }
        if (RJ::now() - prevTimes[request.shellID] > 0.2s && timeRemaining > RJ::Seconds(*_partialReplanLeadTime * 2)) {
            return checkBetter(std::move(request));
        }
        return reuse(std::move(request));
    }

    Trajectory PathTargetPlanner::reuse(PlanRequest&& request) {
        return request.prevTrajectory.empty() ? Trajectory{{request.start}} : std::move(request.prevTrajectory);
    }

    Trajectory PathTargetPlanner::checkBetter(PlanRequest&& request) {
        Trajectory& prevTrajectory = request.prevTrajectory;
        std::shared_ptr<RoboCupStateSpace> stateSpace = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, std::move(request.static_obstacles));
        RobotInstant goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        const RJ::Seconds timeIntoTrajectory =
                RJ::now() - prevTrajectory.begin_time();
        Trajectory preTrajectory = partialPath(prevTrajectory);
        RobotInstant middleInstant = preTrajectory.last();
        Trajectory postTrajectory = RRTTrajectory(middleInstant, goalInstant, request.constraints.mot, request.static_obstacles);
        if (!postTrajectory.empty()) {
            Trajectory comboPath{std::move(preTrajectory),std::move(postTrajectory)};
            if (prevTrajectory.duration() - timeIntoTrajectory > comboPath.duration()) {
                std::cout << "Found A better Path!!!!" << std::endl;
                prevTimes[request.shellID] = RJ::now();
                return std::move(comboPath);
            }
        }
        return std::move(prevTrajectory);
    }

    Trajectory PathTargetPlanner::partialReplan(PlanRequest&& request) {
        RobotInstant startInstant = request.start;
        RobotInstant goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Trajectory& prevTrajectory = request.prevTrajectory;
        std::vector<Point> biasWaypoints;
        biasWaypoints.push_back(startInstant.pose.position() +
                                startInstant.velocity.linear() * 0.2);
        for (auto it = prevTrajectory.iterator(RJ::now(), 100ms);
             (*it).stamp < prevTrajectory.end_time(); ++it) {
            biasWaypoints.push_back((*it).pose.position());
        }
        Trajectory preTrajectory = partialPath(prevTrajectory);
        Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, biasWaypoints);
        if (postTrajectory.empty()) {
            return fullReplan(std::move(request));
        }
        Trajectory comboPath = Trajectory(std::move(preTrajectory),
                                          std::move(postTrajectory));
        prevTimes[request.shellID] = RJ::now();
        return std::move(comboPath);
    }

    Trajectory PathTargetPlanner::fullReplan(PlanRequest&& request) {
        const RobotInstant& goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Trajectory path = RRTTrajectory(request.start, goalInstant, request.constraints.mot, request.static_obstacles);
        if(path.empty()) {
            return reuse(std::move(request));
        }
        prevTimes[request.shellID] = RJ::now();
        return std::move(path);
    }

    Trajectory PathTargetPlanner::plan(PlanRequest &&request) {
        if (std::holds_alternative<WorldVelTargetCommand>(request.motionCommand)) {
            Point targetVelLinear = std::get<WorldVelTargetCommand>(request.motionCommand).worldVel.linear();
            Point targetPoint = request.start.pose.position() + targetVelLinear.normalized(Robot_Diameter);
            RobotInstant pathTarget{Pose{targetPoint, targetVelLinear.angle()}, Twist{targetVelLinear, 0}, RJ::now()};
            request.motionCommand = PathTargetCommand{pathTarget};
            request.constraints.mot.maxSpeed = targetVelLinear.mag();
        }
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
