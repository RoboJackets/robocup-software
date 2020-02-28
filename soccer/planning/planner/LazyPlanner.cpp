#include "Planner.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "planning/planner/EscapeObstaclesPathPlanner.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <vector>

namespace Planning {
    ConfigDouble* LazyPlanner::_partialReplanLeadTime;

    REGISTER_CONFIGURABLE(LazyPlanner);

    void LazyPlanner::createConfiguration(Configuration* cfg) {
        _partialReplanLeadTime = new ConfigDouble(cfg, "LazyPlanner/partialReplanLeadTime", 0.0);
    }

    Trajectory LazyPlanner::reuse(PlanRequest&& request) {
        Trajectory& prevTrajectory = request.prevTrajectory;
        if(prevTrajectory.empty()) {
            Trajectory out{{request.start}};
            out.setDebugText("Empty");
            return std::move(out);
        }
        RJ::Seconds timeElapsed = RJ::now() - prevTrajectory.begin_time();
        if(timeElapsed < prevTrajectory.duration()) {
            prevTrajectory.trimFront(timeElapsed);
            Trajectory out = std::move(prevTrajectory);
            out.setDebugText("Reuse");
            return std::move(out);
        }
        Trajectory out{{prevTrajectory.last()}};
        out.setDebugText("Reusing Past End");
        return std::move(out);
    }

    bool LazyPlanner::veeredOffPath(const PlanRequest& request) const {
        const auto &currentInstant = request.start;
        const MotionConstraints &motionConstraints = request.constraints.mot;
        const Trajectory& prevTrajectory = request.prevTrajectory;
        if (prevTrajectory.empty()) return false;
        RJ::Seconds timeIntoPath =
                (RJ::now() - prevTrajectory.begin_time()) + RJ::Seconds(1) / 60;
        std::optional<RobotInstant> optTarget = prevTrajectory.evaluate(timeIntoPath);
        // If we went off the end of the path, use the end for calculations.
        RobotInstant target = optTarget ? *optTarget : prevTrajectory.last();
        // invalidate path if current position is more than the replanThreshold away
        // from where it's supposed to be right now
        float pathError = (target.pose.position() - currentInstant.pose.position()).mag();
        float replanThreshold = *motionConstraints._replan_threshold;
        if (replanThreshold != 0 && pathError > replanThreshold) {
            return true;
        }
        return false;
    }

    // todo(Ethan) use pathGoal heading and angular velocity?
    /**
     * Implement lazy planning. depending on the situation we full replan, partial
     * replan, or reuse the previous path.
     * @tparam CommandType
     * @param request
     * @return trajectory
     */
    Trajectory LazyPlanner::plan(PlanRequest &&request) {
        using namespace Geometry2d;
        //setup angle function and motion command
        AngleFunction angleFn = getAngleFunction(request);

        std::optional<double> angle_override =
                request.context->robot_intents[request.shellID].angle_override;
        if(angle_override) {
            angleFn = AngleFns::faceAngle(*angle_override);
        }

        const Trajectory& prevTrajectory = request.prevTrajectory;

        // adjust the requested goal out of an obstacle if necessary
        std::optional<Point> prevGoal;
        if (!prevTrajectory.empty()) {
            prevGoal = prevTrajectory.last().pose.position();
        }
        request.motionCommand = PathTargetCommand{getGoalInstant(request)};
        RobotInstant& goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Point& goalPoint = goalInstant.pose.position();
        goalPoint = EscapeObstaclesPathPlanner::findNonBlockedGoal(goalPoint,
                                                                   prevGoal,
                                                                   request.static_obstacles);
        // Simple case: no path
        //todo(Ethan) maybe delete this handle it in RRTTrajectory() ?
        if (request.start.pose.position().distTo(goalPoint) < 1e-6) {
            std::list<RobotInstant> instants;
            instants.emplace_back(request.start.pose, Twist(), RJ::now());
            Trajectory result{std::move(instants)};
            result.setDebugText("RRT Basic");
            return std::move(result);
        }
        if (prevTrajectory.empty() || veeredOffPath(request)) {
            return fullReplan(std::move(request), angleFn);
        }
        RJ::Seconds timeIntoTrajectory =
                std::clamp(RJ::Seconds{RJ::now() - prevTrajectory.begin_time()}, RJ::Seconds{0s}, prevTrajectory.duration());
        const RJ::Seconds timeRemaining =
                prevTrajectory.duration() - timeIntoTrajectory;

        RJ::Seconds invalidTime;
        //note: the dynamic check is expensive, so we shortcut it sometimes
        //todo(Ethan) this could be made more efficient by trimming old trajectories
        bool shouldPartialReplan = prevTrajectory.hit(request.static_obstacles, timeIntoTrajectory, &invalidTime)
                                   || prevTrajectory.intersects(request.dynamic_obstacles, RJ::now(), nullptr, &invalidTime);
        if(!shouldPartialReplan && goalChanged(prevTrajectory.last(), goalInstant)) {
            shouldPartialReplan = true;
            invalidTime = prevTrajectory.duration();
        }
        if (shouldPartialReplan) {
            if (invalidTime - timeIntoTrajectory < partialReplanLeadTime() * 2) {
                return fullReplan(std::move(request), angleFn);
            }
            return partialReplan(std::move(request), angleFn);
        }
        // make fine corrections when we are realy close to the target
        // because the old target might be a bit off
        if(request.start.pose.position().distTo(goalPoint) < Robot_Radius) {
            std::optional<RobotInstant> nowInstant = prevTrajectory.evaluate(RJ::now());
            if (nowInstant) {
                request.start = *nowInstant;
                return fullReplan(std::move(request), angleFn);
            }
        }
        if (RJ::now() - prevTimes[request.shellID] > 0.2s && timeRemaining > partialReplanLeadTime() * 2) {
            return checkBetter(std::move(request), angleFn);
        }
        return reuse(std::move(request));
    }

    AngleFunction LazyPlanner::getAngleFunction(const PlanRequest& request) const {
        //todo(Ethan) this is probably the wrong angle function to use by default
        return AngleFns::tangent;
    }

    Trajectory LazyPlanner::checkBetter(PlanRequest&& request, AngleFunction angleFunction) {
        Trajectory& prevTrajectory = request.prevTrajectory;
        std::shared_ptr<RoboCupStateSpace> stateSpace = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, std::move(request.static_obstacles));
        RobotInstant goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        const RJ::Seconds timeIntoTrajectory =
                RJ::now() - prevTrajectory.begin_time();
        Trajectory preTrajectory = partialPath(prevTrajectory);
        Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
        PlanAngles(postTrajectory, preTrajectory.last(), angleFunction, request.constraints.rot);
        if (!postTrajectory.empty()) {
            Trajectory comboPath{std::move(preTrajectory),std::move(postTrajectory)};
            if (prevTrajectory.duration() - timeIntoTrajectory > comboPath.duration()) {
                std::cout << "Found A better Path!!!!" << std::endl;
                updatePrevTime(request.shellID);
                return std::move(comboPath);
            }
        }
        return reuse(std::move(request));
    }

    Trajectory LazyPlanner::partialReplan(PlanRequest&& request, AngleFunction angleFunction) {
        RobotInstant goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Trajectory& prevTrajectory = request.prevTrajectory;
        std::vector<Geometry2d::Point> biasWaypoints;
        for (auto it = prevTrajectory.iterator(RJ::now(), 100ms);
             (*it).stamp < prevTrajectory.end_time(); ++it) {
            biasWaypoints.push_back((*it).pose.position());
        }
        Trajectory preTrajectory = partialPath(prevTrajectory);
        Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles, biasWaypoints);
        if (postTrajectory.empty()) {
            return fullReplan(std::move(request), angleFunction);
        }
        PlanAngles(postTrajectory, preTrajectory.last(), angleFunction, request.constraints.rot);
        Trajectory comboPath = Trajectory(std::move(preTrajectory),
                                          std::move(postTrajectory));
        updatePrevTime(request.shellID);
        return std::move(comboPath);
    }

    Trajectory LazyPlanner::fullReplan(PlanRequest&& request, AngleFunction angleFunction) {
        const RobotInstant& goalInstant = std::get<PathTargetCommand>(request.motionCommand).pathGoal;
        Trajectory path = RRTTrajectory(request.start, goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
        if(path.empty()) {
            return reuse(std::move(request));
        }
        PlanAngles(path, request.start, angleFunction, request.constraints.rot);
        updatePrevTime(request.shellID);
        return std::move(path);
    }

    bool LazyPlanner::goalChanged(const RobotInstant &prevGoal,
                                        const RobotInstant &goal) const {
        double goalPosDiff = (prevGoal.pose.position() -
                              goal.pose.position()).mag();
        double goalVelDiff = (prevGoal.velocity.linear() -
                              goal.velocity.linear()).mag();
        return goalPosDiff > Planner::goalPosChangeThreshold()
               || goalVelDiff > Planner::goalVelChangeThreshold();
    }
}