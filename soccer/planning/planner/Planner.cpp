#include "planning/planner/Planner.hpp"
namespace Planning {

    REGISTER_CONFIGURABLE(Planner);

    ConfigDouble* Planner::_goalPosChangeThreshold;
    ConfigDouble* Planner::_goalVelChangeThreshold;
    ConfigDouble* Planner::_replanTimeout;

    void Planner::createConfiguration(Configuration* cfg) {
        _replanTimeout = new ConfigDouble(cfg, "PlannerForCommandType/replanTimeout", 5);
        _goalPosChangeThreshold =
                new ConfigDouble(cfg, "PlannerForCommandType/goalPosChangeThreshold", 0.025);
        _goalVelChangeThreshold =
                new ConfigDouble(cfg, "PlannerForCommandType/goalVelChangeThreshold", 0.025);
    }

    bool Planner::veeredOffPath(const PlanRequest& request) const {
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

    Trajectory Planner::reuse(PlanRequest&& request) {
        Trajectory& prevTrajectory = request.prevTrajectory;
        if(prevTrajectory.empty()) {
            return Trajectory{{request.start}};
        }
        RJ::Seconds timeElapsed = RJ::now() - prevTrajectory.begin_time();
        if(timeElapsed < prevTrajectory.duration()) {
            prevTrajectory.trimFront(timeElapsed);
            return std::move(prevTrajectory);
        }
        return Trajectory{{prevTrajectory.last()}};
    }
}