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

    //todo(Ethan) check for target change --> replan
    bool Planner::shouldReplan(const PlanRequest &planRequest) const {
        const auto& currentInstant = planRequest.start;
        const MotionConstraints &motionConstraints = planRequest.constraints.mot;
        const Geometry2d::ShapeSet &obstacles = planRequest.obstacles;
        const Trajectory& prevTrajectory = planRequest.prevTrajectory;

        if (prevTrajectory.empty()) return true;

        // if this number of microseconds passes since our last path plan, we
        // automatically replan
        // const RJ::Seconds kPathExpirationInterval = RJ::Seconds(replanTimeout());
        // if ((RJ::now() - prevPath->startTime()) > kPathExpirationInterval) {
        //    return true;
        //}

        // Evaluate where the path says the robot should be right now
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

        // Replan if we enter new obstacles
        RJ::Seconds hitTime;
        if (prevTrajectory.hit(obstacles, timeIntoPath, &hitTime)) {
            return true;
        }

        return false;
    }
}