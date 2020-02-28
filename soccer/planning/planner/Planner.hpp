#pragma once

#include "MotionCommand.hpp"
#include "PlanRequest.hpp"
#include "planning/trajectory/Trajectory.hpp"

namespace Planning {
class Planner {
public:
    Planner() = default;
    virtual ~Planner() = default; //todo(Ethan) add virtual destructors to all the planners

    /**
     * Whether or not this command can be planned by this planner.
     *
     * Implemented by @ref PlannerForCommandType<T>
     *
     * @param command The command to check.
     * @return Whether or not this planner can plan that type of command.
     */
    virtual bool isApplicable(const MotionCommand& command) const = 0;

    /**
     * Plan a trajectory for this request. This is guaranteed to be a request
     * that this planner is able to handle (according to @ref isApplicable)
     *
     * @param request The request to plan.
     * @return A trajectory for the robot to follow.
     */
    virtual Trajectory plan(PlanRequest&& request) = 0;//todo(Ethan) maybe return just a RobotInstant bc ROS

    /**
     * Get a user-readable name for this planner.
     */
    virtual std::string name() const = 0;

    /**
     * create configuration for Qt
     * @param cfg configuration
     */
    static void createConfiguration(Configuration* cfg);

    /**
     * get threshold for change in target position for replanning
     * @return threshold
     */
    static double goalPosChangeThreshold() { return *_goalPosChangeThreshold; }

    /**
     * get threshold for change in target velocity for replanning
     * @return threshold
     */
    static double goalVelChangeThreshold() { return *_goalVelChangeThreshold; }
    /**
     * get replan timeout
     * @return timeout
     */
    static double replanTimeout() { return *_replanTimeout; }

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;
    static ConfigDouble* _replanTimeout;
};

template<typename CommandType>
class PlannerForCommandType : public Planner {
public:
    PlannerForCommandType() = default;
    ~PlannerForCommandType() override = default;

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<CommandType>(command);
    }
};

}
