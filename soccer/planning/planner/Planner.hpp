#pragma once

#include "MotionCommand.hpp"
#include "PlanRequest.hpp"
#include "planning/trajectory/Trajectory.hpp"

namespace Planning {
class Planner {
public:
    explicit Planner(std::string name): _name(name) {}
    virtual ~Planner() = default;

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
    virtual Trajectory plan(PlanRequest&& request) = 0;

    /**
     * reuse previous path
     */
    static Trajectory reuse(PlanRequest&& request);

    /**
     * Get a user-readable name for this planner.
     */
    std::string name() const {
        return _name;
    }

private:
    const std::string _name;
};

template<typename CommandType>
class PlannerForCommandType : public Planner {
public:
    PlannerForCommandType(const std::string& name): Planner(name) {};
    ~PlannerForCommandType() override = default;

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<CommandType>(command);
    }
};

template <typename T>
inline T applyLowPassFilter(const T& oldValue, const T& newValue,
                     double gain) {
    return gain * newValue + (1 - gain) * oldValue;
}
}
