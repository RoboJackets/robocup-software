#pragma once

#include "motion_command.hpp"
#include "plan_request.hpp"
#include "planning/instant.hpp"
#include "planning/trajectory.hpp"

namespace planning {
class Planner {
public:
    explicit Planner(std::string name) : name_(std::move(name)) {}
    virtual ~Planner() = default;

    Planner(Planner&&) noexcept = default;
    Planner& operator=(Planner&&) noexcept = default;
    Planner(const Planner&) = default;
    Planner& operator=(const Planner&) = default;

    /**
     * Whether or not this command can be planned by this planner.
     *
     * Implemented by @ref PlannerForCommandType<T>
     *
     * @param command The command to check.
     * @return Whether or not this planner can plan that type of command.
     */
    [[nodiscard]] virtual bool is_applicable(
        const MotionCommand& command) const = 0;

    /**
     * Plan a trajectory for this request. This is guaranteed to be a request
     * that this planner is able to handle (according to @ref is_applicable)
     *
     * @param request The request to plan.
     * @return A trajectory for the robot to follow.
     */
    virtual Trajectory plan(const PlanRequest& request) = 0;

    /**
     * Reset this planner. Called after the planner is _not_ used to handle a
     * given command for the robot.
     */
    virtual void reset() {}

    /*
     * @return true if planner is done, false otherwise.
     */
    [[nodiscard]] virtual bool is_done() const = 0;

    /**
     * Get a user-readable name for this planner.
     */
    [[nodiscard]] std::string name() const { return name_; }

private:
    std::string name_;
};

template <typename CommandType>
class PlannerForCommandType : public Planner {
public:
    PlannerForCommandType(const std::string& name) : Planner(name){};
    ~PlannerForCommandType() override = default;

    PlannerForCommandType(const PlannerForCommandType& other) = default;
    PlannerForCommandType& operator=(const PlannerForCommandType& other) =
        default;
    PlannerForCommandType(PlannerForCommandType&& other) noexcept = default;
    PlannerForCommandType& operator=(PlannerForCommandType&& other) noexcept =
        default;

    [[nodiscard]] bool is_applicable(
        const MotionCommand& command) const override {
        return std::holds_alternative<CommandType>(command);
    }
};

}  // namespace planning
