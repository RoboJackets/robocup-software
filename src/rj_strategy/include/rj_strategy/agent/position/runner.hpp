#pragma once
#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {

/*
 * The Runner position handles runner behavior: running in a specified shape
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);

    std::string get_current_state() override;

    std::string get_state_name() const override {
        switch (current_state_) {
            case RUNNING_SIDE1:
                return "SIDE1";
            case RUNNING_SIDE2:
                return "SIDE2";
            case RUNNING_SIDE3:
                return "SIDE3";
            case RUNNING_SIDE4:
                return "SIDE4";
        }

        return "UNKNOWN";
    }

private:
    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // possible states of the Runner
    enum State {
        RUNNING_SIDE1,  // running on side 1 of the polygon
        RUNNING_SIDE2,  // running on side 2 of the polygon
        RUNNING_SIDE3,  // running on side 3 of the polygon
        RUNNING_SIDE4,  // running on side 4 of the polygon
    };

    /**
     * @return what the state should be right now. called on each get_task tick
     */
    State next_state();

    /**
     * @return the task to execute. called on each get_task tick AFTER next_state()
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = RUNNING_SIDE1;
};

}  // namespace strategy