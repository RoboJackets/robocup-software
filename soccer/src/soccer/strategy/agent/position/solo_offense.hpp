#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/**
 * The SoloOffense position is a simple offensive template which
 * gets posession and shoots.
 */
class SoloOffense : public Position {
public:
    SoloOffense(const Position& other);
    SoloOffense(int r_id);
    ~SoloOffense() override = default;
    SoloOffense(const SoloOffense& other) = default;
    SoloOffense(SoloOffense&& other) = default;

    std::string get_current_state() override;

private:
    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // States for position.
    enum State {
        DEFAULT,
        MARKER,
        SHOOTING_START,  // collect
        SHOOTING_PIVOT,  // face goal
        SHOOTING_KICK,   // score
    };

    State current_state_ = State::DEFAULT;

    /**
     * @brief A more custom to-string for debugging better.
     */
    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case DEFAULT:
                return "DEFAULT";
            case MARKER:
                return "MARKER";
            case SHOOTING_START:
                return "SHOOTING_START";
            case SHOOTING_PIVOT:
                return "SHOOTING_PIVOT";
            case SHOOTING_KICK:
                return "SHOOTING_KICK";
        }
    }

    /**
     * @brief This FSM has timeouts for certain states.
     * Ideally, these would not be necessary; as planners get more sophisticated
     * they should not get "stuck".
     *
     * The timeouts are a safety mechanism, and should not be the primary reason for a
     * state transition. They are set relatively high for this reason. Here, they're
     * currently used for debugging.
     *
     * @return the maximum duration to stay in a given state, or -1 if there is no maximum.
     */
    static constexpr RJ::Seconds timeout(State s) {
        switch (s) {
            case DEFAULT:
                return RJ::Seconds{-1};
            case MARKER:
                return RJ::Seconds{-1};
            case SHOOTING_START:
                return RJ::Seconds{10};
            case SHOOTING_PIVOT:
                return RJ::Seconds{10};
            case SHOOTING_KICK:
                return RJ::Seconds{10};
        }
    }

    // The time at which the last state started.
    RJ::Time last_time_;

    /**
     * @brief Reset the timeout for the current state
     */
    void reset_timeout() { last_time_ = RJ::now(); }

    /**
     * @return if the current state has timed out
     */
    bool timed_out() const {
        using namespace std::chrono_literals;
        const auto max_time = timeout(current_state_);
        return (max_time > 0s) && (last_time_ + max_time < RJ::now());
    };

    /**
     * @return what the state should be right now. called on each get_task tick
     */
    State next_state();

    /**
     * @return the task to execute. called on each get_task tick AFTER next_state()
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // The shell id of the opponent robot that is closest to the ball (to "mark" them).
    int marking_id_;

    // Distances
    static constexpr double kTightPosessionRadius{kRobotRadius + 0.1};
    static constexpr double kPosessionRadius{kRobotRadius + 0.5};
    static constexpr double kMarkDistance{kRobotRadius * 6};

    // The best shot available.
    rj_geometry::Point shot_target_;

    /**
     * @return the best shot available, always tries something
     */
    rj_geometry::Point calculate_best_shot() const;

    /**
     * @return the distance from opponent robots, used as a heuristic
     */
    double distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const;
};

}  // namespace strategy
