#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

// assuming these are resources rj made that are relevant
#include <rj_common/planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/runner.hpp"
#include "rj_strategy/agent/position/seeker.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {

/**
 * The Runner position is a tutorial
 */
class Runner : public Position {
    /*
     * public class: slop
     */
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);

    // probably universal to all positions or whatever
    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string(state_to_name(current_state_));
    }

private:
    // 1. define states to the shape
    enum class StarState { STAR_1, STAR_2, STAR_3, STAR_4, STAR_5 };

    // 2. ticking functions, overriding from position
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // 3. helper to advance to next state/star machine
    void next_state();

    // 4. debuggers; it would be cringe if it returns unknown
    static constexpr std::string_view state_to_name(StarState s) {
        switch (s) {
            case StarState::STAR_1:
                return "STAR_1";
            case StarState::STAR_2:
                return "STAR_2";
            case StarState::STAR_3:
                return "STAR_3";
            case StarState::STAR_4:
                return "STAR_4";
            case StarState::STAR_5:
                return "STAR_5";
        }
        return "UNKNOWN";
    };

    // 5. minimal member variables : i dont know what that means
    // oh just setting default
    StarState current_state_ = StarState::STAR_1;
};
}  // namespace strategy