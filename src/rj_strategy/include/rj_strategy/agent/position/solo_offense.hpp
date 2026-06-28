#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {

class SoloOffense : public Position {
public:
    SoloOffense(Position&& other);
    SoloOffense(int r_id);
    ~SoloOffense() override = default;
    SoloOffense(const SoloOffense& other) = default;
    SoloOffense(SoloOffense&& other) = default;

    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string(state_to_name(current_state_));
    }

private:
    enum State {
        IDLE,     // The nothing doer
        MARKER,   // Aggressively sit between ball and goal pos
        TO_BALL,  // Collect
        ROTATE,   // After successful collect, aim and fire
        KICK      // The more naive line kick
    };
    State kick_strategy_ =
        TO_BALL;  // set to TO_BALL for collect kicking, set to KICK for line kicking
    planning::LinearMotionInstant kick_target_;  // aiming point

    State current_state_ = IDLE;

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case IDLE:
                return "IDLE";
            case MARKER:
                return "MARKER";
            case TO_BALL:
                return "TO_BALL";
            case ROTATE:
                return "ROTATE";
            case KICK:
                return "KICK";
            default:
                return "unknown?";
        }
    }

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    State next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};

}  // namespace strategy
