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

namespace strategy {

class SoloOffense : public Position {
public:
    SoloOffense(const Position& other);
    SoloOffense(int r_id);
    ~SoloOffense() override = default;
    SoloOffense(const SoloOffense& other) = default;
    SoloOffense(SoloOffense&& other) = default;

    std::string get_current_state() override;

private:

    // State space.
    enum State { DEFAULT, TO_BALL, KICK };
    State current_state_ = DEFAULT;
    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case DEFAULT:
                return "DEFAULT";
            case TO_BALL:
                return "TO_BALL";
            case KICK:
                return "KICK";
        }
        return "NULL";
    }

    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    SoloOffense::State next_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    
    // Timeout forcing.
    RJ::Time last_time_;
    void reset_timeout() { last_time_ = RJ::now(); }
    bool timed_out() const {
        using namespace std::chrono_literals;
        return last_time_ + 4s < RJ::now();
    };

    // Shot calculation.
    rj_geometry::Point shot_target_;
    rj_geometry::Point calculate_best_shot() const;
    double shot_clearance(rj_geometry::Point tail, rj_geometry::Point head) const;

    // Ball position
    rj_geometry::Point cached_ball_pos_;
    rj_geometry::Point get_ball_pos() const;
    bool point_in_red(rj_geometry::Point concerned_point) const;

    // Line-kick needs space to operate. This dictates how far backward to go.
    static constexpr double kBackOffset = 4 * kRobotRadius;
};

}  // namespace strategy
