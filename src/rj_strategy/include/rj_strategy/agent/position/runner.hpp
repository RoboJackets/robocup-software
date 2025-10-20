#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_command.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * This position attempts to run in a square
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() = default;

    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_pass() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_pass_ball() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // possible states of the Runner
    enum State {
        IDLING,          // doing nothing
        BOTTOM_LEFT,
        TOP_LEFT,
        TOP_RIGHT,
        BOTTOM_RIGHT
    };

    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    State next_state();
    rj_geometry::Point get_target_corner(Runner::State state);

    State state_{IDLING};


    rj_geometry::Point square_center_pos_;
    static constexpr double kSquareSize_{2.0};  // meters
    static constexpr double kTolerance_{0.1};  // meters
};

}  // namespace strategy