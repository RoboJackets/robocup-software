#pragma once

#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/**
 * This position constantly attempts to kick the ball into the goal.
 */
class GoalKicker : public Position {
public:
    GoalKicker(int r_id);
    ~GoalKicker() = default;

    string return_current_state() override;
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

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy