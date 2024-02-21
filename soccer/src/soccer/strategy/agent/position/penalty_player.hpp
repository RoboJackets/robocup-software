#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {

/**
 * This position stays 0.15 meters behind the ball at all times.
 */
class PenaltyPlayer : public Position {
public:
    PenaltyPlayer(int r_id);
    ~PenaltyPlayer() = default;

    std::string return_current_state() override;

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
