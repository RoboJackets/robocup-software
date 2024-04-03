// This position is the robots that are not the kicker during the penalty kick state
// The robots should get behind the ball in preparation for the penalty, then they idle.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {
class PenaltyNonKicker : public Position {
public:
    PenaltyNonKicker(int r_id);
    ~PenaltyNonKicker() = default;
    PenaltyNonKicker(const Position& other);

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

    double distance_to_ball() const {
        return last_world_state_->ball.position.dist_to(
            last_world_state_->get_robot(true, robot_id_).pose.position());
    };

    // enum State { GET_AWAY, IDLING };

    // State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // State latest_state_ = GET_AWAY;
};
}  // namespace strategy