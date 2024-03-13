#pragma once

#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"

namespace strategy {
class SmartIdle : public Position {
public:
    Idle(int r_id);
    ~Idle() = default;
    Idle(const Position& other);

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

    enum {
        GET_AWAY,
        IDLING
    }

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};
}  // namespace strategy