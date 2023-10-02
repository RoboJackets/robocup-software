#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/*
 * This robot runs in a square path
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

private:

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
	    RIGHT,
	    UP,
	    LEFT,
	    DOWN
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = LEFT;
};

}  // namespace strategy
