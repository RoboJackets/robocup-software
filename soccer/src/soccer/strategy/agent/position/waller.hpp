#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>
#include <rj_utils/logging.hpp>
#include "rclcpp/utilities.hpp"

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"
#include "role_interface.hpp"


namespace strategy {

/*
 * The Waller role provides the implementation for a defensive robot that implements
 * this class to have a waller-like behavior where it aims to serve as a wall between
 * the ball and the goal.
 */
class Waller : public RoleInterface {
public:
    Waller(int waller_num, int robot_id, int total_wallers);
    ~Waller() = default;

    /**
     * @brief  Returns a waller behavior which aims to intercept the path
     * between the ball and the center of the goal
     *
     * @param [RobotIntent intent] [RobotIntent of the Defensive Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* world_state,
                                        FieldDimensions field_dimensions) override;

private:
    std::string defense_type_;
    int waller_pos_;
    int robot_id_;
    int total_wallers_;

    const float CLEAR_DIST = 0.75;

    static constexpr double robot_diameter_multiplier_ = 1.5;
};

}  // namespace strategy
