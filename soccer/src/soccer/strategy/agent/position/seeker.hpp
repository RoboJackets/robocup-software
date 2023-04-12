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
#include "role_interface.hpp"

namespace strategy {
/*
The seeker role provides the offensive movement for moving off the ball and opening up for a pass.
*/
class Seeker : public RoleInterface {
public:
    Seeker(int seeker_num);
    ~Seeker() = default;

    /**
     * @brief  Returns a seeker behavior which aims to move to an optimal location.
     *
     * @param [RobotIntent intent] [RobotIntent of the Offensive Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* world_state,
                                        FieldDimensions field_dimensions) override;

private:
    rj_geometry::Point get_open_point(const WorldState* world_state, rj_geometry::Point current_loc,
                                      FieldDimensions field_dimensions);
    rj_geometry::Point correct_point(rj_geometry::Point p, FieldDimensions field_dimensions);
    rj_geometry::Point random_noise(double prec);
    rj_geometry::Point calculate_open_point(double, double, rj_geometry::Point,
                                            const WorldState* world_state,
                                            FieldDimensions field_dimensions);
    double max_los(rj_geometry::Point, rj_geometry::Point, const WorldState* world_state);

    int seeker_pos_;
    std::string offense_type_;
};
}  // namespace strategy
