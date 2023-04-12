#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/*
 * The Line position moves a robot to its designated position along a particular line
 */
class Line : public Position {
public:
    Line(int r_id, int scenario);
    ~Line() override = default;

    void derived_acknowledge_pass() override {};
    void derived_pass_ball() override {};
    void derived_acknowledge_ball_in_transit() override {};

private:

    int r_id_;
    int scenario_;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

};

}  // namespace strategy
