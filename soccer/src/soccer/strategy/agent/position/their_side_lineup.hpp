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
 * Vertical line on our side
 */
class TheirSideLineup : public Position {
public:
    TheirSideLineup(int r_id);
    ~TheirSideLineup() override = default;

    void derived_acknowledge_pass() override{};
    void derived_pass_ball() override{};
    void derived_acknowledge_ball_in_transit() override{};

private:
    int r_id_{};

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    rj_geometry::Point init_ball_pos_{};
    bool init_ball_pos_valid_{false};
};

}  // namespace strategy
