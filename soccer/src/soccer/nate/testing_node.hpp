#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_utils/logging.hpp>

#include "rj_msgs/srv/plan_hypothetical_path.hpp"
#include "rj_msgs/msg/empty_motion_command.hpp"
#include "rj_geometry/point.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_common/time.hpp"

class TestingNode : public rclcpp::Node {
public:
    TestingNode(const rclcpp::NodeOptions& options);

private:
    // Put testing stuff here
    rclcpp::Client<rj_msgs::srv::PlanHypotheticalPath>::SharedPtr test_client_;

    void test_client();
};