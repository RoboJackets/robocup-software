#include "testing_node.hpp"

TestingNode::TestingNode(const rclcpp::NodeOptions& options) : Node("testing_node", options) {
    test_client_ = this->create_client<rj_msgs::srv::PlanHypotheticalPath>("hypothetical_trajectory_robot_2");

    test_client();
}

void TestingNode::test_client() {
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = 2;
    auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};

    auto pt = rj_geometry::Point(2.0, 3.0);
    ptmc.target.position = rj_convert::convert_to_ros(pt);
    auto vel = rj_geometry::Point(0.0, 0.0);
    ptmc.target.velocity = rj_convert::convert_to_ros(vel);
    auto face_pt = rj_geometry::Point(1.0, 1.0);
    ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};

    intent.motion_command.path_target_command = {ptmc};

    auto request = std::make_shared<rj_msgs::srv::PlanHypotheticalPath::Request>();
    request->intent = intent;

    while (!test_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            SPDLOG_ERROR("Interrupted while waiting for the service");
        }
    }

    test_client_->async_send_request(
        request,
        [this](std::shared_future<rj_msgs::srv::PlanHypotheticalPath::Response::SharedPtr> response) {
            test_callback(response);
        }
    );
}

void TestingNode::test_callback(std::shared_future<rj_msgs::srv::PlanHypotheticalPath::Response::SharedPtr> response) {
    auto time = rj_convert::convert_from_ros(response.get()->estimate);
    SPDLOG_INFO("\033[94mRESULT: {}\033[0m", time.count());
}