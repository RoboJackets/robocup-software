#include "rj_radio/radio.hpp"

namespace radio {

Radio::Radio()
    : Node{"radio", rclcpp::NodeOptions{}
                        .automatically_declare_parameters_from_overrides(true)}
{
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            if (color->is_blue != blue_team_) {
                blue_team_ = color->is_blue;
                switch_team(color->is_blue);
            }
        });

    for (size_t i = 0; i < kNumShells; i++) {
        robot_status_pubs_.at(i) = create_publisher<rj_msgs::msg::RobotStatus>(
            topics::robot_status_topic(static_cast<int>(i)), rclcpp::QoS(1));
        control_subs_.at(i) = create_subscription<control::ControlCommand::Msg>(
            fmt::format("/robot_{}/control", i), rclcpp::QoS(1),
            //NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this, i](const control::ControlCommand::Msg::SharedPtr msg) {
                last_updates_.at(i) = RJ::now();
                control_commands_.at(i) = rj_convert::convert_from_ros(*msg);
            }
        );
    }

    alive_robots_pub_ =
        create_publisher<rj_msgs::msg::AliveRobots>(topics::kAliveRobotsTopic, rclcpp::QoS(1));

    tick_timer_ = create_wall_timer(tick_period_, [this]() { tick(); });
}

void Radio::publish_robot_status(int robot_id, const rj_msgs::msg::RobotStatus& robot_status) {
    robot_status_pubs_.at(robot_id)->publish(robot_status);
}

void Radio::publish_alive_robots(const rj_msgs::msg::AliveRobots& alive_robots) {
    alive_robots_pub_->publish(alive_robots);
}

bool Radio::blue_team() const { return blue_team_; }

void Radio::tick() {
    poll_receive();

    for (size_t i = 0; i < kNumShells; i++) {
        send_control_message(i, control_commands_.at(i));
    }
}

}  // namespace radio
