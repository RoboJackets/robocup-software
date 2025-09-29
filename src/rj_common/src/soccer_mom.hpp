#pragma once

#include <rclcpp/rclcpp.hpp>

namespace radio {
class SoccerMom : public Radio {
public:
    SoccerMom();

protected:
    void send_control_message(uint8_t robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                            const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                            strategy::Positions role);

    void poll_receive();

    void switch_team(bool blue_team);
}
}