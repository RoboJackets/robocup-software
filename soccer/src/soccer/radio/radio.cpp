#include "radio.hpp"

namespace radio {

DEFINE_FLOAT64(kRadioParamModule, timeout, 0.25,
               "Timeout after which radio will assume a robot is disconnected. Seconds.");

Radio::Radio() : rclcpp::Node("radio"), param_provider_(this, kRadioParamModule) {
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            switch_team(color->is_blue);
        });
    for (int i = 0; i < kNumShells; i++) {
        robot_status_pubs_.at(i) = create_publisher<rj_msgs::msg::RobotStatus>(
            topics::robot_status_pub(i), rclcpp::QoS(1));
        manipulator_subs_.at(i) = create_subscription<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_pub(i), rclcpp::QoS(1),
            [this, i](rj_msgs::msg::ManipulatorSetpoint::SharedPtr manipulator) {  // NOLINT
                manipulators_cached_.at(i) = *manipulator;
            });
        motion_subs_.at(i) = create_subscription<rj_msgs::msg::MotionSetpoint>(
            control::topics::motion_setpoint_pub(i), rclcpp::QoS(1),
            [this, i](rj_msgs::msg::MotionSetpoint::SharedPtr motion) {  // NOLINT
                last_updates_.at(i) = RJ::now();
                send(i, *motion, manipulators_cached_.at(i));
            });
    }

    tick_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { tick(); });
}

void Radio::publish(int robot_id, const rj_msgs::msg::RobotStatus& robot_status) {
    robot_status_pubs_.at(robot_id)->publish(robot_status);
}

void Radio::tick() {
    receive();

    RJ::Time update_time = RJ::now();

    for (int i = 0; i < kNumShells; i++) {
        if (last_updates_.at(i) + RJ::Seconds(PARAM_timeout) < update_time) {
            using rj_msgs::msg::ManipulatorSetpoint;
            using rj_msgs::msg::MotionSetpoint;

            // Send a NOP packet if we haven't got any updates.
            const auto motion = rj_msgs::build<MotionSetpoint>()
                                    .velocity_x_mps(0)
                                    .velocity_y_mps(0)
                                    .velocity_z_radps(0);
            const auto manipulator = rj_msgs::build<ManipulatorSetpoint>()
                                         .shoot_mode(ManipulatorSetpoint::SHOOT_MODE_KICK)
                                         .trigger_mode(ManipulatorSetpoint::TRIGGER_MODE_STAND_DOWN)
                                         .kick_strength(0)
                                         .dribbler_speed(0);
            last_updates_.at(i) = RJ::now();
            send(i, motion, manipulator);
        }
    }
}
}  // namespace radio