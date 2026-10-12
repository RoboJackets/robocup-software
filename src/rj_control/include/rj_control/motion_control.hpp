#pragma once

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/control/motion_setpoint.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_common/ros_debug_drawer.hpp>
#include <rj_common/time.hpp>
#include <rj_common/topic_names.hpp>
#include <rj_common/utils.hpp>
#include <rj_common/world_state.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/util.hpp>
#include <rj_utils/logging.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rj_control/pid.hpp"

namespace control {

namespace testing {

class MotionControlTest;

}  // namespace testing

class MotionControlNode;

/**
 * @brief Limits shared by every robot's controller (top level of /control).
 */
struct MotionControlLimits {
    double max_velocity;          // m/s
    double max_angular_velocity;  // rad/s
    int translation_windup;
    int rotation_windup;
};

/**
 * @brief PID gains for one robot (robot_N.* under /control).
 */
struct PidGains {
    double translation_kp;
    double translation_ki;
    double translation_kd;
    double rotation_kp;
    double rotation_ki;
    double rotation_kd;
};

/**
 * @brief Limits shared by every robot's controller (top level of /control).
 */
struct MotionControlLimits {
    double max_velocity;          // m/s
    double max_angular_velocity;  // rad/s
    int translation_windup;
    int rotation_windup;
};

/**
 * @brief PID gains for one robot (robot_N.* under /control).
 */
struct PidGains {
    double translation_kp;
    double translation_ki;
    double translation_kd;
    double rotation_kp;
    double rotation_ki;
    double rotation_kd;
};

/**
 * @brief Limits shared by every robot's controller (top level of /control).
 */
struct MotionControlLimits {
    double max_velocity;          // m/s
    double max_angular_velocity;  // rad/s
    int translation_windup;
    int rotation_windup;
};

/**
 * @brief PID gains for one robot (robot_N.* under /control).
 */
struct PidGains {
    double translation_kp;
    double translation_ki;
    double translation_kd;
    double rotation_kp;
    double rotation_ki;
    double rotation_kd;
};

/**
 * @brief Handles computer-side motion control
 * @details This class handles the details of creating velocity commands for a
 *     robot given the desired path to follow.
 */
class MotionControl {
public:
    /**
     * @param node Node that owns this controller. robot_<shell_id>.* gains are
     * declared on it.
     * @param limits Limits shared by every controller.
     */
    MotionControl(int shell_id, rclcpp::Node* node, const MotionControlLimits& limits);

    // Subscriptions capture `this`.
    MotionControl(const MotionControl&) = delete;
    MotionControl& operator=(const MotionControl&) = delete;
    MotionControl(MotionControl&&) = delete;
    MotionControl& operator=(MotionControl&&) = delete;
    ~MotionControl() = default;

protected:
    friend class testing::MotionControlTest;

    /**
     * This runs PID control on the position and angle of the robot and
     * sets values in the robot's radio_tx packet.
     */
    void run(const RobotState& state, const planning::Trajectory& trajectory,
             const PlayState::State& play_state, bool is_joystick_controlled,
             MotionSetpoint* setpoint);

private:
    friend class MotionControlNode;

    /**
     * Force stop the motion by setting the setpoint to zero.
     * Also resets PID controllers.
     * @param setpoint
     */
    void stop(MotionSetpoint* setpoint);

    /**
     * Reset all PID controllers. To be used while the robot is not under PID
     * control (stopped or joystick-controlled).
     */
    void reset();

    /**
     * Update PID parameters.
     */
    void update_params();

    void update_gains(const std::vector<rclcpp::Parameter>& params);

    void set_velocity(MotionSetpoint* setpoint, rj_geometry::Twist target_vel);

    int shell_id_;

    /// The last velocity command (in m/s) that we sent / to the robot
    rj_geometry::Twist last_world_vel_command_;

    /// the time when the last velocity command was sent
    RJ::Time last_cmd_time_;

    Pid position_x_controller_;
    Pid position_y_controller_;
    Pid angle_controller_;

    rj_drawing::RosDebugDrawer drawer_;

    PlayState::State play_state_ = PlayState::State::Halt;

    planning::Trajectory trajectory_;

    rclcpp::Subscription<planning::Trajectory::Msg>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<WorldState::Msg>::SharedPtr world_state_sub_;
    rclcpp::Subscription<PlayState::Msg>::SharedPtr play_state_sub_;
    rclcpp::Publisher<MotionSetpoint::Msg>::SharedPtr motion_setpoint_pub_;
    rclcpp::Publisher<RobotState::Msg>::SharedPtr target_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_heading_pub_;

    MotionControlLimits limits_;
    PidGains gains_;
};

}  // namespace control
