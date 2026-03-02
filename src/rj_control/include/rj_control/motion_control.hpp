#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/control/motion_setpoint.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_common/ros_debug_drawer.hpp>
#include <rj_common/time.hpp>
#include <rj_common/utils.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/util.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_utils/logging.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rj_control/pid.hpp"

namespace control {

// DECLARE_FLOAT64(params::kMotionControlParamModule, max_acceleration);
// DECLARE_FLOAT64(params::kMotionControlParamModule, max_velocity);
// DECLARE_FLOAT64(params::kMotionControlParamModule, rotation_kp);
// DECLARE_FLOAT64(params::kMotionControlParamModule, rotation_ki);
// DECLARE_FLOAT64(params::kMotionControlParamModule, rotation_kd);
// DECLARE_INT64(params::kMotionControlParamModule, rotation_windup);
// DECLARE_FLOAT64(params::kMotionControlParamModule, translation_kp);
// DECLARE_FLOAT64(params::kMotionControlParamModule, translation_ki);
// DECLARE_FLOAT64(params::kMotionControlParamModule, translation_kd);
// DECLARE_INT64(params::kMotionControlParamModule, translation_windup);

MotionControl::MotionControl(int shell_id, rclcpp::Node* node) : shell_id_(shell_id) {
    std::string param_prefix = fmt::format("robot_{}", std::to_string(shell_id_));

    // declare params
    // robot specific
    node->declare_parameter(param_prefix + ".translation_kp", 0.6);
    node->declare_parameter(param_prefix + ".translation_ki", 0.0);
    node->declare_parameter(param_prefix + ".translation_kd", 0.3);
    node->declare_parameter(param_prefix + ".rotation_kp", 10.0);
    node->declare_parameter(param_prefix + ".rotation_ki", 0.0);
    node->declare_parameter(param_prefix + ".rotation_kd", 0.9);

    // shared between robots
    node->declare_parameter("translation_windup", 0);
    node->declare_parameter("rotation_windup", 50);
    node->declare_parameter("max_velocity", 2.4);
    node->declare_parameter("max_acceleration", 3.0);
    node->declare_paramter("max_angular_velocity", 5.0);

    // populate params
    // robot specific
    node->get_parameter(param_prefix + ".translation_kp", translation_kp_);
    node->get_parameter(param_prefix + ".translation_ki", translation_ki_);
    node->get_parameter(param_prefix + ".translation_kd", translation_kd_);
    node->get_parameter(param_prefix + ".rotation_kp", rotation_kp_);
    node->get_parameter(param_prefix + ".rotation_ki", rotation_ki_);
    node->get_parameter(param_prefix + ".rotation_kd", rotation_kd_);
    
    // shared between robots
    node->get_parameter("translation_windup", translation_windup_);
    node->get_parameter("rotation_windup", rotation_windup_);
    node->get_parameter("max_velocity", max_velocity_);
    node->get_parameter("max_acceleration", max_acceleration_);
    node->get_parameter("max_angular_velocity", max_angular_velocity_);

}

namespace testing {

class MotionControlTest;

}  // namespace testing

/**
 * @brief Handles computer-side motion control
 * @details This class handles the details of creating velocity commands for a
 *     robot given the desired path to follow.
 */
class MotionControl {
public:
    MotionControl(int shell_id, rclcpp::Node* node);

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

    static void set_velocity(MotionSetpoint* setpoint, rj_geometry::Twist target_vel);

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

    // Robot-specific PID gains
    double translation_kp_;
    double translation_ki_;
    double translation_kd_;

    double rotation_kp_;
    double rotation_ki_;
    double rotation_kd_;

    // Shared limits
    double max_velocity_;
    double max_acceleration_;
    double max_angular_velocity_;

    int translation_windup_;
    int rotation_windup_;
};

}  // namespace control
