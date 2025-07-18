#pragma once

#include <rclcpp/rclcpp.hpp>

#include <context.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/point.hpp>
#include <rj_param_utils/param.hpp>
#include <std_msgs/msg/float64.hpp>

#include <context.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/point.hpp>
#include <rj_param_utils/param.hpp>
#include <std_msgs/msg/float64.hpp>

#include "control/motion_setpoint.hpp"
#include "game_state.hpp"
#include "ros_debug_drawer.hpp"

#include <rc-fshare/pid.hpp>

namespace control {

DECLARE_FLOAT64(params::kMotionControlParamModule, max_acceleration);
DECLARE_FLOAT64(params::kMotionControlParamModule, max_velocity);
DECLARE_FLOAT64(params::kMotionControlParamModule, max_angular_acceleration);
DECLARE_FLOAT64(params::kMotionControlParamModule, max_angular_velocity);

DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_0, rotation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_0, rotation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_0, rotation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_0, rotation_windup);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_0, translation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_0, translation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_0, translation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_0, translation_windup);

DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_1, rotation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_1, rotation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_1, rotation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_1, rotation_windup);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_1, translation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_1, translation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_1, translation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_1, translation_windup);

DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_2, rotation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_2, rotation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_2, rotation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_2, rotation_windup);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_2, translation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_2, translation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_2, translation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_2, translation_windup);

DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_3, rotation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_3, rotation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_3, rotation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_3, rotation_windup);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_3, translation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_3, translation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_3, translation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_3, translation_windup);

DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_4, rotation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_4, rotation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_4, rotation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_4, rotation_windup);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_4, translation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_4, translation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_4, translation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_4, translation_windup);

DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_5, rotation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_5, rotation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_5, rotation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_5, rotation_windup);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_5, translation_kp);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_5, translation_ki);
DECLARE_NS_FLOAT64(params::kMotionControlParamModule, robot_5, translation_kd);
DECLARE_NS_INT64(params::kMotionControlParamModule, robot_5, translation_windup);

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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motion_mag_pub_;

    rclcpp::TimerBase::SharedPtr vel_timer_;
    bool forward_ = true;

    enum Param {
        rotation_kP,
        rotation_kI,
        rotationkD,
        rotation_windup,
        translation_kP,
        translation_kI,
        translation_kD
    };

    static float get_param(int shell_id, Param param);
};

}  // namespace control