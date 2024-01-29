#pragma once

#include <context.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/point.hpp>
#include <rj_param_utils/param.hpp>

#include "control/motion_setpoint.hpp"
#include "game_state.hpp"
#include "ros_debug_drawer.hpp"

#include <rc-fshare/pid.hpp>

namespace control {

DECLARE_FLOAT64(params::kMotionControlParamModule, max_acceleration);
DECLARE_FLOAT64(params::kMotionControlParamModule, max_velocity);
DECLARE_FLOAT64(params::kMotionControlParamModule, rotation_kp);
DECLARE_FLOAT64(params::kMotionControlParamModule, rotation_ki);
DECLARE_FLOAT64(params::kMotionControlParamModule, rotation_kd);
DECLARE_INT64(params::kMotionControlParamModule, rotation_windup);
DECLARE_FLOAT64(params::kMotionControlParamModule, translation_kp);
DECLARE_FLOAT64(params::kMotionControlParamModule, translation_ki);
DECLARE_FLOAT64(params::kMotionControlParamModule, translation_kd);
DECLARE_INT64(params::kMotionControlParamModule, translation_windup);

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
};

}  // namespace control
