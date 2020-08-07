#pragma once

#include <Configuration.hpp>
#include <Context.hpp>
#include <Geometry2d/Point.hpp>
#include <rc-fshare/pid.hpp>
#include <rj_common/time.hpp>

#include "Robot.hpp"
#include "motion/MotionSetpoint.hpp"
/**
 * @brief Handles computer-side motion control
 * @details This class handles the details of creating velocity commands for a
 *     robot given the desired path to follow.  It is responsible for most of
 *     what gets sent out in a RadioTx packet. The MotionControl object is given
 *     an OurRobot at initialization and from then on will set the values in
 *     that robot's RadioTx packet directly whenever run() or stopped() is
 *     called.
 */
class MotionControl {
public:
    MotionControl(Context* context, int shell_id);

    /**
     * This runs PID control on the position and angle of the robot and
     * sets values in the robot's radio_tx packet.
     */
    void run(const RobotState& state, const Planning::Trajectory& path,
             bool is_joystick_controlled, MotionSetpoint* setpoint);

    /**
     * Force stop the motion by setting the setpoint to zero.
     * Also resets PID controllers.
     * @param setpoint
     */
    void stop(MotionSetpoint* setpoint);

    /**
     * Resets all PID controllers.
     */
    void reset_pid_controllers();

    static void create_configuration(Configuration* cfg);

private:
    /**
     * Reset all PID controllers. To be used while the robot is not under PID
     * control (stopped or joystick-controlled).
     */
    void reset();

    /**
     * Update PID parameters.
     */
    void update_params();

    static void set_velocity(MotionSetpoint* setpoint,
                            Geometry2d::Twist target_vel);

    int shell_id_;

    /// The last velocity command (in m/s) that we sent / to the robot
    Geometry2d::Twist last_world_vel_command_;

    /// the time when the last velocity command was sent
    RJ::Time last_cmd_time_;

    Pid position_x_controller_;
    Pid position_y_controller_;
    Pid angle_controller_;

    static ConfigDouble* max_acceleration;
    static ConfigDouble* max_velocity;
    static ConfigDouble* x_multiplier;

    DebugDrawer* drawer_{};
    RobotConfig* config_{};
};
