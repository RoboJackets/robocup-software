#pragma once

#include <geometry2d/point.h>

#include <Configuration.hpp>
#include <Context.hpp>
#include <rc-fshare/pid.hpp>
#include <time.hpp>

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
     * sets values in the robot's radioTx packet.
     */
    void run(const RobotState& state, const Trajectory::Trajectory& trajectory,
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
    void resetPIDControllers();

    static void createConfiguration(Configuration* cfg);

private:
    /**
     * Reset all PID controllers. To be used while the robot is not under PID
     * control (stopped or joystick-controlled).
     */
    void reset();

    /**
     * Update PID parameters.
     */
    void updateParams();

    static void setVelocity(MotionSetpoint* setpoint,
                            geometry2d::Twist target_vel);

    int _shell_id;

    /// The last velocity command (in m/s) that we sent / to the robot
    geometry2d::Twist _last_world_vel_command;

    /// the time when the last velocity command was sent
    RJ::Time _lastCmdTime;

    Pid _positionXController;
    Pid _positionYController;
    Pid _angleController;

    static ConfigDouble* _max_acceleration;
    static ConfigDouble* _max_velocity;
    static ConfigDouble* _x_multiplier;

    DebugDrawer* _drawer{};
    RobotConfig* _config{};
};
