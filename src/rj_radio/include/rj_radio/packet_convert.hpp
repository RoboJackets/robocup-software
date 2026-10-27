#pragma once

#include <set>

#include <rj_common/status.hpp>
#include <rj_common/time.hpp>
#include <rj_geometry/util.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_protos/ssl_simulation_robot_control.pb.h>
#include <rj_protos/ssl_simulation_robot_feedback.pb.h>
#include <rj_common/game_constants.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_common/radio/robot_status.hpp>
#include <rj_common/robot_intent.hpp>

#include "rj_radio/messages/robot_status_message.hpp"
#include "rj_radio/messages/control_message.hpp"


/**
 * There are several different structs used throughout our code used to
 * represent the data sent to/from our robot.
 *
 * For Rx (robot to soccer), this includes:
 *  - RTP. This is the packed format we use to send to and from real robots
 *  - sim. This is a protobuf-based format used to communicate with sim
 *  - RobotStatus. This is the in-memory representation used in Context.
 *
 * For Tx (soccer to robot), the structs are similar with one exception:
 * instead of a single equivalent to RobotStatus, Tx data arrives as separate
 * ManipulatorSetpoint and MotionSetpoint ROS messages.
 */

/**
 * @brief Kicker and dribbler limits needed to convert setpoints into robot
 * or simulator commands. The radio node reads these from its params.
 */
struct RobotHardwareParams {
    // Chip speed range (m/s) that maps onto the chipper's 0..kMaxKick strength.
    double min_chip_speed;
    double max_chip_speed;
    // Chip angle sent to the simulator (degrees).
    double chip_angle;
    // Simulator dribbler speed at full dribble (rpm).
    double max_dribbler_speed;
};

namespace ConvertRx {

void rtp_to_status(const RadioMessage::RobotStatusMessage& rtp, RobotStatus* status);

void sim_to_status(const RobotFeedback& sim, RobotStatus* status);

}  // namespace ConvertRx

namespace ConvertTx {

void ros_to_rtp(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                const rj_msgs::msg::MotionSetpoint& motion, int shell,
                RadioMessage::ControlMessage* rtp, bool blue_team,
                const RobotHardwareParams& hardware);

void ros_to_sim(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                const rj_msgs::msg::MotionSetpoint& motion, int shell, RobotCommand* sim,
                const RobotHardwareParams& hardware);

}  // namespace ConvertTx
