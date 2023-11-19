#pragma once

#include <set>

#include <control/motion_setpoint.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_protos/RadioRx.pb.h>
#include <rj_protos/RadioTx.pb.h>
#include <rj_protos/Robot.pb.h>
#include <rj_protos/ssl_simulation_robot_control.pb.h>
#include <rj_protos/ssl_simulation_robot_feedback.pb.h>
#include <robot_intent.hpp>

#include "robot_status.hpp"
#include "strategy/agent/position/positions.hpp"

#include "rc-fshare/rtp.hpp"

/**
 * There are several different structs used throughout our code used to
 * represent the data sent to/from our robot.
 *
 * For Rx (robot to soccer), this includes:
 *  - RTP. This is the packed format we use to send to and from real robots
 *  - sim. This is a protobuf-based format used to communicate with sim
 *  - RobotStatus. This is the in-memory representation used in Context.
 *  - Packet::RadioRx. This is the representation used in the log frame.
 *
 * For Tx (soccer to robot), the structs are similar with one exception:
 * instead of a single equivalent to RobotStatus, in-memory representations of
 * Tx data is split across RobotIntent and MotionSetpoint.
 */

namespace ConvertRx {

void rtp_to_status(const rtp::RobotStatusMessage& rtp, RobotStatus* status);

void sim_to_status(const RobotFeedback& sim, RobotStatus* status);

void status_to_proto(const RobotStatus& status, Packet::RadioRx* proto);

void status_to_ros(const RobotStatus& status, rj_msgs::msg::RobotStatus* msg);

void ros_to_status(const rj_msgs::msg::RobotStatus& msg, RobotStatus* status);

}  // namespace ConvertRx

namespace ConvertTx {

void to_rtp(const RobotIntent& intent, const MotionSetpoint& setpoint,
            int shell, rtp::RobotTxMessage* rtp);

void ros_to_rtp(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                const rj_msgs::msg::MotionSetpoint& motion, int shell, rtp::RobotTxMessage* rtp,
                strategy::Positions role);

void to_proto(const RobotIntent& intent, const MotionSetpoint& setpoint,
              int shell, Packet::Robot* proto);

void to_sim(const RobotIntent& intent, const MotionSetpoint& setpoint,
            int shell, RobotCommand* sim);

void ros_to_sim(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                const rj_msgs::msg::MotionSetpoint& motion, int shell,
                RobotCommand* sim);

}  // namespace ConvertTx

void fill_header(rtp::Header* header);
