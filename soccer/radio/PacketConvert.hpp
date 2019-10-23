#pragma once

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/Robot.pb.h>
#include <RobotIntent.hpp>
#include <motion/MotionSetpoint.hpp>
#include <set>

#include "rc-fshare/rtp.hpp"

void from_robot_tx_proto(const ::Packet::Robot& proto_packet,
                         rtp::RobotTxMessage* msg);

/**
 * @brief Serialize a single outgoing robot command from a protobuf.
 */
void convert_tx_robot_proto_to_rtp(const ::Packet::Robot& proto_packet,
                                   rtp::RobotTxMessage* msg);

/**
 * @brief Serialize a protobuf TX packet (with all robots) into an array of
 * robot messages.
 */
void convert_tx_proto_to_rtp(const ::Packet::RadioTx& proto_packet,
                             rtp::RobotTxMessage* messages);

/**
 * @brief Construct a RadioTx packet from the intents and setpoints
 */
void construct_tx_proto(
    Packet::RadioTx& radioTx,
    const std::array<RobotIntent, Num_Shells>& intents,
    const std::array<MotionSetpoint, Num_Shells>& setpoints,
    const std::set<int>& activeRobots);

/**
 * @brief Deserialize an incoming message from a single robot into a protobuf.
 */
::Packet::RadioRx convert_rx_rtp_to_proto(const rtp::RobotStatusMessage& msg);

void fill_header(rtp::Header* header);
