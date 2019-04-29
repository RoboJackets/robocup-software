#include <Geometry2d/Util.hpp>
#include <iostream>
#include <time.hpp>
#include <status.h>
#include "PacketConvert.hpp"

void from_robot_tx_proto(const Packet::Robot& proto_packet, rtp::RobotTxMessage* msg) {
    Packet::Control control = proto_packet.control();
    msg->uid = proto_packet.uid();

    msg->message.controlMessage.vision_pose_x = static_cast<int16_t>(
        control.vision_pose_x() * rtp::ControlMessage::POSE_SCALE_FACTOR);
    msg->message.controlMessage.vision_pose_y = static_cast<int16_t>(
        control.vision_pose_y() * rtp::ControlMessage::POSE_SCALE_FACTOR);
    msg->message.controlMessage.vision_pose_theta = static_cast<int16_t>(
        control.vision_pose_theta() * rtp::ControlMessage::POSE_SCALE_FACTOR);

    msg->message.controlMessage.goal_pose_x = static_cast<int16_t>(
        control.goal_pose_x() * rtp::ControlMessage::POSE_SCALE_FACTOR);
    msg->message.controlMessage.goal_pose_y = static_cast<int16_t>(
        control.goal_pose_y() * rtp::ControlMessage::POSE_SCALE_FACTOR);
    msg->message.controlMessage.goal_pose_theta = static_cast<int16_t>(
        control.goal_pose_theta() * rtp::ControlMessage::POSE_SCALE_FACTOR);

    msg->message.controlMessage.goal_velocity_x = static_cast<int16_t>(
        control.goal_velocity_x() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    msg->message.controlMessage.goal_velocity_y = static_cast<int16_t>(
        control.goal_velocity_y() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    msg->message.controlMessage.goal_velocity_theta = static_cast<int16_t>(
        control.goal_velocity_theta() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);

    msg->message.controlMessage.goal_acceleration_x = static_cast<int16_t>(
        control.goal_acceleration_x() * rtp::ControlMessage::ACCELERATION_SCALE_FACTOR);
    msg->message.controlMessage.goal_acceleration_y = static_cast<int16_t>(
        control.goal_acceleration_y() * rtp::ControlMessage::ACCELERATION_SCALE_FACTOR);
    msg->message.controlMessage.goal_acceleration_theta = static_cast<int16_t>(
        control.goal_acceleration_theta() * rtp::ControlMessage::ACCELERATION_SCALE_FACTOR);

    msg->message.controlMessage.dribbler =
        clamp(static_cast<uint16_t>(control.dribbler_velocity()) * 2, 0, 255);

    msg->message.controlMessage.shootMode = control.shootmode();
    msg->message.controlMessage.kickStrength = control.kcstrength();
    msg->message.controlMessage.triggerMode = control.triggermode();
    msg->message.controlMessage.song = control.song();

    msg->message.controlMessage.motionMode = control.motion_mode();

    msg->messageType = rtp::RobotTxMessage::ControlMessageType;
}

void convert_tx_proto_to_rtp(const Packet::RadioTx &proto_packet,
                             rtp::RobotTxMessage *messages,
                             int num_robots) {
    if (proto_packet.robots_size() != num_robots) {
        std::cerr << "Error: size mismatch in " __FILE__ << std::endl;
        return;
    }

    for (int slot = 0; slot < num_robots; slot++) {
        rtp::RobotTxMessage* msg = messages + slot;
        if (slot < proto_packet.robots_size()) {
            from_robot_tx_proto(proto_packet.robots(slot), msg);
        } else {
            // Empty slot
            msg->uid = rtp::INVALID_ROBOT_UID;
        }
    }
}

Packet::RadioRx convert_rx_rtp_to_proto(const rtp::RobotStatusMessage &msg) {
    Packet::RadioRx packet;

    packet.set_timestamp(RJ::timestamp());
    packet.set_robot_id(msg.uid);

    packet.set_hardware_version(Packet::RJ2015);
    packet.set_battery(msg.battVoltage *
            rtp::RobotStatusMessage::BATTERY_SCALE_FACTOR);

    if (Packet::BallSenseStatus_IsValid(msg.ballSenseStatus)) {
        packet.set_ball_sense_status(
                Packet::BallSenseStatus(msg.ballSenseStatus));
    }

    // Using same flags as 2011 robot. See common/status.h
    // Report that everything is good b/c the bot currently has no way of
    // detecting kicker issues
    packet.set_kicker_status((msg.kickStatus ? Kicker_Charged : 0) |
                             (msg.kickHealthy ? Kicker_Enabled : 0) |
                             Kicker_I2C_OK);

    // Motor errors
    for (int i = 0; i < 5; i++) {
        bool err = msg.motorErrors & (1 << i);
        packet.add_motor_status(err ? Packet::MotorStatus::Hall_Failure
                                    : Packet::MotorStatus::Good);
    }

    for (std::size_t i = 0; i < 4; i++) {
        packet.add_encoders(msg.encDeltas[i]);
    }

    // FPGA status
    if (Packet::FpgaStatus_IsValid(msg.fpgaStatus)) {
        packet.set_fpga_status(Packet::FpgaStatus(msg.fpgaStatus));
    }

    return packet;
}

void fill_header(rtp::Header* header) {
    header->port = rtp::PortType::CONTROL;
    header->address = rtp::BROADCAST_ADDRESS;
    header->type = rtp::MessageType::CONTROL;
}
