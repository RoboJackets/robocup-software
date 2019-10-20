#include "PacketConvert.hpp"
#include <status.h>
#include <Geometry2d/Util.hpp>
#include <iostream>
#include <time.hpp>

void from_robot_tx_proto(const Packet::Robot& proto_packet,
                         rtp::RobotTxMessage* msg) {
    Packet::Control control = proto_packet.control();
    msg->uid = proto_packet.uid();
    msg->message.controlMessage.bodyX = static_cast<int16_t>(
        control.xvelocity() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    msg->message.controlMessage.bodyY = static_cast<int16_t>(
        control.yvelocity() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    msg->message.controlMessage.bodyW = static_cast<int16_t>(
        control.avelocity() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    msg->message.controlMessage.dribbler =
        clamp(static_cast<uint16_t>(control.dvelocity()) * 2, 0, 255);

    msg->message.controlMessage.shootMode = control.shootmode();
    msg->message.controlMessage.kickStrength = control.kcstrength();
    msg->message.controlMessage.triggerMode = control.triggermode();
    msg->message.controlMessage.song = control.song();
    msg->messageType = rtp::RobotTxMessage::ControlMessageType;
}

void convert_tx_proto_to_rtp(const Packet::RadioTx& proto_packet,
                             rtp::RobotTxMessage* messages) {
    from_robot_tx_proto(proto_packet.robots(0), messages);
}

Packet::RadioRx convert_rx_rtp_to_proto(const rtp::RobotStatusMessage& msg) {
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

    /*for (std::size_t i = 0; i < 14; i++) {
        packet.add_encoders(msg.encDeltas[i]);
        //printf("%9.3f,", (float)msg.encDeltas[i] / 1000);
    }*/
    // printf("\r\n");

    // FPGA status
    if (Packet::FpgaStatus_IsValid(msg.fpgaStatus)) {
        packet.set_fpga_status(Packet::FpgaStatus(msg.fpgaStatus));
    }

    return packet;
}

void construct_tx_proto(Packet::RadioTx& radioTx,
                        const std::array<RobotIntent, Num_Shells>& intents) {
    // I'm assuming this is necessary to do logging. idk
    radioTx.set_txmode(Packet::RadioTx::UNICAST);
    while (radioTx.robots_size() < Num_Shells) {
        radioTx.add_robots();
    }
    for (int i = 0; i < intents.size(); ++i) {
        Packet::Robot* robotPacket;
        robotPacket = radioTx.mutable_robots(i);
        robotPacket->set_uid(i);
        Packet::Control* controlPacket = robotPacket->mutable_control();
        const RobotIntent& intent = intents[i];
        switch (intent.shootmode) {
            case RobotIntent::ShootMode::KICK:
                controlPacket->set_shootmode(Packet::Control::KICK);
                break;
            case RobotIntent::ShootMode::CHIP:
                controlPacket->set_shootmode(Packet::Control::CHIP);
                break;
        }
        switch (intent.triggermode) {
            case RobotIntent::TriggerMode::STAND_DOWN:
                controlPacket->set_triggermode(Packet::Control::STAND_DOWN);
                break;
            case RobotIntent::TriggerMode::IMMEDIATE:
                controlPacket->set_triggermode(Packet::Control::IMMEDIATE);
                break;
            case RobotIntent::TriggerMode::ON_BREAK_BEAM:
                controlPacket->set_triggermode(Packet::Control::ON_BREAK_BEAM);
                break;
        }
        switch (intent.song) {
            case RobotIntent::Song::STOP:
                controlPacket->set_song(Packet::Control::STOP);
                break;
            case RobotIntent::Song::CONTINUE:
                controlPacket->set_song(Packet::Control::CONTINUE);
                break;
            case RobotIntent::Song::FIGHT_SONG:
                controlPacket->set_song(Packet::Control::FIGHT_SONG);
                break;
        }
        controlPacket->set_xvelocity(intent.setpoints.xvelocity);
        controlPacket->set_yvelocity(intent.setpoints.yvelocity);
        controlPacket->set_avelocity(intent.setpoints.avelocity);
        controlPacket->set_dvelocity(intent.dvelocity);
        controlPacket->set_kcstrength(intent.kcstrength);
    }
}

void fill_header(rtp::Header* header) {
    header->port = rtp::PortType::CONTROL;
    header->address = rtp::BROADCAST_ADDRESS;
    header->type = rtp::MessageType::CONTROL;
}
