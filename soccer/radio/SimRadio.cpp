#include "SimRadio.hpp"

#include <protobuf/grSim_Commands.pb.h>
#include <protobuf/grSim_Packet.pb.h>
#include <Geometry2d/Util.hpp>
#include <Network.hpp>
#include <Robot.hpp>
#include <Utils.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "status.h"

using namespace std;
using namespace Packet;

static QHostAddress LocalAddress(QHostAddress::LocalHost);

SimRadio::SimRadio(Context* const context, bool blueTeam)
    : _context(context), _blueTeam(blueTeam) {
    switchTeam(blueTeam);
}

bool SimRadio::isOpen() const { return _tx_socket.isValid(); }

void SimRadio::send(Packet::RadioTx& packet) {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();
    for (int i = 0; i < packet.robots_size(); i++) {
        const Packet::Robot& robot = packet.robots(i);
        grSim_Robot_Command* simRobot = simRobotCommands->add_robot_commands();
        simRobot->set_id(robot.uid());
        simRobot->set_veltangent(robot.control().yvelocity());
        simRobot->set_velnormal(-robot.control().xvelocity());
        simRobot->set_velangular(robot.control().avelocity());
        // simRobot->set_velangular(RadiansToDegrees(robot.control().avelocity()));

        simRobot->set_triggermode(
            (grSim_Robot_Command_TriggerMode)robot.control().triggermode());

        // rough approximation of kick strength assuming the max of 255
        // corresponds to 8 m / s and min is 1 m / s
        const float min_kick_m_s = 2.1f;
        const float max_kick_m_s = 7.0f;
        const float min_chip_m_s = 2.1f;
        const float max_chip_m_s = 5.0f;
        const float chip_angle = 40 * M_PI / 180;  // degrees
        float kc_strength_to_ms;
        uint kick_strength;

        switch (robot.control().shootmode()) {
            case Packet::Control::KICK:
                kc_strength_to_ms = (max_kick_m_s - min_kick_m_s) / 255;
                kick_strength =
                    kc_strength_to_ms * robot.control().kcstrength() +
                    min_kick_m_s;
                simRobot->set_kickspeedx(kick_strength);
                simRobot->set_kickspeedz(0);
                break;
            case Packet::Control::CHIP:
                kc_strength_to_ms = (max_chip_m_s - min_chip_m_s) / 255;
                kick_strength =
                    kc_strength_to_ms * robot.control().kcstrength() +
                    min_chip_m_s;
                simRobot->set_kickspeedx(cos(chip_angle) * kick_strength);
                simRobot->set_kickspeedz(sin(chip_angle) * kick_strength);
                break;
            default:
                break;
        }

        simRobot->set_spinner(robot.control().dvelocity() > 0);
        simRobot->set_wheelsspeed(false);
    }
    simRobotCommands->set_isteamyellow(!_blueTeam);
    simRobotCommands->set_timestamp(RJ::timestamp());

    std::string out;
    simPacket.SerializeToString(&out);
    _tx_socket.writeDatagram(&out[0], out.size(),
                             QHostAddress(QHostAddress::LocalHost),
                             SimCommandPort);
}

void SimRadio::receive() {
    while (_rx_socket.hasPendingDatagrams()) {
        char byte = 0;
        // one byte at a time
        _rx_socket.readDatagram(&byte, 1);

        // grSim really needs to set up a proto packet for robot status.
        // Instead they pack their own byte up in a custom way :/
        //
        // byte structure:
        // 0-2: Robot_ID
        // 3: touching_ball
        // 4: just_kicked
        // 5: robot_on
        const uint8_t robot_id_mask = 0x7;
        const uint8_t touching_ball_mask = 0x1 << 3;
        const uint8_t just_kicked_mask = 0x1 << 4;
        const uint8_t robot_on_mask = 0x1 << 5;

        int robot_id = byte & robot_id_mask;
        bool ball_sense = byte & touching_ball_mask;
        bool just_kicked = byte & just_kicked_mask;
        bool robot_on = byte & robot_on_mask;

        RadioRx rx;
        rx.set_robot_id(robot_id);
        rx.set_hardware_version(RJ2015);
        rx.set_battery(100);

        rx.set_ball_sense_status(ball_sense ? Packet::HasBall : Packet::NoBall);

        const int num_motors = 5;
        // 5 motors including dribbler
        for (int i = 0; i < num_motors; i++) {
            rx.add_motor_status(MotorStatus::Good);
        }

        rx.set_fpga_status(FpgaGood);
        rx.set_timestamp(RJ::timestamp());

        const uint8_t kicker_status_charging = Kicker_Enabled | Kicker_I2C_OK;
        const uint8_t kicker_status_ready =
            Kicker_Charged | kicker_status_charging;
        ;

        rx.set_kicker_status(just_kicked ? kicker_status_charging
                                         : kicker_status_ready);
        rx.set_kicker_voltage(200);

        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        _reversePackets.push_back(rx);
    }
}

void SimRadio::stopRobots() {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();
    for (int i = 0; i < _context->state.self.size(); i++) {
        auto& robot = _context->state.self[i]->robotPacket;
        grSim_Robot_Command* simRobot = simRobotCommands->add_robot_commands();
        simRobot->set_id(robot.uid());
        simRobot->set_veltangent(0);
        simRobot->set_velnormal(0);
        simRobot->set_velangular(0);

        simRobot->set_kickspeedx(0);
        simRobot->set_kickspeedz(0);

        simRobot->set_spinner(0);
        simRobot->set_wheelsspeed(false);
    }
    simRobotCommands->set_isteamyellow(!_blueTeam);
    simRobotCommands->set_timestamp(RJ::timestamp());

    std::string out;
    simPacket.SerializeToString(&out);
    _tx_socket.writeDatagram(&out[0], out.size(),
                             QHostAddress(QHostAddress::LocalHost),
                             SimCommandPort);
}

void SimRadio::switchTeam(bool blueTeam) {
    stopRobots();
    _blueTeam = blueTeam;
    _tx_socket.close();
    _rx_socket.close();
    _channel = blueTeam ? 1 : 0;
    if (!_tx_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
    int status_port = blueTeam ? SimBlueStatusPort : SimYellowStatusPort;
    if (!_rx_socket.bind(status_port)) {
        throw runtime_error(
            QString("Can't bind to the %1 team's radio status port.")
                .arg(blueTeam ? "blue" : "yellow")
                .toStdString());
    }
}
