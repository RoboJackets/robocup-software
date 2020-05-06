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
#include "PacketConvert.hpp"

#include "status.h"

using namespace std;
using namespace Packet;
using namespace boost::asio;

SimRadio::SimRadio(Context* const context, bool blueTeam)
    : _context(context),
      _blueTeam(blueTeam),
      _socket(_io_service, ip::udp::endpoint(ip::udp::v4(),
                                             blueTeam ? SimBlueStatusPort
                                                      : SimYellowStatusPort)) {
    _grsim_endpoint = ip::udp::endpoint(ip::udp::v4(), SimCommandPort);

    // TODO: Make sure the buffer is big enough.
    _buffer.resize(128);
    startReceive();
}

bool SimRadio::isOpen() const { return _socket.is_open(); }

void SimRadio::send(Packet::RadioTx& radioTx) {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();
    for (int i = 0; i < radioTx.robots_size(); i++) {
        const Packet::Robot& robot = radioTx.robots(i);
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

    _socket.send_to(buffer(out), _grsim_endpoint);
}

void SimRadio::receive() { _io_service.poll(); }

void SimRadio::startReceive() {
    // Set a receive callback
    _socket.async_receive(
        boost::asio::buffer(_buffer),
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            receivePacket(error, num_bytes);
        });
}

void SimRadio::receivePacket(const boost::system::error_code& error,
                             std::size_t num_bytes) {
    for (int i = 0; i < num_bytes; i++) {
        handleReceive(_buffer[i]);
    }
    startReceive();
}

void SimRadio::handleReceive(uint8_t data) {
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

    int this_robot_id = data & robot_id_mask;
    bool ball_sense = data & touching_ball_mask;
    bool just_kicked = data & just_kicked_mask;
    bool robot_on = data & robot_on_mask;

    RadioRx rx;
    rx.set_robot_id(this_robot_id);
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
    const uint8_t kicker_status_ready = Kicker_Charged | kicker_status_charging;
    ;

    rx.set_kicker_status(just_kicked ? kicker_status_charging
                                     : kicker_status_ready);
    rx.set_kicker_voltage(200);

    std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
    _reversePackets.push_back(rx);
}

void SimRadio::stopRobots() {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();

    for (int i = 0; i < _context->state.self.size(); i++) {
        grSim_Robot_Command* simRobot = simRobotCommands->add_robot_commands();
        simRobot->set_id(_context->state.self[i]->shell());
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
    _socket.send_to(boost::asio::buffer(out),
                    ip::udp::endpoint(ip::udp::v4(), SimCommandPort));
}

void SimRadio::switchTeam(bool blueTeam) {
    _blueTeam = blueTeam;

    if (_socket.is_open()) {
        // We don't really care what port the tx is on, just the rx
        stopRobots();
        _socket.close();
        _socket.open(ip::udp::v4());
    }

    int status_port = blueTeam ? SimBlueStatusPort : SimYellowStatusPort;

    // Let them throw exceptions
    _socket.bind(ip::udp::endpoint(ip::udp::v4(), status_port));
}
