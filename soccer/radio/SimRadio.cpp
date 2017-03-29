#include "SimRadio.hpp"

#include <protobuf/grSim_Commands.pb.h>
#include <protobuf/grSim_Packet.pb.h>
#include <Network.hpp>
#include <stdexcept>
#include <Utils.hpp>
#include <Geometry2d/Util.hpp>
#include <Robot.hpp>

#include "firmware-common/robot2015/cpu/status.h"

using namespace std;
using namespace Packet;

static QHostAddress LocalAddress(QHostAddress::LocalHost);

SimRadio::SimRadio(SystemState& system_state, bool blueTeam)
    : _state(system_state), _blueTeam(blueTeam) {
    _channel = blueTeam ? 1 : 0;
    if (!_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
}

bool SimRadio::isOpen() const {
    return _socket.isValid();
}

void SimRadio::send(Packet::RadioTx& packet) {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();
    for (int i = 0; i < packet.robots_size(); i++) {
        const Packet::Robot& robot = packet.robots(i);
        grSim_Robot_Command* simRobot = simRobotCommands->add_robot_commands();
        simRobot->set_id(robot.uid());
        simRobot->set_veltangent(robot.control().yvelocity());
        simRobot->set_velnormal(-robot.control().xvelocity());
        simRobot->set_velangular(RadiansToDegrees(robot.control().avelocity()));

        simRobot->set_triggermode((grSim_Robot_Command_TriggerMode) robot.control().triggermode());

        // rough approximation of kick strength assuming the max of 255
        // corresponds to 8 m / s
        const float kc_strength_to_ms = 8.0f / 255;
        uint kick_strength = kc_strength_to_ms * robot.control().kcstrength();
        switch (robot.control().shootmode()) {
            case Packet::Control::KICK:
                simRobot->set_kickspeedx(kick_strength);
                simRobot->set_kickspeedz(0);
                break;
            case Packet::Control::CHIP:
                simRobot->set_kickspeedx(kick_strength);
                simRobot->set_kickspeedz(kick_strength);
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
    _socket.writeDatagram(&out[0], out.size(),
                                   QHostAddress(QHostAddress::LocalHost),
                                   SimCommandPort);
}

void SimRadio::receive() {
    for (int i = 0; i < Robots_Per_Team; i++) {
        RadioRx rx;
        rx.set_robot_id(i);
        rx.set_hardware_version(RJ2015);
        rx.set_battery(100);

        // need to add ball sense flag based on ball position and
        // robot position
        auto& robot_pos = _state.self[i]->pos;
        auto& ball_pos = _state.ball.pos;

        double angleToBall = robot_pos.angleTo(ball_pos);
        double distToBall = robot_pos.distTo(ball_pos);

        float robotAngle = _state.self[i]->angle;
        
        // if the ball is very close to us, and within 25 degrees of the front
        // of us, then activate our ball sense.
        if (std::fabs(angleToBall - robotAngle) < DegreesToRadians(25)
                && distToBall < Robot_Radius * 1.1) {
            rx.set_ball_sense_status(Packet::HasBall);
        } else {
            rx.set_ball_sense_status(Packet::NoBall);
        }
        
        const int num_motors = 5;
        // 5 motors including dribbler
        for (int i = 0; i < num_motors; i++) {
            rx.add_motor_status(MotorStatus::Good);
        }

        // it's a sim rx, so just pretend like everything is good
        rx.set_fpga_status(FpgaGood);
        rx.set_timestamp(RJ::timestamp());
        rx.set_kicker_status(Kicker_Charged | Kicker_Enabled | Kicker_I2C_OK);
        rx.set_kicker_voltage(200);
        _reversePackets.push_back(rx);
    }
}

void SimRadio::switchTeam(bool blueTeam) {
    _socket.close();
    _channel = blueTeam ? 1 : 0;
    if (!_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
    _blueTeam = blueTeam;
}
