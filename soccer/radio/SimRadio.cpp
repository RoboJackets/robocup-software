#include "SimRadio.hpp"
#include <Utils.hpp>
#include <protobuf/grSim_Commands.pb.h>
#include <protobuf/grSim_Packet.pb.h>
#include <Network.hpp>
#include <stdexcept>

using namespace std;
using namespace Packet;

static QHostAddress LocalAddress(QHostAddress::LocalHost);

SimRadio::SimRadio(bool blueTeam) {
    _channel = blueTeam ? 1 : 0;
    if (!_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
}

bool SimRadio::isOpen() const {
    // FIXME - check the socket
    return true;
}

void SimRadio::send(Packet::RadioTx& packet) {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();
    for (int x = 0; x < packet.robots_size(); x++) {
        Packet::RadioTx::Robot const& robot = packet.robots(x);
        grSim_Robot_Command* simRobot = simRobotCommands->add_robot_commands();
        simRobot->set_id(robot.robot_id());
        simRobot->set_veltangent(robot.body_x());
        simRobot->set_velnormal(robot.body_y());
        simRobot->set_velangular(robot.body_w());
        if (robot.has_kick()) {
            uint kick = robot.kick();
            simRobot->set_kickspeedx(kick);
        } else {
            simRobot->set_kickspeedx(0);
        }

        if (robot.has_use_chipper()) {
            uint kick = robot.kick();
            simRobot->set_kickspeedx(kick);
            simRobot->set_kickspeedz(kick);
        } else {
            simRobot->set_kickspeedz(0);
        }

        simRobot->set_spinner(robot.dribbler() > 0);
        simRobot->set_wheelsspeed(false);
    }
    simRobotCommands->set_isteamyellow(true);
    simRobotCommands->set_timestamp(0.0);

    // simPacket.set_allocated_commands(&simRobotCommands);
    std::string out;
    simPacket.SerializeToString(&out);
    _socket.writeDatagram(&out[0], out.size(), LocalAddress,
                          RadioTxPort + _channel);
}

void SimRadio::receive() {
    /*
    while (_socket.hasPendingDatagrams())
    {
        unsigned int n = _socket.pendingDatagramSize();
        string buf;
        buf.resize(n);
        _socket.readDatagram(&buf[0], n);

        _reversePackets.push_back(RadioRx());
        RadioRx &packet = _reversePackets.back();

        if (!packet.ParseFromString(buf))
        {
            printf("Bad radio packet of %d bytes\n", n);
            continue;
        }
    }
    */
    for (int x = 0; x < 6; x++) {
        _reversePackets.push_back(RadioRx());
        RadioRx& packet = _reversePackets.back();
        packet.set_robot_id(x);
        packet.set_timestamp(RJ::timestamp());
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
}
