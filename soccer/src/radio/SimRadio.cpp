#include "SimRadio.hpp"

#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/messages_robocup_ssl_robot_status.pb.h>

#include <Geometry2d/Util.hpp>
#include <Network.hpp>
#include <Robot.hpp>
#include <rj_common/Utils.hpp>
#include <cmath>
#include <stdexcept>

#include "PacketConvert.hpp"
#include "status.h"

using namespace std;
using namespace Packet;
using namespace boost::asio;

SimRadio::SimRadio(Context* context, bool blueTeam)
    : _context(context),
      _blueTeam(blueTeam),
      _socket(_io_service, ip::udp::endpoint(ip::udp::v4(),
                                             blueTeam ? SimBlueStatusPort
                                                      : SimYellowStatusPort)) {
    _grsim_endpoint = ip::udp::endpoint(ip::udp::v4(), SimCommandPort);

    _buffer.resize(1024);
    startReceive();
}

bool SimRadio::isOpen() const { return _socket.is_open(); }

void SimRadio::send(const std::array<RobotIntent, Num_Shells>& intents,
                    const std::array<MotionSetpoint, Num_Shells>& setpoints) {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();

    for (int i = 0; i < Num_Shells; i++) {
        const auto& intent = intents[i];
        if (!intent.is_active) {
            continue;
        }

        const auto& setpoint = setpoints[i];

        grSim_Robot_Command* simRobot = simRobotCommands->add_robot_commands();
        ConvertTx::to_grsim(intent, setpoint, i, simRobot);
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
    std::string data(_buffer.begin(), _buffer.end());
    handleReceive(data);
    startReceive();
}

void SimRadio::handleReceive(const std::string& data) {
    Robots_Status packet;

    packet.ParseFromString(data);

    for (size_t pkt_idx = 0; pkt_idx < packet.robots_status_size(); pkt_idx++) {
        RobotStatus status;
        const Robot_Status& grsim_status = packet.robots_status(pkt_idx);
        ConvertRx::grsim_to_status(grsim_status, &status);

        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        _reversePackets.push_back(status);
    }
}

void SimRadio::stopRobots() {
    grSim_Packet simPacket;
    grSim_Commands* simRobotCommands = simPacket.mutable_commands();

    for (auto* const our_robot : _context->state.self) {
        grSim_Robot_Command* simRobot = simRobotCommands->add_robot_commands();
        simRobot->set_id(our_robot->shell());
        simRobot->set_veltangent(0);
        simRobot->set_velnormal(0);
        simRobot->set_velangular(0);

        simRobot->set_kickspeedx(0);
        simRobot->set_kickspeedz(0);

        simRobot->set_spinner(false);
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
        stopRobots();
        _socket.close();
    }

    _socket.open(ip::udp::v4());

    int status_port = blueTeam ? SimBlueStatusPort : SimYellowStatusPort;

    // Let them throw exceptions
    _socket.bind(ip::udp::endpoint(ip::udp::v4(), status_port));
}
