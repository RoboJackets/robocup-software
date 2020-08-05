#include "SimRadio.hpp"

#include <cmath>
#include <stdexcept>

#include <Geometry2d/Util.hpp>
#include <Robot.hpp>
#include <rj_common/Network.hpp>
#include <rj_common/status.h>
#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/messages_robocup_ssl_robot_status.pb.h>

#include "PacketConvert.hpp"

using namespace std;
using namespace Packet;
using namespace boost::asio;

SimRadio::SimRadio(Context* context, bool blue_team)
    : _context(context),
      _blueTeam(blue_team),
      _socket(_io_service, ip::udp::endpoint(ip::udp::v4(),
                                             blue_team ? SimBlueStatusPort
                                                       : SimYellowStatusPort)) {
    _grsim_endpoint = ip::udp::endpoint(ip::udp::v4(), SimCommandPort);

    _buffer.resize(1024);
    startReceive();
}

bool SimRadio::isOpen() const { return _socket.is_open(); }

void SimRadio::send(const std::array<RobotIntent, Num_Shells>& intents,
                    const std::array<MotionSetpoint, Num_Shells>& setpoints) {
    grSim_Packet sim_packet;
    grSim_Commands* sim_robot_commands = sim_packet.mutable_commands();

    for (int i = 0; i < Num_Shells; i++) {
        const auto& intent = intents[i];
        if (!intent.is_active) {
            continue;
        }

        const auto& setpoint = setpoints[i];

        grSim_Robot_Command* sim_robot =
            sim_robot_commands->add_robot_commands();
        ConvertTx::to_grsim(intent, setpoint, i, sim_robot);
    }

    sim_robot_commands->set_isteamyellow(!_blueTeam);
    sim_robot_commands->set_timestamp(RJ::timestamp());

    std::string out;
    sim_packet.SerializeToString(&out);

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
    grSim_Packet sim_packet;
    grSim_Commands* sim_robot_commands = sim_packet.mutable_commands();

    for (auto* const our_robot : _context->state.self) {
        grSim_Robot_Command* sim_robot =
            sim_robot_commands->add_robot_commands();
        sim_robot->set_id(our_robot->shell());
        sim_robot->set_veltangent(0);
        sim_robot->set_velnormal(0);
        sim_robot->set_velangular(0);

        sim_robot->set_kickspeedx(0);
        sim_robot->set_kickspeedz(0);

        sim_robot->set_spinner(false);
        sim_robot->set_wheelsspeed(false);
    }

    sim_robot_commands->set_isteamyellow(!_blueTeam);
    sim_robot_commands->set_timestamp(RJ::timestamp());

    std::string out;
    sim_packet.SerializeToString(&out);
    _socket.send_to(boost::asio::buffer(out),
                    ip::udp::endpoint(ip::udp::v4(), SimCommandPort));
}

void SimRadio::switchTeam(bool blue_team) {
    _blueTeam = blue_team;

    if (_socket.is_open()) {
        stopRobots();
        _socket.close();
    }

    _socket.open(ip::udp::v4());

    int status_port = blue_team ? SimBlueStatusPort : SimYellowStatusPort;

    // Let them throw exceptions
    _socket.bind(ip::udp::endpoint(ip::udp::v4(), status_port));
}
