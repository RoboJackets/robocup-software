#include "sim_radio.hpp"

#include <cmath>
#include <stdexcept>

#include <Geometry2d/Util.hpp>
#include <robot.hpp>
#include <rj_common/network.hpp>
#include <rj_common/status.hpp>
#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/messages_robocup_ssl_robot_status.pb.h>

#include "packet_convert.hpp"

using namespace std;
using namespace Packet;
using namespace boost::asio;

SimRadio::SimRadio(Context* context, bool blue_team)
    : context_(context),
      blue_team_(blue_team),
      socket_(io_service_, ip::udp::endpoint(ip::udp::v4(), blue_team ? kSimBlueStatusPort
                                                                      : kSimYellowStatusPort)) {
    grsim_endpoint_ = ip::udp::endpoint(ip::udp::v4(), kSimCommandPort);

    buffer_.resize(1024);
    start_receive();
}

bool SimRadio::is_open() const { return socket_.is_open(); }

void SimRadio::send(const std::array<RobotIntent, kNumShells>& intents,
                    const std::array<MotionSetpoint, kNumShells>& setpoints) {
    grSim_Packet sim_packet;
    grSim_Commands* sim_robot_commands = sim_packet.mutable_commands();

    for (int i = 0; i < kNumShells; i++) {
        const auto& intent = intents[i];
        if (!intent.is_active) {
            continue;
        }

        const auto& setpoint = setpoints[i];

        grSim_Robot_Command* sim_robot = sim_robot_commands->add_robot_commands();
        ConvertTx::to_grsim(intent, setpoint, i, sim_robot);
    }

    sim_robot_commands->set_isteamyellow(!blue_team_);
    sim_robot_commands->set_timestamp(RJ::timestamp());

    std::string out;
    sim_packet.SerializeToString(&out);

    socket_.send_to(buffer(out), grsim_endpoint_);
}

void SimRadio::receive() { io_service_.poll(); }

void SimRadio::start_receive() {
    // Set a receive callback
    socket_.async_receive(boost::asio::buffer(buffer_),
                          [this](const boost::system::error_code& error, std::size_t num_bytes) {
                              receive_packet(error, num_bytes);
                          });
}

void SimRadio::receive_packet(const boost::system::error_code& error, std::size_t num_bytes) {
    std::string data(buffer_.begin(), buffer_.end());
    handle_receive(data);
    start_receive();
}

void SimRadio::handle_receive(const std::string& data) {
    Robots_Status packet;

    packet.ParseFromString(data);

    for (size_t pkt_idx = 0; pkt_idx < packet.robots_status_size(); pkt_idx++) {
        RobotStatus status;
        const Robot_Status& grsim_status = packet.robots_status(pkt_idx);
        ConvertRx::grsim_to_status(grsim_status, &status);

        std::lock_guard<std::mutex> lock(reverse_packets_mutex_);
        reverse_packets_.push_back(status);
    }
}

void SimRadio::stop_robots() {
    grSim_Packet sim_packet;
    grSim_Commands* sim_robot_commands = sim_packet.mutable_commands();

    for (auto* const our_robot : context_->state.self) {
        grSim_Robot_Command* sim_robot = sim_robot_commands->add_robot_commands();
        sim_robot->set_id(our_robot->shell());
        sim_robot->set_veltangent(0);
        sim_robot->set_velnormal(0);
        sim_robot->set_velangular(0);

        sim_robot->set_kickspeedx(0);
        sim_robot->set_kickspeedz(0);

        sim_robot->set_spinner(false);
        sim_robot->set_wheelsspeed(false);
    }

    sim_robot_commands->set_isteamyellow(!blue_team_);
    sim_robot_commands->set_timestamp(RJ::timestamp());

    std::string out;
    sim_packet.SerializeToString(&out);
    socket_.send_to(boost::asio::buffer(out), ip::udp::endpoint(ip::udp::v4(), kSimCommandPort));
}

void SimRadio::switch_team(bool blue_team) {
    blue_team_ = blue_team;

    if (socket_.is_open()) {
        stop_robots();
        socket_.close();
    }

    socket_.open(ip::udp::v4());

    int status_port = blue_team ? kSimBlueStatusPort : kSimYellowStatusPort;

    // Let them throw exceptions
    socket_.bind(ip::udp::endpoint(ip::udp::v4(), status_port));
}
