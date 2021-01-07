#include "sim_radio.hpp"

#include <cmath>
#include <stdexcept>

#include <rj_common/network.hpp>
#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/messages_robocup_ssl_robot_status.pb.h>

#include "packet_convert.hpp"

using namespace std;
using namespace Packet;
using namespace boost::asio;

namespace radio {

SimRadio::SimRadio(bool blue_team)
    : Radio(),
      blue_team_(blue_team),
      socket_(io_service_, ip::udp::endpoint(ip::udp::v4(), blue_team ? kSimBlueStatusPort
                                                                      : kSimYellowStatusPort)) {
    grsim_endpoint_ = ip::udp::endpoint(ip::udp::v4(), kSimCommandPort);

    buffer_.resize(1024);
    start_receive();
}

void SimRadio::send(RobotId robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                    const rj_msgs::msg::ManipulatorSetpoint& manipulator) {
    grSim_Packet sim_packet;
    grSim_Commands* sim_robot_commands = sim_packet.mutable_commands();

    // Send a grSim packet with a single robot. grSim can handle many robots, but our commands may
    // come in at different times and it should be fine to just recalculate like this.
    // TODO(Kyle): Verify that this is okay.
    grSim_Robot_Command* sim_robot = sim_robot_commands->add_robot_commands();
    ConvertTx::ros_to_grsim(manipulator, motion, robot_id, sim_robot);

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
        rj_msgs::msg::RobotStatus status_ros;
        RobotStatus status;
        const Robot_Status& grsim_status = packet.robots_status(pkt_idx);
        ConvertRx::grsim_to_status(grsim_status, &status);
        ConvertRx::status_to_ros(status, &status_ros);

        publish(status_ros.robot_id, status_ros);
    }
}

void SimRadio::stop_robots() {
    grSim_Packet sim_packet;
    grSim_Commands* sim_robot_commands = sim_packet.mutable_commands();

    for (RobotId shell = 0; shell < kNumShells; shell++) {
        grSim_Robot_Command* sim_robot = sim_robot_commands->add_robot_commands();
        sim_robot->set_id(shell);
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
    if (blue_team == blue_team_) {
        return;
    }

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

}  // namespace radio
