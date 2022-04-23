#include "sim_radio.hpp"

#include <cmath>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include <rj_common/network.hpp>
#include <rj_protos/ssl_simulation_control.pb.h>
#include <rj_protos/ssl_simulation_robot_control.pb.h>
#include <rj_protos/ssl_simulation_robot_feedback.pb.h>

#include "packet_convert.hpp"

using namespace std;
using namespace Packet;
using namespace boost::asio;

namespace radio {

DEFINE_STRING(kRadioParamModule, interface, "172.25.0.10", "The interface for sim radio operation");

static SimulatorCommand convert_placement_to_proto(
    const rj_msgs::srv::SimPlacement::Request& placement) {
    SimulatorCommand packet;
    auto* control = packet.mutable_control();
    if (!placement.ball.position.empty()) {
        const auto& position = placement.ball.position.at(0);
        control->mutable_teleport_ball()->set_x(static_cast<float>(position.x));
        control->mutable_teleport_ball()->set_y(static_cast<float>(position.y));
        control->mutable_teleport_ball()->set_z(0);
        control->mutable_teleport_ball()->set_vx(0);
        control->mutable_teleport_ball()->set_vy(0);
        control->mutable_teleport_ball()->set_vz(0);
        control->mutable_teleport_ball()->set_teleport_safely(true);
    }

    if (!placement.ball.velocity.empty()) {
        const auto& velocity = placement.ball.velocity.at(0);
        control->mutable_teleport_ball()->set_vx(velocity.x);
        control->mutable_teleport_ball()->set_vy(velocity.y);
        control->mutable_teleport_ball()->set_vz(0);
    }

    for (const auto& robot : placement.robots) {
        auto* robot_proto = control->add_teleport_robot();
        robot_proto->set_x(static_cast<float>(robot.pose.position.x));
        robot_proto->set_y(static_cast<float>(robot.pose.position.y));
        robot_proto->set_orientation(static_cast<float>(robot.pose.heading));
        robot_proto->set_v_x(0);
        robot_proto->set_v_y(0);
        robot_proto->set_v_angular(0);
        robot_proto->mutable_id()->set_id(robot.robot_id);
        robot_proto->mutable_id()->set_team(robot.is_blue_team ? Team::BLUE : Team::YELLOW);
        robot_proto->set_present(true);
    }
    return packet;
}

SimRadio::SimRadio(bool blue_team)
    : Radio(),
      blue_team_(blue_team),
      socket_(io_service_, ip::udp::endpoint(ip::udp::v4(), blue_team ? kSimBlueStatusPort
                                                                      : kSimYellowStatusPort)) {
    for (int i = 0; i < kNumShells; ++i) {
        last_sent_diff.emplace_back(RJ::now());
    }
    auto address = boost::asio::ip::make_address(PARAM_interface).to_v4();
    robot_control_endpoint_ =
        ip::udp::endpoint(address, blue_team ? kSimBlueCommandPort : kSimYellowCommandPort);
    sim_control_endpoint_ = ip::udp::endpoint(address, kSimCommandPort);

    buffer_.resize(1024);
    start_receive();

    const auto& placement_callback =
        [this](const rj_msgs::srv::SimPlacement::Request::SharedPtr request,  // NOLINT
               [[maybe_unused]] const rj_msgs::srv::SimPlacement::Response::SharedPtr
                   response) {  // NOLINT
            send_sim_command(convert_placement_to_proto(*request));
        };
    sim_placement_service_ = create_service<rj_msgs::srv::SimPlacement>(
        sim::topics::kSimPlacementSrv, placement_callback);
}

void SimRadio::send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                    const rj_msgs::msg::ManipulatorSetpoint& manipulator) {
    RobotControl sim_packet;

    // Send a sim packet with a single robot. The simulator can handle many robots, but our commands
    // may come in at different times and it should be fine to just recalculate like this.
    // TODO(Kyle): Verify that this is okay.
    if (RJ::now() - last_sent_diff.at(robot_id) < 5.1ms) return;
    last_sent_diff.at(robot_id) = RJ::now();
    RobotCommand* sim_robot = sim_packet.add_robot_commands();
    ConvertTx::ros_to_sim(manipulator, motion, robot_id, sim_robot);

    std::string out;
    sim_packet.SerializeToString(&out);
    if (sim_robot->kick_speed() > 0) {
        SPDLOG_ERROR("sim_robot: {} {} {} \n", sim_robot->id(), sim_robot->kick_speed(),
                     sim_robot->dribbler_speed());
    }

    socket_.send_to(buffer(out), robot_control_endpoint_);
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
    RobotControlResponse packet;

    packet.ParseFromString(data);

    for (size_t pkt_idx = 0; pkt_idx < packet.feedback_size(); pkt_idx++) {
        rj_msgs::msg::RobotStatus status_ros;
        RobotStatus status;
        const RobotFeedback& sim_status = packet.feedback(pkt_idx);
        ConvertRx::sim_to_status(sim_status, &status);
        ConvertRx::status_to_ros(status, &status_ros);

        publish(status_ros.robot_id, status_ros);
    }
}

void SimRadio::stop_robots() {
    RobotControl sim_packet;

    for (int shell = 0; shell < kNumShells; shell++) {
        auto* sim_robot = sim_packet.add_robot_commands();
        sim_robot->set_id(shell);
        auto* local_velocity = sim_robot->mutable_move_command()->mutable_local_velocity();
        local_velocity->set_forward(0);
        local_velocity->set_left(0);
        local_velocity->set_angular(0);

        sim_robot->set_kick_speed(0);
        sim_robot->set_kick_angle(0);

        sim_robot->set_dribbler_speed(0);
    }

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
    socket_.bind(ip::udp::endpoint(ip::make_address("172.25.0.11").to_v4(), status_port));

    auto address = boost::asio::ip::make_address(PARAM_interface).to_v4();
    robot_control_endpoint_ =
        ip::udp::endpoint(address, blue_team ? kSimBlueCommandPort : kSimYellowCommandPort);
}

void SimRadio::send_sim_command(const SimulatorCommand& cmd) {
    std::string out;
    cmd.SerializeToString(&out);
    size_t bytes = socket_.send_to(boost::asio::buffer(out), sim_control_endpoint_);
    if (bytes == 0) {
        SPDLOG_ERROR("Sent 0 bytes.");
    }
}

}  // namespace radio
