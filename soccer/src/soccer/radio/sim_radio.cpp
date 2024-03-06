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

        // TODO(Kevin): make this only happen in sim (set a param)
        // this line sets robots with an ID above 6 to "not present", meaning
        // they will be removed on their next sim command
        // see "rj_protos/proto/ssl_simulation_control.proto"
        //
        // to see this in effect, click and drag the robots expected to be not
        // present and they should disappear
        if (robot.robot_id <= 5) {
            robot_proto->set_present(true);
        } else {
            robot_proto->set_present(false);
        }
    }
    return packet;
}

SimRadio::SimRadio(bool blue_team)
    : Radio(),
      socket_(io_service_, ip::udp::endpoint(ip::udp::v4(), blue_team ? kSimBlueStatusPort
                                                                      : kSimYellowStatusPort)),
      blue_team_(blue_team) {
    for (size_t i = 0; i < kNumShells; ++i) {
        last_sent_diff_.emplace_back(RJ::now());
    }

    /* IP addr our radio should bind to
     * see PR #1887 for last time this file was used w/ external interface
     * run ifconfig to see list of interfaces on this computer

     * NOTE: on field comp, make sure this IP is "172.16.1.1" (router IP)
     * on sim, make sure this IP is "127.0.0.1" (localhost)
     */
    // TODO(Kevin): this default shouldn't be necessary, but sim2play refuses to accept the param
    // file probably an issue with the hacky way I got param files to dynamically load
    std::string localhost = "127.0.0.1";
    this->get_parameter_or("interface", param_radio_interface_, localhost);
    SPDLOG_INFO("SimRadio param_radio_interface_ {}", param_radio_interface_);
    address_ = boost::asio::ip::make_address(param_radio_interface_).to_v4();
    robot_control_endpoint_ =
        ip::udp::endpoint(address_, blue_team_ ? kSimBlueCommandPort : kSimYellowCommandPort);
    sim_control_endpoint_ = ip::udp::endpoint(address_, kSimCommandPort);

    // assume robots 0-6 are always alive in sim
    for (uint8_t robot_id = 0; robot_id < 7; robot_id++) {
        alive_robots_[robot_id] = true;
    }
    alive_robots_pub_ =
        this->create_publisher<rj_msgs::msg::AliveRobots>(strategy::topics::kAliveRobots, rclcpp::QoS(1));

    alive_robots_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        rj_msgs::msg::AliveRobots alive_message{};
        alive_message.alive_robots = alive_robots_;
        publish_alive_robots(alive_message);
    });


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

void SimRadio::send_control_message(uint8_t robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                                    const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                                    strategy::Positions role) {
    RobotControl sim_packet;

    // Send a sim packet with a single robot. The simulator can handle many robots, but our commands
    // may come in at different times and it should be fine to just recalculate like this.
    // TODO(Kyle): Verify that this is okay.
    if (RJ::now() - last_sent_diff_.at(robot_id) < 5.1ms) {
        return;
    }
    last_sent_diff_.at(robot_id) = RJ::now();
    RobotCommand* sim_robot = sim_packet.add_robot_commands();
    ConvertTx::ros_to_sim(manipulator, motion, robot_id, sim_robot);

    std::string out;
    sim_packet.SerializeToString(&out);

    // print kick speed
    // TODO(Alex): replace with UI indicator
    /* if (sim_robot->kick_speed() > 0) { */
    /*     SPDLOG_ERROR("sim_robot: {} {} {} \n", sim_robot->id(), sim_robot->kick_speed(), */
    /*                  sim_robot->dribbler_speed()); */
    /* } */

    socket_.send_to(buffer(out), robot_control_endpoint_);
}

void SimRadio::poll_receive() { io_service_.poll(); }

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

    for (int pkt_idx = 0; pkt_idx < packet.feedback_size(); pkt_idx++) {
        rj_msgs::msg::RobotStatus status_ros;
        RobotStatus status;
        const RobotFeedback& sim_status = packet.feedback(pkt_idx);
        ConvertRx::sim_to_status(sim_status, &status);
        ConvertRx::status_to_ros(status, &status_ros);

        publish_robot_status(status_ros.robot_id, status_ros);
    }
}

void SimRadio::stop_robots() {
    RobotControl sim_packet;

    for (size_t shell = 0; shell < kNumShells; shell++) {
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
    // TODO(Kevin): fix me, in scrim-2022 we used the below line; what is this
    // IP supposed to be?
    socket_.bind(ip::udp::endpoint(ip::udp::v4(), status_port));
    // socket_.bind(ip::udp::endpoint(ip::make_address("172.25.0.11").to_v4(), status_port));

    // remake the robot_control_endpoint_ based on new team color
    robot_control_endpoint_ =
        ip::udp::endpoint(address_, blue_team ? kSimBlueCommandPort : kSimYellowCommandPort);
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
