#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <boost/asio.hpp>
#include <spdlog/spdlog.h>

#include <rj_protos/ssl_simulation_control.pb.h>
#include <rj_protos/ssl_simulation_robot_control.pb.h>
#include <rj_protos/ssl_simulation_robot_feedback.pb.h>
#include <rj_radio_msgs/msg/alive_robots.hpp>
#include <rj_time/time.hpp>
#include <rj_ui_msgs/srv/sim_placement.hpp>

#include "rj_radio/packet_convert.hpp"
#include "rj_radio/radio.hpp"

namespace radio {

static const int kSimBlueStatusPort = 30011;
static const int kSimYellowStatusPort = 30012;
static const int kSimCommandPort = 10300;
static const int kSimBlueCommandPort = 10301;
static const int kSimYellowCommandPort = 10302;

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    SimRadio(bool blue_team = false);

protected:
    //
    void send_control_message(
        uint8_t robot_id, const rj_control_msgs::msg::MotionSetpoint& motion,
        const rj_control_msgs::msg::ManipulatorSetpoint& manipulator) override;

    // Poll the asynchronous receiver
    void poll_receive() override;

    // Switch teams and let the UI know
    void switch_team(bool blue_team) override;

private:
    void stop_robots();

    void handle_receive(const std::string& data);
    void start_receive();
    void receive_packet(const boost::system::error_code& error, size_t num_bytes);

    // For ball and robot placement
    void send_sim_command(const SimulatorCommand& cmd);

    // Timer to publish alive robots (i.e. all robots are alive)
    rclcpp::TimerBase::SharedPtr alive_robots_timer_;
    std::array<bool, kNumShells> alive_robots_{};
    rclcpp::Publisher<rj_radio_msgs::msg::AliveRobots>::SharedPtr alive_robots_pub_;

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;

    // created based on ROS param given from Radio superclass
    std::string param_radio_interface_;
    boost::asio::ip::address_v4 address_;
    boost::asio::ip::udp::endpoint sim_control_endpoint_;
    boost::asio::ip::udp::endpoint robot_control_endpoint_;

    std::vector<char> buffer_;
    std::vector<RJ::Time> last_sent_diff_;

    bool blue_team_;

    rclcpp::Service<rj_ui_msgs::srv::SimPlacement>::SharedPtr sim_placement_service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr interface_callback_handle_;
};

}  // namespace radio
