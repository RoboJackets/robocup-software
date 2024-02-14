#pragma once

#include <cstdint>

#include <boost/asio.hpp>

#include <rj_common/time.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/srv/sim_placement.hpp>
#include <rj_protos/ssl_simulation_control.pb.h>

#include "context.hpp"
#include "radio.hpp"

namespace radio {

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    SimRadio(bool blue_team = false);

protected:
    void send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
              const rj_msgs::msg::ManipulatorSetpoint& manipulator,
              strategy::Positions role) override;
    void receive() override;
    void switch_team(bool blue) override;

private:
    void stop_robots();

    void handle_receive(const std::string& data);
    void start_receive();
    void receive_packet(const boost::system::error_code& error, size_t num_bytes);

    // For ball and robot placement
    void send_sim_command(const SimulatorCommand& cmd);

    // Publishing Alive Robots
    rclcpp::TimerBase::SharedPtr alive_robots_timer_;
    rclcpp::Publisher<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_pub_;
    void publish_alive_robots();

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

    rclcpp::Service<rj_msgs::srv::SimPlacement>::SharedPtr sim_placement_service_;
};

}  // namespace radio
