#pragma once

#include <mutex>

#include <boost/asio.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/config.hpp>

#include <rj_msgs/msg/alive_robots.hpp>
#include <robot_intent.hpp>

#include "radio.hpp"
#include "strategy/agent/position/positions.hpp"

#include "rc-fshare/rtp.hpp"

namespace radio {

/**
 * @brief Interface for the radio over regular network interface
 *
 * TODO(Kyle): Clean this up by removing dual-radio support.
 */
class NetworkRadio : public Radio {
public:
    NetworkRadio();

protected:
    void send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
              const rj_msgs::msg::ManipulatorSetpoint& manipulator,
              strategy::Positions role) override;
    void receive() override;
    void switch_team(bool blue) override;

    struct RobotConnection {
        boost::asio::ip::udp::endpoint endpoint;
        RJ::Time last_received;
    };

    // Connections to the robots, indexed by robot ID.
    std::vector<std::optional<RobotConnection>> connections_{};

    // Map from IP address to robot ID.
    std::map<boost::asio::ip::udp::endpoint, int> robot_ip_map_{};

    void receive_packet(const boost::system::error_code& error, std::size_t num_bytes);

    void start_receive();

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    int param_server_port_;

    // Written by `async_receive_from`.
    std::array<char, rtp::ReverseSize> recv_buffer_;
    boost::asio::ip::udp::endpoint robot_endpoint_;

    // Read from by `async_send_to`
    std::vector<std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)>> send_buffers_{};

    constexpr static std::chrono::duration kTimeout = std::chrono::milliseconds(250);

    rclcpp::Publisher<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_pub_;
    rclcpp::TimerBase::SharedPtr alive_robots_timer_;

    /**
     * @brief Publish a vector of alive robots to the "strategy/alive_robots" endpoint
     *
     */
    void publish_alive_robots();
};

}  // namespace radio
