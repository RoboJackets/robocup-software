#pragma once

#include <mutex>

#include <boost/asio.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/config.hpp>

#include <rj_common/network.hpp>
#include <rj_common/time.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <robot_intent.hpp>

#include "radio.hpp"
#include "strategy/agent/position/positions.hpp"

#include "rc-fshare/rtp.hpp"

namespace radio {

/**
 * @brief Interface for the radio over regular network interface
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

    boost::asio::ip::udp::endpoint base_station_endpoint = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(kBaseStationAddress), kBaseStationPort);

    boost::asio::ip::udp::endpoint robot_status_endpoint = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string("127.0.0.1"), kIncomingBaseStationDataPort);
    boost::asio::ip::udp::endpoint alive_robots_endpoint = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string("127.0.0.1"), kIncomingBaseStationAliveRobotsPort);

    // Fuck Around and Find Out
    // https://stackoverflow.com/questions/26243008/error-initializing-a-boost-udp-socket-with-a-boost-io-service
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;

    void receive_packet(const boost::system::error_code& error, std::size_t num_bytes);

    void start_receive();

    // Written by `async_receive_from`.
    std::array<char, sizeof(rtp::RobotStatusMessage)> robot_status_buffer_;

    // Written by 'async_receive_from`.
    std::array<char, 2> alive_robots_buffer_;

    // Read from by `async_send_to`
    std::vector<std::array<uint8_t, sizeof(rtp::ControlMessage)>> send_buffers_{};

    rclcpp::Publisher<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_pub_;

    /**
     * @brief Publish a vector of alive robots to the "strategy/alive_robots" endpoint
     *
     */
    void publish_alive_robots(const boost::system::error_code& error, std::size_t num_bytes);

private:
    bool _blue_team = false;
};

}  // namespace radio
