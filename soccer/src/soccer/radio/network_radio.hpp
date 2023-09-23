#pragma once

#include <mutex>

#include <boost/asio.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/config.hpp>

#include <rj_msgs/msg/alive_robots.hpp>
#include <robot_intent.hpp>

#include "radio.hpp"
#include "strategy/coach/coach_node.hpp"

#include "rc-fshare/rtp.hpp"

#include <rj_common/time.hpp>
#include <rj_common/network.hpp>

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

    boost::asio::ip::udp::endpoint base_station_endpoint = 
        boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(kBaseStationAddress), kBaseStationPort);
    // TODO: Make this not hardcoded
    boost::asio::ip::udp::endpoint bound_endpoint = 
        boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), kIncomingBaseStationDataPort);

    // Fuck Around and Find Out
    // https://stackoverflow.com/questions/26243008/error-initializing-a-boost-udp-socket-with-a-boost-io-service
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;

    void receive_packet(const boost::system::error_code& error, std::size_t num_bytes);

    void start_receive();

    // Written by `async_receive_from`.
    std::array<char, sizeof(rtp::RobotStatusMessage)> recv_buffer_;

    // Read from by `async_send_to`
    std::vector<std::array<uint8_t, sizeof(rtp::ControlMessage)>> send_buffers_{};

    constexpr static std::chrono::duration kTimeout = std::chrono::milliseconds(250);

    rclcpp::Publisher<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_pub_;
    rclcpp::TimerBase::SharedPtr alive_robots_timer_;
    std::array<RJ::Time, 6> last_heard_from;

    /**
     * @brief Publish a vector of alive robots to the "strategy/alive_robots" endpoint
     *
     */
    void publish_alive_robots();
};

}  // namespace radio
