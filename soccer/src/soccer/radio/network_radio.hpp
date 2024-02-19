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
    // Send Control Message through the Base Station to the Robots
    void send_control_message(uint8_t robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                              const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                              strategy::Positions role) override;

    // Poll the asynchronous boost::asio receiver
    void poll_receive() override;

    // Switch teams and let the base station know
    void switch_team(bool blue_team) override;

private:
    void start_robot_status_receive();

    void start_alive_robots_receive();

    /**
     * @brief Parse the alive robots from a packet received via the base station.
     *
     * @param error
     * @param num_bytes
     */
    void receive_robot_status(const boost::system::error_code& error, size_t num_bytes);

    /**
     * @brief Parse the alive robots from a packet received via the base station.
     *
     * @param error
     * @param num_bytes
     */
    void receive_alive_robots(const boost::system::error_code& error, size_t num_bytes);

    // Where to send control messages to
    boost::asio::ip::udp::endpoint control_message_endpoint_ = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(kBaseStationAddress), kBaseStationControlMessagePort);
    // Buffer to send a set of control messages
    std::vector<std::array<uint8_t, sizeof(rtp::ControlMessage)>> send_buffers_{};

    // What local endpoint to expect robot statuses to be received at
    boost::asio::ip::udp::endpoint robot_status_endpoint_ = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(kBaseStationAddress), kBaseStationRobotStatusMessagePort);
    // Buffer for an incoming robot status from the base station
    std::array<uint8_t, sizeof(rtp::RobotStatusMessage)> robot_status_buffer_{};

    // Where local endpoint to expect alive robots to be received at
    boost::asio::ip::udp::endpoint alive_robots_endpoint_ = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(kBaseStationAddress), kBaseStationAliveRobotsMessagePort);
    // Buffer for an alive robots message from the base station
    std::array<uint8_t, 2> alive_robots_buffer_{};
    // if alive_robots_[robot_id] = true => robot[robot_id] is alive
    std::array<bool, kNumShells> alive_robots_{};

    // Keep io_service above the socket
    // https://stackoverflow.com/questions/26243008/error-initializing-a-boost-udp-socket-with-a-boost-io-service
    boost::asio::io_service io_service_;
    // The socket used to send control messages to the base station
    boost::asio::ip::udp::socket control_message_socket_;
    // The socket used to receive robot status messages from the base station
    boost::asio::ip::udp::socket robot_status_socket_;
    // The socket used to receive alive robots messages from the base station
    boost::asio::ip::udp::socket alive_robots_socket_;
};

}  // namespace radio
