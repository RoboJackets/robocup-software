#pragma once

#include <mutex>

#include <boost/asio.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/config.hpp>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <rj_common/network.hpp>
#include <rj_common/radio/messages/control_message.hpp>
#include <rj_common/radio/messages/robot_status_message.hpp>
#include <rj_common/radio/packet_convert.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/status.hpp>
#include <rj_common/strategy/positions.hpp>
#include <rj_common/time.hpp>
#include <rj_geometry/util.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_param_utils/global_params.hpp>
#include <rj_utils/logging.hpp>

#include "rj_radio/radio.hpp"

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

    /**
     * @brief Parse a robot status from a packet received via the base station.
     *
     * @param error
     * @param num_bytes
     */
    void receive_robot_status(const boost::system::error_code& error, size_t num_bytes);

    /**
     * @brief Publish which robots are alive, derived from recent RobotStatus
     * reception (the same signal the UI's robot list is built from). A robot is
     * alive iff it has reported within PARAM_timeout.
     */
    void publish_alive_from_status();

    // Where to send control messages to
    boost::asio::ip::udp::endpoint control_message_endpoint_ = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(kBaseStationAddress), kControlMessageSocketPort);
    // Buffer to send a set of control messages
    std::vector<std::array<uint8_t, sizeof(RadioMessage::ControlMessage)>> send_buffers_{};

    // What local endpoint to expect robot statuses to be received at
    boost::asio::ip::udp::endpoint robot_status_endpoint_ = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string("0.0.0.0"), kRobotStatusMessageSocketPort);
    // Buffer for an incoming robot status from the base station
    std::array<uint8_t, sizeof(RadioMessage::RobotStatusMessage)> robot_status_buffer_{};

    // Periodically republishes alive robots derived from RobotStatus reception.
    rclcpp::TimerBase::SharedPtr alive_robots_timer_;

    // Keep io_service above the socket
    // https://stackoverflow.com/questions/26243008/error-initializing-a-boost-udp-socket-with-a-boost-io-service
    boost::asio::io_service io_service_;
    // The socket used to send control messages to the base station
    boost::asio::ip::udp::socket control_message_socket_;
    // The socket used to receive robot status messages from the base station
    boost::asio::ip::udp::socket robot_status_socket_;
};

}  // namespace radio
