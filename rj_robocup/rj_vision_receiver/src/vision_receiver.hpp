#pragma once

#include <rj_config_client/config_client.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>

#include <Network.hpp>
#include <boost/asio.hpp>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <time.hpp>
#include <vector>
#include <vision/VisionPacket.hpp>

namespace vision_receiver {
/**
 * @brief Receives vision packets over UDP and places them in a buffer until
 * they are read.
 *
 * @details When start() is called, a new thread is spawned that listens on a
 * UDP port for packets. If sim = true, it tries both simulator ports until one
 * works. Otherwise, it connects to the port specified in the constructor.
 *
 * Whenever a new packet comes in (encoded as Google Protobuf), it is parsed
 * into an SSL_WrapperPacket and placed onto the circular buffer @_packets.
 * They remain there until they are retrieved with getPackets().
 */
class VisionReceiver : public rclcpp::Node {
public:
    explicit VisionReceiver(int port = SharedVisionPortSinglePrimary);

    /// Copies the vector of packets and then clears it. The vector contains
    /// only packets received since the last time this was called (or since the
    /// VisionReceiver was started, if getPackets has never been called).
    ///
    /// The caller is responsible for freeing the packets after this function
    /// returns.
    void getPackets(std::vector<VisionPacket*>& packets);

    void run();

    void setPort(int port);

    RJ::Time getLastVisionTime() const { return _last_receive_time; }

private:
    void startReceive();
    void receivePacket(const boost::system::error_code& error,
                       std::size_t num_bytes);

    // Helper function to decide whether or not to remove a robot at a given
    // x position
    bool shouldRemove(bool defendPlusX, double x);

    void updateGeometryPacket(const SSL_GeometryFieldSize& fieldSize);

    config_client::ConfigClient config_;
    int port_;

    std::vector<uint8_t> _recv_buffer{};

    boost::asio::io_service _io_context;
    boost::asio::ip::udp::socket _socket;

    boost::asio::ip::udp::endpoint _sender_endpoint;

    RJ::Time _last_receive_time;

    std::vector<std::unique_ptr<VisionPacket>> _packets{};
};
}  // namespace vision_receiver
