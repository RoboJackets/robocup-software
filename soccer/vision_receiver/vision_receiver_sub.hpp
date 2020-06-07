#pragma once

#include <config_client/config_client.h>
#include <network/network_constants.h>
#include <rj_robocup_protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <vision/vision_packet.h>

#include <boost/asio.hpp>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rj_robocup/msg/raw_protobuf.hpp>
#include <rj_robocup/msg/vision_packet.hpp>
#include <time.hpp>
#include <vector>

namespace vision_receiver {
using RawProtobufMsg = rj_robocup::msg::RawProtobuf;
using VisionPacketMsg = rj_robocup::msg::VisionPacket;

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
    explicit VisionReceiver();

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

    /**
     * \brief Process new packets
     *
     * 1. Publish the raw packet
     * 2. If packet has geometry info, publish that
     * 3. Remove balls detected on the excluded half of the field
     * 4. Remove robots detected on the excluded half of the field
     */
    void processNewPackets();

    /**
     * \brief Serializes the SSL_WrapperPacket and publishes it to
     * raw_packet_pub
     *
     * @param packet
     */
    void publishRawPacket(const SSL_WrapperPacket& packet);

    /**
     * \brief Publishes the passed in VisionPacket
     * @param packet
     */
    void publishVisionPacket(std::unique_ptr<VisionPacket> packet);

    void updateGeometryPacket(const SSL_GeometryFieldSize& fieldSize);

    config_client::ConfigClient config_;
    int port_;

    std::vector<uint8_t> _recv_buffer{};

    boost::asio::io_service _io_context;
    boost::asio::ip::udp::socket _socket;

    boost::asio::ip::udp::endpoint _sender_endpoint;

    RJ::Time _last_receive_time;

    std::vector<std::unique_ptr<VisionPacket>> _packets{};
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<RawProtobufMsg>::SharedPtr raw_packet_pub_;
    rclcpp::Publisher<VisionPacketMsg>::SharedPtr vision_packet_pub_;
};
}  // namespace vision_receiver
