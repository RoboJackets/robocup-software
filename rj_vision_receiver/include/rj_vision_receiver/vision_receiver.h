#pragma once

#include <config_client/config_client.h>
#include <rj_protos/messages_robocup_ssl_wrapper.pb.h>
#include <rj_vision_receiver/stamped_wrapper_packet.h>

#include <boost/asio.hpp>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rj_common/Network.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>
#include <vector>

namespace vision_receiver {
using RawProtobufMsg = rj_msgs::msg::RawProtobuf;
using DetectionFrameMsg = rj_msgs::msg::DetectionFrame;
using DetectionBallMsg = rj_msgs::msg::DetectionBall;
using DetectionRobotMsg = rj_msgs::msg::DetectionRobot;

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

    void run();

    void setPort(int port);

private:
    void startReceive();
    void receivePacket(const boost::system::error_code& error,
                       std::size_t num_bytes);

    // Helper function to decide whether or not to remove a robot at a given
    // x position
    /**
     * @brief Given the current config on which half we are enabling vision on,
     * return whether the x coordinate lies in an enabled half.
     * @param defend_plus_x Whether we are defending x>0.
     * @param x The x coordinate.
     * @return Whether the x coordinate lies in an enabled half.
     */
    [[nodiscard]] bool InUsedHalf(bool defend_plus_x, double x) const;

    /**
     * @brief Process new packets
     *
     * Publishes the raw packet. If the packet has geometry info, publish that.
     * If it has detection information, also publish that.
     */
    void processNewPackets();

    /**
     * @brief Converts from the janky floating point time that is used in
     * SSL_DetectionFrame to a proper time that uses integers.
     * @param time_since_epoch_s Floating point time
     * @return rclcpp::Time
     */
    static rclcpp::Time ToROSTime(double time_since_epoch_s);

    /**
     * @brief "Syncs" up the timestamp of the proto (because the computer clock
     * isn't necessarily synchronized with the vision computer's clock) by
     * assuming that the vision packet was sent when we received it with 0
     * latency.
     * @param frame
     * @param receive_time The time we received the message on this computer.
     */
    static void SyncDetectionTimestamp(DetectionFrameMsg* frame,
                                const rclcpp::Time& receive_time);

    void UpdateGeometryPacket(const SSL_GeometryFieldSize& fieldSize);

    /**
     * @brief Converts from a SSL_WrapperPacket to a DetectionFrameMsg,
     * respecting the "exclude half" config as well as performing timestamp
     * synchronization.
     * @param packet
     * @param received_time Time that this packet was received.
     * @return
     */
    [[nodiscard]] DetectionFrameMsg ToROSMsg(
        const SSL_DetectionFrame& detection_frame, const rclcpp::Time& received_time) const;

    /**
     * @brief Converts from SSL_WrapperPacket to a RawProtobufMsg.
     * @param wrapper
     * @return
     */
    [[nodiscard]] static RawProtobufMsg::UniquePtr ToROSMsg(
        const SSL_WrapperPacket& wrapper) ;

    /**
     * @brief Converts from SSL_DetectionBall to a DetectionBallMsg.
     * @param ball
     * @return
     */
    [[nodiscard]] static DetectionBallMsg ToROSMsg(
        const SSL_DetectionBall& ball) ;

    /**
     * @brief Converts from SSL_DetectionRobot to a DetectionRobotMsg.
     * @param robot
     * @return
     */
    [[nodiscard]] static DetectionRobotMsg ToROSMsg(
        const SSL_DetectionRobot& robot) ;

    config_client::ConfigClient config_;
    int port_;

    std::vector<uint8_t> _recv_buffer{};

    boost::asio::io_service _io_context;
    boost::asio::ip::udp::socket _socket;

    boost::asio::ip::udp::endpoint _sender_endpoint;

    std::vector<StampedSSLWrapperPacket::UniquePtr> _packets{};
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<RawProtobufMsg>::SharedPtr raw_packet_pub_;
    rclcpp::Publisher<DetectionFrameMsg>::SharedPtr detection_frame_pub_;
};
}  // namespace vision_receiver
