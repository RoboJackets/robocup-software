#pragma once

#include <rj_robocup_protobuf/messages_robocup_ssl_wrapper.pb.h>

#include <rj_robocup/msg/vision_packet.hpp>
#include <time.hpp>

using VisionPacketMsg = rj_robocup::msg::VisionPacket;

/**
 * An unfiltered vision packet from SSL vision.
 *
 * Only the vision filter should attempt to use this directly.
 */
class VisionPacket {
public:
    /// Local time when the packet was received
    RJ::Time receivedTime;

    /// protobuf message from the vision system
    SSL_WrapperPacket wrapper;

    [[nodiscard]] VisionPacketMsg toMsg() const {
        VisionPacketMsg msg{};

        msg.time = RJ::ToROS(receivedTime);

        const auto packet_size = wrapper.ByteSizeLong();
        msg.wrapper.data.reserve(packet_size);

        wrapper.SerializeWithCachedSizesToArray(msg.wrapper.data.data());

        return msg;
    }

    static std::unique_ptr<VisionPacket> fromMsg(const VisionPacketMsg& msg) {
        auto packet = std::make_unique<VisionPacket>();
        packet->receivedTime = RJ::fromROS(rclcpp::Time{msg.time});
        packet->wrapper.ParseFromArray(msg.wrapper.data.data(),
                                       msg.wrapper.data.size());

        return packet;
    }
};
