#pragma once

#include <rj_protos/messages_robocup_ssl_wrapper.pb.h>

#include <rclcpp/time.hpp>

namespace vision_receiver {
/**
 * @brief A struct that adds receive time information to a SSL_WrapperPacket.
 */
struct StampedSSLWrapperPacket {
    using UniquePtr = std::unique_ptr<StampedSSLWrapperPacket>;

    SSL_WrapperPacket wrapper;
    rclcpp::Time receive_time;
};
}  // namespace vision_receiver
