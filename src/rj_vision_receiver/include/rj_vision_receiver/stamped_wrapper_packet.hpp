#pragma once

#include <rclcpp/time.hpp>

#include <rj_protos/vision/ssl_vision_wrapper.pb.h>

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
