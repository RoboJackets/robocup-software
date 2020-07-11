#pragma once

#include <rj_protos/messages_robocup_ssl_wrapper.pb.h>

#include <rj_common/time.hpp>

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
};
