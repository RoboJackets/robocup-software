#pragma once

#include <rj_geometry/point.hpp>
#include <rj_protos/Point.pb.h>

namespace rj_geometry {

/// Protobuf log-frame boundary. Point itself does not include Point.pb.h.
[[nodiscard]] inline Packet::Point to_packet_point(const Point& point) {
    Packet::Point out;
    out.set_x(static_cast<float>(point.x()));
    out.set_y(static_cast<float>(point.y()));
    return out;
}

[[nodiscard]] inline Point point_from_packet(const Packet::Point& point) {
    return Point(point.x(), point.y());
}

}  // namespace rj_geometry
