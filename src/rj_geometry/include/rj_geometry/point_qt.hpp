#pragma once

#include <QtCore/QPoint>
#include <QtCore/QPointF>

#include <rj_geometry/point.hpp>

namespace rj_geometry {

/// Qt drawing boundary. Point itself does not include Qt.
[[nodiscard]] inline QPointF to_qpointf(const Point& point) {
    return QPointF(point.x(), point.y());
}

[[nodiscard]] inline Point point_from_qpointf(const QPointF& point) {
    return Point(point.x(), point.y());
}

[[nodiscard]] inline Point point_from_qpoint(const QPoint& point) {
    return Point(static_cast<double>(point.x()), static_cast<double>(point.y()));
}

/// Same name the log viewer already uses for protobuf points, so a geometry
/// point converts with qpointf(point) via argument-dependent lookup.
[[nodiscard]] inline QPointF qpointf(const Point& point) { return to_qpointf(point); }

}  // namespace rj_geometry
