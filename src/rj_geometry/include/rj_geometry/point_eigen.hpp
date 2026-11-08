#pragma once

#include <Eigen/Dense>

#include <rj_geometry/point.hpp>

namespace rj_geometry {

/// Eigen boundary. Point itself does not include Eigen.
[[nodiscard]] inline Eigen::Vector2d to_eigen(const Point& point) {
    return Eigen::Vector2d(point.x(), point.y());
}

[[nodiscard]] inline Point point_from_eigen(const Eigen::Vector2d& vector) {
    return Point(vector.x(), vector.y());
}

}  // namespace rj_geometry
