#include <cmath>

#include <rj_geometry/line.hpp>
#include <rj_geometry/point.hpp>

namespace rj_geometry {

// Has Simple Test
float Line::dist_to(Point p) const {
    const Point d = delta();
    const auto top = static_cast<float>(d.x() * (points_[0].y() - p.y()) -
                                        (points_[0].x() - p.x()) * d.y());

    return std::fabs(top) / static_cast<float>(d.mag());
}

// Very Simple Test
Point Line::nearest_point(Point p) const {
    Point v_hat = delta().normalized();
    return points_[0] + v_hat * v_hat.dot(p - points_[0]);
}
}  // namespace rj_geometry
