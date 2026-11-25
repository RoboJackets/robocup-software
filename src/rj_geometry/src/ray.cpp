#include "rj_geometry/ray.hpp"

#include <algorithm>

#include "rj_geometry/point.hpp"

namespace rj_geometry {

Point Ray::nearest_point(const Point& p) const {
    // direction_ is unit (or zero), so this is the signed distance along the ray
    const double t = std::max(0.0, direction_.dot(p - origin_));
    return origin_ + direction_ * t;
}

float Ray::dist_to(const Point& p) const {
    return static_cast<float>((p - nearest_point(p)).mag());
}

}  // namespace rj_geometry
