#include <rj_geometry/point.hpp>
#include <rj_geometry/util.hpp>

namespace rj_geometry {
bool Point::nearly_equals(Point other) const {
    return nearly_equal(static_cast<float>(x()), static_cast<float>(other.x())) &&
           nearly_equal(static_cast<float>(y()), static_cast<float>(other.y()));
}
}  // namespace rj_geometry
