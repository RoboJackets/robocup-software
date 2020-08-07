#include <Geometry2d/Point.hpp>
#include <Geometry2d/Util.hpp>

namespace Geometry2d {
bool Point::nearly_equals(Point other) const {
    return nearly_equal(static_cast<float>(x()), static_cast<float>(other.x())) &&
           nearly_equal(static_cast<float>(y()), static_cast<float>(other.y()));
}
}  // namespace Geometry2d
