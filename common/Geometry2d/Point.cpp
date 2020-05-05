#include "Point.hpp"

#include "Util.hpp"

namespace Geometry2d {
bool Point::nearlyEquals(Point other) const {
    return nearlyEqual(static_cast<float>(x()),
                       static_cast<float>(other.x())) &&
           nearlyEqual(static_cast<float>(y()), static_cast<float>(other.y()));
}
}  // namespace Geometry2d
