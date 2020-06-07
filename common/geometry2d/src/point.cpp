#include <geometry2d/point.h>
#include <geometry2d/util.h>

namespace geometry2d {
bool Point::nearlyEquals(Point other) const {
    return nearlyEqual(static_cast<float>(x()),
                       static_cast<float>(other.x())) &&
           nearlyEqual(static_cast<float>(y()), static_cast<float>(other.y()));
}
}  // namespace geometry2d
