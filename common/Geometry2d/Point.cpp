#include "Point.hpp"
#include "Util.hpp"

namespace Geometry2d {
bool Point::nearlyEquals(Point other) const {
    return nearlyEqual(x(), other.x()) && nearlyEqual(y(), other.y());
}
}
