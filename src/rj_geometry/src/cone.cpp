#include <rj_geometry/cone.hpp>

namespace rj_geometry {
Shape* Cone::clone() { return new Cone(*this); }
}  // namespace rj_geometry