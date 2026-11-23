#include "rj_geometry/stadium_shape.hpp"

#include <cassert>
#include <memory>
#include <vector>

#include "rj_geometry/circle.hpp"
#include "rj_geometry/point.hpp"
#include "rj_geometry/polygon.hpp"

namespace rj_geometry {

namespace {

// The four corners of the stadium's body, in boundary order: the rectangle
// whose long sides run c1->c2, offset by r on either side of that axis.
std::vector<Point> _body_vertices(Point c1, Point c2, float r) {
    Point cross_vec = (c2 - c1).normalized();
    Point normal_vec = Point(cross_vec.y(), -cross_vec.x());
    return {c1 + r * normal_vec, c2 + r * normal_vec, c2 - r * normal_vec, c1 - r * normal_vec};
}

}  // namespace

StadiumShape::StadiumShape(Point c1, Point c2, float r) {
    std::shared_ptr<Circle> endcap1 = std::make_shared<Circle>(c1, r);
    std::shared_ptr<Circle> endcap2 = std::make_shared<Circle>(c2, r);
    std::shared_ptr<Polygon> body = std::make_shared<Polygon>(_body_vertices(c1, c2, r));
    add(endcap1);
    add(endcap2);
    add(body);
}

void StadiumShape::pad(float padding) {
    std::shared_ptr<Circle> endcap1 = std::dynamic_pointer_cast<Circle>(subshapes_.at(0));
    std::shared_ptr<Circle> endcap2 = std::dynamic_pointer_cast<Circle>(subshapes_.at(1));
    std::shared_ptr<Polygon> body = std::dynamic_pointer_cast<Polygon>(subshapes_.at(2));
    assert(endcap1 != nullptr && endcap2 != nullptr && body != nullptr);

    // Use the setters to widen the radii.
    float r = endcap1->radius() + padding;
    endcap1->set_radius(r);
    endcap2->set_radius(r);

    // Rebuild the body from the endcap centers and the new radius.
    *body = Polygon(_body_vertices(endcap1->center(), endcap2->center(), r));
}

std::unique_ptr<StadiumShape> StadiumShape::clone() const {
    return std::unique_ptr<StadiumShape>(clone_impl());
}

StadiumShape* StadiumShape::clone_impl() const {
    return new StadiumShape(*this);
}

}  // namespace rj_geometry