#include <rj_geometry/composite_shape.hpp>

namespace rj_geometry {

Shape* CompositeShape::clone() const { return new CompositeShape(*this); }

bool CompositeShape::contains_point(Point pt) const {
    for (const auto& subshape : subshapes_) {
        if (subshape->contains_point(pt)) {
            return true;
        }
    }
    return false;
}

bool CompositeShape::near_point(Point pt, float threshold) const {
    for (const auto& subshape : subshapes_) {
        if (subshape->near_point(pt, threshold)) {
            return true;
        }
    }
    return false;
}

void CompositeShape::add(const std::shared_ptr<Shape>& shape) {
    if (shape != nullptr) {
        subshapes_.push_back(shape);
    }
}

void CompositeShape::add(const CompositeShape& comp_shape) {
    for (const auto& shape : comp_shape.subshapes()) {
        add(shape);
    }
}

void CompositeShape::clear() { subshapes_.clear(); }

}  // namespace rj_geometry
