#include <Geometry2d/CompositeShape.hpp>

namespace Geometry2d {

Shape* CompositeShape::clone() const { return new CompositeShape(*this); }

bool CompositeShape::containsPoint(Point pt) const {
    for (const auto& subshape : _subshapes) {
        if (subshape->containsPoint(pt)) {
            return true;
        }
    }
    return false;
}

bool CompositeShape::nearPoint(Point pt, float threshold) const {
    for (const auto& subshape : _subshapes) {
        if (subshape->nearPoint(pt, threshold)) {
            return true;
        }
    }
    return false;
}

void CompositeShape::add(const std::shared_ptr<Shape>& shape) {
    if (shape != nullptr) {
        _subshapes.push_back(shape);
    }
}

void CompositeShape::add(const CompositeShape& comp_shape) {
    for (const auto& shape : comp_shape.subshapes()) {
        add(shape);
    }
}

void CompositeShape::clear() { _subshapes.clear(); }

}  // namespace Geometry2d
