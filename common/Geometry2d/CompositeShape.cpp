#include "CompositeShape.hpp"

namespace Geometry2d {

Shape* CompositeShape::clone() const { return new CompositeShape(*this); }

bool CompositeShape::containsPoint(Point pt) const {
    for (auto subshape : _subshapes) {
        if (subshape->containsPoint(pt)) return true;
    }
    return false;
}

void CompositeShape::add(std::shared_ptr<Shape> shape) {
    if (shape) {
        _subshapes.push_back(shape);
    }
}

void CompositeShape::add(const CompositeShape& compShape) {
    for (auto shape : compShape.subshapes()) {
        add(shape);
    }
}

void CompositeShape::clear() { _subshapes.clear(); }

}  // namespace Geometry2d
